#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#define FREEGLUT_STATIC 1

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <GL/glu.h>

// parse_line and trim
#include "helper_funcs.h"
#include "shaders_embedded.h"



#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <unistd.h> //read(), close()
#endif

#ifdef _WIN32
HANDLE hPipeReaderThread = NULL;
DWORD pipeReaderThreadId = 0;
#else
pthread_t pipeReaderThread;
#endif

// Atomic variable shared between background pipe_reader thread and OpenGL main thread
#include <atomic>
std::atomic<unsigned int> current_TID(0);

#include "cm/cm.h" //color_mapper
int compile_color_config(FILE* fp);
void map_color(Track* t);

#define SPEED 0.3f
#define SIDEBAR_WIDTH 270

typedef enum { CAM_FREECAM, CAM_ORBIT } CamMode;
CamMode currentCamMode = CAM_FREECAM;

//Camera variables
float camX = 0.0f, camY = 0.0f, camZ = 50.0f;
float camRoll = 0.0f, camPitch = 15.0f, camYaw = 0.0f;

//Specific Orbit mode variables
float orbitRadius = 50.0f;
float orbitAngle = 0.0f;     // Azimuth angle around the target in XY plane
float orbitPitch = 15.0f;    // Elevation angle above/below XY plane

// Smoothed target position for interpolation
float smoothTargetX = 0.0f, smoothTargetY = 0.0f, smoothTargetZ = 0.0f;
float targetX = 0.0f, targetY = 0.0f, targetZ = 0.0f;

int lastX = 0, lastY = 0;

int frameCount = 0;
float fps = 0.0f;
int previousTime = 0;
char fpsString[16] = "FPS: 0";

unsigned int rate_of_change = 1;
char rocString[32] = "Rate of Change: 1";

// Ambient Light Control Variable
float ambientIntensity = 0.2f;
char ambientString[32] = "Ambient Light Intensity: 0.2";

char TID_String[16] = "T_ID: 0";
char camStrPos[32] =     "Pos: (0.0, 0.0, 0.0)";
char camStrTarget[32] =  "Target: (0.0, 0.0, 0.0)";
char camStrYaw[32] =     "Yaw: 0.0°";
char camStrPitch[32] =   "Pitch: 0.0°";
char camStrTYaw[32] =    "Target Yaw: 0.0°";
char camStrTPitch[32] =  "Target Pitch: 0.0°";
char camStrDir[32] =     "Direction: (0.0, 0.0, 0.0)";
char camStrDis[32] =     "Distance: 0.0";
char camStrForward[32] = "Forward: (0.0, 0.0, 0.0)";

//----ui button coords-----//
rect btn_cam_mode = {0};

int buttonDown = 0;
int keys[256] = {0};

// Track data
Track* tracks = NULL;
unsigned int numTracks = 0;

Config config = {0};

// OpenGL / Instancing Handles
GLuint cylinderVBO = 0;
GLuint cylinderNormalVBO = 0;
GLuint instanceVBO = 0;
GLuint shaderProgram = 0;
int cylinderVertexCount = 0;

// Instance data structure for GPU
typedef struct {
    float start[3];
    float end[3];
    float radii[2]; // hradius, vradius
    float color[3];
} TrackInstanceData;

/************************** GLSL Shaders **************************************/
GLuint compileShader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    const char* typeStr = (type == GL_VERTEX_SHADER) ? "VERTEX" : "FRAGMENT";
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        fprintf(stderr, "ERROR: %s shader compilation failed:\n%s\n", typeStr, infoLog);
    } else {
        printf("SUCCESS: %s shader compiled successfully.\n", typeStr);
    }
    return shader;
}

void initShaders() {
    GLuint vs = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);

    shaderProgram = glCreateProgram();

    glAttachShader(shaderProgram, vs);
    printf("DEBUG: Vertex shader attached to program ID %u.\n", shaderProgram);

    glAttachShader(shaderProgram, fs);
    printf("DEBUG: Fragment shader attached to program ID %u.\n", shaderProgram);

    glLinkProgram(shaderProgram);

    GLint success;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
        fprintf(stderr, "ERROR: Shader program linking failed:\n%s\n", infoLog);
    } else {
        printf("SUCCESS: Shader program linked successfully (Program ID: %u).\n", shaderProgram);
    }

    glDeleteShader(vs);
    glDeleteShader(fs);
}

/************************** File / Str Functions **********************************/


//Reads uint32 values from stdin, and saves them to current_TID atomic.
#ifdef _WIN32
DWORD WINAPI pipe_reader_thread(LPVOID lpParam) {
#else
void* pipe_reader_thread(void* arg) {
#endif
    uint32_t received_value;

    // Read raw bytes from stdin file descriptor
    while (1) {
        ssize_t bytes_read;
#ifdef _WIN32
        DWORD total_read = 0;
        // Using ReadFile for Windows handles/stdin descriptor
        BOOL success = ReadFile(GetStdHandle(STD_INPUT_HANDLE), &received_value, sizeof(uint32_t), &total_read, NULL);
        if (!success || total_read == 0) { break; }
#else
        bytes_read = read(0, &received_value, sizeof(uint32_t));
        if (bytes_read <= 0) { break; }
#endif

        if (received_value < numTracks) {
            current_TID = received_value;
        } else {
            current_TID = numTracks - 1;
            fprintf(stderr, "[3D_Vis] Warning: Received out of bounds T_ID: %u, clamped to %u!\n",
              received_value, current_TID.load());
        }
        printf("[3D_Vis] Background thread received T_ID: %u\n", current_TID.load());
    }

    return 0;
}


void start_pipe_listener_thread() {
#ifdef _WIN32
    hPipeReaderThread = CreateThread(NULL, 0, pipe_reader_thread, NULL, 0, &pipeReaderThreadId);
    if (hPipeReaderThread == NULL) {
        fprintf(stderr, "Error: Failed to create Windows pipe reader thread.\n");
    }
#else
    int rc = pthread_create(&pipeReaderThread, NULL, pipe_reader_thread, NULL);
    if (rc != 0) {
        fprintf(stderr, "Error: Failed to create POSIX pthread.\n");
    }
#endif
}



//Close pipeReaderThread, when exiting the 3D_Vis Application
void close_window_callback(void) {
    static int is_closing = 0;
    if (is_closing) return; // Prevent re-entry
    is_closing = 1;

    printf("[3D_Vis] Window closed. Shutting down background pipe reader...\n");

    //Force close the underlying input file descriptor/handle
#ifdef _WIN32
    //TODO test on windows!
    HANDLE hStdin = GetStdHandle(STD_INPUT_HANDLE);
    if (hStdin != INVALID_HANDLE_VALUE) {
        CancelSynchronousIo(hPipeReaderThread);
        CloseHandle(hStdin);
    }
#else
    // Cancel the background thread blocked on read(0, ...)
    pthread_cancel(pipeReaderThread);
    close(0);
#endif

    //Join the background thread
#ifdef _WIN32
    if (hPipeReaderThread != NULL) {
        DWORD waitResult = WaitForSingleObject(hPipeReaderThread, 1000);
        if (waitResult == WAIT_TIMEOUT) {
            fprintf(stderr, "[3D_Vis] Warning: Pipe reader thread did not exit in time, terminating...\n");
            TerminateThread(hPipeReaderThread, 1);
        }
        CloseHandle(hPipeReaderThread);
        hPipeReaderThread = NULL;
    }
#else
    void* res;
    int rc = pthread_join(pipeReaderThread, &res);
    if (rc != 0 && rc != ESRCH) {
        fprintf(stderr, "[3D_Vis] Error: Failed to join POSIX pthread (code: %d).\n", rc);
    }
#endif

    printf("[3D_Vis] Exiting 3D_Visualizer...\n");
    exit(0);
}


//----------------

int read_config(const char* filename, Config* config) {
    printf("Opening %s...\n", filename);
    FILE* file = fopen(filename, "r");
    if (file == NULL) {
        fprintf(stderr, "Error: Could not open config file %s\n", filename);
        return FAILURE;
    }

    char line[512];
    while (fgets(line, sizeof(line), file)) {
        char* trimmed = trim(line);
        if (strlen(trimmed) == 0 || trimmed[0] == '/' || trimmed[0] == '#') {
            continue;
        }
        char* key;
        char* value;
        parse_line(trimmed, &key, &value);
        if (!key || !value) continue;

        if (strcmp(key, "tracks_to_plot") == 0) {
            strncpy(config->tracks_to_plot, value, sizeof(config->tracks_to_plot) - 1);
            config->tracks_to_plot[sizeof(config->tracks_to_plot) - 1] = '\0';
        } else if (strcmp(key, "horizontal_radius") == 0) {
            config->horizontal_radius = atof(value);
        } else if (strcmp(key, "vertical_radius") == 0) {
            config->vertical_radius = atof(value);
        } else if (strcmp(key, "debug_prints") == 0) {
            config->debug = atoi(value);
        }
    }
    fclose(file);
    return SUCCESS;
}

void read_track_list(const char* filename) {
    printf("Opening %s...\n", filename);
    FILE* file = fopen(filename, "r");
    if (!file) {
        printf("Failed to open %s\n", filename);
        exit(1);
    }

    const char* fname = "color.cmap";
    FILE* colorf = fopen(fname, "r");
    if(compile_color_config(colorf) == SUCCESS) {
      printf("Compiled %s successfully!\n",fname);
    } else {
      fprintf(stderr,"Failed to compile %s!\n",fname);
      //NOTE should we exit here?
      exit(1);
    }

    char line[256];
    fgets(line, sizeof(line), file); // Skip header

    int count = 0;
    //TODO Replace with fseek end, see NOTE
    while (fgets(line, sizeof(line), file)) count++;
    numTracks = count;
    tracks = (Track*)calloc(numTracks, sizeof(Track));
    rewind(file); //NOTE seek END and then rewind should be better
    fgets(line, sizeof(line), file); // Skip header again

    int i = 0;
    while (fgets(line, sizeof(line), file) && i < numTracks) {
        sscanf(line, "%u,%f,%f,%f, %f,%f,%f, %f,%f, %u, %u,%f, %f",
               &tracks[i].id,
               &tracks[i].start_x, &tracks[i].start_y, &tracks[i].start_z,
               &tracks[i].end_x, &tracks[i].end_y, &tracks[i].end_z,
               &tracks[i].b, &tracks[i].c,
               &tracks[i].g_code,
               &tracks[i].laser_status,  &tracks[i].laser_power,
               &tracks[i].machine_speed);
        tracks[i].hradius = config.horizontal_radius;
        tracks[i].vradius = config.vertical_radius;
        tracks[i].color = (Color){0,0,0};
        map_color(&tracks[i]); //NOTE Uses compiled color_mapper AST
        i++;
    }
    fclose(file);
    if (colorf) fclose(colorf);
}

/****************************************************************/

void initInstancedBuffers() {
    const int segments = 12;
    cylinderVertexCount = segments * 6;

    float* vertices = (float*)malloc(cylinderVertexCount * 3 * sizeof(float));
    float* normals = (float*)malloc(cylinderVertexCount * 3 * sizeof(float));

    int v_idx = 0;
    for (int i = 0; i < segments; i++) {
        float theta1 = (float)i * 2.0f * M_PI / segments;
        float theta2 = (float)(i + 1) * 2.0f * M_PI / segments;

        float x1 = cosf(theta1), y1 = sinf(theta1);
        float x2 = cosf(theta2), y2 = sinf(theta2);

        // Triangle 1
        vertices[v_idx*3 + 0] = x1; vertices[v_idx*3 + 1] = y1; vertices[v_idx*3 + 2] = 0.0f;
        normals[v_idx*3 + 0]  = x1; normals[v_idx*3 + 1]  = y1; normals[v_idx*3 + 2]  = 0.0f;
        v_idx++;
        vertices[v_idx*3 + 0] = x1; vertices[v_idx*3 + 1] = y1; vertices[v_idx*3 + 2] = 1.0f;
        normals[v_idx*3 + 0]  = x1; normals[v_idx*3 + 1]  = y1; normals[v_idx*3 + 2]  = 0.0f;
        v_idx++;
        vertices[v_idx*3 + 0] = x2; vertices[v_idx*3 + 1] = y2; vertices[v_idx*3 + 2] = 0.0f;
        normals[v_idx*3 + 0]  = x2; normals[v_idx*3 + 1]  = y2; normals[v_idx*3 + 2]  = 0.0f;
        v_idx++;

        // Triangle 2
        vertices[v_idx*3 + 0] = x2; vertices[v_idx*3 + 1] = y2; vertices[v_idx*3 + 2] = 0.0f;
        normals[v_idx*3 + 0]  = x2; normals[v_idx*3 + 1]  = y2; normals[v_idx*3 + 2]  = 0.0f;
        v_idx++;
        vertices[v_idx*3 + 0] = x1; vertices[v_idx*3 + 1] = y1; vertices[v_idx*3 + 2] = 1.0f;
        normals[v_idx*3 + 0]  = x1; normals[v_idx*3 + 1]  = y1; normals[v_idx*3 + 2]  = 0.0f;
        v_idx++;
        vertices[v_idx*3 + 0] = x2; vertices[v_idx*3 + 1] = y2; vertices[v_idx*3 + 2] = 1.0f;
        normals[v_idx*3 + 0]  = x2; normals[v_idx*3 + 1]  = y2; normals[v_idx*3 + 2]  = 0.0f;
        v_idx++;
    }

    // Base mesh buffers
    glGenBuffers(1, &cylinderVBO);
    glBindBuffer(GL_ARRAY_BUFFER, cylinderVBO);
    glBufferData(GL_ARRAY_BUFFER, cylinderVertexCount * 3 * sizeof(float), vertices, GL_STATIC_DRAW);

    glGenBuffers(1, &cylinderNormalVBO);
    glBindBuffer(GL_ARRAY_BUFFER, cylinderNormalVBO);
    glBufferData(GL_ARRAY_BUFFER, cylinderVertexCount * 3 * sizeof(float), normals, GL_STATIC_DRAW);

    free(vertices);
    free(normals);

    // Build Instance Data Array
    TrackInstanceData* instanceData = (TrackInstanceData*)malloc(numTracks * sizeof(TrackInstanceData));
    int validCount = 0;
    for (int i = 0; i < numTracks; i++) {
        instanceData[validCount].start[0] = tracks[i].start_x;
        instanceData[validCount].start[1] = tracks[i].start_y;
        instanceData[validCount].start[2] = tracks[i].start_z;

        instanceData[validCount].end[0] = tracks[i].end_x;
        instanceData[validCount].end[1] = tracks[i].end_y;
        instanceData[validCount].end[2] = tracks[i].end_z;

        if (tracks[i].color.r == 0 && tracks[i].color.g == 0 && tracks[i].color.b == 0) {
            instanceData[validCount].color[0] = 0.0f;
            instanceData[validCount].color[1] = 0.0f;
            instanceData[validCount].color[2] = 0.0f;

            //vertex shader will calculate a zero-sized cylinder,
            //GPU rasterizer will discard it
            //-> do not render tracks with color {0,0,0}
            instanceData[validCount].radii[0] = 0.0f;
            instanceData[validCount].radii[1] = 0.0f;
        } else {
            instanceData[validCount].color[0] = tracks[i].color.r / 255.0f;
            instanceData[validCount].color[1] = tracks[i].color.g / 255.0f;
            instanceData[validCount].color[2] = tracks[i].color.b / 255.0f;

            instanceData[validCount].radii[0] = tracks[i].hradius;
            instanceData[validCount].radii[1] = tracks[i].vradius;
        }
        validCount++;
    }

    glGenBuffers(1, &instanceVBO);
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    glBufferData(GL_ARRAY_BUFFER, numTracks * sizeof(TrackInstanceData), instanceData, GL_STATIC_DRAW);
    free(instanceData);
}

void cameraText(void)
{
    unsigned int cTID = current_TID.load();
    if (cTID >= numTracks) {cTID = numTracks-1;}

    // Vector from camera position to track 0's start point.
    float dx = tracks[cTID].start_x - camX;
    float dy = tracks[cTID].start_y - camY;
    float dz = tracks[cTID].start_z - camZ;

    float horizontal = sqrtf(dx * dx + dy * dy);
    float distance = sqrtf(dx * dx + dy * dy + dz * dz);

    // Z-up convention:
    // yaw = angle in XY plane, measured from +Y toward +X
    // pitch = elevation above the XY plane
    float targetYaw = atan2f(dx, dy) * 180.0f / (float)M_PI;
    float targetPitch = atan2f(dz, horizontal)
                      * 180.0f / (float)M_PI;

    // Direction vector from camera toward track start.
    float dirX = 0.0f, dirY = 0.0f, dirZ = 0.0f;
    if (distance > 1e-6f) {
        dirX = dx / distance;
        dirY = dy / distance;
        dirZ = dz / distance;
    }

    float yawRad = camYaw * (float)M_PI / 180.0f;
    float pitchRad = camPitch * (float)M_PI / 180.0f;

    // Forward vector consistent with yaw/pitch convention
    float forwardX = sinf(yawRad) * cosf(pitchRad);
    float forwardY = cosf(yawRad) * cosf(pitchRad);
    float forwardZ = sinf(pitchRad);

    snprintf(TID_String, sizeof(TID_String), "T_ID: %u", current_TID.load());
    snprintf(camStrPos, sizeof(camStrPos), "Pos: (%.2f, %.2f, %.2f)",
      camX, camY, camZ);
    snprintf(camStrTarget, sizeof(camStrTarget), "Target: (%.2f, %.2f, %.2f)",
      tracks[cTID].start_x,tracks[cTID].start_y,tracks[cTID].start_z);
    snprintf(camStrYaw, sizeof(camStrYaw), "Yaw: %.2f°",
      camYaw);
    snprintf(camStrPitch, sizeof(camStrPitch), "Pitch: %.2f°",
      camPitch);
    snprintf(camStrTYaw, sizeof(camStrTYaw), "Target Yaw: %.2f°",
      targetYaw);
    snprintf(camStrTPitch, sizeof(camStrTPitch), "Target Pitch: %.2f°",
      targetPitch);
    snprintf(camStrDir, sizeof(camStrDir), "Direction: (%.2f, %.2f, %.2f)",
      dirX, dirY, dirZ);
    snprintf(camStrDis, sizeof(camStrDis), "Distance: %.2f",
      distance);
    snprintf(camStrForward, sizeof(camStrForward), "Forward: (%.2f, %.2f, %.2f)",
      forwardX, forwardY, forwardZ);
}

// Render 2D UI Sidebar, with Text and Buttons
void renderUI() {
    //-------Setup-----------//
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    int width = glutGet(GLUT_WINDOW_WIDTH);
    int height = glutGet(GLUT_WINDOW_HEIGHT);
    gluOrtho2D(0, width, height, 0); //Origin->top left corner

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();


    //--------Sidebar Background Panel (Dark Gray)------//
    glColor3f(0.15f, 0.15f, 0.15f);
    glBegin(GL_QUADS);
        glVertex2f(0, 0);
        glVertex2f(SIDEBAR_WIDTH, 0);
        glVertex2f(SIDEBAR_WIDTH, height);
        glVertex2f(0, height);
    glEnd();

    //------------Debug-Text-------------//
    glColor3f(0.0f, 0.6f, 0.6f);
    // Debug Text in Sidebar
    int textY = 0;
    glRasterPos2i(10, textY + 20);
    for (char* c = fpsString; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    glRasterPos2i(10, textY + 40);
    for (char* c = rocString; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    glRasterPos2i(10, textY + 60);
    for (char* c = TID_String; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    glRasterPos2i(10, textY + 80);
    for (char* c = ambientString; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    glRasterPos2i(10, textY + 120);
    for (char* c = camStrPos; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    glRasterPos2i(10, textY + 140);
    for (char* c = camStrTarget; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    glRasterPos2i(10, textY + 160);
    for (char* c = camStrYaw; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    glRasterPos2i(10, textY + 180);
    for (char* c = camStrPitch; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    glRasterPos2i(10, textY + 200);
    for (char* c = camStrTYaw; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    glRasterPos2i(10, textY + 220);
    for (char* c = camStrTPitch; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    glRasterPos2i(10, textY + 240);
    for (char* c = camStrDir; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    glRasterPos2i(10, textY + 260);
    for (char* c = camStrDis; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    glRasterPos2i(10, textY + 280);
    for (char* c = camStrForward; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);

    //---------------Draw Button ("Toggle Camera Mode")----------------//
    int btnY = textY + 300;
    //vec2D Origin = {0, 300};
    btn_cam_mode.xleft = 10;
    btn_cam_mode.xright = 220;
    btn_cam_mode.yupper = btnY;
    btn_cam_mode.ylower = btnY + 40;
      //----Button Box ("Toggle Camera Mode")---//
      glColor3f(0.3f, 0.3f, 0.5f);
      glBegin(GL_QUADS);
          glVertex2f(btn_cam_mode.xleft,  btn_cam_mode.yupper);
          glVertex2f(btn_cam_mode.xright, btn_cam_mode.yupper);
          glVertex2f(btn_cam_mode.xright, btn_cam_mode.ylower);
          glVertex2f(btn_cam_mode.xleft,  btn_cam_mode.ylower);
      glEnd();

      //---Button Text ("Toggle Camera Mode")-//
      glColor3f(1.0f, 1.0f, 1.0f);
      uint16_t btn_c_height = btn_cam_mode.ylower - btn_cam_mode.yupper;
      glRasterPos2i(20, btnY + btn_c_height / 2);
      char btnText[64];
      snprintf(btnText, sizeof(btnText), "Mode: %s", (currentCamMode == CAM_FREECAM) ? "Freecam" : "Orbit");
      for (char* c = btnText; *c != '\0'; c++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
    //---------Clean up--------//
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glEnable(GL_DEPTH_TEST);
}


void display() {
    frameCount++;
    int currentTime = glutGet(GLUT_ELAPSED_TIME);
    int timeInterval = currentTime - previousTime;

    if (timeInterval > 1000) {
        fps = frameCount / (timeInterval / 1000.0f);
        previousTime = currentTime;
        frameCount = 0;
        snprintf(fpsString, sizeof(fpsString), "FPS: %.1f", fps);
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    int winWidth = glutGet(GLUT_WINDOW_WIDTH);
    int winHeight = glutGet(GLUT_WINDOW_HEIGHT);
    int renderWidth = winWidth - SIDEBAR_WIDTH;

    // ==========================================
    // Render 3D Scene, Restricted to Right Side
    // ==========================================
    // Set viewport only for the 3D portion
    glViewport(SIDEBAR_WIDTH, 0, renderWidth, winHeight);

    float aspect = (float)glutGet(GLUT_WINDOW_WIDTH) / (float)glutGet(GLUT_WINDOW_HEIGHT);

    // Projection Matrix (gluPerspective equivalent)
    float fov = 45.0f * M_PI / 180.0f;
    float f = 1.0f / tanf(fov / 2.0f);
    float zNear = 0.1f, zFar = 1000.0f;
    GLfloat projMat[16] = {
        f / aspect, 0.0f, 0.0f, 0.0f,
        0.0f, f, 0.0f, 0.0f,
        0.0f, 0.0f, (zFar + zNear) / (zNear - zFar), -1.0f,
        0.0f, 0.0f, (2.0f * zFar * zNear) / (zNear - zFar), 0.0f
    };

    // Calculate camera position based on mode
    unsigned int cTID = current_TID.load();
    if (cTID >= numTracks) cTID = numTracks - 1;
    targetX = tracks[cTID].start_x;
    targetY = tracks[cTID].start_y;
    targetZ = tracks[cTID].start_z;

    // Smoothly interpolate (lerp) current target toward the goal target
    // A factor around  0.05f provides a snappy yet smooth glide
    float lerpFactor = 0.001f; //TODO set this via plot.cconf
    smoothTargetX += (targetX - smoothTargetX) * lerpFactor;
    smoothTargetY += (targetY - smoothTargetY) * lerpFactor;
    smoothTargetZ += (targetZ - smoothTargetZ) * lerpFactor;

    if (currentCamMode == CAM_ORBIT) {
        float radAngle = orbitAngle * (float)M_PI / 180.0f;
        float radPitch = orbitPitch * (float)M_PI / 180.0f;

        // Spherical coordinates mapping relative to target vertex (XY plane base)
        camX = smoothTargetX + orbitRadius * cosf(radPitch) * sinf(radAngle);
        camY = smoothTargetY + orbitRadius * cosf(radPitch) * cosf(radAngle);
        camZ = smoothTargetZ + orbitRadius * sinf(radPitch);
    }

    glMatrixMode(GL_MODELVIEW); //NOTE why in the middle
    glLoadIdentity();

    if (currentCamMode == CAM_FREECAM) {
        float yawRad = camYaw * (float)M_PI / 180.0f;
        float pitchRad = camPitch * (float)M_PI / 180.0f;
        float forwardX = sinf(yawRad) * cosf(pitchRad);
        float forwardY = cosf(yawRad) * cosf(pitchRad);
        float forwardZ = sinf(pitchRad);

        gluLookAt(
            camX, camY, camZ,
            camX + forwardX, camY + forwardY, camZ + forwardZ,
            0.0f, 0.0f, 1.0f
        ); // Z is up
    } else {
        // Orbit mode: Look directly at the target vertex
        gluLookAt(
            camX, camY, camZ,
            smoothTargetX, smoothTargetY, smoothTargetZ,
            0.0f, 0.0f, 1.0f
        );
    }


    GLfloat viewMat[16];
    glGetFloatv(GL_MODELVIEW_MATRIX, viewMat);

    // Use Custom Shader Program
    glUseProgram(shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, projMat);
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, viewMat);
    glUniform1f(glGetUniformLocation(shaderProgram, "ambientIntensity"), ambientIntensity);

    // Bind Cylinder Mesh & Instance Attributes
    glEnableVertexAttribArray(0); // aPos
    glBindBuffer(GL_ARRAY_BUFFER, cylinderVBO);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    glEnableVertexAttribArray(1); // aNormal
    glBindBuffer(GL_ARRAY_BUFFER, cylinderNormalVBO);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

    // Instance attributes from instanceVBO
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    size_t stride = sizeof(TrackInstanceData);

    glEnableVertexAttribArray(2); // aStart
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
    glVertexAttribDivisor(2, 1);

    glEnableVertexAttribArray(3); // aEnd
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));
    glVertexAttribDivisor(3, 1);

    glEnableVertexAttribArray(4); // aRadii
    glVertexAttribPointer(4, 2, GL_FLOAT, GL_FALSE, stride, (void*)(6 * sizeof(float)));
    glVertexAttribDivisor(4, 1);

    glEnableVertexAttribArray(5); // aColor
    glVertexAttribPointer(5, 3, GL_FLOAT, GL_FALSE, stride, (void*)(8 * sizeof(float)));
    glVertexAttribDivisor(5, 1);

    // Draw all instances up to current_TID in a single call!
    unsigned int drawCount = (cTID < numTracks) ? cTID : numTracks - 1;
    if (drawCount > 0) {
        glDrawArraysInstanced(GL_TRIANGLES, 0, cylinderVertexCount, drawCount);
    }

    // Reset divisors
    glVertexAttribDivisor(2, 0);
    glVertexAttribDivisor(3, 0);
    glVertexAttribDivisor(4, 0);
    glVertexAttribDivisor(5, 0);

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
    glDisableVertexAttribArray(3);
    glDisableVertexAttribArray(4);
    glDisableVertexAttribArray(5);
    glUseProgram(0);


    //=================================================
    // Render UI sidebar overlay, Full Window Viewport
    //=================================================
    glViewport(0, 0, winWidth, winHeight); //Reset viewport to cover entire window
    cameraText();
    renderUI();

    glutSwapBuffers();
}

void reshape(int w, int h) {
      //Just trigger a redisplay; the viewports are handled in display()
      glutPostRedisplay();
}

// Handle Mouse Clicks for UI Buttons
void uiMouseHandler(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        // Check if click is inside the sidebar button bounds
        if (x >= btn_cam_mode.xleft &&  x <= btn_cam_mode.xright &&
            y >= btn_cam_mode.yupper && y <= btn_cam_mode.ylower) {

              if (currentCamMode == CAM_ORBIT) {
                // Switching from Orbit to Freecam:
                // Keep current camX, camY, camZ and synchronize Freecam Yaw,Pitch
                // so that we look directly at the smoothed target vertex.
                float dx = targetX - camX;
                float dy = targetY - camY;
                float dz = targetZ - camZ;

                float horizontal = sqrtf(dx * dx + dy * dy);
                camYaw = atan2f(dx, dy) * 180.0f / (float)M_PI;
                camPitch = atan2f(dz, horizontal) * 180.0f / (float)M_PI;

                currentCamMode = CAM_FREECAM;
                printf("[UI] Camera mode switched to: Freecam\n");

              } else{
                currentCamMode = CAM_ORBIT;
                printf("[UI] Camera mode switched to: Orbit\n");
              }

            glutPostRedisplay();
        }
    }
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        buttonDown = (state == GLUT_DOWN);
        lastX = x;
        lastY = y;
        uiMouseHandler(button, state, x, y);
    }
}

void mouseEntry(int state) {
    if (state == GLUT_ENTERED) {
        rate_of_change = 1;
        for (int i = 0; i < 256; i++) keys[i] = 0;
    }
}

void mouseWheel(int button, int direction, int x, int y) {
    float wheel_speed = SPEED * rate_of_change * 2.0f;

    // direction > 0 means scroll forward (wheel up), corresponding to 'w'
    // direction < 0 means scroll backward (wheel down), corresponding to 's'
    if (direction > 0) {
        if (currentCamMode == CAM_ORBIT) {
            orbitRadius -= wheel_speed;
            if (orbitRadius < 1.0f) orbitRadius = 1.0f;
        } else {
            // Freecam: scroll forward moves forward
            float yawRad = camYaw * M_PI / 180.0f;
            camX += sinf(yawRad) * wheel_speed;
            camY += cosf(yawRad) * wheel_speed;
        }
    } else {
        if (currentCamMode == CAM_ORBIT) {
            orbitRadius += wheel_speed;
        } else {
            // Freecam: scroll backward moves backward
            float yawRad = camYaw * M_PI / 180.0f;
            camX -= sinf(yawRad) * wheel_speed;
            camY -= cosf(yawRad) * wheel_speed;
        }
    }
    glutPostRedisplay();
}


void motion(int x, int y) {
    if (buttonDown) {
        if (currentCamMode == CAM_FREECAM) {
            camYaw += (x - lastX) * 0.1f;
            camPitch -= (y - lastY) * 0.1f;
            if (camPitch > 89.0f) camPitch = 89.0f;
            if (camPitch < -89.0f) camPitch = -89.0f;
        } else {
            // Orbit mode mouse panning / looking around target
            orbitAngle += (x - lastX) * 0.2f;
            orbitPitch += (y - lastY) * 0.2f;
            if (orbitPitch > 89.0f) orbitPitch = 89.0f;
            if (orbitPitch < -89.0f) orbitPitch = -89.0f;
        }
        lastX = x;
        lastY = y;
        glutPostRedisplay();
    }
}


void keyDown(unsigned char key, int x, int y) { keys[tolower(key)] = 1; }
void keyUp(unsigned char key, int x, int y) { keys[tolower(key)] = 0; }

void specialKeyDown(int key, int x, int y) {
    // mode keys
    if (key == GLUT_KEY_CTRL_L || key == GLUT_KEY_CTRL_R) {
        rate_of_change *= 100;
    }
    if (key == GLUT_KEY_SHIFT_L || key == GLUT_KEY_SHIFT_R) {
        rate_of_change *= 10;
    }

    // arrow keys
    unsigned int current_TID_changed = 0;

    if (key == GLUT_KEY_LEFT) {
        current_TID -= rate_of_change;
        if (numTracks <= current_TID) { // underflow feature
            current_TID = numTracks;
        }
        current_TID_changed = 1;
    } else if (key == GLUT_KEY_RIGHT) {
        current_TID += rate_of_change;
        if (numTracks <= current_TID) { // overflow feature
            current_TID = 0;
        }
        current_TID_changed = 1;
    }

    if (current_TID_changed) {
        snprintf(TID_String, sizeof(TID_String), "T_ID: %u", current_TID.load());
    }
}

void specialKeyUp(int key, int x, int y) {
    // mode keys
    if (key == GLUT_KEY_CTRL_L || key == GLUT_KEY_CTRL_R) {
        rate_of_change /= 100;
    }
    if (key == GLUT_KEY_SHIFT_L || key == GLUT_KEY_SHIFT_R) {
        rate_of_change /= 10;
    }

    if (rate_of_change < 1) { rate_of_change = 1; } // lower clamp to 1
}

void handle_movement(int garbage) {
    float speed = SPEED * rate_of_change;

    if (currentCamMode == CAM_FREECAM) {
        float yawRad = camYaw * M_PI / 180.0f;
        float forwardX = sinf(yawRad), forwardY = cosf(yawRad);
        float rightX = cosf(yawRad), rightY = -sinf(yawRad);

        if (keys['w']) { camX += forwardX * speed; camY += forwardY * speed; }
        if (keys['s']) { camX -= forwardX * speed; camY -= forwardY * speed; }
        if (keys['a']) { camX -= rightX * speed; camY -= rightY * speed; }
        if (keys['d']) { camX += rightX * speed; camY += rightY * speed; }
        if (keys[' ']) { camZ += speed; }
        if (keys['c']) { camZ -= speed; }
    } else { //CAM_ORBIT
        //TODO set sensitivity via plot.cconf or sensitivity sliders
        //-> perhaps save sens in plot.cconf and adjust them live via sliders
        float orbit_speed_factor = 1.0;
        if (keys['w']) { orbitRadius -= speed * orbit_speed_factor;
        if(orbitRadius < 1.0f) {orbitRadius = 1.0f;}}
        if (keys['s']) { orbitRadius += speed * orbit_speed_factor; }
        // A/D move the camera along the orbit circle around the target vertex
        if (keys['a']) { orbitAngle -= speed * 2.0f; }
        if (keys['d']) { orbitAngle += speed * 2.0f; }
        // SPACE/C can be used to elevate the orbit plane or angle relative to the XY plane
        if (keys[' ']) { orbitPitch += speed * 2.0f; if (orbitPitch > 89.0f) {orbitPitch = 89.0f;}}
        if (keys['c']) { orbitPitch -= speed * 2.0f; if (orbitPitch < -89.0f) {orbitPitch = -89.0f;}}
    }

    unsigned int ambient_changed = 0;
    if (keys['i']) { ambientIntensity += 0.01f; if (ambientIntensity > 1.0f) ambientIntensity = 1.0f; ambient_changed = 1; }
    else if (keys['k']) { ambientIntensity -= 0.01f; if (ambientIntensity < 0.0f) ambientIntensity = 0.0f; ambient_changed = 1; }

    if (ambient_changed) {
        snprintf(ambientString, sizeof(ambientString), "Ambient Light Intensity: %.2f", ambientIntensity);
    }

    snprintf(rocString, sizeof(rocString), "Rate of Change: %u", rate_of_change);

    glutPostRedisplay();
    glutTimerFunc(16, handle_movement, 0);
}

int main(int argc, char** argv) {
    read_config("plot.cconf", &config);
    read_track_list(config.tracks_to_plot);

    // Initialize camera facing track 0's start point
    //TODO make this work
    if (numTracks > 0) {
            targetX = tracks[0].start_x;
            targetY = tracks[0].start_y;
            targetZ = tracks[0].start_z;
            smoothTargetX = targetX;
            smoothTargetY = targetY;
            smoothTargetZ = targetZ;
            camX = targetX;
            camY = targetY - 50.0f;
            camZ = targetZ + 10.0f;
            orbitRadius = 50.0f;
            orbitAngle = 0.0f;
            orbitPitch = 15.0f;
        }

    glutInit(&argc, argv);
    glutSetOption(
        GLUT_ACTION_ON_WINDOW_CLOSE,
        GLUT_ACTION_EXIT);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow("3D Track Visualizer - Instanced");

    if (glewInit() != GLEW_OK) {
        printf("GLEW initialization failed\n");
        return 1;
    }
    const GLubyte* version = glGetString(GL_VERSION);
    printf("OpenGL Version: %s\n", version);

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    // Initialize Shaders and GPU Instanced Buffers
    initShaders();
    initInstancedBuffers();

    //Start the background thread to listen to the Dashboard via stdin pipe
    start_pipe_listener_thread();

    //register callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutEntryFunc(mouseEntry);
    glutMouseFunc(mouse);
    glutMouseWheelFunc(mouseWheel);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyDown);
    glutKeyboardUpFunc(keyUp);
    glutSpecialFunc(specialKeyDown);
    glutSpecialUpFunc(specialKeyUp);
    glutTimerFunc(0, handle_movement, 0);
    glutCloseFunc(close_window_callback);
    glutMainLoop();

    return 0;
}
