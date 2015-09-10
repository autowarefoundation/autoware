#pragma once

#include "ofMain.h"
#include "pointcloud_map.h"
#include "ofxNetwork.h"
#include "ofxOculusDK2.h"
#include "vector_map.h"
#include "ofxAssimpModelLoader.h"


#define BUFSIZE 100000
#define PORT 59630

#define MAX_OFFSET 20
#define JUMP_TIME 2

//#define LOCAL "/data/log/moriyama_150630/udp.txt"        // Run without data transmission
//#define COLOR                                                     // Enable PCD map color
#define VECTOR "/data/data_moriyama_0.2/map/vector_map"  // Draw vector_map
#define CAR_LOC                                                   // Draw other car

typedef struct{
	ofColor color;
	ofVec3f pos;
	ofVec3f floatPos;
	float radius;
    bool bMouseOver;
    bool bGazeOver;
} DemoSphere;


typedef struct{
    float car_x;
    float car_y;
    float car_z;
    float roll;
    float pitch;
    float yaw;
    bool pose = true;
    vector<ofVec3f> vscan;
    vector<ofVec3f> oth;
    vector<ofVec3f> ped;
    vector<ofVec3f> obj;
} udparg;

class testApp : public ofBaseApp
{
  public:

	void setup();
    void setPointCloudMap();
	void update();
	void draw();

	void drawScene();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	ofxOculusDK2		oculusRift;

	ofLight				light;
	ofCamera			cam;
	bool showOverlay;
	bool predictive;
	vector<DemoSphere> demos;

    ofVec3f cursor2D;
    ofVec3f cursor3D;

    ofVec3f cursorRift;
    ofVec3f demoRift;

    ofVec3f cursorGaze;

    vector<PCDPoint> points;
    vector<PCDfile> file;

    float max_x;
    float min_x;
    float max_y;
    float min_y;
    float max_z;
    float min_z;
    float ma_x,ma_y,mi_x,mi_y,ma_z,mi_z;
    int totalPoints = 0;
    int localPoints = 0;
    int readingFileNum = 0;

    ofVec3f camPos;
    ofVec3f car;
    ofVec3f euler;

    float rotAng = 0;
    float fc = 0.0;
    float fclip;
    ifstream fin;
    string line;
    vector<string> lines, pieces;

    ofVboMesh mesh;
    ofVboMesh vs;

    ofCamera cam1,cam2,cam3;

    // Argument for udpThread
    udparg udp;

    int mode = 1;

    // For JUMP
    int count=0;
    int jump = 0;
    float offset = 0;
    float accel = 0;
    float velocity = 0;
    float initial_velocity = 0;
    float fps = 0;
    ofVec3f camera,OculusRPY;
    ofQuaternion OculusQuaternion;

    PointcloudMap pm;
    VectorMap vm;

    ofLight carlight;
    ofxAssimpModelLoader model;
    bool op = false;
    bool disp = true;
    bool mout = true;
    bool fill = false;

    ofLight sun;
};

int string2unixtime(string str);

void* pcdThread(void *);
void* udpThread(void *);
