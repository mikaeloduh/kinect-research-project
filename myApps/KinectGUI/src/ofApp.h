#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

// Windows users:
// You MUST install the libfreenect kinect drivers in order to be able to use
// ofxKinect. Plug in the kinect and point your Windows Device Manager to the
// driver folder in:
//
//     ofxKinect/libs/libfreenect/platform/windows/inf
//
// This should install the Kinect camera, motor, & audio drivers.
//
// You CANNOT use this driver and the OpenNI driver with the same device. You
// will have to manually update the kinect device to use the libfreenect drivers
// and/or uninstall/reinstall it in Device Manager.
//
// No way around the Windows driver dance, sorry.

// uncomment this to read from two kinects simultaneously
#define USE_TWO_KINECTS
#define SAVEPATH "./cloudFiles/"

class ofApp : public ofBaseApp {
public:
	
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	
	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	
    // My fucntion
    void buildMesh(ofxKinect& kinect, ofMesh& mesh) ;
    void transfromMesh(ofMesh& mesh, int& roll, int& pitch, int& Yaw,
                                int& x, int& y, int& z);
    
    void Axis();
    void saveMesh();

    void mergeMesh(const ofMesh& mesh1, const ofMesh& mesh2);
    
	ofxKinect kinect_0;	// kinect 0
    ofMesh mesh_0;		// mesh 0
    ofMesh meshMerged;	// merged mesh
	
	ofxCvColorImage colorImg;		// color image
	ofxCvGrayscaleImage grayImage;	// grayscale image
	bool bDrawPointCloud;
	
	int angle_0;	// tilt angle
    int mode;		// mesh number
    int roll_0, pitch_0, yaw_0, x_0, y_0, z_0;	// transform parameter
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect_1;	// kinect 1
    ofMesh mesh_1;		// mesh 1
    int angle_1;		// tilt angle
    int roll_1, pitch_1, yaw_1, x_1, y_1, z_1;	// transform parameter
#endif

	// used for viewing the point cloud
	ofEasyCam easyCam;
};
