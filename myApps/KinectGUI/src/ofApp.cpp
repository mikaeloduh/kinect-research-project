#include "ofApp.h"


//--------------------------------------------------------------
void ofApp::setup()
{
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect_0.setRegistration(true);
    
	kinect_0.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect_0.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect_0.isConnected())
    {
		ofLogNotice() << "sensor-emitter dist: " << kinect_0.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect_0.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect_0.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect_0.getZeroPlaneDistance() << "mm";
	}
	
	// color; grayscale image 
	colorImg.allocate(kinect_0.width, kinect_0.height);
	grayImage.allocate(kinect_0.width, kinect_0.height);

	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle_0 = 0;
	kinect_1.setCameraTiltAngle(angle_0);

#ifdef USE_TWO_KINECTS
	kinect_1.setRegistration(true);
	kinect_1.init();
	kinect_1.open();
   	kinect_1.setCameraTiltAngle(angle_1);
    mesh_1.setMode(OF_PRIMITIVE_POINTS);
#endif
    
	// start from the front
	bDrawPointCloud = false;
    // start at mesh 0
    mode = 0;
}

//--------------------------------------------------------------
void ofApp::update()
{
	// set background
    ofBackground(100, 100, 100);
	
	kinect_0.update();
#ifdef USE_TWO_KINECTS
	kinect_1.update();
#endif
}

//--------------------------------------------------------------
void ofApp::draw()
{
	
	ofSetColor(255, 255, 255);
	
	if(bDrawPointCloud)
    {
		// start easy cam
		easyCam.begin();
        
        buildMesh(kinect_0, mesh_0);
        transfromMesh(mesh_0, roll_0, pitch_0, yaw_0, x_0, y_0, z_0);
        mesh_0.drawVertices();
        
#ifdef USE_TWO_KINECTS
        buildMesh(kinect_1, mesh_1);
        transfromMesh(mesh_1, roll_1, pitch_1, yaw_1, x_1, y_1, z_1);
        mesh_1.drawVertices();
#endif
        // draw axis at orgin
        ofDrawAxis(50);
		easyCam.end();
	}
    else
    {
		// draw from the live kinect
		//  color 0  |  color 1
		// --------------------
		//  depth 0  |  depth 1
		kinect_0.draw(10, 10, 400, 300);
		kinect_0.drawDepth(10, 320, 400, 300);		
#ifdef USE_TWO_KINECTS
		kinect_1.draw(420, 10, 400, 300);
        kinect_1.drawDepth(420, 320, 400, 300);
#endif
	}
	
	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
        
    if(kinect_1.hasAccelControl())
    {
        reportStream
        << "accel is: " << ofToString(kinect_0.getMksAccel().x, 2) << " / "
        << ofToString(kinect_0.getMksAccel().y, 2) << " / "
        << ofToString(kinect_0.getMksAccel().z, 2) << endl;
    }
    else
    {
        reportStream
        << "Note: this is a newer Xbox Kinect or Kinect For Windows device," << endl
		<< "motor / led / accel controls are not currently supported" << endl << endl;
    }
    
	reportStream
    << "You are using mesh no." << mode << endl
    << "press W S A D Q E to adjust the pitch, roll, and yaw" << endl
    << "press T G F H R Y to adjust the shift" << endl
    << "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect_0.isConnected() << endl
    << "press x to save the mesh" << endl;

    if(kinect_0.hasCamTiltControl())
    {
    	reportStream
        << "press UP and DOWN to change the Kinect no.1 tilt angle: " << angle_0 << " degrees" << endl
        << "press LEFT and RIGHT to change the tilt Kinect no.2 angle: " << angle_1 << " degrees" << endl
        << "press 1-5 & 0 to change the led mode" << endl;
    }
    
	ofDrawBitmapString(reportStream.str(), 20, 620);
    
}

// Construct a mesh from depth map
void ofApp::buildMesh(ofxKinect& kinect, ofMesh& mesh)
{
    mesh.clear();
    int step = 2;
    for(int y = 0; y < kinect.height; y += step)
    {
        for(int x = 0; x < kinect.width; x += step)
        {
            if(kinect.getDistanceAt(x, y) > 0)
            {
                mesh.addColor(kinect.getColorAt(x,y));
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));	// the orgin right front the depth camera
            }
        }
    }
}


// transfrom the mesh
void ofApp::transfromMesh(ofMesh& mesh, int& roll, int& pitch, int& yaw, int& x, int& y, int& z)
{
    vector< ofVec3f > vector = mesh.getVertices();
    
	// creat the transfromation matrix
    ofMatrix4x4 rotX, rotY, rotZ, trans;
    
    rotX.makeRotationMatrix(roll, ofVec3f(1, 0, 0));	// rotate with x-axis
    rotY.makeRotationMatrix(pitch, ofVec3f(0, 1, 0));	// rotate with y-axis
    rotZ.makeRotationMatrix(yaw, ofVec3f(0, 0, 1));		// rotate with z-axis
    trans.makeTranslationMatrix(x, y, z);	// shift
    
	// applied to each vertex..
    for (int i=0; i<vector.size(); i++)
    {
        vector[i] = vector[i] * rotX * rotY * rotZ * trans * ofVec3f(-1, -1, 1);
    }
    mesh.clearVertices();
    mesh.addVertices(vector);
}


// save mesh
void ofApp::saveMesh()
{
    mergeMesh(mesh_0, mesh_1);
    meshMerged.save(SAVEPATH"mergedMesh.ply", false);
    mesh_0.save(SAVEPATH"Mesh_0.ply", false);
    mesh_1.save(SAVEPATH"Mesh_1.ply", false);
}

// merge two mesh into one mesh
void ofApp::mergeMesh(const ofMesh& mesh1, const ofMesh& mesh2)
{
    vector< ofVec3f > vector1 = mesh1.getVertices();
    vector< ofVec3f > vector2 = mesh2.getVertices();
    vector< ofFloatColor > color1 = mesh1.getColors();
    vector< ofFloatColor > color2 = mesh2.getColors();
    
    meshMerged.addVertices(vector1);
    meshMerged.addColors(color1);
    meshMerged.addVertices(vector2);
    meshMerged.addColors(color2);
}

// Draw the axie at orgin
void ofApp::Axis()
{
    ofEnableDepthTest();
    ofLine(0, 0, 0, 100, 0, 0); // x-axie
    ofSetColor(255, 0, 0);
    ofSetLineWidth(3);
    ofLine(0, 0, 0, 0, 100, 0); // y-axie
    ofSetColor(0, 255, 0);
    ofSetLineWidth(3);
    ofLine(0, 0, 0, 0, 0, 100); // z-axie
    ofSetColor(0, 0, 255);
    ofSetLineWidth(3);
    ofDisableDepthTest();
}

//--------------------------------------------------------------
void ofApp::exit()
{
	kinect_0.setCameraTiltAngle(0); // zero the tilt on exit
	kinect_0.close();
	
#ifdef USE_TWO_KINECTS
    kinect_1.setCameraTiltAngle(0);
   	kinect_1.close();
#endif
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key)
{
	switch (key) {
			
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			break;
			
		case 'o':
			kinect_0.setCameraTiltAngle(angle_0); // go back to prev tilt
			kinect_0.open();
			break;
			
		case 'c':
			kinect_0.setCameraTiltAngle(0); // zero the tilt
			kinect_0.close();
			break;
			
		case '1':
			kinect_0.setLed(ofxKinect::LED_GREEN);
			break;
			
		case '2':
			kinect_0.setLed(ofxKinect::LED_YELLOW);
			break;
			
		case '3':
			kinect_0.setLed(ofxKinect::LED_RED);
			break;
			
		case '4':
			kinect_0.setLed(ofxKinect::LED_BLINK_GREEN);
			break;
			
		case '5':
			kinect_0.setLed(ofxKinect::LED_BLINK_YELLOW_RED);
			break;
			
		case '0':
			kinect_0.setLed(ofxKinect::LED_OFF);
			break;
			
		case OF_KEY_UP:
			angle_0++;
			if(angle_0>30) angle_0=30;
			kinect_0.setCameraTiltAngle(angle_0);
			break;
			
		case OF_KEY_DOWN:
			angle_0--;
			if(angle_0<-30) angle_0=-30;
			kinect_0.setCameraTiltAngle(angle_0);
			break;
            
#ifdef USE_TWO_KINECTS
        case OF_KEY_RIGHT:
			angle_1++;
			if(angle_1>30) angle_1=30;
			kinect_1.setCameraTiltAngle(angle_1);
			break;
			
		case OF_KEY_LEFT:
			angle_1--;
			if(angle_1<-30) angle_1=-30;
			kinect_1.setCameraTiltAngle(angle_1);
			break;
#endif
            
        case 'x':
            saveMesh();
        
        case OF_KEY_TAB:
            mode++;
            if(mode > 1)
                mode = 0;
            break;
            
        // adjust the mesh rotation
        case 'e':
            if (mode == 0)
                pitch_0++;
            else if (mode == 1)
                pitch_1++;
            break;
            
        case 'q':
            if (mode == 0)
                pitch_0--;
            else if (mode == 1)
                pitch_1--;
			break;
            
        case 'w':
            if (mode == 0)
                roll_0++;
            else if (mode == 1)
                roll_1++;
			break;
            
        case 's':
            if (mode == 0)
                roll_0--;
            else if (mode == 1)
                roll_1--;
            break;
            
        case 'd':
            if (mode == 0)
                yaw_0++;
            else if (mode == 1)
                yaw_1++;
			break;
            
        case 'a':
            if (mode == 0)
                yaw_0--;
            else if (mode == 1)
                yaw_1--;
			break;

        // Adjust the mesh position
        case 'h':
            if (mode == 0)
                x_0+=10;
            else if (mode == 1)
                x_1+=10;
            break;
            
        case 'f':
            if (mode == 0)
                x_0-=10;
            else if (mode == 1)
                x_1-=10;
			break;
            
        case 'r':
            if (mode == 0)
                y_0+=10;
            else if (mode == 1)
                y_1+=10;
			break;
            
        case 'y':
            if (mode == 0)
                y_0-=10;
            else if (mode == 1)
                y_1-=10;
            break;
            
        case 't':
            if (mode == 0)
                z_0+=10;
            else if (mode == 1)
                z_1+=10;
			break;
            
        case 'g':
            if (mode == 0)
                z_0-=10;
            else if (mode == 1)
                z_1-=10;
			break;
    }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{}