#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetLogLevel(OF_LOG_VERBOSE);
    ofBackground(50, 0);
    
    ofDisableArbTex(); // we need GL_TEXTURE_2D for our models coords.
    
    // model.loadModel("astroBoy_walk.dae", true);
    mesh.load(FILEPATH);

    
    mesh.setMode(OF_PRIMITIVE_POINTS);
}

//--------------------------------------------------------------
void ofApp::update(){


}

//--------------------------------------------------------------
void ofApp::draw(){
    ofSetColor(255);
    
    ofEnableBlendMode(OF_BLENDMODE_ALPHA);
    
	ofEnableDepthTest();
    
    glShadeModel(GL_SMOOTH); //some model / light stuff
    //light.enable();
    ofEnableSeparateSpecularLight();
    
    easyCam.begin();
    ofDrawAxis(50);
    ofPushMatrix();
    //ofTranslate(model.getPosition().x+100, model.getPosition().y, 0);
    //ofRotate(-mouseX, 0, 1, 0);
    //ofTranslate(-model.getPosition().x, -model.getPosition().y, 0);
    mesh.draw();
    ofPopMatrix();

    easyCam.end();
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}