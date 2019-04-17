#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxAssimpModelLoader.h"
#include "ofxKinect.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
#include "CvUtils.h"

#include "bgParticle.h"

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		
    
    std::vector<glm::vec2> noiseOffset;
    float disperseControl = 5.0;
    std::vector<glm::vec2> psSpeed;

    
    
    
    ///--------- Contour Finder
    ofxCv::ContourFinder contourFinder;
    float hullMinumumDefectDepth = 10;
    // a rect for region of interest
    ofRectangle rect;
    // project model on center of hand
    glm::vec3 getContourCenter(std::size_t i);
    //glm::vec3 contourCenters;
    bool detectCountour = false;
    /// contourFinder for kinect, in ofxOpenCV
    ofxCvContourFinder contourFinderOpenCV;
    
    ///------- model & vertex setup
    ofxAssimpModelLoader model;
    void modelSpeedSetup();
    void modelLoad();
    ofMesh mesh;
    ofMesh newMesh;
    vector<glm::vec3> vertices;
    vector<glm::vec3> speed;
    vector<glm::vec3> offsetForVertex;
    int numVertices;
    float lineOpacity = 100;
    float changingScaler = 0;

    
        //------- change controller
        bool goDie;
        float randomOpacity;
        bool opacityDecrease = false;
        float opacityMin, opacityMax;
    
    //------- 10 second timer
    float timeInterval = 7.0;
    float currentTime = 0.0;
    
    //--------- core
    ofEasyCam cam;
    int randomNum = 0;
    bool isPressed = false;
    ofSoundPlayer first, second, bgm;
    bool isfin = false;
    bool reachTime = false;
    
    
    //---------- kinect
    
    ofxKinect kinect;
    
    ofxCvColorImage colorImg;
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
    
    bool bThreshWithOpenCV;
    bool bDrawPointCloud = false;
    
    int nearThreshold;
    int farThreshold;
    int angle;
    
    void drawPointCloud();
    
    //------- gui
    ofxPanel gui;
    ofParameter<float> pushScaler;
    ofParameter<float> fftMax;
    ofParameter<float> fftMin;
    ofParameter<float> timeScaleClick;
    ofParameter<float> smooth;
    ofParameter<float> micMax, micMin;
    
    ofxFloatSlider rSpeed;
    // kinect gui
    ofxPanel kinectGUI;
    ofParameter<float> nearThresholds, farThresholds;
    ofParameter<float> x, y, w, h;
    // controuFinder GUI
    ofxPanel contourFinderGUI;
    ofParameter<float> simplify;
    ofParameter<float> minAreaRadius, maxAreaRadius;
    ofParameter<float> tipSize;
    
    // particle system
    ofxPanel psGUI;
    ofParameter<float> particleSpeed;
    ofParameter<int> addParticles;
    ofParameter<float> bgParticleScaler;
    ofParameter<float> contourSize;
    ofParameter<float> psX, psY;
    ofParameter<float> transX, transY;
    ofParameter<float> meshSize, meshBig;
    ofParameter<float> handMicMin;
    ofParameter<float> mappedXmax, mappedYmax;


    ///------- for some audio-visuals on the background
    
    ///--- bg particles
    void lineBetween();
    vector <bgParticle> bgParticles;
    int particleNum;
    float range = 1000;
    float speedRange = 0.1;
    vector<glm::vec3> offsets;
    ///lineBetween setup
    float bglineOpacity = 100;
    float lineDisMin = 50;
    float lineDisMax = 70;
    ///control parameterf
    float bgtimeScaleClick = 0.5;
    
    //---  audio setup
    void audioIn(ofSoundBuffer & input);
    vector <float> left;
    vector <float> right;
    vector <float> volHistory;
    int 	bufferCounter;
    int 	drawCounter;
    float smoothedVol;
    float scaledVol;
    ofSoundStream soundStream;
    
    


};
