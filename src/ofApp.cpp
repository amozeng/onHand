#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    ofBackground(0);
    
    ///--------- Contour Finder ofxCv
    
    contourFinder.setMinAreaRadius(80);
    contourFinder.setMaxAreaRadius(1000);
    contourFinder.setSimplify(true);
    
    ///------- bg particles setup
    particleNum = 100;
    bgParticles.assign(particleNum, bgParticle());
    
    //assign each point a random position
    for(int i = 0; i < bgParticles.size(); i++ ){
        bgParticles[i].position = glm::vec3(ofRandom(-range/2, range/2), ofRandom(-range/2, range/2), ofRandom(-range/2, range/2));
        //        bgParticles[i].position = glm::vec3(0,0,0);
        bgParticles[i].velocity = glm::vec3(ofRandom(-speedRange, speedRange),ofRandom(-speedRange, speedRange),ofRandom(-speedRange, speedRange));
        //bgParticles[i].velocity = glm::vec3(0,0,0);
        
        bgParticles[i].randomSphereSize = ofRandom(0.3, 1.0);
    }
        //------- bg particle noise offset
        for (int i = 0; i < bgParticles.size(); ++i) {
            offsets.push_back(ofVec3f(ofRandom(0,100000), ofRandom(0,100000), ofRandom(0,100000)));
        }



    //--------- gui
    gui.setup();
    kinectGUI.setName("sound&mic");
    gui.add(pushScaler.set("pushScaler", 0.50, 0.05, 100.00));
    gui.add(fftMax.set("fftMax", 0.700, 0.200, 0.900));
    gui.add(fftMin.set("fftMin", 0.005, 0.001, 0.600));
    gui.add(timeScaleClick.set("timeScaleClick", 2.00, -2.0, 5.0));
    gui.add(rSpeed.setup("rSpeed", 2, 0, 100));
    gui.add(micMax.set("micMax", 10, 0.1, 25));
    gui.add(micMin.set("micMin", 0.001, 0.01, 5));
    gui.add(smooth.set("smooth", 4.0, 0.0, 32.0));
    
    kinectGUI.setup();
    kinectGUI.setName("kinect");
    kinectGUI.add(nearThresholds.set("nearThresholds", 80, 0, 150));
    kinectGUI.add(farThresholds.set("farThresholds", 160, 0, 255));
    kinectGUI.add(x.set("x", 68, 0, 400));
    kinectGUI.add(y.set("y", 92, 0, 400));
    kinectGUI.add(w.set("w", 330, 0, 1500));
    kinectGUI.add(h.set("h", 252, 0, 800));
    
    contourFinderGUI.setup();
    contourFinderGUI.setName("contourFinder");
    contourFinderGUI.add(simplify.set("simplify", 30.0, 0.0, 40.0));
    contourFinderGUI.add(minAreaRadius.set("minAreaRadius", 10.0, 1.0, 100.0));
    contourFinderGUI.add(maxAreaRadius.set("maxAreaRadius", 500.0, 100.0, 1000.0));
    contourFinderGUI.add(tipSize.set("tipSize", 100, 10, 200));
    
    
    psGUI.setup();
    psGUI.setName("ps");
    psGUI.add(particleSpeed.set("particleSpeed", 0.5, 0.1, 10.0));
    psGUI.add(addParticles.set("addParticles", 20, 1, 50));
    psGUI.add(bgParticleScaler.set("bgParticleScaler", 1.0, 1.0, 50));
    psGUI.add(contourSize.set("contourSize", 6.250, 1.0, 10.0));
    psGUI.add(psX.set("psX", -90, -1000, 1000));
    psGUI.add(psY.set("psY", -130, -1000, 1000));
    psGUI.add(transX.set("transX", -1096, -1600, -800));
    psGUI.add(transY.set("transY", -680, -1000, 1000));
    psGUI.add(meshSize.set("meshSize", 4.51, 1.0, 10.0));
    psGUI.add(meshBig.set("meshBig", 0.1, 0.05, 0.3));
    psGUI.add(handMicMin.set("handMicMin", 0.01, 0.001, 0.1));
    psGUI.add(mappedXmax.set("mappedXmax", 1368, 1000, 1800));
    psGUI.add(mappedYmax.set("mappedYmax", 614, 400, 800));






    ///------- microphone
    int bufferSize = 256;
    
    left.assign(bufferSize, 0.0);
    right.assign(bufferSize, 0.0);
    volHistory.assign(400, 0.0);
    
    bufferCounter	= 0;
    drawCounter		= 0;
    smoothedVol     = 0.0;
    scaledVol		= 0.0;
    
    
    ofSoundStreamSettings settings;
    
    // if you want to set the device id to be different than the default
    // auto devices = soundStream.getDeviceList();
    // settings.device = devices[4];
    
    // you can also get devices for an specific api
    // auto devices = soundStream.getDevicesByApi(ofSoundDevice::Api::PULSE);
    // settings.device = devices[0];
    
    // or get the default device for an specific api:
    // settings.api = ofSoundDevice::Api::PULSE;
    
    // or by name
    auto devices = soundStream.getMatchingDevices("default");
    if(!devices.empty()){
        settings.setInDevice(devices[0]);
    }
    
    settings.setInListener(this);
    settings.sampleRate = 44100;
    settings.numOutputChannels = 0;
    settings.numInputChannels = 2; /// here has to be "2"
    settings.bufferSize = bufferSize;
    
    soundStream.setup(settings);

    //soundStream.setup(this, 0, 1, 44100, bufferSize, 4);
    //(ofBaseApp * app, int outChannels, int inChannels, int sampleRate, int bufferSize, int nBuffers)
    
//    soundStream.printDeviceList();
//    
//    auto buffer = ofBufferFromFile("sound_channel.txt");
//    auto line = buffer.getLines().begin();
//    int deviceNumber = ofToInt(ofTrim(*line));
//    
//    std::cout << "Using device: " << deviceNumber << std::endl;
    
    
    

    ///---------- kinect
    
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    // enable depth->video image calibration
    kinect.setRegistration(true);
    
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    
    kinect.open();		// opens first available kinect
    //kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    colorImg.allocate(kinect.width, kinect.height);
    grayImage.allocate(kinect.width, kinect.height);
    grayThreshNear.allocate(kinect.width, kinect.height);
    grayThreshFar.allocate(kinect.width, kinect.height);
    
    nearThreshold = 10;
    farThreshold = 178;
    bThreshWithOpenCV = true;
    
    ofSetFrameRate(60);
    
    // zero the tilt on startup
    angle = 10;
    kinect.setCameraTiltAngle(angle);
    
    // start from the front
    bDrawPointCloud = false;
    
    //------- load bell ring
    first.load("first.mp3");
    second.load("second.mp3");
    bgm.load("bgm.mp3");
    bgm.setVolume(0.5);
    
    ///------- model
    model.loadModel("apple.obj", 20);
    goDie = false;
    randomOpacity = ofRandom(200, 255);
    opacityMin = 100;
    opacityMax = 255;

}

//--------------------------------------------------------------
void ofApp::update(){
    
    ofSoundUpdate();
    
    //lets scale the vol up to a 0-1 range
    scaledVol = ofMap(smoothedVol, 0.0, 0.17, 0.0, smooth, true);
    
    //lets record the volume into an array
    volHistory.push_back( scaledVol );
    
    //if we are bigger the the size we want to record - lets drop the oldest value
    if( volHistory.size() >= 400 ){
        volHistory.erase(volHistory.begin(), volHistory.begin()+1);
    }
    float mappedMic = ofMap(scaledVol, micMin, 2, 1, micMax, true);
    
    // data for DEBUG
//    cout << "smoothedVol: " << smoothedVol << endl;
//    cout << "scaledVol: " << scaledVol << endl;
//    cout << "mappedMic: " << mappedMic << endl;
    
    // update contourFinder variables
    contourFinder.setMinAreaRadius(minAreaRadius);
    contourFinder.setMaxAreaRadius(maxAreaRadius);
    
    ///------- Control of vertex opacity
    if(opacityDecrease){
        if(opacityMin > 0) {
            opacityMin -= 2;
        }
        if(opacityMax > 0) {
            opacityMax -= 2;
        }
        if(lineOpacity > 0){
            lineOpacity -= 1;
        }
    }
    
    ///------ background particles
    for(int i = 0; i < bgParticles.size(); i++)
    {
        // update each particle
        bgParticles[i].update();
        float time = ofGetElapsedTimef();
        
        glm::vec3 timeOffsets = offsets[i];
        
        // make each particle jitter
        
        bgParticles[i].position.x += ofSignedNoise(time * bgtimeScaleClick + timeOffsets.x) * (mappedMic * bgParticleScaler);
        bgParticles[i].position.y += ofSignedNoise(time * bgtimeScaleClick + timeOffsets.x) * (mappedMic * bgParticleScaler);
        bgParticles[i].position.z += ofSignedNoise(time * bgtimeScaleClick + timeOffsets.z) * (mappedMic * bgParticleScaler);
    }

    
    ///------ model
    
    //!check we do load the model or not
    float time = ofGetElapsedTimef();
    
    if(randomNum < 3){
        changingScaler = 0.5;
    }else{
        changingScaler = handMicMin;
    }
    
    if(model.getNumMeshes() > 0){
        
        //1. for model dots
        for(int i = 0; i < vertices.size(); i++ ){
            vertices[i].x += (changingScaler * mappedMic) * ofSignedNoise(time * 1 + offsetForVertex[i].x) ;
            vertices[i].y += (changingScaler * mappedMic) * ofSignedNoise(time * 1+ offsetForVertex[i].y) ;
            vertices[i].z += (changingScaler * mappedMic) * ofSignedNoise(time * 1 + offsetForVertex[i].z) ;
        }
        
        //2. for lines
        for(int i = 0; i < numVertices; i++ ){
            mesh.setVertex(i, vertices[i]);
        }
        
    }

    
    ///------- kinect
    kinect.update();
    
    if(kinect.isFrameNew()) {
        
        // load grayscale depth image from the kinect source
        grayImage.setFromPixels(kinect.getDepthPixels());
        
        // we do two thresholds - one for the far plane and one for the near plane
        // we then do a cvAnd to get the pixels which are a union of the two thresholds
        if(false){
            //            bThreshWithOpenCV) {
            grayThreshNear = grayImage;
            grayThreshFar = grayImage;
            grayThreshNear.threshold(nearThresholds, true);
            grayThreshFar.threshold(farThresholds);
            cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
        } else {
            
            // or we do it ourselves - show people how they can work with the pixels
            ofPixels & pix = grayImage.getPixels();
            int numPixels = pix.size();
            
            rect.set(x,y,w,h);
            
            // Region of Interest
            for(std::size_t y= 0; y < grayImage.getHeight(); y++){
                for(std::size_t x = 0; x < grayImage.getWidth(); x++) {
                    if (!rect.inside(x, y))
                    {
                        pix.setColor(x, y, 255);
                    }
                }
            }
            
            for(int i = 0; i < numPixels; i++) {
                if(255 - pix[i] > nearThresholds && 255 - pix[i] < farThresholds) {
                    pix[i] = 255;
                } else {
                    pix[i] = 0;
                }
            }
        }
        
        grayImage.flagImageChanged();
        contourFinder.findContours(grayImage);
        contourFinderOpenCV.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
    }
    
    // update each countour
    for (auto contourIndex = 0; contourIndex < contourFinder.size(); ++contourIndex)
    {
        ofPolyline contour = contourFinder.getPolylines()[contourIndex];
        ConvexHull convexHull(contour, hullMinumumDefectDepth);
        ofPolyline convexHullSmoothed = convexHull.convexHull();
        convexHullSmoothed.simplify(simplify);
    }
    
    // if new contour detect, ring the bell
    if (!detectCountour && contourFinder.size() > 0)
    {
        first.play(); // play bell sound
        
        // random load a new model
        float randomFloat = ofRandom(1,10);
        randomNum = int(randomFloat);
        if(randomNum <= 7){
            modelLoad();
            opacityMin = 100;
            opacityMax = 255;
            lineOpacity = 100;
            opacityDecrease = false;
        }else{
            opacityMin = 0;
            opacityMax = 0;
            lineOpacity = 0;
        }
        
        /// model starts to vanish after 7 seconds
        currentTime = ofGetElapsedTimef();
        cout << "randomNum: " << randomNum << endl;
    }
    
    /// model vanishs after 7 seconds
    if(!reachTime && ((timeInterval < ofGetElapsedTimef() - currentTime) && ofGetElapsedTimef() - currentTime < 10)) {
        second.play();
        goDie = true;
        opacityDecrease = true;
    }
    if(ofGetElapsedTimef() - currentTime > timeInterval){
        reachTime = true;
    }else{
        reachTime = false;
    }
    /// END: model vanishs after 7 seconds
    
    if (contourFinder.size() > 0)
    {
        detectCountour = true;
        ofPolyline ourBiggestContour = contourFinder.getPolyline(0);
        
    }else{
        detectCountour = false;
    }

    

}

//--------------------------------------------------------------
void ofApp::draw(){
    
    ofEnableDepthTest();
    cam.begin();
    
    if(bDrawPointCloud) {
        drawPointCloud();
    }
    
    //3. model
    //model dots
    if(model.getNumMeshes() > 0){
        if (randomNum > 2) {
            
            for (auto contourIndex = 0; contourIndex < contourFinder.size(); ++contourIndex)
            {
                ofPushMatrix(); //transfer

                    /// translate model's pos to hand center
                    glm::vec3 contourCenters = ofApp::getContourCenter(contourIndex);
    //                ofScale(meshSize,-meshSize,meshSize);
                    ofScale(1,-1,1);
                    float mappedX = ofMap(contourCenters.x, 0, 300, 0, mappedXmax);
                    float mappedY = ofMap(contourCenters.y, 0, 200, 0, mappedYmax);


                    ofTranslate(mappedX + transX, mappedY + transY);

                    ofPushMatrix(); // scale
                        ofScale(meshBig);

                        for (int i = 0; i < vertices.size(); i++)
                        {
                            randomOpacity = ofRandom(opacityMin, opacityMax);
                            if(goDie){
                                vertices[i] = vertices[i] + speed[i];
                            }
                            ofPushMatrix(); //dot
                            ofSetColor(255, randomOpacity);
                            ofRotate(ofGetElapsedTimef() * rSpeed, 0, 1, 0);
                            ofDrawSphere(vertices[i].x, vertices[i].y, vertices[i].z, 1);
                            ofPopMatrix(); //dot
                        }
                        
                        //model frame
                        ofPushMatrix(); //frame
                        ofRotate(ofGetElapsedTimef() * rSpeed, 0, 1, 0);
                        ofSetColor(255, lineOpacity);
                        mesh.drawWireframe();
                        ofPopMatrix(); //frame
                    ofPopMatrix(); // scale
                ofPopMatrix(); //transfer


            }
        }else{
            for (int i = 0; i < vertices.size(); i++)
            {
                randomOpacity = ofRandom(opacityMin, opacityMax);
                if(goDie){
                    vertices[i] = vertices[i] + speed[i];
                }
                ofPushMatrix();
                ofSetColor(255, randomOpacity);
                ofRotate(ofGetElapsedTimef() * rSpeed, 0, 1, 0);
                ofDrawSphere(vertices[i].x, vertices[i].y, vertices[i].z, 1);
                ofPopMatrix();
            }
            
            //model frame
            ofPushMatrix();
            ofRotate(ofGetElapsedTimef() * rSpeed, 0, 1, 0);
            ofSetColor(255, lineOpacity);
            mesh.drawWireframe();
            ofPopMatrix();
            
        }
    }
    cam.end();

    ofPushStyle();
    ofSetColor(255);
    for (auto contourIndex = 0; contourIndex < contourFinder.size(); ++contourIndex)
    {
        const ofPolyline contour = contourFinder.getPolylines()[contourIndex];
        
        ConvexHull convexHull(contour, hullMinumumDefectDepth);
        
        ofPolyline convexHullSmoothed = convexHull.convexHull();
        convexHullSmoothed.simplify(simplify);
        
        // get center of the coutour to project
        glm::vec3 contourCenters = ofApp::getContourCenter(contourIndex);
        
        // for DEBUG: draw center of contour
        //ofSetColor(255, 255, 0, 200); //yellow
        //ofDrawCircle(contourCenters, 30);
    }
    ofPopStyle();
    
    ofSetColor(255, 255, 255);
    
    ///------ bg particles
    ofPushMatrix();
    ofTranslate(ofGetWidth()/2, ofGetHeight()/2);
    ofNoFill();
    ofSetColor(255,200);
    for(int i = 0; i < bgParticles.size(); i++)
    {
        bgParticles[i].draw();
    }
    lineBetween();
    ofPopMatrix();


    ///------- gui for MAPPING and DEBUG
//    ofDisableDepthTest();
//    float guiY = 600;
//    gui.setPosition(0,guiY);
//    gui.draw();
//    kinectGUI.setPosition(gui.getWidth()+10, guiY);
//    kinectGUI.draw();
//    contourFinderGUI.setPosition(kinectGUI.getPosition().x + kinectGUI.getWidth() + 10, guiY);
//    contourFinderGUI.draw();
//    psGUI.setPosition(contourFinderGUI.getPosition().x + contourFinderGUI.getWidth() + 10, guiY);
//    psGUI.draw();


}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

void ofApp::drawPointCloud() {
    int w = 640;
    int h = 480;
    ofMesh mesh;
    mesh.setMode(OF_PRIMITIVE_POINTS);
    int step = 2;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
            if(kinect.getDistanceAt(x, y) > 0) {
                mesh.addColor(kinect.getColorAt(x,y));
                mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
            }
        }
    }
    glPointSize(3);
    ofPushMatrix();
    // the projected points are 'upside down' and 'backwards'
    ofScale(1, -1, -1);
    ofTranslate(0, 0, -1000); // center the points a bit
    ofEnableDepthTest();
    mesh.drawVertices();
    ofDisableDepthTest();
    ofPopMatrix();
}

glm::vec3 ofApp::getContourCenter(std::size_t i){
    
    ofPolyline _contour = contourFinder.getPolylines()[i];
    glm::vec3 center = _contour.getCentroid2D();
    return center;
}


void ofApp::audioIn(ofSoundBuffer & input){
    
    float curVol = 0.0;
    // samples are "interleaved"
    int numCounted = 0;
        //lets go through each sample and calculate the root mean square which is a rough way to calculate volume
    for (int i = 0; i < input.getNumFrames(); i++){
        left[i]		= input[i*2]*0.5;
        right[i]	= input[i*2+1]*0.5;
        
        curVol += left[i] * left[i];
        curVol += right[i] * right[i];
        numCounted+=2;
    }
    
    //this is how we get the mean of rms :)
    curVol /= (float)numCounted;
    
    // this is how we get the root of rms :)
    curVol = sqrt( curVol );
    
    smoothedVol *= 0.93;
    smoothedVol += 0.07 * curVol;
    
    bufferCounter++;
    
}

void ofApp::modelSpeedSetup(){
    mesh = model.getMesh(0);
    vertices = mesh.getVertices();
    numVertices = mesh.getNumVertices();
    for (int i = 0; i < numVertices; ++i) {
        speed.push_back(glm::vec3(ofRandom(-1,1), ofRandom(-1,1), ofRandom(-1,1)));
        speed.back() *= 0.5;
    }
}

void ofApp::modelLoad(){
    
    //1. hana
    if(randomNum == 1){
        model.loadModel("hana-floor.obj", 20);
    }else if (randomNum == 2){
        model.loadModel("flower-floor.obj", 20);
    }else if (randomNum == 3){
        model.loadModel("deer-hand.obj", 20);
    }else if (randomNum == 4){
        model.loadModel("flower-hand.obj", 20);
    }else if (randomNum == 5){
        model.loadModel("plane-hand.obj", 20);
    }else if (randomNum == 6){
        model.loadModel("hana-hand.obj", 20);
    }else if (randomNum == 7){
        model.loadModel("apple.obj", 20);
    }
    //! check load file or not
    if(model.getNumMeshes() > 0){
        model.setRotation(0, 90, 1, 0, 0);
        model.setRotation(0, 90, 0, 1, 0);
        model.setPosition(ofGetWidth()/2, ofGetHeight()/2 + 300, 0);
        numVertices = mesh.getNumVertices();
        modelSpeedSetup();
        for(int i = 0 ; i < vertices.size(); i++){
            offsetForVertex.push_back(glm::vec3(ofRandom(0,100000),ofRandom(0,100000),ofRandom(0,100000)));
        }
    }
}

void ofApp::lineBetween() {
    for(int i = 0; i < bgParticles.size(); i++){
        for(int j = 0; j < bgParticles.size(); j++){
            float distance = glm::distance(bgParticles[j].position, bgParticles[i].position);
            if(distance > lineDisMin && distance < lineDisMax) {
                ofSetColor(255,bglineOpacity);
                ofDrawLine(bgParticles[j].position, bgParticles[i].position);
            }
        }
    }
}
