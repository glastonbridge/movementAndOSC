#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup(){
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect

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
	
	nearThreshold = 230;
	farThreshold = 70;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 10;
	kinect.setCameraTiltAngle(angle);

    moon.loadImage("moon.png");
    
    sender.setup("10.2.5.124", 57120);

    
    for (unsigned int i = 0; i < NUMBER_OF_STARS; ++i)
        stars[i] = ofPoint(ofRandom(0,ofGetWidth()), ofRandom(0,ofGetHeight()));
}

//--------------------------------------------------------------
void testApp::update(){
	ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
        grayThreshNear = grayImage;
        grayThreshFar = grayImage;
        grayThreshNear.threshold(nearThreshold, true);
        grayThreshFar.threshold(farThreshold);
		
		// update the cv images
		grayImage.flagImageChanged();
		
		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, (kinect.width*kinect.height)/64, (kinect.width*kinect.height)/4, 20, false);
        
        float toprightcornerangle = atan(ofGetWidth()/ofGetHeight());
        float bottomrightcornerangle = PI - toprightcornerangle;
        float bottomleftcornerangle = PI + toprightcornerangle;
        float topleftcornerangle = 2*PI - toprightcornerangle;
        if (contourFinder.blobs.size()>0)
        {
            auto blob = contourFinder.blobs.begin();

            ofPoint centroid = blob->centroid;
            
            float spangle = 2 * PI / NUMBER_OF_HARMONICS;
            int lineIndex = 0;
            for (float rot = 0; rot < 2.0f * PI; rot += spangle)
            {
                ofPoint outroid; // the point where the line meets the edge of the screen

                //float m1 = 1.0f/tan(rot); // angle is measured clockwise from y axis
                //float c1 = centroid.y - m1 * centroid.x;
                
                auto p1 = blob->pts.begin();
                auto p2 = blob->pts.begin();
                ++p2;
                while (p2 < blob->pts.end())
                {
                    //float m2 = (p2->y - p1->y) / (p2->x - p1->x);
                    //float c2 = p1->y - m2 * p1->x;
                    
                    float xp1 = sin(rot)*(p1->y-centroid.y) - cos(rot)*(p1->x - centroid.x);
                    
                    float xp2 = sin(rot)*(p2->y-centroid.y) - cos(rot)*(p2->x - centroid.x);
                    
                    if (xp1*xp2 < 0) // only one cross product is less than 0, they must cross the line!
                    {
                        harmonicLengths[lineIndex].a = centroid;
                        harmonicLengths[lineIndex++].b = *p2;  // this is technically wrong
                        break;
                    }
                    
                    ++p1; ++p2;
                }
            }
        }
        
        for (unsigned int i = 0; i < NUMBER_OF_STARS; ++i)
        {
            stars[i].y += STAR_GRAVITY;
            if (stars[i].y > kinect.height)
            {
                stars[i].y = 0;
                stars[i].x = ofRandom(0,kinect.width);
            }
        }
        
        // SEND OUR OSC MESSAGE
        if (contourFinder.blobs.size() > 0)
        {
            for (unsigned int i = 0; i < NUMBER_OF_STARS; ++i)
            {
                if( contourFinder.blobs[0].boundingRect.inside(stars[i]))
                {
                    ofLogNotice() << "sending osc message";
                    
                    ofxOscMessage m;
                    m.setAddress("/tonz");
                    m.addFloatArg(contourFinder.blobs[0].centroid.y);
                    for (int i = 0; i < NUMBER_OF_HARMONICS; ++i)
                        m.addFloatArg(harmonicLengths[i].length());
                    sender.sendMessage(m);
                    
                    stars[i].y = 0;
                    stars[i].x = ofRandom(0,kinect.width);
                }
            }
        }
	}
}

//--------------------------------------------------------------
void testApp::draw()
{    
	ofSetColor(255, 255, 255);
    float scaleX = (float)ofGetWidth() / kinect.width ;
    float scaleY = (float) ofGetHeight() / kinect.height;
    ofPoint scale(scaleX, scaleY);
	
    // draw from the live kinect
    kinect.draw(0,0,ofGetWidth(), ofGetHeight());

    ofSetColor(0,0,180,100);
    
    grayThreshFar.draw(0,0,ofGetWidth(), ofGetHeight());
    
    ofRect(0,0,ofGetWidth(), ofGetHeight());
    
    ofSetColor(255);
    moon.draw(40,40);
    
    for (unsigned int i = 0; i < NUMBER_OF_STARS; ++i)
    {
        ofCircle(stars[i]*scale,2);
    }
    
    if (contourFinder.blobs.size()>0)
    {
        
        ofSetColor(255,255,255,128);
        ofCircle(contourFinder.blobs[0].centroid*scale, 20);
        
        ofSetColor(255,255,255,255);
        for (int i = 0; i < NUMBER_OF_HARMONICS; ++i)
            ofLine(harmonicLengths[i].a*scale, harmonicLengths[i].b*scale);
	}
    
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){

}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}
