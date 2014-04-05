#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxOsc.h"

struct line
{
    ofPoint a,b;
    float length()
    {
        return sqrt(abs(a.x-b.x) + abs(a.y-b.y));
    }
};

class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
private:
    
	ofxKinect kinect;
    
	ofxCvColorImage colorImg;
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	ofxCvContourFinder contourFinder;
    
    ofImage moon;
	
	int nearThreshold;
	int farThreshold;
	
	int angle;
    
    
    ofxOscSender sender;
    
    static const unsigned int NUMBER_OF_HARMONICS = 16;
    static const unsigned int NUMBER_OF_STARS = 4;
    static const unsigned int STAR_GRAVITY = 2;
    
    line harmonicLengths[NUMBER_OF_HARMONICS];
    
    ofPoint stars[NUMBER_OF_STARS];

};
