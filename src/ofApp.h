#pragma once

#include "ofMain.h"
#include <time.h>
#include <iostream>
#include <cstring>
#include <vector>
#include <algorithm>
#include <cmath>
#include "LeapJoint.h"
#include "IIR4.h"
#include "ofxOsc.h"

#define MAX_HANDS 2

extern "C"
{
	// Leap Motion
	extern void DestroyConnection();
	extern float joints[MAX_HANDS][6*3];
	extern int handsPresent;
	extern bool frameReady;
	extern void initLeapMotion();
	extern void* imageBuffer;
	extern int imageWidth;
	extern int imageHeight;
	extern int imageBytesPerPixel;
	extern bool imageReady;
}

class ofApp : public ofBaseApp {

public:
	// Standard methods
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	// Motion descriptor calculations
	bool useSmoothing;
	std::vector<LeapJoint> currentJointPositions[MAX_HANDS], jointPos[MAX_HANDS], jointVel[MAX_HANDS], jointAcc[MAX_HANDS], jointJerk[MAX_HANDS];
	std::vector<IIR4> posFilter[MAX_HANDS];
	float qOM[MAX_HANDS];
	float weight[MAX_HANDS];
	float weightUpperBounds = 500.0; // Based on max value over time
	float time[MAX_HANDS];
	float timeUpperBounds = 2000.0;
	float space[MAX_HANDS];
	float spaceUpperBounds = 500.0;
	float flow[MAX_HANDS];
	float flowUpperBounds = 10000.0;
	float weightMax, timeMax, spaceMax, flowMax = 0.0;
	float weightScaled[MAX_HANDS], timeScaled[MAX_HANDS], spaceScaled[MAX_HANDS], flowScaled[MAX_HANDS];
	std::vector<float> weightVector[MAX_HANDS];

	// Effort meta data
	float filter[MAX_HANDS];
	float rhythm[MAX_HANDS];
	float _free_[MAX_HANDS];
	float depth[MAX_HANDS];
	float modulation[MAX_HANDS];
	float reverb[MAX_HANDS];

	void calculateDescriptors();
	void scaleDescriptors();
	void calculateEffortMetaData();
	void updateVectors();
	void updateVectorsWithSmoothing();
	void calculateQOM();
	void calculateWeightEffort();
	void calculateTimeEffort();
	void calculateSpaceEffort();
	void calculateFlowEffort();

	// Visuals
	ofImage rawLeapImage;
	unsigned char* prevBuffer;
	void granulateImage();

	// General setup
	int numFrames, numJoints, numCoordinates;
	void resizeVectors();
	//int count;
	bool isFullscreen;
	int frameRate;

	// OSC
	ofxOscSender sender;
	void sendDescriptorsViaOSC();
	bool spatialMode;
};