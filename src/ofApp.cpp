#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup() 
{

	// Prepare data acquisition
	numJoints = 6;
	numCoordinates = 36;
	numFrames = 20;
	resizeVectors();
	frameRate = 80;
	initLeapMotion();

	useSmoothing = true;

	for (int h = 0; h < MAX_HANDS; h++){
		for (int i = 0; i < numJoints; i++)
			posFilter[h][i] = IIR4::IIR4(IIR4::LPF, 0.5);
	}

	ofSetWindowTitle("LLEAF table 0.1");
	ofEnableAlphaBlending();
	ofSetupScreen();
	ofBackground(0, 0, 0);
	ofSetFrameRate(frameRate);

	isFullscreen = false;

	ofSetVerticalSync(true);
	ofEnableAlphaBlending();
	ofEnableSmoothing();

	rawLeapImage.allocate(1024, 768, OF_IMAGE_GRAYSCALE);

	ofSetSphereResolution(5);

	ofBackground(0, 0, 0);

	sender.setup("127.0.0.1", 6448);
	spatialMode = false;
}

//--------------------------------------------------------------
void ofApp::update() {

	if (frameReady)
	{
		calculateDescriptors();
		scaleDescriptors();
		calculateEffortMetaData();
		sendDescriptorsViaOSC();
	}

	if (imageReady)
		granulateImage();
	
	if (!isFullscreen)
		rawLeapImage.resize(1024, 768);
	else
		rawLeapImage.resize(ofGetWindowWidth(), ofGetWindowHeight());
}

void ofApp::granulateImage() {
	// Runs only once, when uninitialized
	if (prevBuffer == nullptr)
		prevBuffer = (unsigned char*)imageBuffer;

	unsigned char* currentBuffer = (unsigned char*)imageBuffer;
	unsigned char* granulatedBuffer = prevBuffer;
	int bufferLength = imageWidth * imageHeight * imageBytesPerPixel;

	float pitch = ofMap(flow[0], 500.0, 180000.0, -4.0, 4.0, true);
	float rate = ofMap(flow[0], 0.0, 10000.0, 0.0, 10.0, true);
	float grainLength = ofMap(flow[0], 0.0, 400.0, 1.8, 0.005, true);
	float nOverlaps = std::round(ofMap(flow[0], 0.0, 1000.0, 1.0, 10.0, true));

	int drawPositionX = imageHeight - 1;
	int readPositionX = std::round(0.1 * rate * imageHeight);

	for (int i = 0; i < bufferLength; i++) {
		int setPixelIndex = std::round(i * imageWidth / 2 * grainLength + drawPositionX);
		int getPixelIndex = std::round(i * imageWidth / 2 * grainLength * nOverlaps + readPositionX);
		
		if (setPixelIndex > bufferLength)
			setPixelIndex %= bufferLength;

		if (getPixelIndex > bufferLength)
			getPixelIndex %= bufferLength;

		granulatedBuffer[setPixelIndex] = currentBuffer[getPixelIndex];

		drawPositionX -= pitch;
		readPositionX += pitch;

		if (drawPositionX < 0)
			drawPositionX = imageHeight - 1;
	}

	prevBuffer = currentBuffer;

	if (granulatedBuffer != nullptr)
		rawLeapImage.setFromPixels(granulatedBuffer, imageWidth, imageHeight, OF_IMAGE_GRAYSCALE);
	
	if (pitch < 0.0)
		rawLeapImage.mirror(1, 1);
	else
		rawLeapImage.mirror(0, 1);

	imageReady = false;
}

//--------------------------------------------------------------
void ofApp::draw() {
	if (imageReady)
		rawLeapImage.draw(0, 0);
	 //Uncomment for debugging:
	
	string debug = /*"Flow: " + ofToString(flow[0]) + ", Time: " + ofToString(time[0]) + ", Space: " + ofToString(space[0]) + ", Weight: "
		+ ofToString(weight[0]) + ", QoM: " + ofToString(qOM[0]) + ", hands present: " + ofToString(handsPresent)
		+ "\nMaxValues: Flow: " + ofToString(flowMax) + ", Time: " + ofToString(timeMax) + ", Space: " + ofToString(spaceMax) + ", Weight: " + ofToString(weightMax)
		+ "\nScaledValues: Flow: " + ofToString(flowScaled[0]) + ", Time: " + ofToString(timeScaled[0]) + ", Space: " + ofToString(spaceScaled[0]) + ", Weight: " + ofToString(weightScaled[0]);*/
		"Flow: " + ofToString(flowScaled[0]) + "\nTime: " + ofToString(timeScaled[0]) + "\nSpace: " + ofToString(spaceScaled[0]) + "\nWeight: "
		+ ofToString(weightScaled[0]) + "\nQoM: " + ofToString(qOM[0]) + ", hands present: " + ofToString(handsPresent)
		+ "\nfilter: " + ofToString(filter[0]) + "\nrhythm: " + ofToString(rhythm[0]) + "\n_free_: " + ofToString(_free_[0])
		+ "\ndepth: " + ofToString(depth[0]) + "\nmodulation: " + ofToString(modulation[0]) + "\nreverb: " + ofToString(reverb[0]);

	ofDrawBitmapString(debug, 100, 100);
	
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
	if (key == 'f' || key == 'F') 
	{
		ofToggleFullscreen();
		isFullscreen = !isFullscreen;
	}

	else if (key == 'q' || key == 'Q') 
	{
		if (isFullscreen)
			ofToggleFullscreen();

		DestroyConnection();
		ofExit();
	}

	else if (key == 's' || key == 'S') 
	{
		useSmoothing = !useSmoothing;
		if (useSmoothing)
			std::cout << "Smoothing on" << std::endl;
		else
			std::cout << "Smoothing off" << std::endl;
	}

	else if (key == '1')
	{
		spatialMode = false;
		std::cout << "Mode: effort" << std::endl;
	}

	else if (key == '2')
	{
		spatialMode = true;
		std::cout << "Mode: spatial" << std::endl;
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) 
{

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) 
{

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) 
{

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)  
{

}

void ofApp::calculateDescriptors()
{
	// Fill low-level descriptor vectors
	
	if (!useSmoothing)
		updateVectors();
	
	else
	updateVectorsWithSmoothing();

	// Calculate effort descriptors
	calculateQOM();
	calculateWeightEffort();
	calculateTimeEffort();
	calculateSpaceEffort();
	calculateFlowEffort();

}

void ofApp::scaleDescriptors()
{
	// Scale descriptors with upper bounds derived from measurements

	// Limit values to upper bound
	for (int i = 0; i < MAX_HANDS; i++) {
		flow[i] = MIN(flow[i], flowUpperBounds);
		time[i] = MIN(time[i], timeUpperBounds);
		weight[i] = MIN(weight[i], weightUpperBounds);
		space[i] = MIN(space[i], spaceUpperBounds);

		flowScaled[i] = flow[i] / flowUpperBounds;
		timeScaled[i] = time[i] / timeUpperBounds;
		weightScaled[i] = weight[i] / weightUpperBounds;
		spaceScaled[i] = space[i] / spaceUpperBounds;
		QoMScaled[i] = qOM[i] / QoMUpperBounds;
	}
}

void ofApp::calculateEffortMetaData()
{
	for (int h = 0; h < MAX_HANDS; h++) {
		filter[h] = (spaceScaled[h]);
		rhythm[h] = (flowScaled[h]);
		_free_[h] = 0;
		depth[h] = (weightScaled[h]);
		modulation[h] = (timeScaled[h]);
		reverb[h] = (timeScaled[h] + flowScaled[h] + spaceScaled[h]) / 3;
	}
}

void ofApp::updateVectors() 
{
	//float dt = 1.0 / frameRate;

	//// shift left on low-level descriptor vectors
	//for (int i = 0; i < jointPos[h].size() - numJoints; i++)
	//{
	//	jointPos[h][i] = jointPos[h][i + numJoints];
	//	jointVel[h][i] = jointVel[h][i + numJoints];
	//	jointAcc[h][i] = jointAcc[h][i + numJoints];
	//	jointJerk[h][i] = jointJerk[h][i + numJoints];
	//}

	//// store current position values
	//int c = 0;

	//for (int i = 0; i < numJoints; i++)
	//{
	//	jointPos[h][jointPos[h].size() - numJoints + i].x = joints[c++];
	//	jointPos[h][jointPos[h].size() - numJoints + i].y = joints[c++];
	//	jointPos[h][jointPos[h].size() - numJoints + i].z = joints[c++];
	//}
	//	

	//// calculate current velocity, acceleration, jerk (lagging 1, 1, 2 frames respectively)
	//for (int i = 0; i < numJoints; i++) 
	//{
	//	jointVel[jointVel.size() - numJoints + i] = (jointPos[((numFrames - 1) * numJoints) + i]
	//		- jointPos[((numFrames - 3) * numJoints) + i]) / (2.0 * dt);
	//	jointAcc[jointAcc.size() - numJoints + i] = (jointPos[((numFrames - 1) * numJoints) + i]
	//		- 2.0 * jointPos[((numFrames - 2) * numJoints) + i] + jointPos[((numFrames - 3) * numJoints) + i]) / (dt * dt);
	//	jointJerk[jointJerk.size() - numJoints + i] = (jointPos[((numFrames - 1) * numJoints) + i]
	//		- 2.0 * jointPos[((numFrames - 2) * numJoints) + i] + 2.0 * jointPos[((numFrames - 3) * numJoints) + i]
	//		- jointPos[((numFrames - 4) * numJoints) + i]) / (2.0 * dt * dt * dt);
	//}
}

void ofApp::updateVectorsWithSmoothing() 
{
	float dt = 1.0 / frameRate;

	for (int h = 0; h < MAX_HANDS; h++) {

		// shift left on low-level descriptor vectors
		for (int i = 0; i < jointPos[h].size() - numJoints; i++)
		{
			jointPos[h][i] = jointPos[h][i + numJoints];
			jointVel[h][i] = jointVel[h][i + numJoints];
			jointAcc[h][i] = jointAcc[h][i + numJoints];
			jointJerk[h][i] = jointJerk[h][i + numJoints];
		}

		// smooth and store current position values

		int c = 0;

		for (int i = 0; i < numJoints; i++)
		{
			currentJointPositions[h][i].x = joints[h][c++];
			currentJointPositions[h][i].y = joints[h][c++];
			currentJointPositions[h][i].z = joints[h][c++];
		}

		for (int i = 0; i < numJoints; i++)
			jointPos[h][jointPos[h].size() - numJoints + i] = posFilter[h][i].process(currentJointPositions[h][i]);

		// calculate current velocity, acceleration, jerk (lagging 1, 1, 2 frames respectively)
		for (int i = 0; i < numJoints; i++)
		{
			jointVel[h][jointVel[h].size() - numJoints + i] = (jointPos[h][((numFrames - 1) * numJoints) + i]
				- jointPos[h][((numFrames - 3) * numJoints) + i]) / (2.0 * dt);
			jointAcc[h][jointAcc[h].size() - numJoints + i] = (jointPos[h][((numFrames - 1) * numJoints) + i]
				- 2.0 * jointPos[h][((numFrames - 2) * numJoints) + i] + jointPos[h][((numFrames - 3) * numJoints) + i]) / (dt * dt);
			jointJerk[h][jointJerk[h].size() - numJoints + i] = (jointPos[h][((numFrames - 1) * numJoints) + i]
				- 2.0 * jointPos[h][((numFrames - 2) * numJoints) + i] + 2.0 * jointPos[h][((numFrames - 3) * numJoints) + i]
				- jointPos[h][((numFrames - 4) * numJoints) + i]) / (2.0 * dt * dt * dt);
		}
	}
	
}

void ofApp::calculateQOM() 
{
	// Calculate quantity of motion

	// Vector of weigthing factors (thumb, index, middle, ring, pinky, wrist for both hands)
	for (int h = 0; h < MAX_HANDS; h++) {
		float alpha[6] = { 1, 1, 1, 1, 1, 5 };

		float num = 0;
		float den = 0;

		for (int j = 0; j < numJoints; j++)
		{
			num += 0.001 * jointVel[h][(jointVel[h].size() - numJoints) + j].magnitude() * alpha[j];
			den += alpha[j];
		}

		qOM[h] = num / den;
	}
}

void ofApp::calculateWeightEffort() 
{	
	// Vector of weigthing factors (thumb, index, middle, ring, pinky, palm)
	float alpha[6] = { 1, 1, 1, 0, 0, 4 };

	for (int h = 0; h < MAX_HANDS; h++) {
		if (handsPresent > h) {
			float val = 0;

			for (int j = 0; j < numJoints; j++)
			{
				float mag = 0.001 * jointVel[h][(jointVel[h].size() - numJoints) + j].magnitude();
				val += alpha[j] * mag * mag;
			}

			// update vector
			for (int j = numFrames - 1; j >= 1; j--)
				weightVector[h][j - 1] = weightVector[h][j];
			weightVector[h][numFrames - 1] = val;
			weight[h] = *std::max_element(weightVector[h].begin(), weightVector[h].end());

		}
		else {
			weight[h] = 0.0;
		}

	} 

	weightMax = MAX(weight[0], weightMax);
}

void ofApp::calculateTimeEffort() 
{
	for (int h = 0; h < MAX_HANDS; h++) {
		if (handsPresent > h) {
			int j = 0;

			// Vector of weigthing factors (thumb, index, middle, ring, pinky, palm for both hands)
			float alpha[6] = { 2, 2, 2, 0, 0, 1 };

			float val = 0;

			for (int i = 0; i < numFrames*numJoints; i++) {
				float mag = 0.001 * jointAcc[h][i].magnitude();
				val += alpha[j++] * mag;
				j %= numJoints;
			}

			j = 0;
			time[h] = val / numFrames;

			if (!std::isfinite(time[h]))
				time[h] = 0.0;
		}
		else {
			time[h] = 0.0;
		}
	} 


	timeMax = MAX(time[0], timeMax);
}

void ofApp::calculateFlowEffort() 
{

	// Vector of weigthing factors (thumb, index, middle, ring, pinky, palm for both hands)
	float alpha[6] = { 1, 1, 1, 0, 0, 1 };

	for (int h = 0; h < MAX_HANDS; h++) {
		if (handsPresent > h) {
			int j = 0;
			float val = 0;

			for (int i = 0; i < numFrames * numJoints; i++) {
				float mag = 0.001 * jointJerk[h][i].magnitude();
				val += alpha[j++] * mag;
				j %= 6;
			}

			j = 0;
			flow[h] = val / 20.0;

			if (!std::isfinite(flow[h]))
				flow[h] = 0.0;
		}
		else {
			flow[h] = 0.0;
		}
	}

	flowMax = MAX(flow[0], flowMax);
}

void ofApp::calculateSpaceEffort()
{
	// Vector of weigthing factors (thumb, index, middle, ring, pinky, palm for both hands)
	float alpha[6] = { 1, 5, 1, 0, 0, 1 };

	for (int h = 0; h < MAX_HANDS; h++) {
		if (handsPresent > h) {
			float num = 0;
			float den = 0;
			space[h] = 0;

			for (int i = 0; i < numJoints; i++) {
				LeapJoint diff;

				for (int j = 0; j < 19; j++) {
					diff = jointPos[h][i + (1 + j) * numJoints] - jointPos[h][i + j * numJoints];
					num += diff.magnitude();
				}

				diff = jointPos[h][i] - jointPos[h][i + (numFrames-1) * numJoints];
				den = diff.magnitude();

				space[h] += alpha[i] * num / (den + 1.0e-9); // Add 1e-9 to avoid nan
			}

			if (!std::isfinite(space[h]))
				space[h] = 500.0;
		}
		else {
			space[h] = 0.0;
		}
	}
	spaceMax = MAX(space[0], spaceMax);
}

void ofApp::resizeVectors()
{
	for (int h = 0; h < MAX_HANDS; h++) {
		currentJointPositions[h].resize(numJoints);
		jointPos[h].resize(numJoints * numFrames);
		jointVel[h].resize(numJoints * numFrames);
		jointAcc[h].resize(numJoints * numFrames);
		jointJerk[h].resize(numJoints * numFrames);
		posFilter[h].resize(numJoints);
		weightVector[h].resize(numFrames);
	}
}

void ofApp::sendDescriptorsViaOSC()
{
	for (int i = 0; i < MAX_HANDS; i++)
	{
		ofxOscMessage m0;
		string address = "/efforts" + ofToString(i);
		m0.setAddress(address);

		m0.addFloatArg(filter[i]);
		m0.addFloatArg(rhythm[i]);
		m0.addFloatArg(_free_[i]);
		m0.addFloatArg(depth[i]);
		m0.addFloatArg(modulation[i]);
		m0.addFloatArg(reverb[i]);
		m0.addFloatArg(QoMScaled[i]);
		int handPresent = (handsPresent > i) ? 1 : 0;
		m0.addIntArg(handPresent);
		sender.sendMessage(m0);
	}

	frameReady = false;
}