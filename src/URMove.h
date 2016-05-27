//
//  URMove.h
//  ofxURDriver
//
//  Created by Dan Moore on 2/20/16.
//  Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//


#pragma once
#include "ofMain.h"
#include "UR5KinematicModel.h"
#include "ur_kin.h"
#include "ofxTiming.h"
#include "Synchronized.h"
class URMove {
public:
    URMove();
    ~URMove();
    void setup();
    void update();
    void draw(int i);
    void computeVelocities();
    void updatePathDebug();
    
    void addTargetPoint(Joint target);
    ofMatrix4x4 forwardKinematics(vector<double> pose);
    ofMatrix4x4 forwardKinematics(double o, double t, double th, double f, double fi, double s);
    void urKinematics(vector<double> input);
    void urKinematics(ofMatrix4x4 input);
    void urKinematics(double o, double t, double th, double f, double fi, double s);
    //    void urKinematics(vector<double> input);
    void setCurrentJointPosition(vector<double> & pose);
    float getAcceleration();
    ofParameterGroup movementParams;
    vector<double> getTargetJointPos();
    vector<double> getCurrentSpeed();
    vector<double> getRawJointPos();
    ofEasyCam cam;
    int selectedSolution;
    ofParameter<float> followLerp;
protected:
    ofParameter<float> speedDivider;
    ofParameter<bool> reachPoint;
    ofParameter<float> targetTCPLerpSpeed;
    ofParameter<float> jointSpeedLerpSpeed;
    ofParameter<float> jointAccelerationMultipler;
    ofParameter<float> avgAcceleration;
    
    float distance;
    int selectSolution();
    vector<UR5KinematicModel*> previews;
    //Motion Capture Visualization
    
    URKinematics kinematics;
    vector<double> currentPose;
    Synchronized<vector<vector<double> > > inversePosition;
    vector<vector<double> >  preInversePosition;
    ofMatrix4x4 mat;
    ofParameter<float> maxSpeed;
    ofParameter<float> minSpeed;
    ofParameter<float> deltaTime;
    float deltaT;
    RateTimer deltaTimer;
    vector<double> lastPosition;
    vector<double> currentJointSpeeds;
    vector<double> lastJointSpeeds;
    vector<double> acceleration;
    double avgAccel;
    double lastAvgAccel;
    deque<vector<float> > jointSpeedHistory;
    //    ofEasyCam cam;
    ofPolyline targetLine;
    float totalLength;
    Joint targetPoint;
    deque<Joint> newTargetPoint;
    unsigned int nearestIndex;
    float rotAngle;
    ofNode node;
    float targetLength;
    
    Joint preTargetPoint;
    float epslion;
};