//
//  ofxURDriver.hpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
// Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#pragma once
#include "ofMain.h"
#include "ur_driver.h"
#include "UR5KinematicModel.h"
#include "ofxTiming.h"

class ofxURDriver : public ofThread{
public:
    ofxURDriver();
    ~ofxURDriver();
    void setup(string ipAddress, double minPayload = 0.0, double maxPayload = 1.0);
    void start();
    void disconnect();
    void stopThread();
    void setToolOffset(ofVec3f localPos);
    void threadedFunction();
    ofNode getToolNode();
    ofVec4f getCalculatedTCPOrientation();
    vector<double> getToolPointRaw();
    vector<double> getJointPositions();
    vector<double> getJointAngles();
    bool isDataReady();
    float getThreadFPS();
    bool bDataReady;
    bool bStarted;
    void convertAxisAngle(double rx, double ry, double rz);
    void moveJoints(vector<double> pos);
    void setSpeed(vector<double> speeds, double acceleration = 100.0);
    Joint getToolPose();
    // Robot Arm
    UrDriver* robot;
    condition_variable rt_msg_cond_;
    condition_variable msg_cond_;
    bool has_goal_;
    std::thread* rt_publish_thread_;
    std::thread* mb_publish_thread_;
    double io_flag_delay_;
    double max_velocity_;
    vector<double> joint_offsets_;
    string base_frame_;
    string tool_frame_;
    bool use_ros_control_;
    std::thread* ros_control_thread_;
    vector<double> currentSpeed;
    double acceleration;
    UR5KinematicModel model;
    RateTimer timer;
    float epslion = 0.00000000000000001;
    
    deque<vector<double> > posBuffer;
    deque<vector<double> > speedBuffers;

    bool bMove;
};