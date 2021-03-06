//
//  KinectModel.h
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#pragma once
#include "ofMain.h"
#include "ofxAssimpModelLoader.h"
#include "Synchronized.h"
#include "URJoint.h"


//#include "ofxBullet.h"
class UR5KinematicModel{
public:
    UR5KinematicModel();
    ~UR5KinematicModel();
    void setup();
    void update();
    void draw(ofFloatColor color = ofFloatColor(1, 1, 1, 1), bool bDrawDebug=true);
    void setToolMesh(ofMesh mesh);
    void setPose(vector<double> pose);
    
    void getArmIK( ofVec3f aTargetWorldPos, ofVec3f aElbowWorldPos, bool aBInvertElbow, float aDeltaTimef );
    
    ofNode getTool();
    void setToolOffset(ofVec3f localOffset);
    ofQuaternion getToolPointQuaternion();
    
    void setAngles( vector<double> aTargetRadians );
    
    ofxAssimpModelLoader loader;
    vector<ofMesh> meshs;
    ofMesh toolMesh;
    
    ofShader shader;
    float elapsed_time, last_time;
    ofVec3f pt;
    vector<Joint> joints;

    Joint tool;
    
    Joint dtoolPoint;
    
    ofEasyCam cam;
    ofNode tcpNode;
    vector<ofNode> nodes;
    
    ofParameter<float> stage;
    ofParameter<bool> bDrawModel;
    ofParameter<bool> bDrawTargetModel;
    ofParameter<bool> bUseShader;
    
    
//    ofxBulletWorldRigid			world;
//    vector <ofxBulletBox*>		bounds;
//    ofxBulletCustomShape*		boundsShape;
//    
//    vector<ofxBulletCustomShape*>	phyMesh;
//    vector<ofxBulletJoint*>		phyJoints;
    
};
