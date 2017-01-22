//
//  URToolHead.cpp
//  ofxURDriver
//
//  Created by Dan Moore on 2/20/16.
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
#include "URToolHead.h"

void URToolHead::setup(){
    
}
void URToolHead::setOrientation(ofQuaternion orientation){
    orientation.get(rot);
}
void URToolHead::update(){
    
}
ofMatrix4x4 URToolHead::getMatrix(){
    return rot;
}
void URToolHead::draw(){
    currentTool.mesh.draw();
    ofPushMatrix();
    float angle;
    ofVec3f axis;
    rot.getRotate().getRotate(angle, axis);
    ofRotate(angle, axis.x, axis.y, axis.z);
    ofDrawAxis(100);
    ofPopMatrix();
}
void URToolHead::setTool(Tool t){
    
}
Tool URToolHead::getCurrentTool(){
    
}