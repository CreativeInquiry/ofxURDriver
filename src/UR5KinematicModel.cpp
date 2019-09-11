//
//  KinectModel.cpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
// Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#include "UR5KinematicModel.h"
UR5KinematicModel::UR5KinematicModel(){
    
}
UR5KinematicModel::~UR5KinematicModel(){

}

void UR5KinematicModel::setup(){
    
    
    if(loader.loadModel(ofToDataPath("models/ur5.dae"))){
        for(int i = 0; i < loader.getNumMeshes(); i++){
            meshs.push_back(loader.getMesh(i));
        }
    }else{
        ofLogFatalError()<<"PLEASE PLACE THE 3D FILES OF THE UR ARM IN data/models/ur5.dae"<<endl;
    }
    
    joints.resize(6);
    nodes.resize(6);
    
    
    vector<Joint> foojoints;
    foojoints.resize(6);
    
    joints[0].position.set(0, 0, 0);
    joints[1].position.set(0, -0.072238, 0.083204);
    joints[2].position.set(0, -0.077537,0.51141);
    joints[3].position.set(0, -0.070608, 0.903192);
    joints[4].position.set(0, -0.117242, 0.950973);
    joints[5].position.set(0, -0.164751, 0.996802);
    tool.position.set(joints[5].position + ofVec3f(0,-0.135,0)); // tool tip position
    
    for(int i = 1; i <joints.size(); i++){
        joints[i].offset =joints[i].position-joints[i-1].position;
        
    }
    tool.offset =joints[5].offset;
    
    
    
    joints[0].axis.set(0, 0, 1);
    joints[1].axis.set(0, -1, 0);
    joints[2].axis.set(0, -1, 0);
    joints[3].axis.set(0, -1, 0);
    joints[4].axis.set(0, 0, 1);
    joints[5].axis.set(0, 1, 0);
    tool.axis.set(joints[5].axis);
    
    joints[0].rotation.makeRotate(0,joints[0].axis);
    joints[1].rotation.makeRotate(-90,joints[1].axis);
    joints[2].rotation.makeRotate(0,joints[2].axis);
    joints[3].rotation.makeRotate(-90,joints[3].axis);
    joints[4].rotation.makeRotate(0,joints[4].axis);
    joints[5].rotation.makeRotate(0,joints[5].axis);
    
    nodes[0].setPosition(joints[0].position);
    nodes[0].setOrientation(joints[0].rotation);
    for(int i = 1; i <nodes.size(); i++){
        nodes[i].setParent(nodes[i-1]);
        nodes[i].setPosition(joints[i].offset*1000);
        nodes[i].setOrientation(joints[i].rotation);
    }
    
    
    
    tcpNode.setParent(nodes[5]);
    tcpNode.setPosition(ofVec3f(0.0, -0.2, 0.0)*1000);
    
    
    tool.rotation =joints[5].rotation;
    
    shader.load("shaders/model");
    
    bDrawModel.set("Draw Model", true);
    bDrawTargetModel.set("Draw Target Model", true);
    bUseShader.set("Use Shader", true);
}

ofQuaternion UR5KinematicModel::getToolPointQuaternion(){
    return toOf(nodes[5].getGlobalTransformMatrix()).getRotate();
}



ofNode UR5KinematicModel::getTool(){
    return tcpNode;
}

void UR5KinematicModel::setToolOffset(ofVec3f localOffset){
    tcpNode.setPosition(localOffset);
}

void UR5KinematicModel::setAngles( vector<double> aTargetRadians ){
    for(int i = 0; i < aTargetRadians.size(); i++){
        if(i == 1 || i == 3){
            joints[i].rotation.makeRotate(ofRadToDeg(aTargetRadians[i])+90,joints[i].axis);
        }else{
            joints[i].rotation.makeRotate(ofRadToDeg(aTargetRadians[i]),joints[i].axis);
        }
        nodes[i].setOrientation(joints[i].rotation);
    }
}

void UR5KinematicModel::setPose(vector<double> pose){
    for(int i = 0; i < pose.size(); i++){
        if(i == 1 || i == 3){
            joints[i].rotation.makeRotate(ofRadToDeg(pose[i])+90,joints[i].axis);
        }else{
            joints[i].rotation.makeRotate(ofRadToDeg(pose[i]),joints[i].axis);
        }
         nodes[i].setOrientation(joints[i].rotation);
    }
    
}
void UR5KinematicModel::setToolMesh(ofMesh mesh){
    toolMesh = mesh;
}
void UR5KinematicModel::update(){

}
void UR5KinematicModel::draw(bool bDrawDebug){
    ofPushMatrix();
    ofPushStyle();
    ofEnableDepthTest();
    ofSetColor(255, 255, 255);
    if(bDrawDebug) {
        ofPushStyle(); {
            ofDrawAxis(1000);
            ofSetColor(255, 255, 0);
            ofDrawSphere(tool.position*ofVec3f(1000, 1000, 1000), 4);
            ofSetColor(225, 225, 225);
        } ofPopStyle();
    }
    
    if(bDrawModel){
        ofQuaternion q;
        ofVec3f offset;
        
        ofMatrix4x4 gmat;
        gmat.makeIdentityMatrix();
        gmat.makeScaleMatrix( 1, 1, 1 );
        
        if(bUseShader){
            shader.begin();
        }
        ofPushMatrix();
        {
            ofPushMatrix();
            {
                for(int i = 0; i < joints.size(); i++)
                {
                    float x;
                    ofVec3f axis;
                    q = joints[i].rotation;
                    q.getRotate(x, axis);
                    ofTranslate(joints[i].offset*1000);
                    gmat.translate( joints[i].offset*1000 );
                    
                    if(bDrawDebug) {
                        ofDrawAxis(10);
                    }
                    ofMatrix4x4 tmat;
                    if(i >= 3){
                        ofPushMatrix();
                        {
                            ofRotateZ(-180);
                            ofRotateX(-180);
                            ofScale(100, 100, 100);
                            meshs[i].draw();
                        }
                        ofPopMatrix();
                    }
                    ofRotate(x, axis.x, axis.y, axis.z);
                    if(i < 3){
                        ofPushMatrix(); {
                            ofRotateZ(-180);
                            ofRotateX(-180);
                            ofScale(100, 100, 100);
                            
                            meshs[i].draw();
                        } ofPopMatrix();
                    }
                }
                toolMesh.draw();
            }
            ofPopMatrix();
        }
        ofPopMatrix();
        
        if(bUseShader) shader.end();
        
        if (bDrawDebug) {
            ofPushMatrix();
            {
                //            ofRotate(180, 0, 0, 1);
                for(int i = 0; i < nodes.size(); i++){
                    nodes[i].draw();
                }
                tcpNode.draw();
            }
            ofPopMatrix();
        }
    }
    ofDisableDepthTest();
    ofPopStyle();
    ofPopMatrix();
}
