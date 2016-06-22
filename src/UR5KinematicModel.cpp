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
    
    
//    world.setup();
//    world.disableGrabbing();
//    world.setGravity( ofVec3f(0, 0, 0) );

    
    ofDirectory dir;
    dir.listDir(ofToDataPath("models"));
    dir.sort();
    dir.allowExt("dae");
    
    // load robot mesh
    loader.loadModel(ofToDataPath("models/ur5.dae"));
    for(int i = 0; i < loader.getNumMeshes(); i++){
        meshs.push_back(loader.getMesh(i));
    }
    

    
    
    //    joints.setup(vector<Joint>());
    vector<double> foo;
    foo.assign(6, 0.0);
    jointsRaw.setup(foo);
    toolPointRaw.setup(foo);
    jointsProcessed.setup(foo);
    
    joints.resize(6);
    nodes.resize(6);
    jointsRaw.getBack().assign(6, 0.0);
    jointsProcessed.getBack().assign(6, 0.0);
    toolPointRaw.getBack().assign(6, 0.0);
    
    vector<Joint> foojoints;
    foojoints.resize(6);
    
    joints[0].position.set(0, 0, 0);
    joints[1].position.set(0, -0.072238, 0.083204);
    joints[2].position.set(0,-0.077537,0.51141);
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
    
//    phyMesh.resize( nodes.size() );
//    for (int i = 0; i < phyMesh.size(); i++) {
//        phyMesh[i] = new ofxBulletCustomShape();
//        phyMesh[i]->addMesh(meshs[i], ofVec3f(100, 100, 100), true);
//        phyMesh[i]->setFriction(0.0);
//        phyMesh[i]->setRestitution(0.01);
//        phyMesh[i]->enableKinematic();
//        phyMesh[i]->create( world.world, joints[i].position*1000, joints[i].rotation, 1.);
//        phyMesh[i]->add();
//
//    }
    
    
    tcpNode.setParent(nodes[5]);
    tcpNode.setPosition(ofVec3f(0.0, -0.2, 0.0)*1000);
    
    
    tool.rotation =joints[5].rotation;
    
    shader.load("shaders/model");
    
    bDrawModel.set("Draw Model", true);
    bDrawTargetModel.set("Draw Target Model", false);
    bDrawModel = true;
    
    jointsProcessed.swapBack();
    jointsRaw.swapBack();
    toolPointRaw.swapBack();
    
}

ofQuaternion UR5KinematicModel::getToolPointQuaternion(){
    return nodes[5].getGlobalTransformMatrix().getRotate();
}

ofNode UR5KinematicModel::getTool(){
    return tcpNode;
}

void UR5KinematicModel::setToolOffset(ofVec3f localOffset){
    tcpNode.setPosition(localOffset);
}

void UR5KinematicModel::setToolMesh(ofMesh mesh){
    toolMesh = mesh;
}
void UR5KinematicModel::update(){

    	world.update();
}
void UR5KinematicModel::draw(float stage){
    
    ofDrawAxis(1000);
    ofEnableDepthTest();
    ofSetColor(255, 255, 0);
    ofDrawSphere(tool.position*ofVec3f(1000, 1000, 1000), 4);
    ofSetColor(255, 0, 255);
    ofDisableDepthTest();
    
    
    if(bDrawModel){
        ofEnableDepthTest();
        shader.begin();
        float x;
        ofVec3f axis;
        ofQuaternion q;
        ofVec3f offset;
        ofPushMatrix();
        {
            for(int i = 0; i < joints.size(); i++)
            {
                float x;
                ofVec3f axis;
                q = joints[i].rotation;
                q.getRotate(x, axis);
                ofTranslate(joints[i].offset*1000);
                ofDrawAxis(10);
                if(i >= 3){
                    ofPushMatrix();
                    ofRotateZ(-180);
                    ofRotateX(-180);
                    ofScale(100, 100, 100);
                    meshs[i].draw();
                    ofPopMatrix();
                }
                ofRotate(x, axis.x, axis.y, axis.z);
                if(i < 3){
                    ofPushMatrix();
                    ofRotateZ(-180);
                    ofRotateX(-180);
                    ofScale(100, 100, 100);
                    meshs[i].draw();
                    ofPopMatrix();
                }
            }
            toolMesh.draw();
        }
        ofPopMatrix();
        
        shader.end();
        ofDisableDepthTest();
        
        ofPushMatrix();
        for(int i = 0; i < nodes.size(); i++){
            nodes[i].draw();
        }
        tcpNode.draw();
        ofPopMatrix();
    }
}