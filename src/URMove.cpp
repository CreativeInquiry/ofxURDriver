//
//  URMove.cpp
//  ofxURDriver
//
//  Created by Dan Moore on 2/20/16.
//  Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
#include "URMove.h"
#include "URUtils.h"
URMove::URMove(){
    
}
URMove::~URMove(){
    
}
void URMove::setup(){
    float min = FLT_MIN;
    float max = FLT_MAX;
    movementParams.setName("UR Movements");
    movementParams.add(minSpeed.set("MIN Speed", max, min, max));
    movementParams.add(maxSpeed.set("MAX Speed", min, min, max));
    movementParams.add(targetTCPLerpSpeed.set("TCP LerpSpeed", 0.9, 0.001, 1.0));
    movementParams.add(avgAcceleration.set("Accel", 100, 0, 200));
    movementParams.add(jointAccelerationMultipler.set("Accel Multi", 1, 1, 1000));
    movementParams.add(speedDivider.set("Speed Divider", 1, 1, 10));
    reachPoint.set("Step", false);
    deltaTime.set("Delta T", 0.0, 0.0, 1.0);
    jointSpeedLerpSpeed.set("Joint LerpSpeed", 0.9, 0.001, 1.0);
    
    
    for(int i = 0; i < 8; i++){
        previews.push_back(new UR5KinematicModel());
        previews.back()->setup();
    }
    
    selectedSolution = -1;
    deltaTimer.setSmoothing(false);
    distance = 0;
    acceleration.assign(6, 0.0);
    lastJointSpeeds.assign(6, 0.0);
    currentJointSpeeds.assign(6, 0.0);
    currentPose.assign(6, 0.0);
    inversePosition.setup(vector<vector<double> >());
    
    epslion = 0.00025;
    targetLength = 0.0;
}


void URMove::update(){
    deltaTimer.tick();
    deltaTime = deltaTimer.getPeriod();
    
  
    if(newTargetPoint.size() > 0){
        float speed = ((targetPoint.position-newTargetPoint.front().position)).length()/deltaTime;
        targetLine.getPointAtIndexInterpolated(targetLine.getIndexAtLength(targetLength));
        targetPoint.position = targetPoint.position.interpolate(newTargetPoint.front().position, targetTCPLerpSpeed);
        targetPoint.rotation.slerp(targetTCPLerpSpeed, targetPoint.rotation, newTargetPoint.front().rotation);
        newTargetPoint.pop_front();
    }
    
    mat.setTranslation(targetPoint.position);
    mat.setRotate(targetPoint.rotation);

    selectedSolution = selectSolution();
    urKinematics(mat);
    computeVelocities();
    
}

vector<double> URMove::getTargetJointPos(){
    if(selectedSolution > -1){
        return previews[selectedSolution]->jointsRaw.getFront();
    }else{
        return currentPose;
    }
}
float URMove::getAcceleration(){
    avgAcceleration = avgAccel*jointAccelerationMultipler;
    return avgAccel*jointAccelerationMultipler;
}
vector<double> URMove::getCurrentSpeed(){
    return currentJointSpeeds;
}

void URMove::setCurrentJointPosition(vector<double> & pose){
    for(int i =0; i < pose.size(); i++){
        currentPose[i]= pose[i];
    }
}

void URMove::computeVelocities(){
    if(selectedSolution != -1){
        if(currentPose.size() > 0){
            lastAvgAccel = avgAccel;
            avgAccel = FLT_MIN;
            lastJointSpeeds = currentJointSpeeds;
            previews[selectedSolution]->jointsRaw.swapFront();
            previews[selectedSolution]->jointsProcessed.swapFront();
            for(int i = 0; i < previews[selectedSolution]->jointsRaw.getFront().size(); i++){
                currentJointSpeeds[i] = (previews[selectedSolution]->jointsRaw.getFront()[i]-currentPose[i])/deltaTime/speedDivider;
                
                float tempMin = minSpeed;
                float tempMax = maxSpeed;
                minSpeed = MIN(tempMin, currentJointSpeeds[i]);
                maxSpeed = MAX(tempMax, currentJointSpeeds[i]);
                
                acceleration[i] = currentJointSpeeds[i] - lastJointSpeeds[i];
                avgAccel = MAX(acceleration[i], avgAccel);
                
            }
        }
    }else{
        for(int i = 0; i < currentJointSpeeds.size(); i++){
            currentJointSpeeds[i] = 0;
        }
    }
}

void URMove::addTargetPoint(Joint target){
    
    newTargetPoint.push_back(target);
    targetLine.addVertex(toMM(target.position));
    if(targetLine.size() > 400){
        targetLine.getVertices().erase(targetLine.getVertices().begin(), targetLine.getVertices().begin()+1);
    }
    totalLength = targetLine.getLengthAtIndexInterpolated(targetLine.getIndexAtPercent(1.0));
    
}


void URMove::draw(int i){
    if(inversePosition.getFront().size() > 0 && i < inversePosition.getFront().size()){
        ofSetColor(255, 0, 255, 150);
        previews[i]->draw(i);
        targetLine.draw();
        ofSetColor(255, 0, 255, 200);
        ofDrawSphere(toMM(targetPoint.position), 5);
        ofSetColor(255, 0, 255);
        if(newTargetPoint.size() > 0){
            ofDrawSphere(toMM(newTargetPoint.front().position), 5);
        }
    }
}

int URMove::selectSolution(){
    inversePosition.swapFront();
    vector<int> nearestSolution;
    vector<int> count;
    if(currentPose.size() > 0 && inversePosition.getFront().size() > 0){
        vector<double> minDistances;
        vector<vector<double> > diffs;
        diffs.resize(inversePosition.getFront().size());
        nearestSolution.resize(inversePosition.getFront()[0].size());
        count.resize(inversePosition.getFront().size());
        minDistances.assign(inversePosition.getFront().size(), DBL_MAX);
        for(int i = 0; i < inversePosition.getFront().size(); i++){
            diffs[i].resize(inversePosition.getFront()[i].size());
            for(int j = 0; j < inversePosition.getFront()[i].size(); j++){
                diffs[i][j]=(inversePosition.getFront()[i][j]-abs(currentPose[j]));
            }
        }
        
        for(int i = 0; i < diffs.size(); i++){
            for(int j = 0; j < diffs[i].size(); j++){
                if(diffs[i][j] < minDistances[i]){
                    nearestSolution[j] = i;
                    minDistances[i] = diffs[i][j];
                    distance = minDistances[i];
                }
            }
        }
        vector<int> count;
        count.resize(inversePosition.getFront().size());
        for(int i =0; i < nearestSolution.size();i++){
            count[nearestSolution[i]]++;
        }
        int nearest = INT_MIN;
        int max = INT_MIN;
        for(int i = 0; i < count.size(); i++){
            if(count[i] > max){
                nearest = i;
                max = count[i];
            }
        }
        ofLog(OF_LOG_VERBOSE)<<"nearest "<<nearest<<endl;
        //        if(inversePosition.size() >= 7)
        //            return nearest;
        //        else
        return 0;
        //        return nearest;
    }else{
        return -1;
    }
}

void URMove::urKinematics(vector<double> input){
    if(input.size() == 6){
        urKinematics(input[0], input[1], input[2], input[3], input[4], input[5]);
    }
}

void URMove::urKinematics(ofMatrix4x4 input){
    double q_sols[8*6];
    double* T = new double[16];
    
    T = toUR(input);
    
    int num_sols = kinematics.inverse(T, q_sols);
    preInversePosition = inversePosition.getBack();
    inversePosition.getBack().clear();
    for(int i=0;i<num_sols;i++){
        vector<double> fooSol;
        fooSol.push_back(q_sols[i*6]);
        fooSol.push_back(q_sols[i*6+1]);
        fooSol.push_back(q_sols[i*6+2]);
        fooSol.push_back(q_sols[i*6+3]);
        fooSol.push_back(q_sols[i*6+4]);
        fooSol.push_back(q_sols[i*6+5]);
        inversePosition.getBack().push_back(fooSol);
    }
    
    if(inversePosition.getBack().size() > 0){
        for(int i = 0; i < inversePosition.getBack().size(); i++){
            previews[i]->jointsProcessed.getBack().resize(previews[i]->jointsRaw.getBack().size());
            for(int j = 0; j < previews[i]->joints.size(); j++){
                if(j == 0){
                    inversePosition.getBack()[i][j] = inversePosition.getBack()[i][j]-PI;
                }
                if(j == 1 || j == 3){
                    if(inversePosition.getBack()[i][j] > PI){
                        inversePosition.getBack()[i][j]  = ofMap(inversePosition.getBack()[i][j], PI, TWO_PI, -PI, 0, true);
                    }
                }
                
                previews[i]->jointsRaw.getBack()[j] = inversePosition.getBack()[i][j];
                if(preInversePosition.size() > 0){
                    if(i == selectedSolution){
                        if(preInversePosition[i][j]-inversePosition.getBack()[i][j] > PI){
                            ofLog(OF_LOG_WARNING)<<"JOINT WRAPS SOL "<<ofToString(i)<<" Joint "<<ofToString(j)<<endl;
                        }
                    }
                }
                
                previews[i]->jointsProcessed.getBack()[j] = ofRadToDeg(previews[i]->jointsRaw.getBack()[j]);
                if(j == 1 || j == 3){
                    //correct for -90 zero position in render
                    previews[i]->jointsProcessed.getBack()[j]+=90;
                }
                
                previews[i]->joints[j].rotation.makeRotate(previews[i]->jointsProcessed.getBack()[j], previews[i]->joints[j].axis);
                previews[i]->nodes[j].setOrientation(previews[i]->joints[j].rotation);
            }
            previews[i]->jointsRaw.swapBack();
            previews[i]->jointsProcessed.swapBack();
        }
    }
    inversePosition.swapBack();
}

ofMatrix4x4 URMove::forwardKinematics(vector<double> pose){
    return forwardKinematics(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
}

ofMatrix4x4 URMove::forwardKinematics(double o, double t, double th, double f, double fi, double s){
    double q[6] = {o, t, th, f, fi, s};
    double* T1 = new double[16];
    double* T2 = new double[16];
    double* T3 = new double[16];
    double* T4 = new double[16];
    double* T5 = new double[16];
    double* T6 = new double[16];
    
    kinematics.forward_all(q, T1, T2, T3, T4, T5, T6);
    
    return toOF(T6);
}


vector<double> URMove::getRawJointPos(){
    if(selectedSolution > -1){
        return inversePosition.getFront()[selectedSolution];
    }else{
        return currentPose;
    }
}

void URMove::urKinematics(double o, double t, double th, double f, double fi, double s){
    double q[6] = {o, t, th, f, fi, s};
    double* T = new double[16];
    kinematics.forward(q, T);
    double q_sols[8*6];
    int num_sols = kinematics.inverse(T, q_sols);
    inversePosition.getBack().clear();
    for(int i=0;i<num_sols;i++){
        vector<double> fooSol;
        fooSol.push_back(q_sols[i*6+0]);
        fooSol.push_back(q_sols[i*6+1]);
        fooSol.push_back(q_sols[i*6+2]);
        fooSol.push_back(q_sols[i*6+3]);
        fooSol.push_back(q_sols[i*6+4]);
        fooSol.push_back(q_sols[i*6+5]);
        inversePosition.getBack().push_back(fooSol);
    }
    inversePosition.swapBack();
}

