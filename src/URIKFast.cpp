#include "URIKFast.h"
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
URIKFast::URIKFast(){
    
}
URIKFast::~URIKFast(){
    
}

/// \brief Converts a 4x4 matrix to a 1D array
/// \param input ofMatrix4x4 to convert
/// \return row-major array in UR World Cords
double* toUR(ofMatrix4x4 input){
    double* T = new double[16];
    cout<<"toUR ==================="<<endl;
    
    cout<<"ofMatrix ==================="<<endl;
    cout<<input<<endl;
    
    for(int i = 0; i < 4; i++){
        T[i] = (double)input._mat[i][0];
        T[i+(4)] = (double)input._mat[i][1];
        T[i+(8)] = (double)input._mat[i][2];
        T[i+(12)] = (double)input._mat[i][3];
    }
    
    
    
    
    for(int i=0;i<4;i++) {
        for(int j=i*4;j<(i+1)*4;j++)
            printf("%1.3f ", T[j]);
        printf("\n");
    }
    
    
    
    cout<<"end toUR ==================="<<endl;
    return T;
}

ofMatrix4x4 toOF(double * T){
    ofMatrix4x4 output;
//    cout<<"toOF ==================="<<endl;
//    for(int i=0;i<4;i++) {
//        for(int j=i*4;j<(i+1)*4;j++)
//            printf("%1.3f ", T[j]);
//        printf("\n");
//    }
    
    
//    cout<<"ofMatrix ==================="<<endl;
    for(int i = 0; i < 4; i++){
        output._mat[i][0] = T[i];
        output._mat[i][1] = T[i+(4)];
        output._mat[i][2] = T[i+(8)];
        output._mat[i][3] = T[i+(12)];
    }
//    cout<<output<<endl;
    return output;
}

int URIKFast::selectSolution(vector<vector<double> > & inversePosition)
{
    int selectedSolution = 0;
    
    for(int i = 0; i < inversePosition.size(); i++){
        for(int j = 0; j < inversePosition[i].size(); j++){
            if(j == 0){
                inversePosition[i][j] = inversePosition[i][j]-PI;
            }
            if(j == 1 || j == 3){
                if(inversePosition[i][j] > PI){
                    inversePosition[i][j]  = ofMap(inversePosition[i][j], PI, TWO_PI, -PI, 0, true);
                }
            }
            if(preInversePosition.size() > 0){
                if(i == selectedSolution){
                    if(preInversePosition[i][j]-inversePosition[i][j] > PI){
                        ofLog(OF_LOG_WARNING)<<"JOINT WRAPS SOL "<<ofToString(i)<<" Joint "<<ofToString(j)<<endl;
                    }
                }
            }
        }
    }
    preInversePosition = inversePosition;
    if(inversePosition.size() > 0){
        return 0;
    }else{
        return -1;
    }
}


vector<vector<double> > URIKFast::inverseKinematics(double o, double t, double th, double f, double fi, double s)
{
    double q[6] = {o, t, th, f, fi, s};
    double* T = new double[16];
    double q_sols[8*6];
    int num_sols = kinematics.inverse(T, q_sols);
    vector<vector<double> > sols;
    for(int i=0;i<num_sols;i++){
        vector<double> fooSol;
        fooSol.push_back(q_sols[i*6]);
        fooSol.push_back(q_sols[i*6+1]);
        fooSol.push_back(q_sols[i*6+2]);
        fooSol.push_back(q_sols[i*6+3]);
        fooSol.push_back(q_sols[i*6+4]);
        fooSol.push_back(q_sols[i*6+5]);
        sols.push_back(fooSol);
    }
    return sols;
}


vector<vector<double> > URIKFast::inverseKinematics(vector<double> input)
{
    if(input.size() == 6){
        return inverseKinematics(input[0], input[1], input[2], input[3], input[4], input[5]);
    }
    return vector<vector<double>>();
}
vector<vector<double> > URIKFast::inverseKinematics(Joint pose){
    ofMatrix4x4 matPose;
    matPose.setTranslation(pose.position);
    matPose.setRotate(pose.rotation);
    return inverseKinematics(matPose);
    
}
vector<vector<double> > URIKFast::inverseKinematics(ofMatrix4x4 pose)
{
    double q_sols[8*6];
    double* T = new double[16];
    T = toUR(pose);
    int num_sols = kinematics.inverse(T, q_sols);
    vector<vector<double> > sols;
    for(int i=0;i<num_sols;i++){
        vector<double> fooSol;
        fooSol.push_back(q_sols[i*6]);
        fooSol.push_back(q_sols[i*6+1]);
        fooSol.push_back(q_sols[i*6+2]);
        fooSol.push_back(q_sols[i*6+3]);
        fooSol.push_back(q_sols[i*6+4]);
        fooSol.push_back(q_sols[i*6+5]);
        sols.push_back(fooSol);
    }
    return sols;
}

ofMatrix4x4 URIKFast::forwardKinematics(vector<double> pose)
{
    return toOF(forwardKinematics(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]));
}

double* URIKFast::forwardKinematics(double o, double t, double th, double f, double fi, double s)
{
    double q[6] = {o, t, th, f, fi, s};
    double* T1 = new double[16];
    double* T2 = new double[16];
    double* T3 = new double[16];
    double* T4 = new double[16];
    double* T5 = new double[16];
    double* T6 = new double[16];
    
    kinematics.forward_all(q, T1, T2, T2, T4, T5, T6);
    return T6;
}

