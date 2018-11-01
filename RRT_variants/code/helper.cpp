#include <iostream>
#include "helper.hpp"
#include <math.h>

using namespace std;
// determine if two points are same in K-dim space
bool areConfigsSame(double *config1,double *config2){
    for(int i=0; i<DOF; ++i){
        if (fabs(angDiff(config1[i],config2[i]))!=0){
            return false;
        }
    }
    return true;
}

// // @return difference between two angles within [-pi,pi]
double angDiff(double ang1, double ang2){
    wrapToPI(ang1);
    wrapToPI(ang2);
    double diff=ang1-ang2;
    wrapToPI(diff);
    return diff;
}

double configDiff(double *config1, double *config2){
    double dist=0;
    for(int i=0; i<DOF; ++i){
        dist+=pow(angDiff(config1[i],config2[i]),2);
    }
    return sqrt(dist);
}

void wrapToPI(double &angle){
    angle=angle-2*PI*floor((angle+PI)/(2*PI));
}