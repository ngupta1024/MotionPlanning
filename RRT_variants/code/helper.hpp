#ifndef HELPER_H
#define HELPER_H

const int DOF= 5;

#ifndef PI
#define PI 3.141592654
#endif

bool areConfigsSame(double *config1,double *config2);
double angDiff(double ang1, double ang2);
double configDiff(double *config1, double *config2);
void wrapToPI(double &angle);

#endif