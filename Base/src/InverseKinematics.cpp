//
// Created by noahb on 08.03.2022.
//

#include "InverseKinematics.h"
#include "math.h"

/*
 * XYD = posX posY Difference
 * Beta = Angle of Top Motor
 * Alpha = Angle of Bottom Motor
 * Delta = Angle of Base Motor
 */

double InverseKinematics::getBetaAngle(double xyd, double z , double lengthTop, double lengthBottom){
    double firstStage = pow(xyd,2) + pow(z,2) - pow(lengthBottom, 2) - pow(lengthTop, 2);
    double secondStage = firstStage / (2 * lengthBottom * lengthTop);
    double thirdStage = acos(secondStage) * 180 / M_PI;
    return 180 - thirdStage;
}

double InverseKinematics::getAlphaAngle(double xyd, double z, double lengthTop, double lengthBottom, double beta){
    beta = (360-beta)*M_PI/180;
    double firstStage = (lengthTop * sin(beta)) / (lengthBottom + lengthTop * cos(beta));
    double secondStage = (atan(xyd/z) - atan(firstStage)) * 180 / M_PI;
    return 90 - secondStage;
}

double InverseKinematics::getDeltaAngle(double x, double y){
    double firstStage = atan(y/x) * 180 / M_PI;
    if(x < 0){
        firstStage -= 180;
    }
    return firstStage;
}

double InverseKinematics::getDistance(double x1, double y1, double x2, double y2){
    double firstStage = pow((x1 - x2), 2) + pow((y1-y2), 2);
    double secondStage = sqrt(firstStage);
    return secondStage;
}