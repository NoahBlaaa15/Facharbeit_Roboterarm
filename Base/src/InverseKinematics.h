//
// Created by noahb on 08.03.2022.
//

#ifndef BASE_INVERSEKINEMATICS_H
#define BASE_INVERSEKINEMATICS_H

#include "math.h"

class InverseKinematics {
public:
    constexpr static double lB = 110;
    constexpr static double lT = 145;
    static double getBetaAngle(double xyd, double z, double lengthTop, double lengthBottom);
    static double getAlphaAngle(double xyd, double z, double lengthTop, double lengthBottom, double beta);
    static double getDeltaAngle(double x, double y);
    static double getDistance(double x1, double y1, double x2, double y2);
};


#endif //BASE_INVERSEKINEMATICS_H
