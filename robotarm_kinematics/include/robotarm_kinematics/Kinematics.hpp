#ifndef ROBOTARM_KINEMATICS_KINEMATICS_HPP
#define ROBOTARM_KINEMATICS_KINEMATICS_HPP

#include "robotarm_kinematics/KinematicsCore.hpp"

namespace robotarm_kinematics
{

class Kinematics : public KinematicsCore
{
public:
    Kinematics() = default;
    virtual ~Kinematics() = default;
private:

};

}

#endif