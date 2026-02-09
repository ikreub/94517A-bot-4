#pragma once

#include "EZ-Template/api.hpp"
#include "EZ-Template/piston.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor outtake(-8);
inline pros::Motor intake(6);

inline ez::Piston Middle('A');
inline ez::Piston Wing('D');
inline ez::Piston MatchLoad('C');
inline ez::Piston LowScore('B');
