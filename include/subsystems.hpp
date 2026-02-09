#pragma once

#include "EZ-Template/api.hpp"
#include "EZ-Template/piston.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor outtake(-2);
inline pros::Motor intake(1);

inline ez::Piston Middle('A');
inline ez::Piston Wing('B');
inline ez::Piston MatchLoad('C');
// inline pros::adi::DigitalIn limit_switch('A');