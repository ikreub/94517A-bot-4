#include "../include/dsr.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"

//https://www.desmos.com/calculator/f8689a01ef
//this explains the numbers below, which are used to measure the offsets of the sensors from the center of the robot.
//(these numbers are converted specifically for the way i coded it so 25.4 times what the graph has)

const bool debug = false;

const double Xa = 1.83195123007;
const double Xb = 4.90508341504;
const double Xc = 3.07313218497;
const double Ya = 6.83693506762;
const double Yb = 11.8419189052;
const double Yc = 4.00498383755;

DSRDS::DSRDS(int port, Dir direction, double offset_x, double offset_y) : sensor(port){
    dir = direction;
    dir_string = dir_to_string(direction);
    x_offset = offset_x;
    y_offset = offset_y;
}

double DSRDS::read_raw(){
    return sensor.get_distance();
}

double DSRDS::read_raw_in(int time_out){
    double reading = 9999;
    for(int i = 0; (reading >= 9999 || reading < 0) && time_out > 0; i++){
        reading = read_raw();
        pros::delay(10);
        time_out -= 10;
        if(debug){
            ez::screen_print(dir_string + ": fail " + util::to_string_with_precision(i) + " value: " + util::to_string_with_precision(reading), 7);
        }
    }
    if(debug){
        //ez::screen_print(dir_string + ": success", 7);
    }
    return reading / 25.4;
}

double DSRDS::read(int time_out){
    double angle = util::to_rad(fmod(chassis.odom_theta_get() + 45, 90) - 45);
    return (read_raw_in(time_out) + y_offset) * cos(angle) - x_offset * sin(angle);
}   

void DSRDS::set_x_offset(double x){
    x_offset = x;
}

void DSRDS::set_y_offset(double y){
    y_offset = y;
}

void DSRDS::set_offsets(double x, double y){
    x_offset = x;
    y_offset = y;
}

void DSRDS::set_dir(Dir direction){
    dir = direction;
    dir_string = dir_to_string(direction);
}

double DSRDS::get_x_offset(){
    return x_offset;
}

double DSRDS::get_y_offset(){
    return y_offset;
}

ez::pose DSRDS::get_offsets(){
    return {x_offset, y_offset};
}

Dir DSRDS::get_dir(){
    return dir;
}

std::string DSRDS::get_dir_string(){
    return dir_string;
}

void DSRDS::measure_offsets(double read45, double read30, double read0){
    y_offset = Ya * read45 - Yb * read30 + Yc * read0;
    x_offset = Xa * read45 - Xb * read30 + Xc * read0;
}