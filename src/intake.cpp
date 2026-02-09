#include "intake.hpp"
#include "pros/misc.h"

bool intake_type;
void intake_opcontrol(){
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        intake.move(127);
        outtake.move(intake_type ? 0 : Middle.get() ? -127 : 127);
    }else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        intake.move(-127);
        outtake.move(-127);
    }else{
        intake.move(0);
        outtake.move(0);
    }
    Middle.button_toggle(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN));
    Wing.button_toggle(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1));
    MatchLoad.button_toggle(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2));

    master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B) ? intake_type = !intake_type : intake_type = intake_type;
}