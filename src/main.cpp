#include "main.h"
#include <iomanip>
#include "motor.hpp"
#include "okapi/api.hpp"
#include "gui.h"
#include "MiniPID.h"

/*
cd (change directory)
cd .. (go up one level)
prosv5 make clean (clean everything)
prosv5 build-compile-commands (compile the code)
prosv5 upload --slot 5 (upload the program to V5 slot 5)
prosv5 v5 rm-all
prosv5 v5 rm-file slot_4.bin --erase-all
*/

int8_t lf=11;
int8_t lb=12;
int8_t rf=20;
int8_t rb=19; // do not need to reverse if reversed in the motor define
MiniPID pid=MiniPID(0.3,0,0.1);

using namespace okapi;
auto drive = okapi::ChassisControllerBuilder()
        .withMotors(
          {lf, lb},
          {rf, rb}
        )
        // Green gearset, 4 in wheel diam, 11.5 in wheel track
        .withDimensions(AbstractMotor::gearset::green, {{4_in, 10_in}, imev5GreenTPR })
        .withOdometry() // Use the same scales as the chassis (above)
        .withGains(
       {0.001, 0, 0.0001}, // Distance controller gains
       {0.001, 0, 0.0001}, // Turn controller gains
       {0.001, 0, 0.0001}  // Angle controller gains (helps drive straight)
     )
        .buildOdometry()
;
const double liftkP = 0.001;
const double liftkI = 0.0001;
const double liftkD = 0.0001;

auto liftController = AsyncPosControllerBuilder()
                       .withMotor(5) // lift motor port 3
                       .withGains({liftkP, liftkI, liftkD})
                       .build();

void basemove(double target) {

  pid.setOutputLimits(-70,70);
  pid.setOutputRampRate(5);
  double start=left_front.get_position();
  double ticks = (target*900)/(4*M_PI)+start;
  while (fabs(left_front.get_position()-ticks)>5) {
    double output=pid.getOutput(left_front.get_position(),
        ticks);
    left_back.move(output);
    left_front.move(output);
    right_back.move(output);
    right_front.move(output);
    pros::delay(10);
  }
}

void baseturn(double target) {

  pid.setOutputLimits(-50,50);
  pid.setOutputRampRate(5);
  double start=gyro.get_value();
  double turn = target*10+start;
  while (true) {
    double output=pid.getOutput(gyro.get_value(),
        turn);
    left_back.move(output);
    left_front.move(output);
    right_back.move(-output);
    right_front.move(-output);
    pros::delay(10);
  }
}


void competition_initialize() {

}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::delay(10);
  gui();

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */


/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {



  switch (auton_sel) {
    case 1:
      drive->setMaxVelocity(100);
      drive->moveDistance(24_in); // Drive forward 24 inches
      drive->turnAngle(90_deg);   // Turn in place 90 degrees
      //liftController->setTarget(200); // Move 200 motor degrees upward
      drive->waitUntilSettled();
      drive->moveDistance(-12_in); // Drive backwrad 12 inches
      // auton1();
    break;
    case 2:
      // auton2();
      basemove(12);
      pros::delay(2000);
      baseturn(90);
    break;
    case 3:
      // auton3();
    break;
    case 4:
      // auton4();
    break;
    default:
      // empty_auton();
    break;
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

 void  tray_control(void*) {
 	pros::Controller master(CONTROLLER_MASTER);
 	pros::Task tray_t(tray_pid);
 	bool b_toggle = false;
 	while (true) {
 		if (master.get_digital(DIGITAL_Y)) {
 			b_toggle = !b_toggle;

 			if (b_toggle) {
 				for(int i=0;i<1700;i=i+3) {
 					set_tray_pid(i);
 					pros::delay(5);
 				}
 			} else {
 				set_tray_pid(0);
 			}

 			while (master.get_digital(DIGITAL_Y)) {
 				pros::delay(1);
 			}
 		}

 		pros::delay(20);
 	}
 }

 //Make the right paddle a shift key, making everything into a descore position.
 /*
 void
 arm_control(void*) {
 	pros::Controller master(CONTROLLER_MASTER);
 	pros::Task arm_t(arm_pid);
 	static int HIGH_POLE = 2450, LOW_POLE = 1000, DOWN = 0;
 	int b_toggle = DOWN;
 	while (true) {
 		if (master.get_digital(DIGITAL_R1)) {
 			if (b_toggle == DOWN || b_toggle == LOW_POLE) {
 				b_toggle = HIGH_POLE;
 			} else {
 				b_toggle = DOWN;
 			}
 			set_arm_pid(b_toggle);
 			while (master.get_digital(DIGITAL_R1)) {
 				pros::delay(1);
 			}
 		}
 		if (master.get_digital(DIGITAL_R2)) {
 			if (b_toggle == DOWN || b_toggle == HIGH_POLE) {
 				b_toggle = LOW_POLE;
 			} else {
 				b_toggle = DOWN;
 			}
 			set_arm_pid(b_toggle);
 			while (master.get_digital(DIGITAL_R2)) {
 				pros::delay(1);
 			}
 		}
 		pros::delay(20);
 	}
 }
 */

 void  arm_control(void*) {
 	pros::Controller master(CONTROLLER_MASTER);
 	pros::Task arm_t(arm_pid);
 	bool was_pid;
 	while (true) {
 		if (master.get_digital(DIGITAL_B)) {
 			was_pid = true;
 			arm_t.resume();
 			set_arm_pid(2300);
 		} else if (master.get_digital(DIGITAL_DOWN)) {
 			was_pid = true;
 			arm_t.resume();
 			set_arm_pid(1800);
 		} else {
 			if (master.get_digital(DIGITAL_R1)||master.get_digital(DIGITAL_R2)) {
 				was_pid = false;
 				set_arm((master.get_digital(DIGITAL_R1)-master.get_digital(DIGITAL_R2))*80);// set arm to slow speed
 			} else {
 				if (!was_pid) {
 					set_arm(0);
 				}
 			}
 		}

 		if (!was_pid) {
 			arm_t.suspend();
 		}

 		pros::delay(20);
 	}
 }

 void  opcontrol() {

  std::cout << std::fixed;
  std::cout << std::setprecision(1);
  char mytext[100];
  bool allowPress=true;

  /*Create a screen*/
  lv_obj_t * scr = lv_obj_create(NULL, NULL);
  lv_scr_load(scr);                                   /*Load the screen*/

  lv_obj_t * title = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(title, "Debug");
  lv_obj_align(title, NULL, LV_ALIGN_IN_TOP_MID, 0, 2);  /*Align to the top*/

  /*Create a new style*/
  /*
  static lv_style_t style_txt;
  lv_style_copy(&style_txt, &lv_style_plain);
  style_txt.text.font = &lv_font_dejavu_20;
  style_txt.text.letter_space = 2;
  style_txt.text.line_space = 1;
  style_txt.text.color = LV_COLOR_HEX(0x606060);
  */

  /*Create a new label*/
  lv_obj_t * txt = lv_label_create(lv_scr_act(), NULL);
  //lv_obj_set_style(txt, &style_txt);                    /*Set the created style*/
  lv_label_set_long_mode(txt, LV_LABEL_LONG_BREAK);     /*Break the long lines*/
  lv_label_set_recolor(txt, true);                      /*Enable re-coloring by commands in the text*/
  lv_label_set_align(txt, LV_LABEL_ALIGN_LEFT);       /*Center aligned lines*/
  lv_label_set_text(txt, NULL);
  lv_obj_set_width(txt, 500);                           /*Set a width*/
  lv_obj_align(txt, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 20);      /*Align to center*/




 	pros::Controller master(CONTROLLER_MASTER);
 	pros::Task tray_control_t(tray_control);


 	pros::Task t(arm_control);
 	while (true) {

    sprintf(mytext, "claw potentiameter: %d, claw %8.2f \n"
                "tray: %8.2f, set zero: %d\n"
                "leftfront:%8.2f rightfront:%8.2f\n"
                "gyro:%8.2f\n",
       potentiameter.get_value(),
       arm.get_position(),
       tray.get_position(), limitswitch.get_value(),
       left_front.get_position(), right_front.get_position(),
       gyro.get_value()
     );
lv_label_set_text(txt, mytext);
 		//set_tank(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));

 		//set_arm((master.get_digital(DIGITAL_R1)-master.get_digital(DIGITAL_R2))*127);
    Controller controller;

    drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY),
                          controller.getAnalog(ControllerAnalog::rightX));

 		if (master.get_digital(DIGITAL_L1)) {
 			set_rollers(127);
 		} else if (master.get_digital(DIGITAL_L2)) {
 			set_rollers(-127);
 		} else if (master.get_digital(DIGITAL_RIGHT)) {
 			set_rollers(60);
 		} else {
 			set_rollers(0);
 		}

 		pros::delay(20);
 	}
 }
