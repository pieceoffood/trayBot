#include "main.h"
#include <iomanip>
#include "motor.hpp"
#include "okapi/api.hpp"
#include "gui.h"
#include "MiniPID.h"
#include "userFn.hpp"

/*
cd (change directory)
cd .. (go up one level)
prosv5 make clean (clean everything)
prosv5 build-compile-commands (compile the code)
prosv5 upload --slot 5 (upload the program to V5 slot 5)
prosv5 v5 rm-all
prosv5 v5 rm-file slot_4.bin --erase-all
*/

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
  lv_ex_tabview_1();

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
  lv_tabview_set_tab_act(tabview, 1, LV_ANIM_NONE);

  using namespace okapi;
  int8_t lf=11;
  int8_t lb=12;
  int8_t rf=20;
  int8_t rb=19; // do not need to reverse if reversed in the motor define

  auto drive = okapi::ChassisControllerBuilder()
          .withMotors(
            {lf, lb},
            {rf, rb}
          )
          // Green gearset, 4 in wheel diam, 11.5 in wheel track
          .withDimensions(AbstractMotor::gearset::green, {{4_in, 10_in}, imev5GreenTPR })
          .withOdometry() // Use the same scales as the chassis (above)
          .withGains(
         {0.00128, 0.00000, 0.0000}, // Distance controller gains
         {0.00128, 0.00000, 0.0000}, // Turn controller gains
         {0.00128, 0.00000, 0.0000}  // Angle controller gains (helps drive straight)
       )
          .buildOdometry()
  ;
  //The model pointer stuff is currently necessary to use the xArcade method of the chassis controller.
  //auto xModel = std::dynamic_pointer_cast<XDriveModel>(drive->getModel());
  const double liftkP = 0.00628;
  const double liftkI = 0.0001;
  const double liftkD = 0.0001;

  auto liftController = AsyncPosControllerBuilder()
                         .withMotor(5) // lift motor port 3
                         .withGains({liftkP, liftkI, liftkD})
                         .build();


  auto profileController =AsyncMotionProfileControllerBuilder()
                         .withLimits({0.5, 1.0, 5.0})
                         .withOutput(drive)
                         .buildMotionProfileController();

  switch (auton_sel) {
    case 1:
      // auton1();
      left_roller.move(120);
      right_roller.move(120);
      basemovePID(39);
      pros::delay(10);
      basemovePID(-5);
      pros::delay(10);
      left_roller.move(0);
      right_roller.move(0);
      baseturnPID(-45);
      basemovePID(-24*1.414);
      baseturnPID(45);
      pros::delay(10);
      left_roller.move(120);
      right_roller.move(120);
      basemovePID(30);
      left_roller.move(0);
      right_roller.move(0);

    break;
    case 2:
      // auton2();
      basemovePID(12);
      //pros::delay(2000);
      baseturnPID(90);
    break;
    case 3:
      // auton3();
      drive->setMaxVelocity(100);
      drive->moveDistance(24_in); // Drive forward 24 inches
      drive->waitUntilSettled();
      drive->turnAngle(90_deg);   // Turn in place 90 degrees
      drive->waitUntilSettled();
      //liftController->setTarget(200); // Move 200 motor degrees upward
      drive->waitUntilSettled();
      drive->moveDistance(-12_in); // Drive backwrad 12 inches
    break;
    case 4:
      // auton4();
      profileController->generatePath({
        {0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
        {1_ft, 1_ft, 90_deg}}, // The next point in the profile, 3 feet forward
        "A" // Profile name
      );
      profileController->generatePath({
        {0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
        {3_ft, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
        "B" // Profile name
      );
      profileController->generatePath({
        {0_ft, 0_ft, 0_deg},  // Profile starting position, this will normally be (0, 0, 0)
        {3_ft, 0_ft, 0_deg}}, // The next point in the profile, 3 feet forward
        "C" // Profile name
      );
      profileController->setTarget("A");
      profileController->waitUntilSettled();
      profileController->setTarget("B");
      profileController->waitUntilSettled();
      profileController->setTarget("C", true); // backward
      profileController->waitUntilSettled();

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

 void  opcontrol() {
  std::cout << std::fixed;
  std::cout << std::setprecision(1);
  char mytext[100];
  bool allowPress=true;

  /*Create a screen*/
  lv_tabview_set_tab_act(tabview, 2, LV_ANIM_NONE);

 	pros::Controller master(CONTROLLER_MASTER);
  pros::Task tray_control_t(tray_control);
  pros::Task t(arm_control);

  master.clear();
  master.print(0, 0, "VEX");
  auto timeFlag=pros::millis();

 	while (true) {
    if(pros::millis()-timeFlag>=1000)
             {
                    master.print(1, 0, "arm:%8.2f", arm.get_position());
                    timeFlag=pros::millis();
             }

    sprintf(mytext, "arm potentiameter: %d, arm %8.2f \n"
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
    /*
    xModel->xArcade(
          controller.getAnalog(ControllerAnalog::rightX),
          controller.getAnalog(ControllerAnalog::rightY),
          controller.getAnalog(ControllerAnalog::leftX));
    */
    /*
    drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY),
                          controller.getAnalog(ControllerAnalog::rightX));

    */
 		if (master.get_digital(DIGITAL_L1)) {
 			set_rollers(127);
 		} else if (master.get_digital(DIGITAL_L2)) {
 			set_rollers(-127);
 		} else if (master.get_digital(DIGITAL_RIGHT)) {
 			set_rollers(-60);
 		} else {
 			set_rollers(0);
 		}

 		pros::delay(10);
 	}
 }
