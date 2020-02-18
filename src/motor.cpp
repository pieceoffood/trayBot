#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor left_back (2, MOTOR_GEARSET_18);
pros::Motor left_front (1, MOTOR_GEARSET_18);
pros::Motor right_back (17, MOTOR_GEARSET_18,true);
pros::Motor right_front (16, MOTOR_GEARSET_18, true);

pros::Motor tray(20, MOTOR_GEARSET_18, true);
pros::Motor arm(10, MOTOR_GEARSET_18, true);
pros::Motor left_roller(8, MOTOR_GEARSET_18);
pros::Motor right_roller(9, MOTOR_GEARSET_18, true);

pros::ADIDigitalIn limitswitch  (1);
pros::ADIAnalogIn potentiameter (2);
pros::ADIGyro gyro (3);
