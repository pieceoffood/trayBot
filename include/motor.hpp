#ifndef MOTOR_HPP
#define MOTOR_HPP

pros::Motor left_back;
pros::Motor left_mid;
pros::Motor right_back;
pros::Motor right_mid;

pros::Motor tray;
pros::Motor arm;
pros::Motor left_roller;
pros::Motor right_roller;

void set_tank(int input_l, int input_r);
void set_tray(int input);
void set_arm(int input);
void set_rollers(int input);
void set_tray_pid(int input);
void tray_pid(void*);
void set_arm_pid(int input);
void arm_pid(void*);
void reset();

#endif
