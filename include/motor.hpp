#ifndef MOTOR_HPP
#define MOTOR_HPP

extern pros::Motor left_back;
extern pros::Motor left_mid;
extern pros::Motor right_back;
extern pros::Motor right_mid;

extern pros::Motor tray;
extern pros::Motor arm;
extern pros::Motor left_roller;
extern pros::Motor right_roller;
extern int8_t lf, lb, rf, rb;

void set_tank(int input_l, int input_r);
void set_tray(int input);
void set_arm(int input);
void set_rollers(int input);
void set_tray_pid(int input);
void tray_pid(void*);
void set_arm_pid(int input);
void arm_pid(void*);
void reset();

extern int sgn(int input);
extern int clipnum(int input, int clip);
void reset();
void set_tank(int input_l, int input_r);
void set_tray(int input);
void set_arm(int input);
void set_rollers(int input);
extern int t_target;
void set_tray_pid(int input);
void tray_pid(void*) ;
extern int a_target;
void set_arm_pid(int input);
void arm_pid(void*);

#endif
