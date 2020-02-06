#ifndef USERFN_H
#define USERFN_H


void baseturnPID(double target);
void basemovePID(double target);
void basemove(double distance, int speed);

void  tray_control(void*) ;
void  arm_control(void*) ;




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




void basemovement(double distance, int speed);

void baseturn(int left, int speed);

void moving (double distance, int speed);

void turning (int left, int speed);

void turnDegree(double degree, int speed);

void gyroTurn(double degree, int speed); // right=positive and let=negative

void ballLoad();

void clawmove(int openclose, int speed); //open=1 close=-1

void flip();

#endif
