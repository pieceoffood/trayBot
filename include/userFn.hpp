#ifndef USERFN_H
#define USERFN_H


void baseturnPID(double target);
void basemovePID(double target);


void  tray_control(void*) ;
void  arm_control(void*) ;

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
