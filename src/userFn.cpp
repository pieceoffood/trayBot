#include "main.h"
#include "cmath"
#include "motor.hpp"
#include "MiniPID.h"

void basemovePID(double target) {
  MiniPID pid=MiniPID(0.3,0,0.1);
  char mytext[100];
  lv_obj_t * txt = lv_label_create(lv_scr_act(), NULL);

  pid.setOutputLimits(-80,80);
  pid.setOutputRampRate(5);
  double start=left_front.get_position();
  double ticks = (target*900)/(4*M_PI)+start;
  while (fabs(left_front.get_position()-ticks)>10) {
    double output=pid.getOutput(left_front.get_position(),
        ticks);
    left_back.move(output);
    left_front.move(output);
    right_back.move(output);
    right_front.move(output);

    printf("base start %8.2f, target %8.2f, base %8.2f\n", start, ticks,left_front.get_position());
    sprintf(mytext, "base start %8.2f\n, target %8.2f\n, base %8.2f\n, output  base %8.2f\n",
            start, ticks,left_front.get_position(), output
         );
    lv_label_set_text(txt, mytext);
    pros::delay(10);
  }
}

void baseturnPID(double target) {
  MiniPID pid=MiniPID(0.3,0,0.1);
  char mytext[100];
  lv_obj_t * txt = lv_label_create(lv_scr_act(), NULL);

  pid.setOutputLimits(-50,50);
  pid.setOutputRampRate(5);
  double start=gyro.get_value();
  double turn = target*10+start;
  while (fabs(gyro.get_value()-turn)>3) {
    double output=pid.getOutput(gyro.get_value(),
        turn);
    left_back.move(output);
    left_front.move(output);
    right_back.move(-output);
    right_front.move(-output);
    lv_label_set_text(txt, NULL);
    printf("base start %8.2f, target %8.2f, gyro %8.2f\n", start, turn,gyro.get_value());
    sprintf(mytext, "\n\n\n\n\ngyro start %8.2f\n, target %8.2f\n, gyro %8.2f\n", start, turn,gyro.get_value()
         );
    lv_label_set_text(txt, mytext);
    pros::delay(10);
  }
}


void basemovement(double distance, int speed)
{
  double ticks=(distance*900)/(4*M_PI);
  left_front.move_relative  (ticks, speed);
  left_back.move_relative   (ticks, speed);
  right_front.move_relative (ticks, speed);
  right_back.move_relative  (ticks, speed);
}

void baseturn(int left, int speed) // right=positive and left=negative
{
  double ticks=735*left;
  left_front.move_relative  (ticks, speed);
  left_back.move_relative   (ticks, speed);
  right_front.move_relative (-ticks, speed);
  right_back.move_relative  (-ticks, speed);
}




//Math
int sgn(int input) {
  if (input>0)
    return 1;
  else if (input < 0)
    return -1;
  return 0;
}
int clipnum(int input, int clip) {
  if (input > clip)
    return clip;
  else if (input < -clip)
    return -clip;
  return input;
}

void reset() {
  left_roller.set_brake_mode(MOTOR_BRAKE_HOLD);
  right_roller.set_brake_mode(MOTOR_BRAKE_HOLD);
  arm.set_brake_mode(MOTOR_BRAKE_HOLD);
  arm.set_zero_position(0);
  tray.set_zero_position(0);
}

void set_tray(int input) {
  tray.move(input);
}

void set_arm(int input) {
  arm.move(input);
}

void set_rollers(int input) {
  left_roller.move(input);
  right_roller.move(input);
}

//PID
int t_target;
void set_tray_pid(int input) {
  t_target = input;
}

void tray_pid(void*) {
	while (true) {
		set_tray((t_target-tray.get_position())*0.5);
		pros::delay(20);
	}
}

int a_target;
void set_arm_pid(int input) {
  a_target = input;
}
void arm_pid(void*) {
  while (true) {
    set_arm((a_target-arm.get_position())*0.5);
    pros::delay(20);
  }
}








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
       pros::delay(10);
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
     set_arm_pid(1300);
   } else if (master.get_digital(DIGITAL_DOWN)) {
     was_pid = true;
     arm_t.resume();
     set_arm_pid(900);
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
