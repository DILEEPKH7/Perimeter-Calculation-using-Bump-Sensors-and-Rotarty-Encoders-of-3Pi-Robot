// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _MOTORS_H
#define _MOTORS_H

#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define FWD LOW
#define REV HIGH


// Class to operate the motor(s).
class Motors_c {
  public:

    // Constructor, must exist.
    Motors_c() {

    } 

    // Use this function to 
    // initialise the pins and 
    // state of your motor(s).
    void initialise() {
      pinMode(L_PWM_PIN,OUTPUT);
      pinMode(R_PWM_PIN,OUTPUT);
      pinMode(L_DIR_PIN,OUTPUT);
      pinMode(R_DIR_PIN,OUTPUT);
      
      pinMode(L_PWM_PIN,20);
      pinMode(R_PWM_PIN,20);
      pinMode(L_DIR_PIN,FWD);
      pinMode(R_DIR_PIN,FWD);

    }

    // Write a function to operate
    // your motor(s)
    // ...
    /*void myMotorCommandFunction(){
      digitalWrite(L_DIR_PIN,FWD);
      digitalWrite(R_DIR_PIN,FWD);
      analogWrite( L_PWM_PIN, 20.1);
      analogWrite( R_PWM_PIN, 20);
    }*/
    void setMotorPower(float left_pwm, float right_pwm){
      /*Serial.print(left_pwm);
      Serial.print(",");
      Serial.print(right_pwm);
      Serial.print("\n");*/
      if (left_pwm<0 && left_pwm>= -255 && right_pwm <0 && right_pwm>=-255){
        digitalWrite(L_DIR_PIN,REV);
        analogWrite( L_PWM_PIN, abs(left_pwm));
        digitalWrite(R_DIR_PIN,REV);
        analogWrite( R_PWM_PIN, abs(right_pwm));
      } else if (right_pwm>=0 &&right_pwm<=255 && left_pwm>=0 && left_pwm<=255){
        digitalWrite(R_DIR_PIN,FWD);
        analogWrite( R_PWM_PIN, abs(right_pwm));
        digitalWrite(L_DIR_PIN,FWD);
        analogWrite( L_PWM_PIN, abs(left_pwm));
        //Serial.print("Motors");
        //Serial.print("\n");
      } else if (left_pwm<0 && left_pwm>=-255 && right_pwm>=0 && right_pwm<=255){
        digitalWrite(R_DIR_PIN,FWD);
        analogWrite( R_PWM_PIN, abs(right_pwm));
        digitalWrite(L_DIR_PIN,REV);
        analogWrite( L_PWM_PIN, abs(left_pwm));
      } else if (left_pwm>=0 && left_pwm<=255 && right_pwm<0 && right_pwm>=-255 ){
        digitalWrite(R_DIR_PIN,REV);
        analogWrite( R_PWM_PIN, abs(right_pwm));
        digitalWrite(L_DIR_PIN,FWD);
        analogWrite( L_PWM_PIN, abs(left_pwm));
      }
    }
    
};



#endif
