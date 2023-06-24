// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _KINEMATICS_H
#define _KINEMATICS_H

// Class to track robot position.
class Kinematics_c {
  public:
    float X_M;
    float Y_M;
    float R_M;
    float X_L;
    float Y_L;
    float R_L;
    unsigned long X_M_total;
    unsigned long Y_M_total;
    float R_M_total;
    unsigned long update_ts;
    long count_e1_last; // for difference in encoder counts
    long count_e0_last;
    //float X_L2;
    //float R_L2;
    int conversion;
    float distance = 42.75; //distance between the wheels in mm
    long right_wheel_travelled;
    long left_wheel_travelled;
    // Constructor, must exist.
    Kinematics_c() {

    }

    void initialise() {
      X_M = 0;
      Y_M = 0;
      R_M = 0;
      X_L = 0;
      Y_L = 0;
      R_L = 0;
      /*X_M_total = 0;
        Y_M_total = 0;
        R_M_total = 0;
      */
      count_e1_last = 0;
      count_e0_last = 0;

      update_ts = millis();
    }

    void reset() {
      X_M = 0;
      Y_M = 0;
      R_M = 0;
      X_L = 0;
      Y_L = 0;
      R_L = 0;
    }

    // Use this function to update
    // your kinematics
    void kinematics_cal () {
      right_wheel_travelled = count_e0;
      left_wheel_travelled = count_e1;
      //Serial.print("Right_Encoder:");
      //Serial.print(",");
      //Serial.println(count_e0);
      //Serial.println(right_wheel_travelled);
      //Serial.print("Left_Encoder is:");
      //Serial.print(left_wheel_travelled);
      //Serial.print(",");
      //Serial.println(count_e1);


      unsigned elapsed;
      float diff_e1;
      float diff_e0;

      elapsed = millis() - update_ts;

      //if (elapsed > 20) {
      update_ts = millis();

      diff_e1 = (count_e1 - count_e1_last) * 0.28;
      count_e1_last = count_e1;

      diff_e0 = (count_e0 - count_e0_last) * 0.28;
      count_e0_last = count_e0;
      X_L = (diff_e0 / 2) + (diff_e1 / 2);
      Y_L = 0;
      R_L = ((diff_e1  - diff_e0) / (2 * distance));
      /*Serial.print("right_difference_ is:");
        Serial.print(diff_e0);
        Serial.print("\n");
        Serial.print("left_difference_ is:");
        Serial.print(diff_e1);
        Serial.print("\n");*/
      /*Serial.print("X_local_coordinate_ is:");
      Serial.print(X_L);
      Serial.print("\n");
      Serial.print("R_local_coordinate_ is:");
      Serial.print(R_L);
      Serial.print("\n");*/
      /*if(R_L > 360 || R_L < -360){
        conversion =  R_L;
        R_L = conversion % 360;
        R_M = R_M + R_L;
        //R_M = R_M_total;
        } else {
        R_M = R_M + R_L;
        //R_M = R_M_total;
        }*/
      X_M = X_M + X_L * cos(R_M);
      Y_M = Y_M + X_L * sin(R_M);
      R_M = R_M + R_L;
      /*Serial.print("right_wheel_travelled is ");
        Serial.print(right_wheel_travelled);
        Serial.print("\n");
        Serial.print("left_wheel_travelled is ");
        Serial.print(left_wheel_travelled);
        Serial.print("\n");*/

      /*Serial.print("R_main_coordinate_ is:");
      Serial.println(R_M);
      Serial.print("X_main_coordinate_ is:");
      Serial.println(X_M);
      Serial.print("Y_main_coordinate_ is:");
      Serial.println(Y_M);*/
      //}
    }

    /*right_wheel_travelled = count_e0*0.28; // distance travelled in mm
      left_wheel_travelled = count_e1*0.28; // distnace travelled in mm
      //X_L2 = X_L;
      //R_L2 = R_L;
      X_L = (left_wheel_travelled/2) + (right_wheel_travelled/2);
      Y_L = 0;
      R_L = (((left_wheel_travelled/(2*distance)) - (right_wheel_travelled/(2*distance)))*57.2958);
      if(R_L > 360 || R_L < -360){
      conversion =  R_L;
      R_L = conversion % 360;
      R_M = R_L;
      } else {
      R_M = R_L;
      }
      X_M = X_M + X_L*cos(R_M/57.2958);
      Y_M = Y_M + X_L*sin(R_M/57.2958);
      R_M = R_M + R_L;

      Serial.print("right_wheel_travelled is ");
      Serial.print(right_wheel_travelled);
      Serial.print("\n");
      Serial.print("left_wheel_travelled is ");
      Serial.print(left_wheel_travelled);
      Serial.print("\n");
      Serial.print("X_local_coordinate_ is:");
      Serial.print(X_L);
      Serial.print("\n");
      Serial.print("R_local_coordinate_ is:");
      Serial.print(R_L);
      Serial.print("\n");
      Serial.print("R_main_coordinate_ is:");
      Serial.println(R_M);
      Serial.print("X_main_coordinate_ is:");
      Serial.println(X_M);
      Serial.print("Y_main_coordinate_ is:");
      Serial.println(Y_M);

      }*/

    // Use this function to update
    // your kinematics
    /*    void update() {
          R_M = R_M + R_L;
          X_M = X_M + X_L*cos(R_M);
          Y_M = Y_M + X_L*sin(R_M);
          Serial.print("R_main_coordinate_ is:");
          Serial.println(R_M);
          Serial.print("X_main_coordinate_ is:");
          Serial.println(X_M);
          Serial.print("Y_main_coordinate_ is:");
          Serial.println(Y_M);
        }
    */
};



#endif
