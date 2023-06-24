// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

#define LS_LEFT_IN_PIN A0
#define LS_CENTRE_IN_PIN A2
#define LS_RIGHT_IN_PIN A3
#define LS_LEFT_CORNER 12
#define LS_RIGHT_CORNER A4

#define NB_LS_PINS 5
//#define caliberation_range 20

#define EMIT 11


// Class to operate the linesensor(s).
class LineSensor_c {
  public:
    int ls_pin[NB_LS_PINS] = { LS_LEFT_IN_PIN, LS_CENTRE_IN_PIN, LS_RIGHT_IN_PIN, LS_LEFT_CORNER, LS_RIGHT_CORNER };
    float minimum_value[NB_LS_PINS];
    float maximum_value[NB_LS_PINS];
    float previous_minimum[NB_LS_PINS];
    float previous_maximum[NB_LS_PINS];
    unsigned long previous_values[NB_LS_PINS];
    float range[NB_LS_PINS];
    float s_factor[NB_LS_PINS] = {0.00,0.00,0.00,0.00,0.00};//{0.000340483,0.000357142,0.000314960,0.000539083,0.000324675};
    float bias[NB_LS_PINS] = {0,0,0,0,0};//{728,660,780,885,1070};
    long previous_range[NB_LS_PINS];
    long real_range[NB_LS_PINS];
    float real_sensor_read[ NB_LS_PINS ];
    float sensor_read[ NB_LS_PINS ];
    //float minimum_values[caliberation_range][NB_LS_PINS];

    float e_line;
    float w_left;
    float w_right;
    float sum;
    float sum_all;
    float normalise_left;
    float normalise_centre;
    float normalise_right;
    float turn_pwm;
    long right_encoder;
    long left_encoder;
    int sensor_count_a;
    int sensor_count_b;
    
    // Constructor, must exist.
    LineSensor_c() {
    } 
    
    void initialise(){
      pinMode(LS_LEFT_IN_PIN,INPUT);
      pinMode(LS_CENTRE_IN_PIN, INPUT);
      pinMode(LS_RIGHT_IN_PIN, INPUT);
      pinMode(LS_LEFT_CORNER, INPUT);
      pinMode(LS_RIGHT_CORNER, INPUT);
      pinMode(EMIT,OUTPUT);

      digitalWrite(EMIT,LOW);
      digitalWrite(LS_LEFT_IN_PIN,HIGH);
      digitalWrite(LS_CENTRE_IN_PIN, HIGH);
      digitalWrite(LS_RIGHT_IN_PIN, HIGH);
      digitalWrite(LS_LEFT_CORNER, HIGH);
      digitalWrite(LS_RIGHT_CORNER, HIGH);


      range[0] = 0;
      range[1] = 0;
      range[2] = 0;
      range[3] = 0;
      range[4] = 0;

      sensor_count_a =0;
      sensor_count_b = 0; 

      minimum_value[0] = 1199.00;
      minimum_value[1] = 1199.00;
      minimum_value[2] = 1199.00;
      minimum_value[3] = 1199.00;
      minimum_value[4] = 1199.00;

      maximum_value[0] = 1.00;
      maximum_value[1] = 1.00;
      maximum_value[2] = 1.00;
      maximum_value[3] = 1.00;
      maximum_value[4] = 1.00;

      previous_minimum[0] = 1199.00;
      previous_minimum[1] = 1199.00;
      previous_minimum[2] = 1199.00;
      previous_minimum[3] = 1199.00;
      previous_minimum[4] = 1199.00;

      previous_maximum[0] = 1199.00;
      previous_maximum[1] = 1199.00;
      previous_maximum[2] = 1199.00;
      previous_maximum[3] = 1199.00;
      previous_maximum[4] = 1199.00;

      //Caliberation set up
      
    }

    void caliberation(){
      int which;
      int caliberation_range=20;
      while(caliberation_range>0){
      readLineSensor();
      for( which = 0; which < NB_LS_PINS; which++ ){
        if (sensor_read[which]<minimum_value[which]){
          minimum_value[which] = sensor_read[which];
          bias[which] = minimum_value[which];
          //previous_minimum[which]=minimum_value[which];
        }
        else if (sensor_read[which]>previous_maximum[which]){
          maximum_value[which] = sensor_read[which];
          previous_maximum[which]=sensor_read[which];
        }
        range[which] = maximum_value[which]-minimum_value[which];
        s_factor[which] = ((float)1.00/((float)range[which]));        
      }
      caliberation_range = caliberation_range - 1;
      /*
      Serial.print("Left line sensor 2: " );
      Serial.print(sensor_read[0]);
      Serial.print(",");
      Serial.print(real_sensor_read[0]);
      Serial.print(",");
      Serial.print(bias[0]);
      Serial.print(",");
      Serial.print(minimum_value[0]);
      Serial.print(",");
      Serial.print(maximum_value[0]);
      Serial.print(",");
      Serial.print(range[0]);
      Serial.print(",");
      Serial.print(s_factor[0]);
      Serial.print("\n");
      Serial.print("Centre line sensor 3: " );
      Serial.print(sensor_read[1]);
      Serial.print(",");
      Serial.print( real_sensor_read[1]);
      Serial.print(",");
      Serial.print(bias[1]);
      Serial.print(",");
      Serial.print(minimum_value[1]);
      Serial.print(",");
      Serial.print(maximum_value[1]);
      Serial.print(",");
      Serial.print(range[1]);
      Serial.print(",");
      Serial.print(s_factor[1]);
      Serial.print("\n");
      Serial.print("Right line sensor 4: " );
      Serial.print(sensor_read[2]);
      Serial.print(",");
      Serial.print( real_sensor_read[2]);
      Serial.print(",");
      Serial.print(bias[2]);
      Serial.print(",");
      Serial.print(minimum_value[2]);
      Serial.print(",");
      Serial.print(maximum_value[2]);
      Serial.print(",");
      Serial.print(range[2]);
      Serial.print(",");
      Serial.print(s_factor[2]);
      Serial.print("\n");
      Serial.print("Left corner sensor 1: " );
      Serial.print(sensor_read[3]);
      Serial.print(",");
      Serial.print( real_sensor_read[3]);
      Serial.print(",");
      Serial.print(bias[3]);
      Serial.print(",");
      Serial.print(minimum_value[3]);
      Serial.print(",");
      Serial.print(maximum_value[3]);
      Serial.print(",");
      Serial.print(range[3]);
      Serial.print(",");
      Serial.print(s_factor[3]);
      Serial.print("\n");
      Serial.print("Right corner sensor 5: " );
      Serial.print(sensor_read[4]);
      Serial.print(",");
      Serial.print( real_sensor_read[4]);
      Serial.print(",");
      Serial.print(bias[4]);
      Serial.print(",");
      Serial.print(minimum_value[4]);
      Serial.print(",");
      Serial.print(maximum_value[4]);
      Serial.print(",");
      Serial.print(range[4]);
      Serial.print(",");
      Serial.print(s_factor[4]);
      Serial.print("\n");
      */
      }
    } 

    void readLineSensor(){
      pinMode(ls_pin[0],OUTPUT);
      pinMode(ls_pin[1],OUTPUT);
      pinMode(ls_pin[2],OUTPUT);
      pinMode(ls_pin[3],OUTPUT);
      pinMode(ls_pin[4],OUTPUT);
      digitalWrite(ls_pin[0], HIGH);
      digitalWrite(ls_pin[1], HIGH);
      digitalWrite(ls_pin[2], HIGH);
      digitalWrite(ls_pin[3], HIGH);
      digitalWrite(ls_pin[4], HIGH);

      delayMicroseconds(10);
      
      pinMode(ls_pin[0],INPUT);
      pinMode(ls_pin[1],INPUT);
      pinMode(ls_pin[2],INPUT);
      pinMode(ls_pin[3], INPUT);
      pinMode(ls_pin[4], INPUT);

      int which;

      unsigned long start_time;
      start_time = micros();

      sensor_read[0] = 0;
      sensor_read[1] = 0;
      sensor_read[2] = 0;
      sensor_read[3] = 0;
      sensor_read[4] = 0;

      real_sensor_read[0] = 0;
      real_sensor_read[1] = 0;
      real_sensor_read[2] = 0;
      real_sensor_read[3] = 0;
      real_sensor_read[4] = 0;
      
      int remaining = NB_LS_PINS;
      unsigned long timeout = 5000;
      while( remaining > 0 ) {
        unsigned long current_time = micros();
        unsigned long elapsed_time = current_time - start_time;
        for( which = 0; which < NB_LS_PINS; which++ ) {
          if(digitalRead( ls_pin[ which ] ) == LOW ){
            if(sensor_read[ which ] == 0){
              sensor_read[which] = elapsed_time;
              //Serial.print(sensor_read[which]);
              /*if (sensor_read[which] < 1200) {
                bias[which]=sensor_read[which];
              }
              real_sensor_read[which] = (((float)abs((float)sensor_read[which]-(float)bias[which])))*((float)s_factor[which]);
              */
              
              //Serial.print(sensor_read[which]);
              //Serial.print(",");
              if (sensor_read[which]<1200){// && sensor_read[which]<previous_minimum[which] && sensor_count_a <=100){
                minimum_value[which] = sensor_read[which];
                bias[which] = minimum_value[which];
                //previous_minimum[which] = sensor_read[which];
                //sensor_count_a = sensor_count_a +1;
              }
              if (sensor_read[which]>1200){// && sensor_read[which]>previous_maximum[which] && sensor_count_b <=100){
                maximum_value[which] = sensor_read[which];
                //previous_maximum[which] = sensor_read[which];
                //sensor_count_b = sensor_count_b +1;
              }
              range[which] = maximum_value[which]-minimum_value[which];
              s_factor[which] = ((float)1.00/((float)range[which]));
              real_sensor_read[which] = ((float)abs((float)sensor_read[which] - (float)bias[which]))/(float)range[which];
              remaining = remaining - 1; 
            }// end of second if loop
          } // end of first if loop
        } // end of for loop
        if(elapsed_time >= timeout ){
          for(which =0;which<NB_LS_PINS;which++){
            sensor_read[which] = timeout;
          }
          remaining=0;
        }

      }// end of while loop
      //Weight measurement
      /*
      // Print output.
      Serial.println(e_line);
      Serial.print("Left line sensor 2: " );
      Serial.print(sensor_read[0]);
      Serial.print(",");
      Serial.print(real_sensor_read[0],8);
      Serial.print(",");
      Serial.print(bias[0]);
      Serial.print(",");
      Serial.print(minimum_value[0]);
      Serial.print(",");
      Serial.print(maximum_value[0]);
      Serial.print(",");
      Serial.print(range[0]);
      Serial.print(",");
      Serial.print(s_factor[0]);
      Serial.print("\n");
      Serial.print("Centre line sensor 3: " );
      Serial.print(sensor_read[1]);
      Serial.print(",");
      Serial.print( real_sensor_read[1]);
      Serial.print(",");
      Serial.print(bias[1]);
      Serial.print(",");
      Serial.print(minimum_value[1]);
      Serial.print(",");
      Serial.print(maximum_value[1]);
      Serial.print(",");
      Serial.print(range[1]);
      Serial.print(",");
      Serial.print(s_factor[1]);
      Serial.print("\n");
      Serial.print("Right line sensor 4: " );
      Serial.print(sensor_read[2]);
      Serial.print(",");
      Serial.print( real_sensor_read[2]);
      Serial.print(",");
      Serial.print(bias[2]);
      Serial.print(",");
      Serial.print(minimum_value[2]);
      Serial.print(",");
      Serial.print(maximum_value[2]);
      Serial.print(",");
      Serial.print(range[2]);
      Serial.print(",");
      Serial.print(s_factor[2]);
      Serial.print("\n");
      Serial.print("Left corner sensor 1: " );
      Serial.print(sensor_read[3]);
      Serial.print(",");
      Serial.print( real_sensor_read[3]);
      Serial.print(",");
      Serial.print(bias[3]);
      Serial.print(",");
      Serial.print(minimum_value[3]);
      Serial.print(",");
      Serial.print(maximum_value[3]);
      Serial.print(",");
      Serial.print(range[3]);
      Serial.print(",");
      Serial.print(s_factor[3]);
      Serial.print("\n");
      Serial.print("Right corner sensor 5: " );
      Serial.print(sensor_read[4]);
      Serial.print(",");
      Serial.print( real_sensor_read[4]);
      Serial.print(",");
      Serial.print(bias[4]);
      Serial.print(",");
      Serial.print(minimum_value[4]);
      Serial.print(",");
      Serial.print(maximum_value[4]);
      Serial.print(",");
      Serial.print(range[4]);
      Serial.print(",");
      Serial.print(s_factor[4]);
      Serial.print("\n");
      */
      // Delay, just so that we have time toread the 
      // values when using Serial.print().
      delay(100);
   }
   float getLineError(){
    readLineSensor();
    sum = (float)real_sensor_read[0]+(float)real_sensor_read[1]+(float)real_sensor_read[2];
    sum_all = (float)real_sensor_read[0]+(float)real_sensor_read[1]+(float)real_sensor_read[2]+(float)real_sensor_read[3]+(float)real_sensor_read[4]; 
    if (sum==0) {
      e_line =0;
      /*Serial.print("e_line is:");
      Serial.print(e_line);
      Serial.print("\n");
      Serial.print("sum is:");
      Serial.print(sum);
      Serial.print(",");
      Serial.print(sum_all);
      Serial.print("\n");*/
     }
     else if (sum != 0){
       /*Serial.print("e_line is:");
       Serial.print(e_line);
       Serial.print("\n");
       Serial.print("sum is:");
       Serial.print(sum);
       Serial.print(",");
       Serial.print(sum_all);
       Serial.print("\n");*/
       normalise_left = real_sensor_read[0]/sum;
       //Serial.print("normalise_left is:");
       //Serial.print(normalise_left);
       //Serial.print("\n");
       normalise_centre = real_sensor_read[1]/sum;
       //Serial.print("normalise_centre is:");
       //Serial.print(normalise_centre);
       //Serial.print("\n");
       normalise_right = real_sensor_read[2]/sum;
       //Serial.print("normalise_right is:");
       //Serial.print(normalise_right);
       //Serial.print("\n");
       w_left = normalise_left + (0.5*normalise_centre);  
       //Serial.print("w_right is:");
       //Serial.print(w_right);
       //Serial.print("\n");
       w_right  = normalise_right + (0.5*normalise_centre);
       //Serial.print("w_left is:");
       //Serial.print(w_left);
       //Serial.print("\n");
       e_line  = w_left - w_right;
       //Serial.print("e_line is:");
       //Serial.print(e_line);
       //Serial.print("\n");
     return e_line;
    }
  }
};



#endif
