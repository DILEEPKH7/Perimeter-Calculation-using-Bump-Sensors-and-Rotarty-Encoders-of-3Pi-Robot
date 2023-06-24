// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _BUMPSENSOR_H
#define _BUMPSENSOR_H

#define BS_LEFT 4
#define BS_RIGHT 5

#define NB_BS_PINS 2
//#define caliberation_range 20

#define EMIT 11

// Class to operate the linesensor(s).
class BumpSensor_c {
  public:
    int bs_pin[NB_BS_PINS] = {BS_LEFT, BS_RIGHT};
    float sensor_read[ NB_BS_PINS ];

    float minimum_value[NB_BS_PINS];
    float maximum_value[NB_BS_PINS];
    float previous_minimum[NB_BS_PINS];
    float previous_maximum[NB_BS_PINS];
    unsigned long previous_values[NB_BS_PINS];
    float range[NB_BS_PINS];
    float s_factor[NB_BS_PINS] = {0.00, 0.00}; //{0.000340483,0.000357142,0.000314960,0.000539083,0.000324675};
    float bias[NB_BS_PINS] = {0, 0}; //{728,660,780,885,1070};
    long previous_range[NB_BS_PINS];
    long real_range[NB_BS_PINS];
    float real_sensor_read[ NB_BS_PINS ];

    long right_encoder;
    long left_encoder;

    // Constructor, must exist.
    BumpSensor_c() {
    }

    void initialise() {
      pinMode(BS_LEFT, INPUT);
      pinMode(BS_RIGHT, INPUT);
      pinMode(EMIT, OUTPUT);

      digitalWrite(EMIT, LOW);
      digitalWrite(BS_LEFT, HIGH);
      digitalWrite(BS_RIGHT, HIGH);

      range[0] = 0;
      range[1] = 0;
      range[2] = 0;
      range[3] = 0;
      range[4] = 0;

      minimum_value[0] = 999.00;
      minimum_value[1] = 649.00;

      maximum_value[0] = 0;
      maximum_value[1] = 0;

      previous_minimum[0] = 1199.00;
      previous_minimum[1] = 1199.00;

      previous_maximum[0] = 1199.00;
      previous_maximum[1] = 1199.00;
    }

    void readBumpSensor() {
      pinMode(bs_pin[0], OUTPUT);
      pinMode(bs_pin[1], OUTPUT);
      digitalWrite(bs_pin[0], HIGH);
      digitalWrite(bs_pin[1], HIGH);

      delayMicroseconds(10);

      pinMode(bs_pin[0], INPUT);
      pinMode(bs_pin[1], INPUT);

      int which;

      unsigned long start_time;
      start_time = micros();

      unsigned long start_time_1;
      start_time_1 = micros();


      sensor_read[0] = 0.00;
      sensor_read[1] = 0.00;

      real_sensor_read[0] = 0;
      real_sensor_read[1] = 0;

      int remaining = NB_BS_PINS;
      unsigned long timeout = 2500;
      while ( remaining > 0 ) {
        unsigned long current_time = micros();
        unsigned long elapsed_time = current_time - start_time;
        for ( which = 0; which < NB_BS_PINS; which++ ) {
          if (digitalRead( bs_pin[ which ] ) == LOW ) {
            if (sensor_read[ which ] == 0) {
              sensor_read[which] = elapsed_time;
              remaining = remaining - 1;
            }
          }
        }
      }
      if (sensor_read[0] < 1000) { //|| sensor_read[1]<650){
        minimum_value[0] = sensor_read[0];
        bias[0] = minimum_value[0];
      }
      else if (sensor_read[0] > 1000) { // || sensor_read[1]>650){
        maximum_value[0] = sensor_read[0];
      }
      range[0] = maximum_value[0] - minimum_value[0];
      s_factor[0] = ((float)1.00 / ((float)range[0]));
      real_sensor_read[0] = ((float)abs((float)sensor_read[0] - (float)bias[0])) / (float)range[0];

      if (sensor_read[1] < 650) { //|| sensor_read[1]<650){
        minimum_value[1] = sensor_read[1];
        bias[1] = minimum_value[1];
      }
      else if (sensor_read[1] > 650) { // || sensor_read[1]>650){
        maximum_value[1] = sensor_read[1];
      }
      range[1] = maximum_value[1] - minimum_value[1];
      s_factor[1] = ((float)1.00 / ((float)range[1]));
      real_sensor_read[1] = ((float)abs((float)sensor_read[1] - (float)bias[1])) / (float)range[1];

      //Weight measurement
      //Serial.print(sensor_read[0]);
      //Serial.print(",");
      //Serial.print(real_sensor_read[0]);
      //Serial.print(",");
      /*Serial.print(bias[0]);
      Serial.print(",");
      Serial.print(minimum_value[0]);
      Serial.print(",");
      Serial.print(maximum_value[0]);
      Serial.print(",");
      Serial.print(range[0]);
      Serial.print(",");
      Serial.println(s_factor[0]);

      Serial.print(sensor_read[1]);
      Serial.print(",");*/
      //Serial.println(real_sensor_read[1]);
      /*Serial.print(",");
      Serial.print(bias[1]);
      Serial.print(",");
      Serial.print(minimum_value[1]);
      Serial.print(",");
      Serial.print(maximum_value[1]);
      Serial.print(",");
      Serial.print(range[1]);
      Serial.print(",");
      Serial.println(s_factor[1]);*/
      // Delay, just so that we have time toread the
      // values when using Serial.print().
      delay(100);
    }
};



#endif
