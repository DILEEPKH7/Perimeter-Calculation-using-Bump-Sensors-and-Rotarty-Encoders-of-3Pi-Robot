// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _PID_H
#define _PID_H


// Class to contain generic PID algorithm.
class PID_c {
  public:

    // PID update variables.
    float last_error;
    float p_term;
    float i_term;
    float d_term;
    float i_sum;
    float feedback; 
    float p_gain;
    float i_gain;
    float d_gain;
    unsigned long ms_last_ts;
    float diff_error;
    float feedback_pi;
  
    // Constructor, must exist.
    PID_c() {

    } 

    // Setups up class variables and also sets PID gains
    void initialise(float kp, float ki, float kd) {

      feedback =0;
      last_error =0;
      p_term = 0;
      i_term =0;
      d_term =0;
      i_sum =0;

      p_gain=kp;
      i_gain=ki;
      d_gain=kd;

      ms_last_ts = millis();
    }

    //Reset variables including the timestamp.
    //This is because, a delay() or some other blocking code will cause the i-term to
    // wind-up. It is best to reset the PID controller before its next period of use.
    void reset() {
      p_term = 0;
      i_term =0;
      d_term =0;
      i_sum =0;
      last_error =0;
      feedback = 0;
      ms_last_ts = millis();
      
    }

    float update(float demand, float measurement){
      float error;
      unsigned long ms_now_ts;
      unsigned long ms_dt;
      float float_dt;
      

      //Grab time to calc elapsed time. 
      ms_now_ts = millis();
      ms_dt = ms_now_ts - ms_last_ts;

      // ms_last_t has been used, so update
      // it for the next call of this update.
      ms_last_ts = millis();

      // typecasting the difference of two 
      // unsigned longs is safer
      float_dt = (float)ms_dt;

      //Note a serious error can occur 
      // here if dt is 0, this causes divide 
      //by zero errors. this can happen if 
      //PID.update() is called faster than 1ms.
      //Here, we catch the error by returning 
      //last feedback value. 
      if(float_dt == 0) {
        return feedback;
      }

      //calculate error signal.
      error = demand - measurement;

      //P term, nice and easy
      p_term = p_gain* error;

      //discrete integration
      i_sum = i_sum+(error*float_dt);
      

      //i_term
      i_term = i_gain*i_sum;

      //d term.
      //Note, sometimes this needs to be inverted
      //to last_error - error. It depends on the 
      //error signal sign in relation to the system.
      
      diff_error = (error-last_error)/float_dt;
      last_error = error;
      d_term = diff_error * d_gain;

      // the whole thing !
      // Again, sometimes this needs to be -d term.
      //d_term should counteract sudden changes, which 
      // means it is subtractive. Another way to achieve 
      // this is to have a -ve gain. Best way to trouble 
      //shoot is to plot what is doing. 
      feedback_pi = p_term +i_term;
      feedback = p_term +i_term +d_term;

      //done
      return feedback;
    }
};



#endif
