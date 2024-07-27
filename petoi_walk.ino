#include "Adafruit_PWMServoDriver.h"


#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500
#define FREQUENCY 50

long float_map(float in, float low_in, float high_in, long low_out, long high_out){
    return low_out + (in - low_in) / (high_in - low_in) * (high_out - low_out);
}

class PetoiDog{
public:
    int alpha_maps[4] = {14, 1, 6, 9}; // Servo index mapping for alpha (shoulder)
    int beta_maps[4] = {15, 0, 7, 8}; //Servo index mapping for beta (knees)
    float alphas[4] = {0., 0., 0., 0.}; // shoulder angles, in radius
    float betas[4] = {0., 0., 0., 0.}; // knee angles, in radius

    // Calibrations, in degrees
    float alpha_calibrate[4] = {1. , 6. , 9. , -7.}; 
    float beta_calibrate[4] = {3., 7., 9., -5.}; 


    Adafruit_PWMServoDriver pwm_driver;

    void write_degree(float* alpha_vals=nullptr, float* beta_vals=nullptr){
        long pulsewidth = 0;
        alpha_vals = alpha_vals ? alpha_vals : alphas;
        beta_vals = beta_vals ? beta_vals : betas;

        // write alpha (shoulder) degrees
        // note that for the right side of the body, need to rotate the servo on the other direction

        for (int i=0; i<=3; i++){
            int flip = i >= 2 ? -1 : 1;
            pulsewidth = float_map(flip * (alpha_vals[i] * RAD_TO_DEG + alpha_calibrate[i]), -135, 135, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
            // pulsewidth = min(MAX_PULSE_WIDTH, max(pulsewidth, MIN_PULSE_WIDTH));
            pwm_driver.writeMicroseconds(alpha_maps[i], pulsewidth);
            // Serial.print(pulsewidth); Serial.print(" ");
            pulsewidth = float_map(flip * (beta_vals[i] * RAD_TO_DEG + beta_calibrate[i]), -135, 135, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
            // pulsewidth = min(MAX_PULSE_WIDTH, max(pulsewidth, MIN_PULSE_WIDTH));
            pwm_driver.writeMicroseconds(beta_maps[i], pulsewidth);
            // Serial.print(pulsewidth); Serial.print(" ");
        }
        // Serial.println();
    }

    void initialize(){
        pwm_driver.begin();
        pwm_driver.setPWMFreq(50);
    }

};






PetoiDog dog;

// These values are calcualted from petoi_walk.py
float walk_alphas[6][4] = {
    {0.76272625, 1.13457267, 0.76272625, 1.13457267},
    {0.93000985, 1.18074076, 0.93000985, 1.18074076},
    {1.05577914, 0.99622112, 1.05577914, 0.99622112},

    {1.13457267, 0.76272625, 1.13457267, 0.76272625},
    {1.18074076, 0.93000985, 1.18074076, 0.93000985},
    {0.99622112, 1.05577914, 0.99622112, 1.05577914}
};

float walk_betas[6][4] = {
    {0.33109918, 0.05484791, 0.33109918, 0.05484791},
    {0.28922337, 0.39589408, 0.28922337, 0.39589408},
    {0.19638032, 0.42164591, 0.19638032, 0.42164591},

    {0.05484791, 0.33109918, 0.05484791, 0.33109918},
    {0.39589408, 0.28922337, 0.39589408, 0.28922337},
    {0.42164591, 0.19638032, 0.42164591, 0.19638032}
};



void setup(){
    // Serial.begin(9600);
    dog.initialize();
    dog.write_degree();
}

int idx = 0;
int length = 6;
void loop(){
    idx = idx % length;
    dog.write_degree(
        (float*)walk_alphas[idx],
        (float*)walk_betas[idx]
    );
    delay(100);
    idx += 1;
}