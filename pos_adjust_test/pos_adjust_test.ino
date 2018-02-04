/* Written by : HRUSHIKESH
 * Date: 01/02/18 05:15:43
 */

// ################ Including necessary Libraries #############

#include <PID_v1.h>
#include <Ultrasonic.h>

// ################ Defining pin names ##############

//these pins can not be changed 2/3 are special pins
const unsigned char motor_inA[4] = {58, 60, 62, 64};
const unsigned char motor_inB[4] = {59, 61, 63, 65};
const unsigned char motor_dir[4] = {11, 9, 7, 5};
const unsigned char motor_pwm[4] = {12, 10, 8, 6};

const unsigned char sensor_left[2] = {31,33};
const unsigned char sensor_right[2] = {35,37};

// ################ Declearing global variables ##############

const unsigned char dir_val_f[4] = {0, 0, 1, 1};
const unsigned char dir_val_r[4] = {1, 1, 0, 0};

double pwm_val_d = 0;
double pwm_val_a = 0;

volatile int lastEncoded[4] = {0, 0, 0, 0};
volatile double encoderValue[4] = {0, 0, 0, 0};
int lastMSB[4] = {0, 0, 0, 0};
int lastLSB[4] = {0, 0, 0, 0};

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

double Kp_d = 6;    // PID parameters for maintaining distance of bot from wall
double Ki_d = 20;
double Kd_d = 0;

double Kp_a = 6;    // PID parameters for keeping bot parallel to the wall
double Ki_a = 15;
double Kd_a = 0;

double end_dist = 25;
double end_phi = 0;

byte run = 0;       // flag controlling run and stop of operation in loop
double dist = 0;
double phi = 0;

Ultrasonic ultrasonic_left(sensor_left[0], sensor_left[1]);
Ultrasonic ultrasonic_right(sensor_right[0], sensor_right[1]);
PID distPID(&dist, &pwm_val_d, &end_dist, Kp_d, Ki_d, Kd_d, DIRECT);    // controller for maintaining distance
PID phiPID(&phi, &pwm_val_a, &end_phi, Kp_a, Ki_a, Kd_a, DIRECT);       // controller for maintaining angle 

void setup() {

    // start serial port at 115200 bps:
    Serial.begin(115200);

    attachInterrupt(motor_inA[0], updateEncoder1, CHANGE);
    attachInterrupt(motor_inB[0], updateEncoder1, CHANGE);

    attachInterrupt(motor_inA[1], updateEncoder2, CHANGE);
    attachInterrupt(motor_inB[1], updateEncoder2, CHANGE);

    attachInterrupt(motor_inA[2], updateEncoder3, CHANGE);
    attachInterrupt(motor_inB[2], updateEncoder3, CHANGE);

    attachInterrupt(motor_inA[3], updateEncoder4, CHANGE);
    attachInterrupt(motor_inB[3], updateEncoder4, CHANGE);

    // set direction, speed and pins for controlling motors 
    for(char i = 0; i<4; i++){
        pinMode(motor_dir[i], OUTPUT);  
        pinMode(motor_pwm[i], OUTPUT);
        analogWrite(motor_pwm[i], 0);
        digitalWrite(motor_dir[i], dir_val_f[i]);
    }
    
    distPID.SetMode(AUTOMATIC);
    distPID.SetOutputLimits(-120,120);
    distPID.SetSampleTime(2);

    phiPID.SetMode(AUTOMATIC);
    phiPID.SetOutputLimits(-120,120);
    phiPID.SetSampleTime(2);

}

void loop(){ 
    if(run == 1){
        dist = distance();
        phi = angle();  
        distPID.Compute();
        phiPID.Compute();
        loop_print();
        setPWM(pwm_val_d, pwm_val_a);
    }
    if (stringComplete) {
        inputString = "";
        stringComplete = false;
    }
    
}

void setPWM(int val_d, int val_a){
//    Previous version #####try if new not working
//    if(val>0)
//        for(char i = 0; i<4; i++){
//            digitalWrite(motor_dir[i], dir_val_r[i]);
//            analogWrite(motor_pwm[i], abs(val));
//        }
//    else 
//        for(char i = 0; i<4; i++){
//            digitalWrite(motor_dir[i], dir_val_f[i]);
//            analogWrite(motor_pwm[i], abs(val));
//        }
    int val;
    for(char i = 0; i<4; i++){
        if(dir_val_f[i] == 0){
            val = val_d + val_a;
        }
        else{
            val = val_d - val_a;
        }
        
        if(val < 0){
            digitalWrite(motor_dir[i], dir_val_f[i]);
        }
        else{
            digitalWrite(motor_dir[i], dir_val_r[i]);
        }
        analogWrite(motor_pwm[i], abs(val));
    }
}

void serialEvent() {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
        stringComplete = true;
        switch(inputString.charAt(0)){
            case 's':   //stops the controller
                        run = 0;
                        for(char i = 0; i<4; i++){
                            encoderValue[i] = 0;   
                        }
                        setPWM(0, 0);
                        break;
            
            case 'r':   //runs the controller
                        run = 1;
                        break;
            
            case 'p':   //sets P value for distance controller
                        Kp_d = inputString.substring(inputString.indexOf('=')+1).toFloat();
                        break;
            
            case 'i':   //sets I value for distance controller
                        Ki_d = inputString.substring(inputString.indexOf('=')+1).toFloat();
                        break;
                        
            case ';':   //sets P value for angle controller
                        Kp_a = inputString.substring(inputString.indexOf('=')+1).toFloat();
                        break;
            
            case 'k':   //sets I value for angle controller
                        Ki_a = inputString.substring(inputString.indexOf('=')+1).toFloat();
                        break;            
        }
        distPID.SetTunings(Kp_d, Ki_d, Kd_d);          
        phiPID.SetTunings(Kp_a, Ki_a, Kd_a);
        Serial.println(inputString);
        Serial.print("End dist =");
        Serial.print(end_dist);
        Serial.print("\tKp_d =");
        Serial.print(Kp_d);
        Serial.print("\tKi_d =");
        Serial.print(Ki_d);
        Serial.print("\tKp_a =");
        Serial.print(Kp_a);
        Serial.print("\tKi_a =");
        Serial.print(Ki_a);
        Serial.println("\nPress 'r' to run");
    }
    else
        inputString += inChar;
}

void loop_print(void){
    Serial.print("Distance: ");
    Serial.print(distance());
    Serial.print("\tAngle: ");
    Serial.print(angle());
    Serial.print("\tPWM_d: ");
    Serial.print(-pwm_val_d);
    Serial.print("\tPWM_a: ");
    Serial.println(pwm_val_a);
}

unsigned char distance(void){
    // average of two distances
    return((ultrasonic_left.distanceRead() + ultrasonic_right.distanceRead())/2);    
}

int angle(void){
    // difference of two distances
    int left = ultrasonic_left.distanceRead();
    int right = ultrasonic_right.distanceRead();
    return(left - right);    
    // angle is +ve if bot is slightly left
}

void updateEncoder1(void){
    int MSB = digitalRead(motor_inA[0]);        //MSB = most significant bit
    int LSB = digitalRead(motor_inB[0]);        //LSB = least significant bit
    
    int encoded = (MSB << 1) |LSB;                //converting the 2 pin value to single number
    int sum  = (lastEncoded[0] << 2) | encoded; //adding it to the previous encoded value
    
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
        encoderValue[0] ++;
    else encoderValue[0] --;

    lastEncoded[0] = encoded; //store this value for next time
}

void updateEncoder2(void){
    int MSB = digitalRead(motor_inA[1]);        //MSB = most significant bit
    int LSB = digitalRead(motor_inB[1]);        //LSB = least significant bit
    
    int encoded = (MSB << 1) |LSB;                //converting the 2 pin value to single number
    int sum  = (lastEncoded[1] << 2) | encoded; //adding it to the previous encoded value
    
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
        encoderValue[1] ++;
    else encoderValue[1] --;

    lastEncoded[1] = encoded; //store this value for next time
}

void updateEncoder3(void){
    int MSB = digitalRead(motor_inA[2]);        //MSB = most significant bit
    int LSB = digitalRead(motor_inB[2]);        //LSB = least significant bit
    
    int encoded = (MSB << 1) |LSB;                //converting the 2 pin value to single number
    int sum  = (lastEncoded[2] << 2) | encoded; //adding it to the previous encoded value
    
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
        encoderValue[2] ++;
    else encoderValue[2] --;

    lastEncoded[2] = encoded; //store this value for next time
}

void updateEncoder4(void){
    int MSB = digitalRead(motor_inA[3]);        //MSB = most significant bit
    int LSB = digitalRead(motor_inB[3]);        //LSB = least significant bit
    
    int encoded = (MSB << 1) |LSB;                //converting the 2 pin value to single number
    int sum  = (lastEncoded[3] << 2) | encoded; //adding it to the previous encoded value
    
    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
        encoderValue[3] ++;
    else encoderValue[3] --;

    lastEncoded[3] = encoded; //store this value for next time
}

