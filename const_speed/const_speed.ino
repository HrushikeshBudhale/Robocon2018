/* Written by : HRUSHIKESH
 * Date: 03/02/18 05:56:43
 */

// ################ Including necessary Libraries #############

#include <PID_v1.h>

// ################ Defining pin names ##############

//these pins can not be changed 2/3 are special pins
const unsigned char motor_inA[4] = {58, 60, 62, 64};
const unsigned char motor_inB[4] = {59, 61, 63, 65};
const unsigned char motor_dir[4] = {11, 9, 7, 5};
const unsigned char motor_pwm[4] = {12, 10, 8, 6};

// ################ Declearing global variables ##############

const unsigned char dir_val_f[4] = {0, 0, 1, 1};
const unsigned char dir_val_r[4] = {1, 1, 0, 0};

volatile int lastEncoded[4] = {0, 0, 0, 0};
double encoderValue[4] = {0, 0, 0, 0};

double pre_encoder[4] = {0, 0, 0, 0};

int lastMSB[4] = {0, 0, 0, 0};
int lastLSB[4] = {0, 0, 0, 0};

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

double Kp_soft = 0.0005;    // PID parameters for soft start
double Ki_soft = 0.2;
double Kd_soft = 0;

double Kp_v = 2;    // PID parameters for maintaining constant speed
double Ki_v = 1;
double Kd_v = 0;

int soft_start = 1;
double end_pos =40000 ;

double end_velocity = 0;
double pwm_val[4] = {0, 0, 0, 0};
double curr_rpm[4] = {0, 0, 0, 0};

double Kp_p = 0.15;    // PID parameters for maintaining stopping position
double Ki_p = 0.01;
double Kd_p = 0.001;

long unsigned int now = 0;
long unsigned int pre_now = 0;

byte run = 0;       // flag controlling run and stop of operation in loop

PID postionPID(&encoderValue[2], &end_velocity, &end_pos, Kp_soft, Ki_soft, Kd_soft, DIRECT);      // controller for maintaining stopping position

PID velocityPID_1(&curr_rpm[0], &pwm_val[0], &end_velocity, Kp_v, Ki_v, Kd_v, DIRECT);    // controller for maintaining speed
PID velocityPID_2(&curr_rpm[1], &pwm_val[1], &end_velocity, Kp_v, Ki_v, Kd_v, DIRECT);    
PID velocityPID_3(&curr_rpm[2], &pwm_val[2], &end_velocity, Kp_v, Ki_v, Kd_v, DIRECT);    
PID velocityPID_4(&curr_rpm[3], &pwm_val[3], &end_velocity, Kp_v, Ki_v, Kd_v, DIRECT);    

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
    
    velocityPID_1.SetMode(AUTOMATIC);
    velocityPID_1.SetOutputLimits(-200,200);
    velocityPID_1.SetSampleTime(10);

    velocityPID_2.SetMode(AUTOMATIC);
    velocityPID_2.SetOutputLimits(-200,200);
    velocityPID_2.SetSampleTime(10);

    velocityPID_3.SetMode(AUTOMATIC);
    velocityPID_3.SetOutputLimits(-200,200);
    velocityPID_3.SetSampleTime(10);

    velocityPID_4.SetMode(AUTOMATIC);
    velocityPID_4.SetOutputLimits(-200,200);
    velocityPID_4.SetSampleTime(10);

    postionPID.SetMode(AUTOMATIC);
    postionPID.SetOutputLimits(-150,150);
    postionPID.SetSampleTime(10);

    delay(5000);
}

void loop(){ 
    if(run == 1){
        
        if((end_pos - encoderValue[2]) < 10000){
            postionPID.Compute();
        }
        
        if((soft_start == 1) && (curr_rpm[2] > 50)){
            postionPID.SetTunings(Kp_p, Ki_p, Kd_p);
            soft_start = 0;
        }
        
        now = millis();
        if((now - pre_now) >= 10){
            for(char i=0; i<4; i++){
                if(dir_val_f[i]){
                    curr_rpm[i] = ((encoderValue[i] - pre_encoder[i])*6)/(now - pre_now);    
                }
                else{
                    curr_rpm[i] = ((pre_encoder[i] - encoderValue[i])*6)/(now - pre_now);
                }
                pre_encoder[i] = encoderValue[i];
            }
            pre_now = now;
            loop_print();
        }
        postionPID.Compute();
        velocityPID_1.Compute();
        velocityPID_2.Compute();
        velocityPID_3.Compute();
        velocityPID_4.Compute();
        
        setPWM();
        
    }
    if (stringComplete) {
        inputString = "";
        stringComplete = false;
    }
    
}

void setPWM(void){
    for(char i=0; i<4; i++){
        if(pwm_val[i] > 0){
            digitalWrite(motor_dir[i], dir_val_f[i]);
            analogWrite(motor_pwm[i], abs(pwm_val[i]));
        }
        else{
            digitalWrite(motor_dir[i], dir_val_r[i]);
            analogWrite(motor_pwm[i], abs(pwm_val[i]));
        }
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
                            curr_rpm[i] = 0;   
                            analogWrite(motor_pwm[i], 0);
                        }
                        break;
            
            case 'r':   //runs the controller
                        run = 1;
                        break;
            
            case 'p':   //sets P value for distance controller
                        Kp_soft = inputString.substring(inputString.indexOf('=')+1).toFloat();
                        break;
            
            case 'i':   //sets I value for distance controller
                        Ki_soft = inputString.substring(inputString.indexOf('=')+1).toFloat();
                        break;
                        
            case 'd':   //sets P value for angle controller
                        Kd_soft = inputString.substring(inputString.indexOf('=')+1).toFloat();
                        break;
        }
        velocityPID_1.SetTunings(Kp_v, Ki_v, Kd_v);
        velocityPID_2.SetTunings(Kp_v, Ki_v, Kd_v);
        velocityPID_3.SetTunings(Kp_v, Ki_v, Kd_v);
        velocityPID_4.SetTunings(Kp_v, Ki_v, Kd_v);
        postionPID.SetTunings(Kp_soft, Kp_soft, Kp_soft);
        
        Serial.println(inputString);
        Serial.print("End speed =");
        Serial.print(end_velocity);
        Serial.print("\tKp_v =");
        Serial.print(Kp_v);
        Serial.print("\tKi_v =");
        Serial.print(Ki_v);
        Serial.print("\tKd_v =");
        Serial.print(Kd_v);
        Serial.println("\nPress 'r' to run");
    }
    else
        inputString += inChar;
}

void loop_print(void){
//    for(char i=0; i<4; i++){
//        Serial.print(curr_rpm[i]);
//        Serial.print("\t");
//    }
//
//    for(char i=0; i<4; i++){
//        Serial.print(pwm_val[i]);
//        Serial.print("\t");
//    }
    Serial.print(encoderValue[2]);
    Serial.print("\t");
    Serial.print(end_velocity);
    Serial.print("\t");
    Serial.print(end_pos);
    Serial.print("\n");
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

