/* Written by : HRUSHIKESH
 * Date: 07/02/18 11:34:43
 */

// ################ Including necessary Libraries #############

#include <PID_v1.h>

// ################ Defining pin names ##############

#define FRONT 0
#define LEFT 1
#define ACW 2

//these pins can not be changed 2/3 are special pins
const unsigned char motor_inA[4] = {59, 61, 62, 64};
const unsigned char motor_inB[4] = {58, 60, 63, 65};
const unsigned char motor_dir[4] = {11, 9, 7, 5};
const unsigned char motor_pwm[4] = {12, 10, 8, 6};

// ################ Declearing global variables ##############

int front_dir[4] = {1, 1, 1, 1};
int left_dir[4] = {-1, 1, -1, 1};
int acw_dir[4] = {-1, -1, 1, 1};

volatile int lastEncoded[4] = {0, 0, 0, 0};
volatile double encoderValue[4] = {0, 0, 0, 0};
int lastMSB[4] = {0, 0, 0, 0};
int lastLSB[4] = {0, 0, 0, 0};

int pwm_val[4] = {0, 0, 0, 0};

double front_pwm_val = 0;
double left_pwm_val = 0;
double acw_pwm_val = 0;

double pos_wrt_front = 0;
double pos_wrt_left = 0;
double pos_wrt_acw = 0;

double front_dest = 0;
double left_dest = 0;
double acw_dest = 0;

double Kp_front = 0.01;    // PID parameters for maintaining stopping position in front
double Ki_front = 0.01;
double Kd_front = 0;

double Kp_left = 0.01;    // PID parameters for maintaining stopping position in left
double Ki_left = 0.01;
double Kd_left = 0;

double Kp_acw = 0.01;    // PID parameters for maintaining stopping position in acw
double Ki_acw = 0.01;
double Kd_acw = 0;


String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

byte run = 0;       // flag controlling run and stop of operation in loop

PID front_PID(&pos_wrt_front, &front_pwm_val, &front_dest, Kp_front, Ki_front, Kd_front, DIRECT);      // controller for maintaining stopping position
PID left_PID(&pos_wrt_left, &left_pwm_val, &left_dest, Kp_left, Ki_left, Kd_left, DIRECT);
PID acw_PID(&pos_wrt_acw, &acw_pwm_val, &acw_dest, Kp_acw, Ki_acw, Kd_acw, DIRECT);

void setup() {
    // start serial port at 115200 bps:
    Serial.begin(115200);

    // Attaches all encoder pins to interrupt
    attachAllInt();

    // set direction, speed and pins for controlling motors 
    for(char i = 0; i<4; i++){
        pinMode(motor_dir[i], OUTPUT);  
        pinMode(motor_pwm[i], OUTPUT);
        analogWrite(motor_pwm[i], 0);
        digitalWrite(motor_dir[i], front_dir[i]);
    }

    pid_init(); // sets mode, limits and sampleTime
}

void loop(){
    if(run == 1){
        pos_wrt_front = ret_pos(FRONT);
        pos_wrt_left = ret_pos(LEFT);
        pos_wrt_acw = ret_pos(ACW);
        
        front_PID.Compute();
        left_PID.Compute();
        acw_PID.Compute();
        pwm_compute();  // Calculates pwm value for each wheel
        setPWM();       // sets pwm for all wheels considering diretion
        
        loop_print();
    }

    if (stringComplete) {
        inputString = "";
        stringComplete = false;
    }
}

void pwm_compute(void){
    for(char i=0; i<4; i++){
        pwm_val[i] = (front_pwm_val * front_dir[i]) + (left_pwm_val * left_dir[i]) + (acw_pwm_val * acw_dir[i]);
        //pwm_val[i] = (front_pwm_val + left_pwm_val + acw_pwm_val);
    }
}

void serialEvent(void) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
        stringComplete = true;
        switch(inputString.charAt(0)){
            case 's':   //stops the controller
                        run = 0;
                        for(char i = 0; i<4; i++){
                            encoderValue[i] = 0;
                            pwm_val[i] = 0;   
                        }
                        setPWM();
                        break;
            
            case 'r':   //runs the controller
                        run = 1;
                        break;
            
            case 'p':   //sets P value for distance controller
                        Kp_front = extract_num();
                        break;
            
            case 'i':   //sets I value for distance controller
                        Ki_front = extract_num();
                        break;
                        
            case ';':   //sets P value for angle controller
                        Kp_left = extract_num();
                        break;
            
            case 'k':   //sets I value for angle controller
                        Ki_left = extract_num();
                        break;            

            case 'f':   //sets I value for angle controller
                        front_dest = extract_num();
                        break;            

            case 'l':   //sets I value for angle controller
                        left_dest = extract_num();
                        break;            
            
            case 'a':   //sets I value for angle controller
                        acw_dest = extract_num();
                        break;            
                        
        }
        front_PID.SetTunings(Kp_front, Ki_front, Kd_front);          
        left_PID.SetTunings(Kp_left, Ki_left, Kd_left);
        Serial.println(inputString);
        Serial.print("F=");
        Serial.print(front_dest);
        Serial.print("\tL=");
        Serial.print(left_dest);
        Serial.print("\tA=");
        Serial.print(acw_dest);
        Serial.print("\tKpf=");
        Serial.print(Kp_front);
        Serial.print("\tKif=");
        Serial.print(Ki_front);
        Serial.print("\tKpl=");
        Serial.print(Kp_left);
        Serial.print("\tKil=");
        Serial.print(Ki_left);
        Serial.println("\nPress 'r' to run");
    }
    else
        inputString += inChar;
}

double extract_num(void) {
    return(inputString.substring(inputString.indexOf('=')+1).toFloat());
}

void loop_print(void){
    Serial.print("F: ");
    Serial.print(ret_pos(FRONT));
    Serial.print("\tL: ");
    Serial.print(ret_pos(LEFT));
    Serial.print("\tA: ");
    Serial.print(ret_pos(ACW));
    Serial.print(" \t");    
    for(char i=0; i<4; i++){
        Serial.print(encoderValue[i]);
        Serial.print(" ");
    }
//
//    for(char i=0; i<4; i++){
//        Serial.print(pwm_val[i]);
//        Serial.print(" ");
//    }
//    
//    Serial.print("\tPWM_f: ");
//    Serial.print(front_pwm_val);
//    Serial.print("\tPWM_l: ");
//    Serial.println(left_pwm_val);
    Serial.print("\n");
}

long int ret_pos(char wrt){
    switch(wrt){
        case FRONT: return((encoderValue[3] + encoderValue[2] + encoderValue[1] + encoderValue[0])/4);

        case LEFT:  return((encoderValue[3] - encoderValue[2] + encoderValue[1] - encoderValue[0])/4);

        case ACW:   return((encoderValue[3] + encoderValue[2] - encoderValue[1] - encoderValue[0])/4);
    }
}

void setPWM(void){
    for(char i = 0; i<4; i++){
        if(pwm_val[i] < 0)
            digitalWrite(motor_dir[i], LOW);
        else
            digitalWrite(motor_dir[i], HIGH);
        analogWrite(motor_pwm[i], abs(pwm_val[i]));
    }
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

void attachAllInt(void){
    attachInterrupt(motor_inA[0], updateEncoder1, CHANGE);
    attachInterrupt(motor_inB[0], updateEncoder1, CHANGE);

    attachInterrupt(motor_inA[1], updateEncoder2, CHANGE);
    attachInterrupt(motor_inB[1], updateEncoder2, CHANGE);

    attachInterrupt(motor_inA[2], updateEncoder3, CHANGE);
    attachInterrupt(motor_inB[2], updateEncoder3, CHANGE);

    attachInterrupt(motor_inA[3], updateEncoder4, CHANGE);
    attachInterrupt(motor_inB[3], updateEncoder4, CHANGE);
}

void pid_init(void){
    front_PID.SetMode(AUTOMATIC);
    front_PID.SetOutputLimits(-140,140);
    front_PID.SetSampleTime(2);

    left_PID.SetMode(AUTOMATIC);
    left_PID.SetOutputLimits(-140,140);
    left_PID.SetSampleTime(2);   

    acw_PID.SetMode(AUTOMATIC);
    acw_PID.SetOutputLimits(-140,140);
    acw_PID.SetSampleTime(2);       
}
