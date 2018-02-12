/* Written by : HRUSHIKESH
 * Date: 11/02/18 11:02:13
 */

// ################ Including necessary Libraries #############

#include <PID_v1.h>
#include <Ultrasonic.h>


// ################ Defining pin names and constants ##############

#define th_dir 3
#define th_pwm 4
#define valve 2
#define encoderPin1 66
#define encoderPin2 67
#define laser_sensor 51

#define FRONT 0
#define LEFT 1
#define ACW 2

const unsigned char motor_inA[4] = {59, 61, 62, 64};
const unsigned char motor_inB[4] = {58, 60, 63, 65};
const unsigned char motor_dir[4] = {11, 9, 7, 5};
const unsigned char motor_pwm[4] = {12, 10, 8, 6};

const unsigned char sensor_left[2] = {31,33};
const unsigned char sensor_right[2] = {35,37};


// ################ Declearing global variables for drive ##############

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

double Kp_front = 0.1;    // PID parameters for maintaining stopping position in front
double Ki_front = 0.1;
double Kd_front = 0;

double Kp_left = 0.1;    // PID parameters for maintaining stopping position in left
double Ki_left = 0.1;
double Kd_left = 0;

double Kp_acw = 0.01;    // PID parameters for maintaining stopping position in acw
double Ki_acw = 0.01;
double Kd_acw = 0;

// ################ Declearing global variables for throw ##############

double release_val = 23300;
double pwm_val_th = 0;

volatile int lastEncod_th = 0;
double encoderVal_th = 0;
double pre_encoder = 0;

double Kp_th=0.1;
double Ki_th=0.05;
double Kd_th=0.001;
double reqd_pos = -3000;
int throw_velocity = -160;
int travel_velocity = 20;

double now = 0;
double pre_now = 0;

// ################ Declearing global variables for ultrasonic ##############

double Kp_d = 8;    // PID parameters for maintaining distance of bot from wall
double Ki_d = 20;
double Kd_d = 0;

double Kp_a = 6;    // PID parameters for keeping bot parallel to the wall
double Ki_a = 15;
double Kd_a = 0;

double end_dist = 25;
double end_phi = 0;

double dist = 0;
double phi = 0;

double pwm_val_d = 0;
double pwm_val_a = 0;

// ################ Declearing global variables for state machine ##############

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

byte run = 0;       // flag controlling run and stop of operation in loop
byte throw_state = 0;
int state = 0;
char key = 0;  // permission for setting state



PID front_PID(&pos_wrt_front, &front_pwm_val, &front_dest, Kp_front, Ki_front, Kd_front, DIRECT);      // controller for maintaining stopping position
PID left_PID(&pos_wrt_left, &left_pwm_val, &left_dest, Kp_left, Ki_left, Kd_left, DIRECT);
PID acw_PID(&pos_wrt_acw, &acw_pwm_val, &acw_dest, Kp_acw, Ki_acw, Kd_acw, DIRECT);

PID throw_PID(&encoderVal_th, &pwm_val_th, &reqd_pos, Kp_th, Ki_th, Kd_th, DIRECT);

Ultrasonic ultrasonic_left(sensor_left[0], sensor_left[1]);
Ultrasonic ultrasonic_right(sensor_right[0], sensor_right[1]);
PID distPID(&dist, &pwm_val_d, &end_dist, Kp_d, Ki_d, Kd_d, DIRECT);    // controller for maintaining distance
PID phiPID(&phi, &pwm_val_a, &end_phi, Kp_a, Ki_a, Kd_a, DIRECT);       // controller for maintaining angle 



void setup() {
    // start serial port at 115200 bps:
    Serial.begin(115200);

    // Attaches all encoder pins to interrupt
    attachAllInt();

    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(laser_sensor, INPUT);
    
    pinMode(th_dir, OUTPUT); 
    digitalWrite(th_dir,HIGH);
    
    pinMode(th_pwm,OUTPUT);
    analogWrite(th_pwm, 0);

    pinMode(valve,OUTPUT);
    digitalWrite(valve,LOW);
    
    // set direction, speed and pins for controlling motors 
    for(char i = 0; i<4; i++){
        pinMode(motor_dir[i], OUTPUT);  
        pinMode(motor_pwm[i], OUTPUT);
        analogWrite(motor_pwm[i], 0);
        digitalWrite(motor_dir[i], front_dir[i]);
    }

    pid_init(); // sets mode, limits and sampleTime
    left_dest = 140416;
    front_dest = 0;

    while(Serial.available() <= 0);
}

void loop(){
    digitalWrite(LED_BUILTIN,digitalRead(laser_sensor));
    if(run == 1){
        loop_calc();
        switch(state){
           case -1: left_dest = 106132;
                    front_dest = 0;
                    stop();
                    state = 0;
                    stringComplete = true;
                    break;

           case 0:  if(pos_wrt_left > 106132 || key == 1){  // To left
                        stop();
                        state = 1;
                        grab();
                        front_dest = -74882;
                        left_dest = 0;
                        stringComplete = true;
                    }
                    break;

           case 1:  if(pos_wrt_front < -74882 || key == 1){    // TO 1st throw
                        stop();
                        state = 2;
                        throw1(-165,23400);
                        front_dest = 74882;
                        left_dest = 0;
                        stringComplete = true;
                    }
                    break;

           case 2:  if(pos_wrt_front > 74882 || key == 1){ // To home
                        stop();
                        state = 3;
                        left_dest = 47169;
                        front_dest = 0;
                        stringComplete = true;
                    }
                    break;

           case 3:  if(pos_wrt_left > 47169 || key == 1){   // TO corner
                        stop();
                        state = 4;
                        grab();
                        front_dest = -74882;
                        left_dest = 0;
                        stringComplete = true;
                    }
                    break;

           case 4:  if(pos_wrt_front < -74882 || key == 1){    // To 2nd pos
                        stop();
                        state = 5;
                        throw1(-165,23400);
                        front_dest = 74882;
                        left_dest = 0;
                        stringComplete = true;
                    }
                    break;

           case 5:  if(pos_wrt_front > 74882 || key == 1){ // To corner
                        stop();
                        state = 6;
                        grab();
                        front_dest = -149646;
                        left_dest = 0;
                        stringComplete = true;
                    }
                    break;

           case 6:  if(pos_wrt_front < -149646 || key == 1){   // To 3rd pos
                        stop();
                        state = 5;
                        throw1(-165,23400);
                        stringComplete = true;
                        front_dest = 0;
                        left_dest = 0;
                        Serial.print("Complete!!");
                    }
                    break;
        }
        key = 0;
    }
    
    if (stringComplete) {
        inputString = "";
        stringComplete = false;
        print_data();
    }
}

void grab(void){
    for(char i=0; i<5; i++){
        throw_states(i);
    }
}

void throw1(int pwm, long int angle){
//    throw_states(5);
    throw_velocity = pwm;
    release_val = angle;
    setThPWM(throw_velocity);
    while(abs(encoderVal_th) < release_val) {
        now = micros();
        if((now - pre_now) >= 1000){    // calculates velocity and angle in degree
            Serial.print(abs(encoderVal_th*0.03));
            Serial.print("\t");
            Serial.println((abs(encoderVal_th - pre_encoder)*5000.0)/(now - pre_now));
            pre_now = now;
            pre_encoder = encoderVal_th;
        }
    }
    digitalWrite(valve,HIGH); 
    Serial.print("Released at: ");
    Serial.println(encoderVal_th);
    setThPWM(0);
}

void throw_states(char throw_state){
    int t1;
    switch(throw_state){
            case 0 :    setThPWM(travel_velocity);
            
                        // wait till arm reaches initial pos
                        while(digitalRead(laser_sensor) == LOW);
                            //Serial.println(digitalRead(laser_sensor));

                        encoderVal_th = 0;
                        Serial.println("Reached Zero !");
                        setThPWM(0);
                        break;
        
            case 1:     reqd_pos = -3000;
                        t1 = millis();
                        while((millis()-t1) < 3000){
                            throw_PID.Compute();
                            Serial.print(pwm_val_th);
                            Serial.print("\t");
                            Serial.print(encoderVal_th);
                            Serial.print("\t");
                            Serial.println(reqd_pos);
                            setThPWM(pwm_val_th);
    
                            // open valve when reached
                            if(encoderVal_th < -2500 && encoderVal_th > -3500){
                                digitalWrite(valve, HIGH);
                            }
                        }
                        break;
            
            case 2:     digitalWrite(valve, LOW);
                        t1= millis();
                        while((millis()-t1) < 3000){
                            throw_PID.Compute();
                            Serial.print("Grabbed !! \n");
//                            Serial.print(pwm_val_th);
//                            Serial.print("\t");
//                            Serial.println(encoderVal_th);
                            setThPWM(pwm_val_th); 
                        }
                        break;
        
            case 3:     // wait till arm reaches initial pos
                        setThPWM(travel_velocity);
                        while(digitalRead(laser_sensor) == LOW){
//                            Serial.println("");
//                            Serial.print("Laser \t");
//                            Serial.print(digitalRead(laser_sensor));
//                            Serial.println("");
                            digitalWrite(LED_BUILTIN,digitalRead(laser_sensor));
                        }
                        encoderVal_th = 0;
                        Serial.println("Reached Zero !");
                        setThPWM(0);
                        break;
                        
            case 4:     reqd_pos = 0;
                        t1= millis();
                        while((millis()-t1) < 5000){
                            throw_PID.Compute();
                            setThPWM(pwm_val_th);
                            Serial.print(pwm_val_th);
                            Serial.print("\t");
                            Serial.println(encoderVal_th);
                        }
                        break;
                        
//            case 5:     setThPWM(throw_velocity);
//                        while(abs(encoderVal_th) < release_val) {
//                            now = micros();
//                            if((now - pre_now) >= 1000){    // calculates velocity and angle in degree
////                                Serial.print(abs(encoderVal_th*0.03));
////                                Serial.print("\t");
////                                Serial.println((abs(encoderVal_th - pre_encoder)*5000.0)/(now - pre_now));
//                                pre_now = now;
//                                pre_encoder = encoderVal_th;
//                            }
//                        }
//                        digitalWrite(valve,HIGH); 
//                        Serial.print("Released at: ");
//                        Serial.println(encoderVal_th);
//                        setThPWM(0);
//                        break;
    }
}

void loop_calc(void){
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

void print_data(void){
    Serial.print("\tCurr_state=");
    Serial.println(state);
}

void stop(void){
    run = 0;
    acw_dest = 0;
    setThPWM(0);
    encoderVal_th = 0;
                        
    for(char i = 0; i<4; i++){
        encoderValue[i] = 0;
        pwm_val[i] = 0;   
    }
    setPWM();                       
}

void pwm_compute(void){
    for(char i=0; i<4; i++){
        pwm_val[i] = (front_pwm_val * front_dir[i]) + (left_pwm_val * left_dir[i]) + (acw_pwm_val * acw_dir[i]);
    }
}

void adjustPosition(void){
    int t1 = millis();
    while((t1 - millis()) < 6000){
        dist = distance();
        phi = angle();  
        distPID.Compute();
        phiPID.Compute();
        for(char i=0; i<4; i++){
            pwm_val[i] = (pwm_val_a * acw_dir[i]) - (pwm_val_d * front_dir[i]);
        }
        setPWM();       // sets pwm for all wheels considering diretion    
    }
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

void serialEvent(void) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
        stringComplete = true;
        switch(inputString.charAt(0)){
            case 's':   //stops the controller
                        stop();
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
            case 'v':   //sets release value of encoder
                        release_val = extract_num();
                        break;
            case 'h':   //sets power of motor
                        throw_velocity = extract_num();
                        break;                  
            case 'm':   //sets I value for angle controller
                        state = extract_num();
                        state--;
                        key = 1;
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
    Serial.print("\tfront: ");
    Serial.print(front_dest);
    Serial.print("\tleft: ");
    Serial.print(left_dest);


//    for(char i=0; i<4; i++){
//        Serial.print(encoderValue[i]);
//        Serial.print(" ");
//    }
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

void setThPWM(int val){
    if(val>0)
        digitalWrite(th_dir, HIGH);
    else 
        digitalWrite(th_dir, LOW);
    analogWrite(th_pwm,abs(val));
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

void updateThEncoder(void){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncod_th << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
    encoderVal_th ++;
  else encoderVal_th --;

  lastEncod_th = encoded; //store this value for next time
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

    digitalWrite(encoderPin1, HIGH); // turn pullup resistor on
    digitalWrite(encoderPin2, HIGH); // turn pullup resistor on
    
    attachInterrupt(encoderPin1, updateThEncoder, CHANGE); 
    attachInterrupt(encoderPin2, updateThEncoder, CHANGE);
    
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

    throw_PID.SetMode(AUTOMATIC);
    throw_PID.SetOutputLimits(-100,100);
    throw_PID.SetSampleTime(2);
}
