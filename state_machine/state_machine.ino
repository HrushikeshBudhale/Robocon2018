/* Written by : HRUSHIKESH
 * Date: 24/01/18 14:34:27
 * board : Arduino UNO
 */

// ################ Including necessary Libraries #############

#include <PID_v1.h>

// ################ Defining pin names #######################

#define th_dir 3
#define th_pwm 4
#define valve 2
#define encoderPin1 66
#define encoderPin2 67
#define laser_sensor 51

// ################ global variables decleration ##############

double release_val = 23300;
double pwm_val_th = 0;

volatile int lastEncod_th = 0;
double encoderVal_th = 0;
double pre_encoder = 0;

String inputString = "";        // a String to hold incoming data
boolean stringComplete = true;  // whether the string is complete

double Kp_th=0.1;
double Ki_th=0.05;
double Kd_th=0.001;
double reqd_pos = -3000;
int throw_velocity = -160;
int travel_velocity = 20;

double now = 0;
double pre_now = 0;


byte throw_state = 0;
char run = 0;
unsigned char flag = 0;

PID throw_PID(&encoderVal_th, &pwm_val_th, &reqd_pos, Kp_th, Ki_th, Kd_th, DIRECT);

void setup() {

    //temperary
    pinMode(LED_BUILTIN, OUTPUT);
    
    throw_PID.SetMode(AUTOMATIC);
    throw_PID.SetOutputLimits(-100,100);
    throw_PID.SetSampleTime(1);
  
    pinMode(laser_sensor, INPUT);
    
    pinMode(th_dir, OUTPUT); 
    digitalWrite(th_dir,HIGH);
    
    pinMode(th_pwm,OUTPUT);
    analogWrite(th_pwm, 0);

    pinMode(valve,OUTPUT);
    digitalWrite(valve,LOW);
    
    digitalWrite(encoderPin1, HIGH); // turn pullup resistor on
    digitalWrite(encoderPin2, HIGH); // turn pullup resistor on
    
    // call updateThEncoder() when any high/low changed seen
    attachInterrupt(encoderPin1, updateThEncoder, CHANGE); 
    attachInterrupt(encoderPin2, updateThEncoder, CHANGE);
    
    Serial.begin (115200);
    while(Serial.available() <= 0);
}

void loop(){ 
    digitalWrite(LED_BUILTIN,digitalRead(laser_sensor));
    if(run == 1) {
        switch(throw_state){
            case 0 :    setThPWM(travel_velocity);
            
                        // wait till arm reaches initial pos
                        while(digitalRead(laser_sensor) == LOW);
                            //Serial.println(digitalRead(laser_sensor));

                        encoderVal_th = 0;
                        Serial.println("Reached Zero !");
                        setThPWM(0);
                        reqd_pos = -3000;
                        throw_state = 1;
                        
                        // for testing purpose
//                        run = 0;
//                        stringComplete = true;
                        break;
        
            case 1:     throw_PID.Compute();
//                        Serial.print(pwm_val_th);
//                        Serial.print("\t");
//                        Serial.println(encoderVal_th);
                        setThPWM(pwm_val_th);

                        // open valve when reached
                        if(encoderVal_th < -2500 && encoderVal_th > -3500){
                            digitalWrite(valve, HIGH);
                        }

                        // wait for keypress
                        if(flag == 1){
                            throw_state = 2;
                            flag = 0;
                            digitalWrite(valve, LOW);
                        }
                        break;
            
            case 2:     
                        throw_PID.Compute();
                        Serial.print("Grabbed !! \n");
//                        Serial.print(pwm_val_th);
//                        Serial.print("\t");
//                        Serial.println(encoderVal_th);
                        setThPWM(pwm_val_th);
                        if(flag == 1){
                            throw_state = 3;
                            flag = 0;
                            setThPWM(travel_velocity);
                        }
                        break;
        
            case 3:     // wait till arm reaches initial pos
                        while(digitalRead(laser_sensor) == LOW){
//                            Serial.println("");
//                            Serial.print("Laser \t");
//                            Serial.print(digitalRead(laser_sensor));
//                            Serial.println("");
//                            digitalWrite(LED_BUILTIN,digitalRead(laser_sensor));
                        }
                        
                        encoderVal_th = 0;
                        Serial.println("Reached Zero !");
                        setThPWM(0);
                        reqd_pos = 0;
                        throw_state = 4;
                        
                        // for testing purpose
//                        run = 0;
//                        stringComplete = true;
                        break;
                        
            case 4:     throw_PID.Compute();
                        setThPWM(pwm_val_th);
//                        Serial.print(pwm_val_th);
//                        Serial.print("\t");
//                        Serial.println(encoderVal_th);
                        if(flag == 1){
                            throw_state = 5;
                            flag = 0;
                            setThPWM(throw_velocity);
                            Serial.println("Running pid");
                        }
                        break;
                        
            case 5:     now = micros();
                        if((now - pre_now) >= 1000){    // calculates velocity and angle in degree
                            Serial.print(abs(encoderVal_th*0.03));
                            Serial.print("\t");
                            Serial.println((abs(encoderVal_th - pre_encoder)*5000.0)/(now - pre_now));
                            pre_now = now;
                            pre_encoder = encoderVal_th;
                        }
                        if (abs(encoderVal_th) > release_val) {
                            digitalWrite(valve,HIGH); 
                            Serial.print("Released at: ");
                            Serial.println(encoderVal_th);
                            setThPWM(0);
                            encoderVal_th = 0;
                            throw_state = 0;
                        }
                        break;
        }
    }
    
    if (stringComplete) {
        inputString = "";
        print_data();
        stringComplete = false;
    }
}
    
void serialEvent() {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
        stringComplete = true;
        switch(inputString.charAt(0)){
            case 's':   //stops operation
                        run = 0;
                        setThPWM(0);
                        encoderVal_th = 0;
                        break;
            case 'r':   //runs the states
                        run = 1;
                        break;
            case 'e':   //sets flag for next state
                        flag = 1;
                        break;
            case 'p':   //sets P value for PID controller
                        Kp_th = extract_num();
                        break;
            case 'i':   //sets I value for PID controller
                        Ki_th = extract_num();
                        break;
            case 'd':   //sets D value for PID controller
                        Kd_th = extract_num();
                        break;
            case 'v':   //sets release value of encoder
                        release_val = extract_num();
                        break;
            case 'f':   //sets power of motor
                        throw_velocity = extract_num();
                        break;                  
            case 'm':   //sets state number
                        throw_state = extract_num();
                        break;
        }
        throw_PID.SetTunings(Kp_th, Ki_th, Kd_th);
        
    }
    else
        inputString += inChar;
}

double extract_num() {
    return(inputString.substring(inputString.indexOf('=')+1).toFloat());
}

void print_data(void) {
    Serial.println(inputString);
    Serial.print("Encoder=");
    Serial.print(encoderVal_th);
    Serial.print("\tCurr_state=");
    Serial.print(throw_state);
    Serial.print("\tKp=");
    Serial.print(Kp_th);
    Serial.print("\tKi=");
    Serial.print(Ki_th);
    Serial.print("\tKd=");
    Serial.print(Kd_th);
    Serial.print("\tPWM=");
    Serial.print(pwm_val_th);
    Serial.print("\tRelease=");
    Serial.print(release_val);
    Serial.println("\nPress 'r' to run");
}

void setThPWM(int val){
    if(val>0)
        digitalWrite(th_dir, HIGH);
    else 
        digitalWrite(th_dir, LOW);
    analogWrite(th_pwm,abs(val));
}

void updateThEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncod_th << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
    encoderVal_th ++;
  else encoderVal_th --;

  lastEncod_th = encoded; //store this value for next time
}

