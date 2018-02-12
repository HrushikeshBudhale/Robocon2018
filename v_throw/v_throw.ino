/* Written by : HRUSHIKESH
 * Date: 24/01/18 14:34:27
 * board : Arduino UNO
 */

// ################ Including necessary Libraries #############

#include <PID_v1.h>

// ################ Defining pin names #######################

#define dir 3
#define pwm 4
#define valve 2
#define encoderPin1 66
#define encoderPin2 67
#define laser_sensor 51

// ################ global variables decleration ##############

double release_val = 6000;
double pwm_val = 0;
char r_dir=1;

volatile int lastEncoded = 0;
double encoderValue = 0;
double pre_encoder = 0;

String inputString = "";        // a String to hold incoming data
boolean stringComplete = true;  // whether the string is complete

double Kp=0.1;
double Ki=0.05;
double Kd=0.001;
double reqd_pos = 3000;
int throw_velocity = 160;
int travel_velocity = -40;

byte state = 0 ;
char run = 0;
unsigned char flag = 0;

double now = 0;
double pre_now = 0;

PID myPID(&encoderValue, &pwm_val, &reqd_pos, Kp, Ki, Kd, DIRECT);

void setup() {

    //temperary
    pinMode(LED_BUILTIN, OUTPUT);
    
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-100,100);
    myPID.SetSampleTime(1);
  
    pinMode(encoderPin1, INPUT); 
    pinMode(encoderPin2, INPUT);
    pinMode(laser_sensor, INPUT);
    
    pinMode(dir, OUTPUT); 
    digitalWrite(dir,r_dir);
    
    pinMode(pwm,OUTPUT);
    analogWrite(pwm, 0);

    pinMode(valve,OUTPUT);
    digitalWrite(valve,LOW);
    
    digitalWrite(encoderPin1, HIGH); // turn pullup resistor on
    digitalWrite(encoderPin2, HIGH); // turn pullup resistor on
    
    // call updateEncoder() when any high/low changed seen
    // on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
    attachInterrupt(encoderPin1, updateEncoder, CHANGE); 
    attachInterrupt(encoderPin2, updateEncoder, CHANGE);
    
    Serial.begin (115200);
    while(Serial.available() <= 0);
}

void loop(){ 
    digitalWrite(LED_BUILTIN,digitalRead(laser_sensor));
    if(run == 1) {
        switch(state){
            case 0 :    setPWM(travel_velocity);
            
                        // wait till arm reaches initial pos
                        while(digitalRead(laser_sensor) == LOW);
                            //Serial.println(digitalRead(laser_sensor));

                        encoderValue = 0;
                        Serial.println("Reached Zero !");
                        setPWM(0);
                        reqd_pos = 3000;
                        state = 1;
                        
                        // for testing purpose
//                        run = 0;
//                        stringComplete = true;
                        break;
        
            case 1:     myPID.Compute();
                        Serial.print(pwm_val);
                        Serial.print("\t");
                        Serial.println(encoderValue);
                        setPWM(pwm_val);

                        // wait for keypress
                        if(flag == 1){
                            state = 2;
                            flag = 0;
                            setPWM(throw_velocity);
                        }
                        break;
            
            case 2:     
                        now = micros();
                        if((now - pre_now) >= 1000){
                            Serial.print("v");
                            Serial.println(((encoderValue - pre_encoder)*5000.0)/(now - pre_now));
                            pre_now = now;
                            
                        }
                        if (abs(encoderValue) > release_val) {
                            Serial.print("Released at: ");
                            Serial.println(encoderValue);
                            setPWM(-40);
                            encoderValue = 0;
                            state = 0;
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
                        setPWM(0);
                        encoderValue = 0;
                        break;
            case 'r':   //runs the states
                        run = 1;
                        break;
            case 'e':   //sets flag for next state
                        flag = 1;
                        break;
            case 'p':   //sets P value for PID controller
                        Kp = extract_num();
                        break;
            case 'i':   //sets I value for PID controller
                        Ki = extract_num();
                        break;
            case 'd':   //sets D value for PID controller
                        Kd = extract_num();
                        break;
            case 'v':   //sets release value of encoder
                        release_val = extract_num();
                        break;
            case 'f':   //sets power of motor
                        throw_velocity = extract_num();
                        break;                  
            case 'm':   //sets state number
                        state = extract_num();
                        break;
        }
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
    Serial.print(encoderValue);
    Serial.print("\tCurr_state=");
    Serial.print(state);
    Serial.print("\tKp=");
    Serial.print(Kp);
    Serial.print("\tKi=");
    Serial.print(Ki);
    Serial.print("\tKd=");
    Serial.print(Kd);
    Serial.print("\tPWM=");
    Serial.print(pwm_val);
    Serial.print("\tRelease=");
    Serial.print(release_val);
    Serial.println("\nPress 'r' to run");
}

void setPWM(int val){
    if(val>0)
        digitalWrite(dir, HIGH);
    else 
        digitalWrite(dir, LOW);
    analogWrite(pwm,abs(val));
}

void updateEncoder(){
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) 
    encoderValue ++;
  else encoderValue --;

  lastEncoded = encoded; //store this value for next time
}

