/* Written by : HRUSHIKESH
 * Date: 24/01/18 14:34:27
 * board : Arduino UNO
 */

// ################ Including necessary Libraries #############

#include <PID_v1.h>

// ################ Defining pin names #######################

//these pins can not be changed 2/3 are special pins
#define dir 8
#define pwm 9
#define valve 4
#define angle_sensor 5
#define encoderPin1 2
#define encoderPin2 3

// ################ global variables decleration ##############

double release_val = 50300;
double pwm_val = 0;
char r_dir=1;

volatile int lastEncoded = 0;
volatile double encoderValue = 0;

String inputString = "";        // a String to hold incoming data
boolean stringComplete = true;  // whether the string is complete

double Kp=0.3;
double Ki=0.0;
double Kd=0.007;
double reqd_pos = 3000;

byte state = 0 ;
char run = 0;

PID myPID(&encoderValue, &pwm_val, &reqd_pos, Kp, Ki, Kd, DIRECT);

void setup() {
    
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-100,100);
    myPID.SetSampleTime(1);
  
    pinMode(encoderPin1, INPUT); 
    pinMode(encoderPin2, INPUT);
    
    pinMode(dir, OUTPUT); 
    digitalWrite(dir,r_dir);
    
    pinMode(pwm,OUTPUT);
    analogWrite(pwm,pwm_val);

    pinMode(valve,OUTPUT);
    digitalWrite(valve,LOW);
    
    digitalWrite(encoderPin1, HIGH); // turn pullup resistor on
    digitalWrite(encoderPin2, HIGH); // turn pullup resistor on
    
    // call updateEncoder() when any high/low changed seen
    // on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
    attachInterrupt(0, updateEncoder, CHANGE); 
    attachInterrupt(1, updateEncoder, CHANGE);
    
    Serial.begin (115200);
    while(Serial.available() <= 0);
}

void loop(){ 
    if(run == 1) {
        switch(state){
            case 0 :    encoderValue = 0;
                        Serial.print(encoderValue);
                        reqd_pos = 3000;
                        state = 1;
                        // for testing purpose
                        run = 0;
                        stringComplete = true;
                        break;
        
            case 1:     myPID.Compute();
                        Serial.print(pwm_val);
                        Serial.print("\t");
                        Serial.println(encoderValue);
                        setPWM(pwm_val);
                        //if keypress{ 
                            digitalWrite(valve,LOW);
                            state = 2;
                        }
                        break;
            
            case 2:     myPID.Compute();
                        Serial.print(pwm_val);
                        Serial.print("\t");
                        Serial.println(encoderValue);
                        setPWM(pwm_val);
                        //if keypress then pwm_val = 100; state = 3;
                        break;
        
            case 3:     //apply PID for postion 0
                        //wait till keypress
                        //if keypress then pwm_val = 100; state = 4;
                        break;
        
            case 4:     if (abs(encoderValue) > release_val) {
                            analogWrite(pwm,0);
                            digitalWrite(valve,HIGH); 
                            Serial.print("Released at: ");
                            Serial.println(encoderValue);
                            state = 0;
                            // move until zero point reached 
                            analogWrite(pwm,20);
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
                        break;
            case 'r':   //runs the states
                        run = 1;
                        break;
            case 'e':   //sets encoder value for const pos
                        reqd_pos = extract_num();
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
                        pwm_val = extract_num();
                        break;                  
            case 'm':   //sets state number
                        state = extract_num();
                        break;
        }
    }
    else
        inputString += inChar;
}

double extract_num(String str) {
    return(str.substring(str.indexOf('=')+1).toFloat());
}

void print_data(void) {
    Serial.println(inputString);
    Serial.print("Encoder=");
    Serial.print(encoderValue);
    Serial.print("PID_Pos=");
    Serial.print(reqd_pos);
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

void setPWM(char val){
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
  //if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;
  else encoderValue --;

  lastEncoded = encoded; //store this value for next time
}

