/* Written by : HRUSHIKESH
 * Date: 14/01/18 02:34:27
 */

// ################ Including necessary Libraries #############

#include <PID_v1.h>

// ################ Defining motor pin names ##############

//these pins can not be changed 2/3 are special pins
#define dir 8
#define pwm 9
#define valve 4
#define release_rev 5.1
#define encoderPin1 2
#define encoderPin2 3

double pwm_val = 0;
char r_dir=1;

volatile int lastEncoded = 0;
volatile double encoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete


double Kp=0.0001;
double Ki=0.1136;
double Kd=0.0284;
double reqd_pos = 3000;

byte run = 0;

PID myPID(&encoderValue, &pwm_val, &reqd_pos, Kp, Ki, Kd, DIRECT);

void setup() {
    Serial.begin (115200);
    while(Serial.available() <= 0);
    
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-100,100);
    myPID.SetSampleTime(1);
  
    pinMode(encoderPin1, INPUT); 
    pinMode(encoderPin2, INPUT);
    
    pinMode(dir, OUTPUT); 
    pinMode(pwm,OUTPUT);
    
    digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
    digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
    
    //call updateEncoder() when any high/low changed seen
    //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
    attachInterrupt(0, updateEncoder, CHANGE); 
    attachInterrupt(1, updateEncoder, CHANGE);
    
    digitalWrite(dir,r_dir);
    analogWrite(pwm,pwm_val);

}

void loop(){ 
    if(run == 1){
        myPID.Compute();
        Serial.print(pwm_val);
        Serial.print("\t");
        Serial.println(encoderValue);
        setPWM(pwm_val);
    }
    if (stringComplete) {
        inputString = "";
        stringComplete = false;
    }
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

void setPWM(char val){
    if(val>0)
        digitalWrite(dir, HIGH);
    else 
        digitalWrite(dir, LOW);
    analogWrite(pwm,abs(val));
}

void serialEvent() {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
        stringComplete = true;
        switch(inputString.charAt(0)){
            case 's':   //stops the controller
                        run = 0;
                        encoderValue = 0;
                        setPWM(0);
                        break;
            
            case 'r':   //runs the controller
                        run = 1;
                        break;
            
            case 'e':   //sets encoder value
                        reqd_pos = inputString.substring(inputString.indexOf('=')+1).toFloat();
                        break;
            
            case 'p':   //sets P value for PID controller
                        Kp = inputString.substring(inputString.indexOf('=')+1).toFloat();
                        break;
            case 'i':   //sets I value for PID controller
                        Ki = inputString.substring(inputString.indexOf('=')+1).toFloat();
                        break;
                        
            case 'd':   //sets D value for PID controller
                        Kd = inputString.substring(inputString.indexOf('=')+1).toFloat();
                        break;
        }
        Serial.println(inputString);
        Serial.print("End Pos =");
        Serial.print(reqd_pos);
        Serial.print("\tKp =");
        Serial.print(Kp);
        Serial.print("\tKi =");
        Serial.print(Ki);
        Serial.print("\tKd =");
        Serial.print(Kd);
        Serial.println("\nPress 'r' to run");
    }
    else
        inputString += inChar;
}
