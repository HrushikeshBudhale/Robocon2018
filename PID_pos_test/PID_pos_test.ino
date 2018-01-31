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
long encount;
int lastMSB = 0;
int lastLSB = 0;

int integerValue;
char incomingByte;
int i = 0;

double Kp=0.3;
double Ki=0.0;
double Kd=0.007;
double consts[] = {Kp,Ki,Kd};
double reqd_pos = 3000;


PID myPID(&encoderValue, &pwm_val, &reqd_pos, consts[0], consts[1], consts[2], DIRECT);

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
  
//    if(Serial.available() > 0){   // something came across serial
//        integerValue = 0;         // throw away previous integerValue
//      
//        while(1) {            // force into a loop until 7'n' is received
//            incomingByte = Serial.read();
//            if(incomingByte == '\n') {
//                i++;break;   // exit the while(1), we're done receiving
//                if(i==3) i = 0;
//            }
//            if (incomingByte == -1)
//                continue;  // if no characters are in the buffer read() returns -1
//            integerValue *= 10;  // shift left 1 decimal place
//            // convert ASCII to integer, add, and shift left 1 decimal place
//            integerValue = ((incomingByte - 48) + integerValue);
//        }
//        consts[i] = integerValue/10.0;
//        myPID.SetTunings(consts[0], consts[1], consts[2]);
//        Serial.println(consts[i]);   // Do something with the value
//    }
    myPID.Compute();
    Serial.print(pwm_val);
    Serial.print("\t");
    
    Serial.println(encoderValue);
    setPWM(pwm_val);
    if(abs(reqd_pos - encoderValue))
    reqd_pos += 1;
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
