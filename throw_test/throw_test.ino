//these pins can not be changed 2/3 are special pins
#define dir 3//8
#define pwm 4//9
#define valve 2//4
#define encoderPin1 64//2
#define encoderPin2 65//3

double release_val = 51100;
byte r_dir = 1;
double pwm_val = 100;

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

char stop = 1;
char gripped = 0;

void setup() {
    Serial.begin (115200);
    while(Serial.available() <= 0);
    
    pinMode(encoderPin1, INPUT);
    pinMode(encoderPin2, INPUT);
    
    pinMode(dir, OUTPUT);
    digitalWrite(dir,r_dir);
    
    pinMode(pwm,OUTPUT);
    analogWrite(pwm,0);
    
    pinMode(valve,OUTPUT);
    digitalWrite(valve,LOW);
    
    digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
    digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
    
    //call updateEncoder() when any high/low changed seen
    //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
    attachInterrupt(0, updateEncoder, CHANGE); 
    attachInterrupt(1, updateEncoder, CHANGE);
}

void loop(){ 
    Serial.print(encoderValue);
    Serial.print("\n");
    if(abs(encoderValue)>release_val && stop == 0)
    {
        analogWrite(pwm,0);
        digitalWrite(valve,HIGH); 
        delay(1000);
        digitalWrite(valve,LOW);
        stop = 1;
        gripped = 0;
    }
    if(stop == 0){
        analogWrite(pwm,pwm_val);         
    }

    if(encoderValue > 2500 && encoderValue < 3500 && gripped == 0){
        digitalWrite(valve,HIGH); 
        delay(1000);
        digitalWrite(valve,LOW);
        gripped = 1;
    }
 
    if (stringComplete) {
        inputString = "";
        stringComplete = false;
    }
}


void serialEvent() {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
        stringComplete = true;
        switch(inputString.charAt(0)){
            case 's':   //stops the controller
                        stop = 1;
                        encoderValue = 0;
                        break;
            
            case 'r':   //runs the controller
                        stop = 0;
                        break;

            case 'e':   //sets encoder value
                        release_val = inputString.substring(inputString.indexOf('=')+1).toInt();
                        break;

            case 'p':   //sets encoder value
                        pwm_val = inputString.substring(inputString.indexOf('=')+1).toInt();
                        break;
        }
        Serial.println(inputString);
        Serial.println("\nPress 'r' to run");
    }
    else
        inputString += inChar;
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
