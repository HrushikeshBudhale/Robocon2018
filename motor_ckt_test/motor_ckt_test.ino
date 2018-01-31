//#include <quadrature.h>
//#include <Encoder.h>
    
#define motor1_inA 58
#define motor1_inB 59
#define motor1_dir 11
#define motor1_pwm 12

#define motor2_inA 60
#define motor2_inB 61
#define motor2_dir 9
#define motor2_pwm 10

#define motor3_inA 62
#define motor3_inB 63
#define motor3_dir 7
#define motor3_pwm 8

#define motor4_inA 64
#define motor4_inB 65
#define motor4_dir 5
#define motor4_pwm 6

#define PWM_VAL 100

//###############################################################################

long m1_encoder_prev = 0;
long m2_encoder_prev = 0;
long m3_encoder_prev = 0;
long m4_encoder_prev = 0;

long m1_encoder_curr = 0;
long m2_encoder_curr = 0;
long m3_encoder_curr = 0;
long m4_encoder_curr = 0;

float m1_speed = 0;
float m2_speed = 0;
float m3_speed = 0;
float m4_speed = 0;

unsigned char m1_state = 0;
unsigned char m2_state = 0;
unsigned char m3_state = 0;
unsigned char m4_state = 0;

double volatile m1_count = 0;
double volatile m2_count = 0;
double volatile m3_count = 0;
double volatile m4_count = 0;

unsigned long int now = 0;

void setup() {
    attachInterrupt(motor1_inA, m1_int, CHANGE);
    attachInterrupt(motor1_inB, m1_int, CHANGE);
    
    attachInterrupt(motor2_inA, m2_int, CHANGE);
    attachInterrupt(motor2_inB, m2_int, CHANGE);
    
    attachInterrupt(motor3_inA, m3_int, CHANGE);
    attachInterrupt(motor3_inB, m3_int, CHANGE);
    
    attachInterrupt(motor4_inA, m4_int, CHANGE);
    attachInterrupt(motor4_inB, m4_int, CHANGE);

    pinMode(motor1_dir, OUTPUT);  
    pinMode(motor1_pwm, OUTPUT);
    
    pinMode(motor2_dir, OUTPUT);  
    pinMode(motor2_pwm, OUTPUT);
    
    pinMode(motor3_dir, OUTPUT);  
    pinMode(motor3_pwm, OUTPUT);
    
    pinMode(motor4_dir, OUTPUT);  
    pinMode(motor4_pwm, OUTPUT);
    
    // start serial port at 115200 bps:
    Serial.begin(115200);
    
    delay(4000);
    
    analogWrite(motor1_pwm, PWM_VAL);
    digitalWrite(motor1_dir, 1);
    analogWrite(motor2_pwm, PWM_VAL);
    digitalWrite(motor2_dir, 1);
    analogWrite(motor3_pwm, PWM_VAL);
    digitalWrite(motor3_dir, 0);
    analogWrite(motor4_pwm, PWM_VAL);
    digitalWrite(motor4_dir, 0);
    
    now = millis();
}

void loop() {
    while((millis()-now) < 5000) {
        m1_speed = (m1_encoder_curr - m1_encoder_prev)*0.0485;
        m2_speed = (m2_encoder_curr - m2_encoder_prev)*0.0485;
        m3_speed = (m3_encoder_curr - m3_encoder_prev)*0.0485;
        m4_speed = (m4_encoder_curr - m4_encoder_prev)*0.0485;
    
        m1_encoder_prev = m1_encoder_curr;
        m2_encoder_prev = m2_encoder_curr;
        m3_encoder_prev = m3_encoder_curr;
        m4_encoder_prev = m4_encoder_curr;
        
    //    Serial.print(m1_encoder_curr);
    //    Serial.print(",\t");
    //    Serial.print(m2_encoder_curr);
    //    Serial.print(",\t");
    //    Serial.print(m3_encoder_curr);
    //    Serial.print(",\t");
    //    Serial.print(m4_encoder_curr);
    //    Serial.print(",\t\t");
        Serial.print(m1_speed);
        Serial.print(",\t");
        Serial.print(m2_speed);
        Serial.print(",\t");
        Serial.print(m3_speed);
        Serial.print(",\t");
        Serial.println(m4_speed);
        
        delay(20);
    }
    analogWrite(motor1_pwm, 0);
    analogWrite(motor2_pwm, 0);
    analogWrite(motor3_pwm, 0);
    analogWrite(motor4_pwm, 0);
}

void m1_int(void) {
    unsigned char a = digitalRead(motor1_inA);
    unsigned char b = digitalRead(motor1_inB);
    if(b == 0 && a == 0){
        if(m1_state == 1){
          m1_encoder_curr--;
        }
        if(m1_state == 3){
          m1_encoder_curr++;
        }
        m1_state = 0;
      }
    
      if(b == 1 && a == 0){
        if(m1_state == 0){
          m1_encoder_curr++;
        }
        if(m1_state == 2){
          m1_encoder_curr--;
        }
        m1_state = 1;
      }
    
      if(b == 1 && a == 1){
        if(m1_state == 3){
          m1_encoder_curr--;
        }
        if(m1_state == 1){
          m1_encoder_curr++;
        }
        m1_state = 2;
      }
    
      if(b == 0 && a == 1){
        if(m1_state == 2){
          m1_encoder_curr++;
        }
        if(m1_state == 0){
          m1_encoder_curr--;
        }
        m1_state = 3;
      }
}


void m2_int(void) {
    unsigned char a = digitalRead(motor2_inA);
    unsigned char b = digitalRead(motor2_inB);
    if(b == 0 && a == 0){
        if(m2_state == 1){
          m2_encoder_curr--;
        }
        if(m2_state == 3){
          m2_encoder_curr++;
        }
        m2_state = 0;
      }
    
      if(b == 1 && a == 0){
        if(m2_state == 0){
          m2_encoder_curr++;
        }
        if(m2_state == 2){
          m2_encoder_curr--;
        }
        m2_state = 1;
      }
    
      if(b == 1 && a == 1){
        if(m2_state == 3){
          m2_encoder_curr--;
        }
        if(m2_state == 1){
          m2_encoder_curr++;
        }
        m2_state = 2;
      }
    
      if(b == 0 && a == 1){
        if(m2_state == 2){
          m2_encoder_curr++;
        }
        if(m2_state == 0){
          m2_encoder_curr--;
        }
        m2_state = 3;
      }
}


void m3_int(void) {
    unsigned char a = digitalRead(motor3_inA);
    unsigned char b = digitalRead(motor3_inB);
    if(b == 0 && a == 0){
        if(m3_state == 1){
          m3_encoder_curr--;
        }
        if(m3_state == 3){
          m3_encoder_curr++;
        }
        m3_state = 0;
      }
    
      if(b == 1 && a == 0){
        if(m3_state == 0){
          m3_encoder_curr++;
        }
        if(m3_state == 2){
          m3_encoder_curr--;
        }
        m3_state = 1;
      }
    
      if(b == 1 && a == 1){
        if(m3_state == 3){
          m3_encoder_curr--;
        }
        if(m3_state == 1){
          m3_encoder_curr++;
        }
        m3_state = 2;
      }
    
      if(b == 0 && a == 1){
        if(m3_state == 2){
          m3_encoder_curr++;
        }
        if(m3_state == 0){
          m3_encoder_curr--;
        }
        m3_state = 3;
      }
}


void m4_int(void) {
    unsigned char a = digitalRead(motor4_inA);
    unsigned char b = digitalRead(motor4_inB);
    if(b == 0 && a == 0){
        if(m4_state == 1){
          m4_encoder_curr--;
        }
        if(m4_state == 3){
          m4_encoder_curr++;
        }
        m4_state = 0;
      }
    
      if(b == 1 && a == 0){
        if(m4_state == 0){
          m4_encoder_curr++;
        }
        if(m4_state == 2){
          m4_encoder_curr--;
        }
        m4_state = 1;
      }
    
      if(b == 1 && a == 1){
        if(m4_state == 3){
          m4_encoder_curr--;
        }
        if(m4_state == 1){
          m4_encoder_curr++;
        }
        m4_state = 2;
      }
    
      if(b == 0 && a == 1){
        if(m4_state == 2){
          m4_encoder_curr++;
        }
        if(m4_state == 0){
          m4_encoder_curr--;
        }
        m4_state = 3;
      }
}

