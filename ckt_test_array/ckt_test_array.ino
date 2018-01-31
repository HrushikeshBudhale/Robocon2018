const unsigned char motor_inA[4] = {58, 60, 62, 64};
const unsigned char motor_inB[4] = {59, 61, 63, 65};
const unsigned char motor_dir[4] = {11, 9, 7, 5};
const unsigned char motor_pwm[4] = {12, 10, 8, 6};

const unsigned char dir_val[4] = {1, 1, 0, 0};

const unsigned char PWM_VAL = 100;

//###############################################################################

long m_encoder_prev[4] = {0, 0, 0, 0};
long m_encoder_curr[4] = {0, 0, 0, 0};
float m_speed[4] = {0, 0, 0, 0};
unsigned char m_state[4] = {0, 0, 0, 0};
double volatile m_count[4] = {0, 0, 0, 0};

unsigned long int now;

void setup() {

    attachInterrupt(motor_inA[0], m1_int, CHANGE);
    attachInterrupt(motor_inB[0], m1_int, CHANGE);
    
    attachInterrupt(motor_inA[1], m2_int, CHANGE);
    attachInterrupt(motor_inB[1], m2_int, CHANGE);
    
    attachInterrupt(motor_inA[2], m3_int, CHANGE);
    attachInterrupt(motor_inB[2], m3_int, CHANGE);
    
    attachInterrupt(motor_inA[3], m4_int, CHANGE);
    attachInterrupt(motor_inB[3], m4_int, CHANGE);

    for(char i = 0; i<4; i++){
        pinMode(motor_dir[i], OUTPUT);  
        pinMode(motor_pwm[i], OUTPUT);
    }
    
    // start serial port at 115200 bps:
    Serial.begin(115200);
    
    delay(4000);

    for(char i=0; i<4; i++){
        analogWrite(motor_pwm[i], PWM_VAL);
        digitalWrite(motor_dir[i], dir_val[i]);
        
    }
    now = millis();
}

void loop() {
    while((millis()-now) < 5000) {
        for(char i=0; i<4; i++){
            m_speed[i] = (m_encoder_curr[i] - m_encoder_prev[i])*0.0485;
        }

        for(char i=0; i<4; i++){
            m_encoder_prev[i] = m_encoder_curr[i];
        }
        
    //    Serial.print(m_encoder_curr[0]);
    //    Serial.print(",\t");
    //    Serial.print(m_encoder_curr[1]);
    //    Serial.print(",\t");
    //    Serial.print(m_encoder_curr[2]);
    //    Serial.print(",\t");
    //    Serial.print(m_encoder_curr[3]);
    //    Serial.print(",\t\t");
        for(char i=0; i<4; i++){
            Serial.print(m_speed[i]);
            Serial.print(",\t");
        }
        Serial.println(" ");
        delay(20);
    }
    for(char i=0; i<4; i++){
        analogWrite(motor_pwm[i], 0);    
    }
}

void m1_int(void) {
    unsigned char a = digitalRead(motor_inA[0]);
    unsigned char b = digitalRead(motor_inB[0]);
    if(b == 0 && a == 0){
        if(m_state[0] == 1){
          m_encoder_curr[0]--;
        }
        if(m_state[0] == 3){
          m_encoder_curr[0]++;
        }
        m_state[0] = 0;
      }
    
      if(b == 1 && a == 0){
        if(m_state[0] == 0){
          m_encoder_curr[0]++;
        }
        if(m_state[0] == 2){
          m_encoder_curr[0]--;
        }
        m_state[0] = 1;
      }
    
      if(b == 1 && a == 1){
        if(m_state[0] == 3){
          m_encoder_curr[0]--;
        }
        if(m_state[0] == 1){
          m_encoder_curr[0]++;
        }
        m_state[0] = 2;
      }
    
      if(b == 0 && a == 1){
        if(m_state[0] == 2){
          m_encoder_curr[0]++;
        }
        if(m_state[0] == 0){
          m_encoder_curr[0]--;
        }
        m_state[0] = 3;
      }
}


void m2_int(void) {
    unsigned char a = digitalRead(motor_inA[1]);
    unsigned char b = digitalRead(motor_inB[1]);
    if(b == 0 && a == 0){
        if(m_state[1] == 1){
          m_encoder_curr[1]--;
        }
        if(m_state[1] == 3){
          m_encoder_curr[1]++;
        }
        m_state[1] = 0;
      }
    
      if(b == 1 && a == 0){
        if(m_state[1] == 0){
          m_encoder_curr[1]++;
        }
        if(m_state[1] == 2){
          m_encoder_curr[1]--;
        }
        m_state[1] = 1;
      }
    
      if(b == 1 && a == 1){
        if(m_state[1] == 3){
          m_encoder_curr[1]--;
        }
        if(m_state[1] == 1){
          m_encoder_curr[1]++;
        }
        m_state[1] = 2;
      }
    
      if(b == 0 && a == 1){
        if(m_state[1] == 2){
          m_encoder_curr[1]++;
        }
        if(m_state[1] == 0){
          m_encoder_curr[1]--;
        }
        m_state[1] = 3;
      }
}


void m3_int(void) {
    unsigned char a = digitalRead(motor_inA[2]);
    unsigned char b = digitalRead(motor_inB[2]);
    if(b == 0 && a == 0){
        if(m_state[2] == 1){
          m_encoder_curr[2]--;
        }
        if(m_state[2] == 3){
          m_encoder_curr[2]++;
        }
        m_state[2] = 0;
      }
    
      if(b == 1 && a == 0){
        if(m_state[2] == 0){
          m_encoder_curr[2]++;
        }
        if(m_state[2] == 2){
          m_encoder_curr[2]--;
        }
        m_state[2] = 1;
      }
    
      if(b == 1 && a == 1){
        if(m_state[2] == 3){
          m_encoder_curr[2]--;
        }
        if(m_state[2] == 1){
          m_encoder_curr[2]++;
        }
        m_state[2] = 2;
      }
    
      if(b == 0 && a == 1){
        if(m_state[2] == 2){
          m_encoder_curr[2]++;
        }
        if(m_state[2] == 0){
          m_encoder_curr[2]--;
        }
        m_state[2] = 3;
      }
}


void m4_int(void) {
    unsigned char a = digitalRead(motor_inA[3]);
    unsigned char b = digitalRead(motor_inB[3]);
    if(b == 0 && a == 0){
        if(m_state[3] == 1){
          m_encoder_curr[3]--;
        }
        if(m_state[3] == 3){
          m_encoder_curr[3]++;
        }
        m_state[3] = 0;
      }
    
      if(b == 1 && a == 0){
        if(m_state[3] == 0){
          m_encoder_curr[3]++;
        }
        if(m_state[3] == 2){
          m_encoder_curr[3]--;
        }
        m_state[3] = 1;
      }
    
      if(b == 1 && a == 1){
        if(m_state[3] == 3){
          m_encoder_curr[3]--;
        }
        if(m_state[3] == 1){
          m_encoder_curr[3]++;
        }
        m_state[3] = 2;
      }
    
      if(b == 0 && a == 1){
        if(m_state[3] == 2){
          m_encoder_curr[3]++;
        }
        if(m_state[3] == 0){
          m_encoder_curr[3]--;
        }
        m_state[3] = 3;
      }
}

