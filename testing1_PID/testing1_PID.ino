/* Written by : HRUSHIKESH
 * Date: 14/01/18 02:34:27
 */
// ################ Including necessary Libraries #############

#include <Encoder.h>
#include <PID_v1.h>

// ################ Defining motor pin names ##############

#define motor1_inA 14
#define motor1_inB 15
#define motor1_dir 3
#define motor1_pwm 7
double m1_pwm_val = 0;

#define motor2_inA 16
#define motor2_inB 17
#define motor2_dir 2
#define motor2_pwm 6
double m2_pwm_val = 0;

#define motor3_inA 18
#define motor3_inB 19
#define motor3_dir 1
#define motor3_pwm 5
double m3_pwm_val = 0;

#define motor4_inA 20
#define motor4_inB 21
#define motor4_dir 0
#define motor4_pwm 4
double m4_pwm_val = 0;

// ############# Creating instances of class encoder ###########

Encoder m1_encoder(motor1_inA, motor1_inB);
Encoder m2_encoder(motor2_inA, motor2_inB);
Encoder m3_encoder(motor3_inA, motor3_inB);
Encoder m4_encoder(motor4_inA, motor4_inB);

long m1_encoder_prev = 0;
long m2_encoder_prev = 0;
long m3_encoder_prev = 0;
long m4_encoder_prev = 0;

long m1_encoder_curr = 0;
long m2_encoder_curr = 0;
long m3_encoder_curr = 0;
long m4_encoder_curr = 0;

double m1_speed = 0;
double m2_speed = 0;
double m3_speed = 0;
double m4_speed = 0;

double reqd_speed = 150;
boolean transmit = 1;

// ############# PID related variables and objects initialization ###############

// Specify the links and initial tuning parameters
double m1_Kp=2;
double m2_Kp=2;
double m3_Kp=2;
double m4_Kp=2;

double m1_Ki=0;
double m2_Ki=0;
double m3_Ki=0;
double m4_Ki=0;

double m1_Kd=0;
double m2_Kd=0;
double m3_Kd=0;
double m4_Kd=0;

// creating instances of class PID
// .......((&Input  , &Output    , &Setpoint  , Kp, Ki, Kd..............

PID m1_PID(&m1_speed, &m1_pwm_val, &reqd_speed, m1_Kp, m1_Ki, m1_Kd, P_ON_M, DIRECT);
PID m2_PID(&m2_speed, &m2_pwm_val, &reqd_speed, m2_Kp, m2_Ki, m2_Kd, P_ON_M, DIRECT);
PID m3_PID(&m3_speed, &m3_pwm_val, &reqd_speed, m3_Kp, m3_Ki, m3_Kd, P_ON_M, DIRECT);
PID m4_PID(&m4_speed, &m4_pwm_val, &reqd_speed, m4_Kp, m4_Ki, m4_Kd, P_ON_M, DIRECT);

// ##################### setup function ########################
void setup() {
    
    m1_encoder.write(0);
    m2_encoder.write(0);
    m3_encoder.write(0);
    m4_encoder.write(0);
    
    pinMode(motor1_dir, OUTPUT);  
    pinMode(motor1_pwm, OUTPUT);
    
    pinMode(motor2_dir, OUTPUT);  
    pinMode(motor2_pwm, OUTPUT);
    
    pinMode(motor3_dir, OUTPUT);  
    pinMode(motor3_pwm, OUTPUT);
    
    pinMode(motor4_dir, OUTPUT);  
    pinMode(motor4_pwm, OUTPUT);

    m1_PID.SetMode(AUTOMATIC);
    m2_PID.SetMode(AUTOMATIC);
    m3_PID.SetMode(AUTOMATIC);
    m4_PID.SetMode(AUTOMATIC);

    // start serial port at 115200 bps:
    Serial.begin(115200);
    while (Serial.available() <= 0);

    set_direction(1);
    
    delay(4000);
}

void loop() {

    //################ Measure Speed of each motor ###############
    
    m1_encoder_curr = m1_encoder.read()/1000;
    m2_encoder_curr = m2_encoder.read()/1000;
    m3_encoder_curr = m3_encoder.read()/1000;
    m4_encoder_curr = m4_encoder.read()/1000;

    m1_speed = m1_encoder_curr - m1_encoder_prev;
    m2_speed = m2_encoder_curr - m2_encoder_prev;
    m3_speed = m3_encoder_curr - m3_encoder_prev;
    m4_speed = m4_encoder_curr - m4_encoder_prev;

    m1_encoder_prev = m1_encoder_curr;
    m2_encoder_prev = m2_encoder_curr;
    m3_encoder_prev = m3_encoder_curr;
    m4_encoder_prev = m4_encoder_curr;

    // ############## Compute required PID values #################

    m1_PID.Compute();
    m2_PID.Compute();
    m3_PID.Compute();
    m4_PID.Compute();

    // ##################### Set the speed(PWM) ###################

    analogWrite(motor1_pwm, m1_pwm_val);
    analogWrite(motor2_pwm, m2_pwm_val);
    analogWrite(motor3_pwm, m3_pwm_val);
    analogWrite(motor4_pwm, m4_pwm_val);
    
    // ################### display data #################
    if(transmit) {
        //display_data();
        String out = String(m1_speed + ", ");
        out = String(m2_speed + ", ");
        out = String(m3_speed + ", ");
        //out = String(m4_speed + "\t ");
    
    }
    
    delay(10);

    // ################### Logic for stopping stransmission ##################
    
    if (Serial.available()) {
        Serial.read();
        Serial.println("end of Transmission !");
        transmit = 0;
    }
}


//void display_data() {
//    /* Function to display required data on serial monitor
//     */
//    String out = String(m1_speed + ", ");
//    out = String(m2_speed + ", ");
//    out = String(m3_speed + ", ");
//    //out = String(m4_speed + "\t ");
//    out = String(out + m1_encoder_curr + ", " + m2_encoder_curr + ", " + m3_encoder_curr + ", " + m4_encoder_curr + "\t ");
//    //out = out + m1_pwm_val + ", " + m2_pwm_val + ", " + m3_pwm_val + ", " + m4_pwm_val + "\t ";
//    Serial.println(out);
//}

void set_direction(int dir) {
    /* set_direction function sets the direction of 4 motors
     *  arguments : 1,2,3,4
     *  1- forward, 2- right, 3- backward, 4- left
     */
    switch(dir){
        case 1 :    digitalWrite(motor1_dir, 0);
                    digitalWrite(motor1_dir, 0);
                    digitalWrite(motor1_dir, 0);
                    digitalWrite(motor1_dir, 0);
                    break;

        case 2 :    digitalWrite(motor1_dir, 0);
                    digitalWrite(motor1_dir, 0);
                    digitalWrite(motor1_dir, 0);
                    digitalWrite(motor1_dir, 0);
                    break;

        case 3 :    digitalWrite(motor1_dir, 0);
                    digitalWrite(motor1_dir, 0);
                    digitalWrite(motor1_dir, 0);
                    digitalWrite(motor1_dir, 0);
                    break;

        case 4 :    digitalWrite(motor1_dir, 0);
                    digitalWrite(motor1_dir, 0);
                    digitalWrite(motor1_dir, 0);
                    digitalWrite(motor1_dir, 0);
                    break;                    
    }
}
