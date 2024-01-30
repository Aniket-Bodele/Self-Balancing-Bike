#include <Arduino.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <Encoder.h>


MPU6050 mpu(Wire);
unsigned long timer = 0;

Encoder DC_Encoder(2, 3);
#define enA 6
#define in1 A2
#define in2 A3

//Bo Motor
#define enB 5
#define in3 9
#define in4 4

// Variables init
#define ALPHA_1 0.98
#define ALPHA_2 0.90
float rotation = 0;
float radian = 0;
float old_radian = 0;
float a = 0;
float a_dot = 0;
long old_ticks = -999;
float r = 3;  // radius of wheel;
float l = 24; // length of axis;
float setpoint = 15,
      pitch = 0,
      yaw = 0,
      yaw_dot = 0,
      prev_pitch = 0,
      prev_yaw = 0,
      torque = 0,
      yaw_setpoint = 0,
      alpha_change = 0,
      theta_dot_change = 0,
      yaw_degree = 0,
      angle = 0,
      alpha_dot = 0,
      delta_yaw_dot = 0,
      delta_theta_dot = 0;

float I_total = 0.01830661;
float dt =0.01;
unsigned long millisOld = 0;
float pwm_factor = 4;
float delta_pwm = 0, prev_pwm = 0;


float pwm = 0;
float delta = 0.0;
float velocity = 0;
float new_setpoint = 0;

//Nxt plan: Low Kp and increase pos_kp
// Checkpoint 1 best
// 2: Second Best
// 3: Third
//Angular PID parameters
float Kp = 3.5; //proportional gain //4.25
float Ki = 0.0; //integral gain //0.0
float Kd = 0.5; //derivative gain //0.1

//rotational PID parameters
float pos_Kp = 0.004; //proportional gain //0.003
float pos_Ki = 0.00; //integral gain //0
float pos_Kd = 0.3; //derivative gain //0.35 doing well
float acc_Kp = 0;

const float motor_pwm_offset = 69; //59


int sign(float val){
  return (val>=0)?1:-1;
}

// ================================================================
// ===                      ANGULAR PID                         ===
// ================================================================

//Angular PID compute parameters
unsigned long last_time = 0;
float delta_error = 0;
float total_error = 0, last_error = 0;
unsigned short int delta_time = 0;

void angular_pid(float angle){

    unsigned long current_time = millis(); //returns the number of milliseconds passed since the Arduino started running the program
    delta_time = current_time - last_time; //delta time interval 
    //P
    double error = setpoint + new_setpoint - angle;

    //I
    if(abs(error)<200)
    total_error += error; 
    else
    total_error = 0;
    // if (abs(total_error) >= 400)
    // total_error = 400*total_error/abs(total_error + 1e-3);


    //D
    if(delta_time > 50){
      delta_error = (error - last_error); //difference of error for derivative term
      last_error = error;
      last_time = current_time;
    }
    //Output
    pwm = Kp*error + (Ki)*total_error + (Kd)*delta_error; //PID control compute


    // Serial.print("Angular PID\t");
    // Serial.print(error);
    // Serial.print("\t");
    // Serial.print(total_error);
    // Serial.print("\t");
    // Serial.println(delta_error);
    // Serial.println(error);
    // Serial.print("\t");
    // Serial.print(total_error);
    // Serial.print("\t");
    // Serial.println(pwm);
}

// ================================================================
// ===                      rotation PID                        ===
// ================================================================

//rotation PID parameters
unsigned long pos_last_time = 0;
double current_velocity = 0;
double current_acceleration = 0;
double previous_velocity = 0;
float velocity_error = 0;
float pos_total_error=0, pos_last_rotation=0;
unsigned short int pos_delta_time = 0;
long int velocity_sum_error = 0.0;
long target_rotation = 0.0;

void rotation_pid(double pos_rotation){

    unsigned long pos_current_time = millis(); //returns the number of milliseconds passed since the Arduino started running the program
    pos_delta_time = pos_current_time - pos_last_time; //delta time interval 

    //D
    if(pos_delta_time > 50){
      current_velocity = (pos_rotation - pos_last_rotation); //difference of error for derivative term
      current_acceleration = (current_velocity - previous_velocity);
      velocity_error = velocity - current_velocity;
      pos_last_rotation = pos_rotation;
      previous_velocity = current_velocity;
      pos_last_time = pos_current_time;
      velocity_sum_error += velocity_error*pos_Ki;
    }
    //double D

    // if (abs(current_velocity) > 7)
    // pos_Kd = 0.9;
    // else
    // pos_Kd = 1.15;

    //Output
    new_setpoint = (pos_Kp)*(target_rotation - pos_rotation) + (pos_Kd)*velocity_error + velocity_sum_error + current_acceleration*acc_Kp; //PID control compute
}

// ================================================================
// ===                      FUNCTIONS                           ===
// ================================================================

//BO MOTOR//
void bo_motor_init(){
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void backward(int pwm){
  analogWrite(enB, pwm);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);  
}

void forward(int pwm){
  analogWrite(enB, pwm);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);  
}

void dc_motor_init()
{
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}

void motor_control(float pwm)
{
    if (pwm < 0)
    {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        pwm = -pwm;
    }
    else if (pwm > 0)
    {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }

    if(abs(pwm)+motor_pwm_offset>150)
    pwm = 150-motor_pwm_offset;

    if(pwm ==0)
        {digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(enA, 0);}
    else
        analogWrite(enA, pwm + motor_pwm_offset);
}

float encoder()
{
    long ticks = DC_Encoder.read();
    if (ticks != old_ticks)
    {
        old_ticks = ticks;
    }
    // Serial.println(ticks);
    return ticks;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    Serial.begin(115200);
    Wire.begin();
    dc_motor_init();
    bo_motor_init();

    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0)
    {
    } // stop everything if could not connect to MPU6050

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.setGyroOffsets(1.44,0.64,0.83);
    mpu.setAccOffsets(0.05,-0.01,0.26);
    //0.05 0.07 0.25 
    // 1.54 0.69 0.78

    //0.05 0.05 0.25 
    // 1.52 0.69 0.77
    Serial.println("Done!\n");
}

void loop() {
    rotation = encoder();
    mpu.update();
    dt=(millis()-millisOld)/1000.;
    millisOld=millis();
    // if(millisOld > 9999 && millisOld < 39999)
    //     {forward(230);
    //     // Kp=5.5;
    //     pos_Kd=0.2;}
    // if(millisOld > 39999 && millisOld < 19999+2000)
    //     {forward(0);
    //     // Kp=4.25;
    //     pos_Kd=0.35;}

    // ================================================================
    // ===                      MAIN LOOP                           ===
    // ================================================================
    angle = (ALPHA_1 * (angle + ((mpu.getGyroX()) * dt)) + (1 - ALPHA_1) * (mpu.getAccAngleX()));
    rotation_pid(rotation);
    angular_pid(angle*10);

    if(abs(angle) < 50)
        motor_control(pwm);
    else
        motor_control(0);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  Print  //
    Serial.print(pwm);
    Serial.print("\t");
    Serial.println(angle); 

}