#ifndef HEADER_H
#define HEADER_H

#include "Arduino.h"
#include "Wire.h"
#include "ESP32Servo.h"

#define gyro_address 0x68                  //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile float pid_p_gain_roll = 0.4;                     //Gain setting for the pitch and roll P-controller (default = 1.3).
volatile float pid_i_gain_roll = 0.002;                    //Gain setting for the pitch and roll I-controller (default = 0.04).
volatile float pid_d_gain_roll = 10.0;                    //Gain setting for the pitch and roll D-controller (default = 18.0).
int pid_max_roll = 400;                                   //Maximum output of the PID-controller (+/-).

volatile float pid_p_gain_pitch = pid_p_gain_roll;        //Gain setting for the pitch P-controller.
volatile float pid_i_gain_pitch = pid_i_gain_roll;        //Gain setting for the pitch I-controller.
volatile float pid_d_gain_pitch = pid_d_gain_roll;        //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;                         //Maximum output of the PID-controller (+/-).

volatile float pid_p_gain_yaw = 3.0;                      //Gain setting for the pitch P-controller (default = 4.0).
volatile float pid_i_gain_yaw = 0.002;                     //Gain setting for the pitch I-controller (default = 0.02).
volatile float pid_d_gain_yaw = 0.0;                      //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                                    //Maximum output of the PID-controller (+/-).

volatile boolean auto_level = true;                      //Auto level on (true) or off (false).

//Manual accelerometer calibration values for IMU angles:
volatile int16_t manual_acc_pitch_cal_value = 171;
volatile int16_t manual_acc_roll_cal_value = 5;

//Manual gyro calibration values.
//Set the use_manual_calibration variable to true to use the manual calibration variables.
uint8_t use_manual_calibration = true;                      // Set to false or true;
volatile int16_t manual_gyro_pitch_cal_value = 59;
volatile int16_t manual_gyro_roll_cal_value = 446;
volatile int16_t manual_gyro_yaw_cal_value = 35;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer
uint8_t start, error;

int16_t esc_1, esc_2, esc_3, esc_4;
int16_t throttle, cal_int;
volatile int16_t temperature;
int16_t count_var;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;


volatile int32_t channel_1, channel_2, channel_3, channel_4;
int32_t acc_total_vector;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

uint32_t loop_timer;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;

const int esc1_pin = 33;
const int esc2_pin = 32;
const int esc3_pin = 19;
const int esc4_pin = 18;
Servo esc1_pwm, esc2_pwm, esc3_pwm, esc4_pwm;

void pwmSetup() {

    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
    
    esc1_pwm.setPeriodHertz(400);
    esc2_pwm.setPeriodHertz(400);
    esc3_pwm.setPeriodHertz(400);
    esc4_pwm.setPeriodHertz(400);

    esc1_pwm.attach(esc1_pin, 1000, 2000);
    esc2_pwm.attach(esc2_pin, 1000, 2000);
    esc3_pwm.attach(esc3_pin, 1000, 2000);
    esc4_pwm.attach(esc4_pin, 1000, 2000);
    esc1_pwm.writeMicroseconds(1000);
    esc2_pwm.writeMicroseconds(1000);
    esc3_pwm.writeMicroseconds(1000);
    esc4_pwm.writeMicroseconds(1000);
    delay(100);
}

void i2c_setup() {
    Wire.begin();                                                   //Start the I2C as master
    Wire.setClock(400000);
    Wire.beginTransmission(gyro_address);                           //Start communication with the MPU-6050.
    error = Wire.endTransmission();                                 //End the transmission and register the exit status.
    while (error != 0) {                                            //Stay in this loop because the MPU-6050 did not responde.
        error = 2;                                                  //Set the error status to 2.
        digitalWrite(2, !digitalRead(2));                           //Show the error via the LED.
        delay(500);
    }
}

void gyro_setup(){
    Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
    Wire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex).
    Wire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro.
    Wire.endTransmission();                                      //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
    Wire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale).
    Wire.endTransmission();                                      //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
    Wire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex).
    Wire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range).
    Wire.endTransmission();                                      //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
    Wire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex).
    Wire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
    Wire.endTransmission();                                      //End the transmission with the gyro.
}

void gyro_signalen() {
    Wire.beginTransmission(gyro_address);                        //Start communication with the gyro.
    Wire.write(0x3B);                                            //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                      //End the transmission.
    Wire.requestFrom(gyro_address, 14);                          //Request 14 bytes from the MPU 6050.
    acc_y       = Wire.read() << 8 | Wire.read();                //Add the low and high byte to the acc_x variable.
    acc_x       = Wire.read() << 8 | Wire.read();                //Add the low and high byte to the acc_y variable.
    acc_z       = Wire.read() << 8 | Wire.read();                //Add the low and high byte to the acc_z variable.
    temperature = Wire.read() << 8 | Wire.read();                //Add the low and high byte to the temperature variable.
    gyro_roll   = Wire.read() << 8 | Wire.read();                //Read high and low part of the angular data.
    gyro_pitch  = Wire.read() << 8 | Wire.read();                //Read high and low part of the angular data.
    gyro_yaw    = Wire.read() << 8 | Wire.read();                //Read high and low part of the angular data.
    gyro_pitch *= -1;                                            //Invert the direction of the axis.
    gyro_yaw   *= -1;                                            //Invert the direction of the axis.

    acc_y      -= manual_acc_pitch_cal_value;                    //Subtact the manual accelerometer pitch calibration value.
    acc_x      -= manual_acc_roll_cal_value;                     //Subtact the manual accelerometer roll calibration value.
    gyro_roll  -= manual_gyro_roll_cal_value;                    //Subtact the manual gyro roll calibration value.
    gyro_pitch -= manual_gyro_pitch_cal_value;                   //Subtact the manual gyro pitch calibration value.
    gyro_yaw   -= manual_gyro_yaw_cal_value;                     //Subtact the manual gyro yaw calibration value.
}

void calibrate_gyro() {
    if (use_manual_calibration)cal_int = 2000;                                          //If manual calibration is used set cal_int to 2000 to skip the calibration.
    else {
        cal_int = 0;                                                                    //If manual calibration is not used.
        manual_gyro_pitch_cal_value = 0;                                                //Set the manual pitch calibration variable to 0.
        manual_gyro_roll_cal_value = 0;                                                 //Set the manual roll calibration variable to 0.
        manual_gyro_yaw_cal_value = 0;                                                  //Set the manual yaw calibration variable to 0.
    }

    if (cal_int != 2000) {
        //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
        for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                //Take 2000 readings for calibration.
            if (cal_int % 25 == 0) digitalWrite(2, !digitalRead(2));                    //Change the led status every 125 readings to indicate calibration.
            gyro_signalen();                                                            //Read the gyro output.
            gyro_roll_cal += gyro_roll;                                                 //Ad roll value to gyro_roll_cal.
            gyro_pitch_cal += gyro_pitch;                                               //Ad pitch value to gyro_pitch_cal.
            gyro_yaw_cal += gyro_yaw;                                                   //Ad yaw value to gyro_yaw_cal.
            delay(4);                                                                   //Small delay to simulate a 250Hz loop during calibration.
        }
        digitalWrite(2, LOW);
        //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
        gyro_roll_cal /= 2000;                                                          //Divide the roll total by 2000.
        gyro_pitch_cal /= 2000;                                                         //Divide the pitch total by 2000.
        gyro_yaw_cal /= 2000;                                                           //Divide the yaw total by 2000.
        manual_gyro_pitch_cal_value = gyro_pitch_cal;                                   //Set the manual pitch calibration variable to the detected value.
        manual_gyro_roll_cal_value = gyro_roll_cal;                                     //Set the manual roll calibration variable to the detected value.
        manual_gyro_yaw_cal_value = gyro_yaw_cal;                                       //Set the manual yaw calibration variable to the detected value.
    }
}

void calculate_pid(){
    //Roll calculations
    pid_error_temp = gyro_roll_input - pid_roll_setpoint;
    pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
    if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
    else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

    pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
    if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
    else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

    pid_last_roll_d_error = pid_error_temp;

    //Pitch calculations
    pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
    pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
    if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
    else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

    pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
    if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
    else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

    pid_last_pitch_d_error = pid_error_temp;

    //Yaw calculations
    pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
    pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
    if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
    else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

    pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
    if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
    else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

    pid_last_yaw_d_error = pid_error_temp;
}

void setManualCalibrationData() {
    int32_t gyro_axis_cal[4], acc_axis_cal[4];
    int16_t acc_axis[4], gyro_axis[4];
    acc_axis_cal[1] = acc_axis_cal[2] = gyro_axis_cal[1] = gyro_axis_cal[2] = gyro_axis_cal[3] = 0;
    manual_acc_pitch_cal_value = manual_acc_roll_cal_value = manual_gyro_roll_cal_value = manual_gyro_pitch_cal_value = manual_gyro_yaw_cal_value = 0;

    //Let's take multiple gyro data samples to stabilize the gyro.
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                            //Take 2000 readings for calibration.
        if (cal_int % 125 == 0) digitalWrite(2, !digitalRead(2));                               //Change the led status to indicate calibration.
        Wire.beginTransmission(gyro_address);                                                   //Start communication with the gyro.
        Wire.write(0x3B);                                                                       //Start reading @ register 43h and auto increment with every read.
        Wire.endTransmission();                                                                 //End the transmission.
        Wire.requestFrom(gyro_address, 14);                                                     //Request 14 bytes from the MPU 6050.
        acc_axis[1] = Wire.read() << 8 | Wire.read();                                           //Add the low and high byte to the acc_x variable.
        acc_axis[2] = Wire.read() << 8 | Wire.read();                                           //Add the low and high byte to the acc_y variable.
        acc_axis[3] = Wire.read() << 8 | Wire.read();                                           //Add the low and high byte to the acc_z variable.
        temperature = Wire.read() << 8 | Wire.read();                                           //Add the low and high byte to the temperature variable.
        gyro_axis[1] = Wire.read() << 8 | Wire.read();                                          //Read high and low part of the angular data.
        gyro_axis[2] = Wire.read() << 8 | Wire.read();                                          //Read high and low part of the angular data.
        gyro_axis[3] = Wire.read() << 8 | Wire.read();                                          //Read high and low part of the angular data.
        gyro_axis[2] *= -1;                                                                     //Invert gyro so that nose up gives positive value.
        gyro_axis[3] *= -1;                                                                     //Invert gyro so that nose right gives positive value.
        acc_axis[1]  -= manual_acc_pitch_cal_value;                                             //Subtact the manual accelerometer pitch calibration value.
        acc_axis[2]  -= manual_acc_roll_cal_value;                                              //Subtact the manual accelerometer roll calibration value.
        gyro_axis[1] -= manual_gyro_roll_cal_value;                                             //Subtact the manual gyro roll calibration value.
        gyro_axis[2] -= manual_gyro_pitch_cal_value;                                            //Subtact the manual gyro pitch calibration value.
        gyro_axis[3] -= manual_gyro_yaw_cal_value;                                              //Subtact the manual gyro yaw calibration value.
        delay(4);                                                                               //Small delay to simulate a 250Hz loop during calibration.
    }

    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 4000 ; cal_int ++) {                                            //Take 2000 readings for calibration.
        if (cal_int % 125 == 0) digitalWrite(2, !digitalRead(2));                               //Change the led status to indicate calibration.
        Wire.beginTransmission(gyro_address);                                                   //Start communication with the gyro.
        Wire.write(0x3B);                                                                       //Start reading @ register 43h and auto increment with every read.
        Wire.endTransmission();                                                                 //End the transmission.
        Wire.requestFrom(gyro_address, 14);                                                     //Request 14 bytes from the MPU 6050.
        acc_axis[1] = Wire.read() << 8 | Wire.read();                                           //Add the low and high byte to the acc_x variable.
        acc_axis[2] = Wire.read() << 8 | Wire.read();                                           //Add the low and high byte to the acc_y variable.
        acc_axis[3] = Wire.read() << 8 | Wire.read();                                           //Add the low and high byte to the acc_z variable.
        temperature = Wire.read() << 8 | Wire.read();                                           //Add the low and high byte to the temperature variable.
        gyro_axis[1] = Wire.read() << 8 | Wire.read();                                          //Read high and low part of the angular data.
        gyro_axis[2] = Wire.read() << 8 | Wire.read();                                          //Read high and low part of the angular data.
        gyro_axis[3] = Wire.read() << 8 | Wire.read();                                          //Read high and low part of the angular data.
        gyro_axis[2] *= -1;                                                                     //Invert gyro so that nose up gives positive value.
        gyro_axis[3] *= -1;                                                                     //Invert gyro so that nose right gives positive value.
        acc_axis[1]  -= manual_acc_pitch_cal_value;                                             //Subtact the manual accelerometer pitch calibration value.
        acc_axis[2]  -= manual_acc_roll_cal_value;                                              //Subtact the manual accelerometer roll calibration value.
        gyro_axis[1] -= manual_gyro_roll_cal_value;                                             //Subtact the manual gyro roll calibration value.
        gyro_axis[2] -= manual_gyro_pitch_cal_value;                                            //Subtact the manual gyro pitch calibration value.
        gyro_axis[3] -= manual_gyro_yaw_cal_value;                                              //Subtact the manual gyro yaw calibration value.
        acc_axis_cal[1] += acc_axis[1] + manual_acc_pitch_cal_value;                        //Ad the Y accelerometer value to the calibration value.
        acc_axis_cal[2] += acc_axis[2] + manual_acc_roll_cal_value;                         //Ad the X accelerometer value to the calibration value.
        gyro_axis_cal[1] += gyro_axis[1] + manual_gyro_roll_cal_value;                      //Ad roll value to gyro_roll_cal.
        gyro_axis_cal[2] += gyro_axis[2] + manual_gyro_pitch_cal_value;                     //Ad pitch value to gyro_pitch_cal.
        gyro_axis_cal[3] += gyro_axis[3] + manual_gyro_yaw_cal_value;                       //Ad yaw value to gyro_yaw_cal.
        delay(4);                                                                           //Small delay to simulate a 250Hz loop during calibration.
    }
    digitalWrite(2, LOW);
    //Now that we have 4000 measures, we need to devide by 4000 to get the average gyro offset.
    acc_axis_cal[1] /= 4000;                                                                //Divide the accelerometer Y value by 4000.
    acc_axis_cal[2] /= 4000;                                                                //Divide the accelerometer X value by 4000.
    gyro_axis_cal[1] /= 4000;                                                               //Divide the roll total by 4000.
    gyro_axis_cal[2] /= 4000;                                                               //Divide the pitch total by 4000.
    gyro_axis_cal[3] /= 4000;                                                               //Divide the yaw total by 4000.

    manual_acc_pitch_cal_value = acc_axis_cal[1];
    manual_acc_roll_cal_value = acc_axis_cal[2];
    manual_gyro_pitch_cal_value = gyro_axis_cal[2];
    manual_gyro_roll_cal_value = gyro_axis_cal[1];
    manual_gyro_yaw_cal_value = gyro_axis_cal[3];
}


#endif //HEADER_H
