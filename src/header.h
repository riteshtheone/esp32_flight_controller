#ifndef HEADER_H
#define HEADER_H

#include "Arduino.h"
#include "Wire.h"
#include "ESP32Servo.h"

#define BATTERY_VOLTAGE_PIN 35
#define gyro_address 0x68                  //The I2C address of the MPU-6050 is 0x68 in hexadecimal form.
#define MS5611_address 0x76
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

volatile float pid_p_gain_yaw = 4.0;                      //Gain setting for the pitch P-controller (default = 4.0).
volatile float pid_i_gain_yaw = 0.008;                     //Gain setting for the pitch I-controller (default = 0.02).
volatile float pid_d_gain_yaw = 0.0;                      //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                                    //Maximum output of the PID-controller (+/-).

volatile float pid_p_gain_altitude = 0.8;           //Gain setting for the altitude P-controller (default = 1.4).
volatile float pid_i_gain_altitude = 0.002;           //Gain setting for the altitude I-controller (default = 0.2).
volatile float pid_d_gain_altitude = 0.75;          //Gain setting for the altitude D-controller (default = 0.75).
int pid_max_altitude = 400;                         //Maximum output of the PID-controller (+/-).

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


//barometer ------------------------------------------------------------------------
//Pressure variables.
float pid_error_gain_altitude, pid_throttle_gain_altitude;
uint16_t C[7];
uint8_t barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, altutude_hold_pressure;
int32_t dT, dT_C5;
//Altitude PID variables
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;

uint8_t takeoff_detected, manual_altitude_change;
uint8_t flight_mode, flight_mode_counter, flight_mode_led;
int16_t manual_throttle;

int16_t acc_z_average_short[26], acc_z_average_long[51];
int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total ;

uint8_t acc_z_average_short_rotating_mem_location, acc_z_average_long_rotating_mem_location;
int32_t acc_alt_integrated;


int16_t motor_idle_speed = 1170;           //Enter the minimum throttle pulse of the motors when they idle (between 1000 and 1200). 1170 for DJI
volatile int16_t manual_takeoff_throttle = 0;    //Enter the manual hover point when auto take-off detection is not desired (between 1400 and 1600).

volatile float press = 1.5;
volatile int16_t throttle_diff = 1530;
//barometer end -------------------------------------------------------------------------





///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int16_t = signed 16 bit integer
//uint16_t = unsigned 16 bit integer
volatile uint8_t start, error;

int16_t esc_1, esc_2, esc_3, esc_4;
int16_t throttle, takeoff_throttle, cal_int;
volatile int16_t temperature;
int16_t count_var;
int16_t acc_x, acc_y, acc_z;
int16_t gyro_pitch, gyro_roll, gyro_yaw;


volatile int32_t channel_1, channel_2, channel_3, channel_4, channel_5;
int32_t acc_total_vector, acc_total_vector_at_start;
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

volatile float battery_voltage;
void calculateBatteryVoltage() {
    // to measure voltage under 3.3 volt, multiplication factor = 0.0033
    // to measure voltage under 16.8 volt, multiplication factor = 0.0168
    battery_voltage = float(map(analogRead(BATTERY_VOLTAGE_PIN), 0, 4095, 0, 1000) * 0.0168);
}

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

void read_barometer(void) {
  barometer_counter ++;

  //Every time this function is called the barometer_counter variable is incremented. This way a specific action
  //is executed at the correct moment. This is needed because requesting data from the MS5611 takes around 9ms to complete.

  if (barometer_counter == 1) {                                                 //When the barometer_counter variable is 1.
    if (temperature_counter == 0) {                                             //And the temperature counter is 0.
      //Get temperature data from MS-5611
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611
      Wire.write(0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
      Wire.requestFrom(MS5611_address, 3);                                     //Poll 3 data bytes from the MS5611.
      // Store the temperature in a 5 location rotating memory to prevent temperature spikes.
      raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
      raw_temperature_rotating_memory[average_temperature_mem_location] = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
      raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
      average_temperature_mem_location++;
      if (average_temperature_mem_location == 5)average_temperature_mem_location = 0;
      raw_temperature = raw_average_temperature_total / 5;                      //Calculate the avarage temperature of the last 5 measurements.
    }
    else {
      //Get pressure data from MS-5611
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      Wire.write(0x00);                                                        //Send a 0 to indicate that we want to poll the requested data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
      Wire.requestFrom(MS5611_address, 3);                                     //Poll 3 data bytes from the MS5611.
      raw_pressure = Wire.read() << 16 | Wire.read() << 8 | Wire.read();     //Shift the individual bytes in the correct position and add them to the raw_pressure variable.
    }

    temperature_counter ++;                                                     //Increase the temperature_counter variable.
    if (temperature_counter == 20) {                                            //When the temperature counter equals 20.
      temperature_counter = 0;                                                  //Reset the temperature_counter variable.
      //Request temperature data
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611.
      Wire.write(0x58);                                                        //Send a 0x58 to indicate that we want to request the temperature data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
    }
    else {                                                                      //If the temperature_counter variable does not equal 20.
      //Request pressure data
      Wire.beginTransmission(MS5611_address);                                  //Open a connection with the MS5611
      Wire.write(0x48);                                                        //Send a 0x48 to indicate that we want to request the pressure data.
      Wire.endTransmission();                                                  //End the transmission with the MS5611.
    }
  }
  if (barometer_counter == 2) {                                                 //If the barometer_counter variable equals 2.
    //Calculate pressure as explained in the datasheet of the MS-5611.
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    dT += raw_temperature;
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
    //To get a smoother pressure value we will use a 20 location rotating memory.
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];                          //Subtract the current memory position to make room for the new value.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                                                //Calculate the new change between the actual pressure and the previous measurement.
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];                          //Add the new value to the long term avarage value.
    pressure_rotating_mem_location++;                                                                         //Increase the rotating memory location.
    if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;                              //Start at 0 when the memory location 20 is reached.
    actual_pressure_fast = (float)pressure_total_avarage / 20.0;                                              //Calculate the average pressure of the last 20 pressure readings.

    //To get better results we will use a complementary fillter that can be adjusted by the fast average.
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                       //Calculate the difference between the fast and the slow avarage value.
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
    //If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0;
    actual_pressure = actual_pressure_slow;                                                                   //The actual_pressure is used in the program for altitude calculations.
  }

  if (barometer_counter == 3) {                                                                               //When the barometer counter is 3

    barometer_counter = 0;                                                                                    //Set the barometer counter to 0 for the next measurements.
    //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
    //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
    if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;                       //During manual altitude change the up/down detection is disabled.
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
    pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
    parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.

    if (flight_mode >= 2 && takeoff_detected == 1) {                                                          //If the quadcopter is in altitude mode and flying.
      if (pid_altitude_setpoint == 0)pid_altitude_setpoint = actual_pressure;                                 //If not yet set, set the PID altitude setpoint.
      //When the throttle stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
      //will indicate if the altitude of the quadcopter is changed by the pilot.
      manual_altitude_change = 0;                                                    //Preset the manual_altitude_change variable to 0.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0.
      if (channel_3 > 1600) {                                                        //If the throtttle is increased above 1600us (60%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (channel_3 - 1600) / 3;                                    //To prevent very fast changes in hight limit the function of the throttle.
      }
      if (channel_3 < 1400) {                                                        //If the throtttle is lowered below 1400us (40%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (channel_3 - 1400) / 5;                                    //To prevent very fast changes in hight limit the function of the throttle.
      }

      //Calculate the PID output of the altitude hold.
      pid_altitude_input = actual_pressure;                                          //Set the setpoint (pid_altitude_input) of the PID-controller.
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;                   //Calculate the error between the setpoint and the actual pressure value.

      //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
      //The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
      pid_error_gain_altitude = 0;                                                   //Set the pid_error_gain_altitude to 0.
      if (pid_error_temp > 10 || pid_error_temp < -10) {                             //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
        pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;                 //The positive pid_error_gain_altitude variable is calculated based based on the error.
        if (pid_error_gain_altitude > 3)pid_error_gain_altitude = 3;                 //To prevent extreme P-gains it must be limited to 3.
      }

      //In the following section the I-output is calculated. It's an accumulation of errors over time.
      //The time factor is removed as the program loop runs at 250Hz.
      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
      if (pid_i_mem_altitude > pid_max_altitude)pid_i_mem_altitude = pid_max_altitude;
      else if (pid_i_mem_altitude < pid_max_altitude * -1)pid_i_mem_altitude = pid_max_altitude * -1;
      //In the following line the PID-output is calculated.
      //P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp.
      //I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp (see above).
      //D = pid_d_gain_altitude * parachute_throttle.
      pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
      //To prevent extreme PID-output the output must be limited.
      if (pid_output_altitude > pid_max_altitude)pid_output_altitude = pid_max_altitude;
      else if (pid_output_altitude < pid_max_altitude * -1)pid_output_altitude = pid_max_altitude * -1;
    }

    //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
    else if (flight_mode < 2 && pid_altitude_setpoint != 0) {                        //If the altitude hold mode is not set and the PID altitude setpoint is still set.
      pid_altitude_setpoint = 0;                                                     //Reset the PID altitude setpoint.
      pid_output_altitude = 0;                                                       //Reset the output of the PID controller.
      pid_i_mem_altitude = 0;                                                        //Reset the I-controller.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0 .
      manual_altitude_change = 1;                                                    //Set the manual_altitude_change to 1.
    }
  }
}

void vertical_acceleration_calculations(void) {
  acc_z_average_short_rotating_mem_location++;
  if (acc_z_average_short_rotating_mem_location == 25)acc_z_average_short_rotating_mem_location = 0;

  acc_z_average_short_total -= acc_z_average_short[acc_z_average_short_rotating_mem_location];
  acc_z_average_short[acc_z_average_short_rotating_mem_location] = acc_total_vector;
  acc_z_average_short_total += acc_z_average_short[acc_z_average_short_rotating_mem_location];

  if (acc_z_average_short_rotating_mem_location == 0) {
    acc_z_average_long_rotating_mem_location++;

    if (acc_z_average_long_rotating_mem_location == 50)acc_z_average_long_rotating_mem_location = 0;

    acc_z_average_long_total -= acc_z_average_long[acc_z_average_long_rotating_mem_location];
    acc_z_average_long[acc_z_average_long_rotating_mem_location] = acc_z_average_short_total / 25;
    acc_z_average_long_total += acc_z_average_long[acc_z_average_long_rotating_mem_location];
  }
  acc_z_average_total = acc_z_average_long_total / 50;


  acc_alt_integrated += acc_total_vector - acc_z_average_total;
  if (acc_total_vector - acc_z_average_total < 400 || acc_total_vector - acc_z_average_total > 400) {
    if (acc_z_average_short_total / 25 - acc_z_average_total < 500 && acc_z_average_short_total / 25 - acc_z_average_total > -500)
      if (acc_alt_integrated > 200)acc_alt_integrated -= 200;
      else if (acc_alt_integrated < -200)acc_alt_integrated += 200;
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the starting, stopping and take-off detection is managed.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void start_stop_takeoff(void) {
  if (channel_3 < 1050 && channel_4 < 1050)start = 1;                              //For starting the motors: throttle low and yaw left (step 1).
  if (start == 1 && channel_3 < 1050 && channel_4 > 1450) {                        //When yaw stick is back in the center position start the motors (step 2).
    digitalWrite(2, LOW);
    start = 2;                                                                     //Set the start variable to 2 to indicate that the quadcopter is started.
    throttle = motor_idle_speed;                                                   //Set the base throttle to the motor_idle_speed variable.
    angle_pitch = angle_pitch_acc;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    ground_pressure = actual_pressure;                                             //Register the pressure at ground level for altitude calculations.
    acc_total_vector_at_start = acc_total_vector;                                  //Register the acceleration when the quadcopter is started.
    acc_alt_integrated = 0;                                                        //Reset the integrated acceleration value.
    if (manual_takeoff_throttle > 1400 && manual_takeoff_throttle < 1600) {        //If the manual hover throttle is used and valid (between 1400us and 1600us pulse).
      takeoff_throttle = manual_takeoff_throttle - 1500;                           //Use the manual hover throttle.
      takeoff_detected = 1;                                                        //Set the auto take-off detection to 1, indicated that the quadcopter is flying.
      //Reset the PID controllers for a smooth take-off.
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_output_roll = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_output_pitch = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
      pid_output_yaw = 0;
    }
    else if (manual_takeoff_throttle) {                                            //If the manual hover throttle value is invalid.
      error = 5;                                                                   //Error = 5.
      takeoff_throttle = 0;                                                        //No hover throttle compensation.
      start = 0;                                                                   //Set the start variable to 0 to stop the motors.
    }
  }
  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
    digitalWrite(2, HIGH);
    start = 0;                                                                     //Set the start variable to 0 to disable the motors.
    takeoff_detected = 0;                                                          //Reset the auto take-off detection.
  }

  if (takeoff_detected == 0 && start == 2) {                                       //When the quadcopter is started and no take-off is detected.
    if (channel_3 > 1480 && throttle < 1750) throttle++;                           //When the throttle is half way or higher, increase the throttle.
    if (throttle == 1750)error = 6;                                                //If take-off is not detected when the throttle has reached 1700: error = 6.
    if (channel_3 <= 1480) {                                                       //When the throttle is below the center stick position.
      if (throttle > motor_idle_speed)throttle--;                                  //Lower the throttle to the motor_idle_speed variable.
      //Reset the PID controllers for a smooth take-off.
      else {                                                                       //When the throttle is back at idle speed reset the PID controllers.
        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_output_roll = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_output_pitch = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
        pid_output_yaw = 0;
      }
    }
    if (acc_z_average_short_total / 25 - acc_total_vector_at_start > 800) {        //A take-off is detected when the quadcopter is accelerating.
      takeoff_detected = 1;                                                        //Set the take-off detected variable to 1 to indicate a take-off.
      pid_altitude_setpoint = ground_pressure - press;       // 2.2meter - 22                        //Set the altitude setpoint at groundlevel + approximately 2.2 meters.
      if (throttle > 1400 && throttle < 1700) takeoff_throttle = throttle - throttle_diff;  //If the automated throttle is between 1400 and 1600us during take-off, calculate take-off throttle.
      else {                                                                       //If the automated throttle is not between 1400 and 1600us during take-off.
        takeoff_throttle = 0;                                                      //No take-off throttle is calculated.
        error = 7;                                                                 //Show error 7 on the red LED.
      }
    }
  }
}



#endif //HEADER_H
