#include "header.h"
#include "WiFi.h"
#include "esp_now.h"
#include "debug_server.h"

long debug_timer;
boolean webpid = true;
boolean debug = false;
boolean webdebug = false;

struct Signal {
    uint16_t throttle = 1000;
    uint16_t yaw = 1500;
    uint16_t pitch = 1500;
    uint16_t roll = 1500;
};
Signal received_signal;

void onDataRecv(const uint8_t *mac_addr, const uint8_t *_data, int data_len) {
    memcpy(&received_signal, _data, sizeof(received_signal));
    channel_1 = received_signal.roll;
    channel_2 = map(received_signal.pitch, 1000, 2000, 2000, 1000);;
    channel_3 = map(received_signal.throttle, 1000, 2000, 1000, 1600);
    channel_4 = received_signal.yaw;
}

void espNowSetup() {
    WiFi.mode(WIFI_MODE_STA);
    while (esp_now_init() != ESP_OK) digitalWrite(2, LOW);
    esp_now_register_recv_cb(onDataRecv);
    delay(100);
}

void setup() {
    pwmSetup();                                                     //start esc pwm output 1000 micros
    if (debug) Serial.begin(115200);
    // pid_p_gain_roll = pid_i_gain_roll = pid_d_gain_roll = pid_p_gain_pitch = pid_i_gain_pitch = pid_d_gain_pitch = pid_p_gain_yaw = pid_i_gain_yaw = pid_d_gain_yaw = 0;
    pinMode(2, OUTPUT);
    pinMode(BATTERY_VOLTAGE_PIN, INPUT);
    espNowSetup();                                                  //start signal receiver
    i2c_setup();                                                    //Start the I2C as master
    gyro_setup();                                                   //Initiallize the gyro and set the correct registers.

    if (!use_manual_calibration) {
        //Create a 5 second delay before calibration.
        for (count_var = 0; count_var < 1250; count_var++) {        //1250 loops of 4 microseconds = 5 seconds
            if (count_var % 125 == 0)                             //Every 125 loops (500ms).
                digitalWrite(2, !digitalRead(2));                   //Change the led status.
            delay(4);                                               //Delay 4 microseconds
        }
        count_var = 0;                                              //Set start back to 0.
    }

    setManualCalibrationData();                                     //Calibrate the gyro offset.
    calibrate_gyro();

    if (debug) {
        Serial.print("manual_acc_pitch_cal_value = ");
        Serial.println(manual_acc_pitch_cal_value);
        Serial.print("manual_acc_roll_cal_value = ");
        Serial.println(manual_acc_roll_cal_value);
        Serial.print("manual_gyro_pitch_cal_value = ");
        Serial.println(manual_gyro_pitch_cal_value);
        Serial.print("manual_gyro_roll_cal_value = ");
        Serial.println(manual_gyro_roll_cal_value);
        Serial.print("manual_gyro_yaw_cal_value = ");
        Serial.print(manual_gyro_yaw_cal_value);
        Serial.print("temperature = ");
        Serial.println(temperature);
    }

    if (webpid) pid_serverSetup();
    if (webdebug) setupWebSocket();
    if (webpid || webdebug) server.begin();

    //Wait until the receiver is active.
//    channel_1 = 1500;channel_2 = 1500;channel_4 = 1500;channel_3 = 1000;
    digitalWrite(2, LOW);
    while (channel_1 < 990 || channel_2 < 990 || channel_4 < 990 || channel_3 < 990 || channel_3 > 1050) yield();
    error = 0;                                                      //Reset the error status to 0.
    digitalWrite(2, HIGH);
    loop_timer = micros();                                          //Set the timer for the first loop.
}

void loop() {
    gyro_signalen();                                                                    //Read the gyro and accelerometer data.
    //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
    gyro_roll_input = (gyro_roll_input * 0.7) + (((float) gyro_roll / 65.5) * 0.3);      //Gyro pid input is deg/sec.
    gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float) gyro_pitch / 65.5) * 0.3);   //Gyro pid input is deg/sec.
    gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float) gyro_yaw / 65.5) * 0.3);         //Gyro pid input is deg/sec.

    //Gyro angle calculations
    //0.0000611 = 1 / (250Hz / 65.5)
    angle_pitch += (float) gyro_pitch * 0.0000611;                                       //Calculate the traveled pitch angle and add this to the angle_pitch variable.
    angle_roll += (float) gyro_roll * 0.0000611;                                         //Calculate the traveled roll angle and add this to the angle_roll variable.

    //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians and not degrees.
    angle_pitch -= angle_roll * sin((float) gyro_yaw * 0.000001066);                     //If the IMU has yawed transfer the roll angle to the pitch angel.
    angle_roll += angle_pitch * sin((float) gyro_yaw * 0.000001066);                     //If the IMU has yawed transfer the pitch angle to the roll angel.

    //Accelerometer angle calculations
    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));       //Calculate the total accelerometer vector.

    if (abs(acc_y) < acc_total_vector)                                                  //Prevent the asin function to produce a NaN.
        angle_pitch_acc = asin((float) acc_y / acc_total_vector) * 57.296;              //Calculate the pitch angle.
    if (abs(acc_x) < acc_total_vector)                                                  //Prevent the asin function to produce a NaN.
        angle_roll_acc = asin((float) acc_x / acc_total_vector) * 57.296;               //Calculate the roll angle.

    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                      //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                         //Correct the drift of the gyro roll angle with the accelerometer roll angle.

    pitch_level_adjust = angle_pitch * 18;                                              //Calculate the pitch angle correction.
    roll_level_adjust  = angle_roll  * 18;                                                //Calculate the roll angle correction.
    // 13 = 37 degrees
    // 14 = 35 degrees
    // 15 = 33 degrees
    // 16 = 31 degrees
    // 17 = 29 degrees
    // 18 = 27 degrees

    if (!auto_level) {                                                                  //If the quadcopter is not in auto-level mode
        pitch_level_adjust = 0;                                                         //Set the pitch angle correction to zero.
        roll_level_adjust = 0;                                                          //Set the roll angle correcion to zero.
    }


    //For starting the motors: throttle low and yaw left (step 1).
    if (channel_3 < 1050 && channel_4 < 1050) start = 1;
    //When yaw stick is back in the center position start the motors (step 2).
    if (start == 1 && channel_3 < 1050 && channel_4 > 1450) {
        digitalWrite(2, LOW);
        start = 2;

        angle_pitch = angle_pitch_acc;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
        angle_roll = angle_roll_acc;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

        //Reset the PID controllers for a bumpless start.
        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
    }
    //Stopping the motors: throttle low and yaw right.
    if (start == 2 && channel_3 < 1050 && channel_4 > 1950) {
        digitalWrite(2, HIGH);
        start = 0;
    }


    //The PID set point in degrees per second is determined by the roll receiver input.
    //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_roll_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if (channel_1 > 1508)pid_roll_setpoint = channel_1 - 1508;
    else if (channel_1 < 1492)pid_roll_setpoint = channel_1 - 1492;

    pid_roll_setpoint -= roll_level_adjust;                                          //Subtract the angle correction from the standardized receiver roll input value.
    pid_roll_setpoint /= 3.0;                                                        //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


    //The PID set point in degrees per second is determined by the pitch receiver input.
    //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_pitch_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if (channel_2 > 1508)pid_pitch_setpoint = channel_2 - 1508;
    else if (channel_2 < 1492)pid_pitch_setpoint = channel_2 - 1492;

    pid_pitch_setpoint -= pitch_level_adjust;                                        //Subtract the angle correction from the standardized receiver pitch input value.
    pid_pitch_setpoint /= 3.0;                                                       //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

    //The PID set point in degrees per second is determined by the yaw receiver input.
    //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    pid_yaw_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if (channel_3 > 1050) { //Do not yaw when turning off the motors.
        if (channel_4 > 1508)pid_yaw_setpoint = (channel_4 - 1508) / 3.0;
        else if (channel_4 < 1492)pid_yaw_setpoint = (channel_4 - 1492) / 3.0;
    }

    calculate_pid();                                                                   //PID inputs are known. So we can calculate the pid output.

    throttle = channel_3;                                                              //We need the throttle signal as a base signal.
    if (start == 2) {                                                                  //The motors are started.
        if (throttle > 1800) throttle = 1800;                                          //We need some room to keep full control at full throttle.
        esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
        esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
        esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
        esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

        if (esc_1 < 1100) esc_1 = 1100;                                                //Keep the motors running.
        if (esc_2 < 1100) esc_2 = 1100;                                                //Keep the motors running.
        if (esc_3 < 1100) esc_3 = 1100;                                                //Keep the motors running.
        if (esc_4 < 1100) esc_4 = 1100;                                                //Keep the motors running.

        if (esc_1 > 2000)esc_1 = 2000;                                                 //Limit the esc-1 pulse to 2000us.
        if (esc_2 > 2000)esc_2 = 2000;                                                 //Limit the esc-2 pulse to 2000us.
        if (esc_3 > 2000)esc_3 = 2000;                                                 //Limit the esc-3 pulse to 2000us.
        if (esc_4 > 2000)esc_4 = 2000;                                                 //Limit the esc-4 pulse to 2000us.
    } else esc_1 = esc_2 = esc_3 = esc_4 = 1000;                                       //If start is not 2 keep a 1000us pulse for esc-1, esc-2, esc-3, esc-4

    esc1_pwm.writeMicroseconds(esc_1);
    esc2_pwm.writeMicroseconds(esc_2);
    esc3_pwm.writeMicroseconds(esc_3);
    esc4_pwm.writeMicroseconds(esc_4);

    if (debug && millis() - debug_timer > 100) {
        debug_timer = millis();
        if (debug_type == 0) {
            //monitor receiver signal data
            Serial.print(start);
            Serial.print(", ");
            Serial.print(channel_1);
            Serial.print(", ");
            Serial.print(channel_2);
            Serial.print(", ");
            Serial.print(channel_3);
            Serial.print(", ");
            Serial.println(channel_4);
        } else if (debug_type == 1) {
            //esc output pulse:-
            Serial.print(start);
            Serial.print(", ");
            Serial.print(esc_1);
            Serial.print(", ");
            Serial.print(esc_2);
            Serial.print(", ");
            Serial.print(esc_3);
            Serial.print(", ");
            Serial.println(esc_4);
        } else if (debug_type == 2) {
            //gyro roll, pith, yaw value:-
            Serial.print(gyro_roll);
            Serial.print(", ");
            Serial.print(gyro_pitch);
            Serial.print(", ");
            Serial.print(gyro_yaw);
            Serial.print(", ");
            Serial.println(temperature);
        }


        //Serial.println("acceleration data:- ");
        //Serial.print(acc_x);
        //Serial.print(", ");
        //Serial.print(acc_y);
        //Serial.print(", ");
        //Serial.println(acc_z);

        //Serial.println("roll level adjust, pitch level adjust:- ");
        //Serial.print(roll_level_adjust);
        //Serial.print(", ");
        //Serial.println(pitch_level_adjust);
    }

    if (webdebug) {
        debug_timer = millis();
        if(wsClient != nullptr && wsClient->canSend()) {
            String s = String(gyro_roll) +", "+ String(gyro_pitch) +", "+ String(gyro_yaw);
            wsClient->text(s);
        }
    }

    if (micros() - loop_timer > 4050) digitalWrite(2, HIGH);                         //Turn on the LED if the loop time exceeds 4050us.
    while (micros() - loop_timer < 4000) yield();                                    //We wait until 4000us are passed.
    loop_timer = micros();                                                           //Set the timer for the next loop.
}
