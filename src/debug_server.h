#ifndef ESP32FLIGHT_DEBUG_SERVER_H
#define ESP32FLIGHT_DEBUG_SERVER_H

#include "ESPAsyncWebServer.h"
#include "ArduinoJson.h"
#include "htmlpage.h"
#include "header.h"

int debug_type = 1;     //0:receiver  1:esc_output  2:gyro

JsonDocument json;
String jsonString;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
AsyncWebSocketClient *wsClient;

void pid_serverSetup() {
    WiFi.softAP("pidserver", "0123456789");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");

    server.on("/", HTTP_ANY, [](AsyncWebServerRequest *request) {
        if (request->hasArg("rpP") && request->hasArg("rpI") && request->hasArg("rpD") &&
            request->hasArg("yP") && request->hasArg("yI"), request->hasArg("yD")) {

            String rpP, rpI, rpD, yP, yI, yD;
            rpP = request->arg("rpP");
            rpI = request->arg("rpI");
            rpD = request->arg("rpD");

            yP = request->arg("yP");
            yI = request->arg("yI");
            yD = request->arg("yD");

            pid_p_gain_roll = pid_p_gain_pitch = rpP.toFloat();
            pid_i_gain_roll = pid_i_gain_pitch = rpI.toFloat();
            pid_d_gain_roll = pid_d_gain_pitch = rpD.toFloat();

            pid_p_gain_yaw = yP.toFloat();
            pid_i_gain_yaw = yI.toFloat();
            pid_d_gain_yaw = yD.toFloat();
        }
        return request->send(200, "text/html", page);
    });
    server.on("/pid", HTTP_GET, [](AsyncWebServerRequest *request) {
        json.clear(); jsonString = "";
        json["rpP"] = String(pid_p_gain_roll, 3);
        json["rpI"] = String(pid_i_gain_roll, 4);
        json["rpD"] = String(pid_d_gain_roll, 3);
        json["yP"] = String(pid_p_gain_yaw, 3);
        json["yI"] = String(pid_i_gain_yaw, 4);
        json["yD"] = String(pid_d_gain_yaw, 3);
        serializeJson(json, jsonString);
        return request->send(200, "text/plain", jsonString);
    });

    server.on("/calibration_data", HTTP_GET, [](AsyncWebServerRequest *request) {
        String str = "<div>accPitch: "+ String(manual_acc_pitch_cal_value) +"  accRoll: "+ String(manual_acc_roll_cal_value) +"</div>";
        str += "<div>gyroPitch: "+ String(manual_gyro_pitch_cal_value) +"  gyroRoll: "+ String(manual_gyro_roll_cal_value) +"  gyroYaw: "+ String(manual_gyro_yaw_cal_value) +"</div>";
        str += "<div>temperature: "+ String(temperature) +"</div>";
        return request->send(200, "text/html", str);
    });
    server.on("/debug", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("type")) {
            String str = request->getParam("type")->value();
            if (str.equals("0")) debug_type = 0;
            if (str.equals("1")) debug_type = 1;
            if (str.equals("2")) debug_type = 2;
        }
        request->send(200, "text/plain", String(debug_type));
    });

    server.on("/level", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("set")) {
            String str = request->getParam("set")->value();
            auto_level = str.equals("1") ? true : false;
        }
        request->send(200, "text/plain", String(auto_level));
    });
    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not found");
    });
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data,size_t len) {
    if (type == WS_EVT_CONNECT) wsClient = client;
    else if (type == WS_EVT_DISCONNECT) wsClient = nullptr;
}

void setupWebSocket() { 
    WiFi.softAP("flightdebug", "0123456789");
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
}

#endif //ESP32FLIGHT_DEBUG_SERVER_H
