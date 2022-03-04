#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>

const char* WIFI_NAME =  "Noah";
const char* WIFI_PASS = "testtest01";

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

bool toggle = 1;
bool toggleLast = 0;

int X = 0;
int Y = 0;

int posX = 500; //Min: 0; Max: 1000
int posY = 500;
int posZ = 500;

int changeJoyX();
int changeJoyY();
int sanitizeInput(int sanInt, int oldInt);
void convertJoyToPos();

void setup(void){
    Serial.begin(9600);
    u8g2.begin();
    WiFi.begin(WIFI_NAME, WIFI_PASS);

    int i = 0;
    while (WiFi.status() != WL_CONNECTED) { // Wait for the Wi-Fi to connect
        u8g2.setFont(u8g2_font_finderskeepers_tf);	// choose a suitable font
        u8g2.clearBuffer();
        u8g2.drawBox(0,0, 128, 9);
        u8g2.setDrawColor(0);
        u8g2.drawStr(40,8, "Connecting...");
        u8g2.setDrawColor(1);
        u8g2.sendBuffer();
        delay(500);
    }

    /*webSocket.begin("192.168.0.123", 81, "/");
    webSocket.onEvent(webSocketEvent);
    webSocket.setAuthorization(WIFI_NAME, WIFI_PASS);
    webSocket.setReconnectInterval(5000);*/

    pinMode(13, INPUT);
    X = map(changeJoyX(), 13, 236, 0, 50);
    Y =  map(changeJoyY(), 13, 236, 0, 50);
}

void loop(void){
    X = sanitizeInput(changeJoyX(), X);
    delay(10);
    Y = sanitizeInput(changeJoyY(), Y);
    convertJoyToPos();

    if(toggle){
        u8g2.clearBuffer();
        u8g2.drawBox(0,0, 128, 9);
        u8g2.setDrawColor(0);
        u8g2.drawStr(40,8, "Select X/Y");
        u8g2.drawStr(0, 8, String(X).c_str());
        u8g2.drawStr(110, 8, String(Y).c_str());
        u8g2.setDrawColor(1);
        u8g2.drawFrame(37,10,54,54);
        u8g2.drawStr(map(posX, 0, 1000, 36, 86),map(posY, 0, 1000, 65, 14), "+");
        u8g2.drawStr(0,26, String(posX).c_str());
        u8g2.drawStr(0,36, String(posY).c_str());
        u8g2.drawStr(108,26, String(posZ).c_str());
        u8g2.drawStr(0,52, "Con");
        u8g2.drawStr(4,60, "v");
        u8g2.drawStr(112,52, "Res");
        u8g2.drawStr(118,60, "v");
        u8g2.sendBuffer();
    }else{
        u8g2.clearBuffer();
        u8g2.drawBox(0,0, 128, 9);
        u8g2.setDrawColor(0);
        u8g2.drawStr(40,8, "Select Z");
        u8g2.drawStr(0, 8, String(X).c_str());
        u8g2.drawStr(110, 8, String(Y).c_str());
        u8g2.setDrawColor(1);
        u8g2.drawFrame(37,10,54,54);
        u8g2.drawStr(61,map(posZ, 0, 1000, 65, 14), "+");
        u8g2.drawStr(0,26, String(posX).c_str());
        u8g2.drawStr(0,36, String(posY).c_str());
        u8g2.drawStr(108,26, String(posZ).c_str());
        u8g2.drawStr(0,52, "Con");
        u8g2.drawStr(4,60, "v");
        u8g2.drawStr(112,52, "Res");
        u8g2.drawStr(118,60, "v");
        u8g2.sendBuffer();
    }

    //Joystick Button
    if(digitalRead(13) == 0){
        if(toggleLast == 0){
            toggle = !toggle;
        }
        toggleLast = 1;
    }else{
        toggleLast = 0;
    }

    //ConfirmButton

    //ResetButton

    //Websocket Connection
    //webSocket.loop();

    delay(10);
}

void convertJoyToPos(){
    if(toggle) {
        posX = (posX + (X - 21)) > 1000 ? 1000 : (posX + (X - 21)) < 0 ? 0 : (posX + (X - 21) < posX - 1 ||
                                                                              posX + (X - 21) > posX + 1) ? posX +
                                                                                                            (X - 21)
                                                                                                          : posX; //Von 0 zu 50 zu -2.5 bis +2.5
        posY = (posY + (Y - 21)) > 1000 ? 1000 : (posY + (Y - 21)) < 0 ? 0 : (posY + (Y - 21) < posY - 1 ||
                                                                              posY + (Y - 21) > posY + 1) ? posY +
                                                                                                            (Y - 21): posY;
    }else{
        posZ = (posZ + (Y - 21)) > 1000 ? 1000 : (posZ + (Y - 21)) < 0 ? 0 : (posZ + (Y - 21) < posZ - 1 ||
                                                                              posZ + (Y - 21) > posZ + 1) ? posZ +
                                                                                                            (Y - 21): posZ;
    }
}


int sanitizeInput(int sanInt, int oldInt){
    int newInt = map(sanInt, 13, 235, 0, 50);
    if(newInt > 50){
        newInt = 50;
    }
    if(newInt < 0){
        newInt = 0;
    }

    if(newInt < oldInt-1 || newInt > oldInt+1){ oldInt = newInt;}

    return oldInt;
}

int changeJoyX(){
    pinMode(14,INPUT);
    pinMode(12,OUTPUT);
    digitalWrite(12,LOW);
    return analogRead(A0);
}

int changeJoyY(){
    pinMode(12,INPUT);
    pinMode(14,OUTPUT);
    digitalWrite(14,LOW);
    return analogRead(A0);
}
