#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <InverseKinematics.h>
#include <AccelStepper.h>

#define standardG 0
#define standardB 90
#define standardT 90

const char* WIFI_NAME =  "Noah";
const char* WIFI_PASS = "testtest01";

int posX = 0;
int posY = 0;
int posZ = 0;

double currentPositionG = standardG; //G = Ground
double currentPositionB = standardB; // B = Bottom
double currentPositionT = standardT; //T = Top

WebSocketsServer webSocket = WebSocketsServer(81);

AccelStepper stepperBase(AccelStepper::FULL4WIRE, 33, 32, 35, 34);
AccelStepper stepperBottom(AccelStepper::FULL4WIRE, 14, 27, 26, 25);
AccelStepper stepperTop(AccelStepper::FULL4WIRE, 17, 5, 18, 19);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_TEXT: {
            String coords = reinterpret_cast<char *>(payload);
            Serial.printf("[%u] get Text: %s\n", num, coords.c_str());
            if(coords.equalsIgnoreCase("Connected")){
                //Go to normal position so after restart we know where the motors are
                //current - standard = steps to make to reach start position
                stepperBase.moveTo(0);
                stepperBottom.moveTo(0);
                stepperTop.moveTo(0);
            }else {
                char *charString = strdup(coords.c_str());
                char *processedString = strtok(charString, ",");
                posX = map(atoi(processedString), 0, 1000, -175, 175);
                Serial.printf("%d\n", posX);
                processedString = strtok(NULL, ",");
                posY = map(atoi(processedString), 0, 1000, -175, 175);
                Serial.printf("%d\n", posY);
                processedString = strtok(NULL, ",");
                posZ = map(atoi(processedString), 0, 1000, 0, 180);
                Serial.printf("%d\n", posZ);
                free(charString);
                double dist = InverseKinematics::getDistance(0,0, posX, posY);
                Serial.println("Distance: " + String( dist ));
                double newPosX = InverseKinematics::getDeltaAngle(posX, posY);
                Serial.println("Ground Angle: " + String( newPosX));
                double angleOne = InverseKinematics::getBetaAngle(dist, posZ, InverseKinematics::lT, InverseKinematics::lB);
                Serial.println("Upper Angle: " + String( angleOne));
                double angleTwo = InverseKinematics::getAlphaAngle(dist, posZ, InverseKinematics::lT, InverseKinematics::lB, angleOne);
                Serial.println("Down Angle: " + String( angleTwo ));
                Serial.println(String(newPosX - currentPositionG));
                Serial.println(String(angleOne - currentPositionT));
                Serial.println(String(angleTwo - currentPositionB));
                stepperBase.moveTo(map(newPosX - currentPositionG, -360, 360, -1024, 1024));
                stepperBottom.moveTo(map(angleTwo - currentPositionB, -360, 360, -1024, 1024));
                stepperTop.moveTo(map(angleOne - currentPositionT, -360, 360, -1024, 1024));
                currentPositionG = newPosX;
                currentPositionB = angleTwo;
                currentPositionT = angleOne;
            }
            break;
        }
        default:
            break;
    }

}

void setup(){
    Serial.begin(9600);

    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAP(WIFI_NAME, WIFI_PASS);

    webSocket.onEvent(webSocketEvent);
    webSocket.begin();

    stepperBase.setMaxSpeed(16);
    stepperBase.setAcceleration(128);
    stepperBottom.setMaxSpeed(16);
    stepperBottom.setAcceleration(128);
    stepperTop.setMaxSpeed(16);
    stepperTop.setAcceleration(128);
}


void loop(){
    webSocket.loop();
    stepperBase.run();
    stepperBottom.run();
    stepperTop.run();
    delay(10);
}

