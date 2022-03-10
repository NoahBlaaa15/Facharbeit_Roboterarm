#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <InverseKinematics.h>
#include <AccelStepper.h>

#define IN1 14
#define IN2 27
#define IN3 26
#define IN4 25

#define standardG 0
#define standardB 20
#define standardT 20

const char* WIFI_NAME =  "Noah";
const char* WIFI_PASS = "testtest01";

int posX = 0;
int posY = 0;
int posZ = 0;

double currentPositionG = standardG; //G = Ground
double currentPositionB = standardB; // B = Bottom
double currentPositionT = standardT; //T = Top

WebSocketsServer webSocket = WebSocketsServer(81);

AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_DISCONNECTED:
            //Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
        {
            /*IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);*/
        }
            break;
        case WStype_TEXT: {
            String coords = reinterpret_cast<char *>(payload);
            //Serial.printf("[%u] get Text: %s\n", num, coords.c_str());
            if(coords.equalsIgnoreCase("Reset")){
                //Go to normal position so after restart we know where the motors are
            }else {
                char *charString = strdup(coords.c_str());
                char *processedString = strtok(charString, ",");
                posX = map(atoi(processedString), 0, 1000, -175, 175);
                //Serial.printf("%d\n", posX);
                processedString = strtok(NULL, ",");
                posY = map(atoi(processedString), 0, 1000, -175, 175);
                //Serial.printf("%d\n", posY);
                processedString = strtok(NULL, ",");
                posZ = map(atoi(processedString), 0, 1000, 0, 180);
                //Serial.printf("%d\n", posZ);
                free(charString);
                double dist = InverseKinematics::getDistance(0,0, posX, posY);
                //Serial.println("Distance: " + String( dist ));
                double newPosX = InverseKinematics::getDeltaAngle(posX, posY);
                //Serial.println("Ground Angle: " + String( newPosX));
                double angleOne = InverseKinematics::getBetaAngle(dist, posZ, InverseKinematics::lT, InverseKinematics::lB);
                //Serial.println("Upper Angle: " + String( angleOne));
                double angleTwo = InverseKinematics::getAlphaAngle(dist, posZ, InverseKinematics::lT, InverseKinematics::lB, angleOne);
                //Serial.println("Down Angle: " + String( angleTwo ));
                Serial.println(String(angleOne) + "," + String(angleTwo));
                stepper.setMaxSpeed(64);
                stepper.setAcceleration(120);
                stepper.moveTo(map(newPosX, 0, 360, 0, 4096));
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
}


void loop(){
    webSocket.loop();
    stepper.run();
    delay(10);
}

