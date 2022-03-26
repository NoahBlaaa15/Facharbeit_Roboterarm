//--Erforderliche Bibliotheken--
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <InverseKinematics.h>
#include <AccelStepper.h>

//--WLAN-Netzwerk--
const char* WIFI_NAME =  "Noah";
const char* WIFI_PASS = "testtest01";

//--3D-Position--
int posX = 0; //Min: 0; Max: 1000
int posY = 0;
int posZ = 0;

//--Websocket--
WebSocketsServer webSocket = WebSocketsServer(81);

//--Schrittmotoren--
AccelStepper stepperBase(AccelStepper::FULL4WIRE, 13, 32, 12, 33);
AccelStepper stepperBottom(AccelStepper::FULL4WIRE, 25, 27, 26, 14);
AccelStepper stepperTop(AccelStepper::FULL4WIRE, 19, 5, 18, 17);

//--WebSocket Event--
//Hier wird direkt die Rotation der Schrittmotoren ausgerechnet und auch angewendet
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

    switch(type) {
        case WStype_TEXT: {
            String coords = reinterpret_cast<char *>(payload); //Das Payload aus uint_t Array Buchstaben wird in char Array Buchstaben konvertiert, was wir in einem String für weitere Zugriffe speichern
            Serial.printf("[%u] get Text: %s\n", num, coords.c_str()); //Gibt den String aus, um festzustellen, ob die konvertierung geklappt hat
            if(coords.equalsIgnoreCase("Connected")){
                //Wenn der Arm "Connected" empfängt gehen die Schrittmotoren auf ihre anfangs Position zurück
                //Also am Anfang die Position die sie gerade haben
                stepperBase.moveTo(0);
                stepperBottom.moveTo(map(90, -180, 180, -256, 256));
                stepperTop.moveTo(map(90, 0, 180, 512, -512));
            }else {
                char *charString = strdup(coords.c_str()); //String wird wieder in ein char Array konvertiert damit wir ihn splitten können und die einzelnen X/Y/Z Werte ermitteln können
                char *processedString = strtok(charString, ","); //char Array wird mit strtok in Teil Arrays aufgesplittet, wobei processedString nun den ersten Wert(X) enthält
                posX = map(atoi(processedString), 0, 1000, -175, 175); //Zahlen als Buchstaben werden mit atoi in Integer Zahlen konvertiert, map verändert die Werte so, dass sie für Inverse Kinematics passen
                Serial.printf("%d\n", posX); //Ausgabe zu Testzwecken
                processedString = strtok(NULL, ","); //Geht weiter zum nächsten Teilstück weswegen kein neuer Pointer, sondern NULL angegeben werden muss
                posY = map(atoi(processedString), 0, 1000, -175, 175); //Y Wert
                Serial.printf("%d\n", posY); //Ausgabe zu Testzwecken
                processedString = strtok(NULL, ","); //Letztes Element
                posZ = map(atoi(processedString), 0, 1000, 0, 180); //Z Wert
                Serial.printf("%d\n", posZ); //Ausgabe zu Testzwecken
                free(charString); //Pointer und die Adresse, auf die der Pointer zeigt, können wieder freigegeben werden
                //Wendet die Formeln aus "2.2.1. Inverse Kinematics" an
                double dist = InverseKinematics::getDistance(0,0, posX, posY);
                Serial.println("Distance: " + String( dist ));
                double newPosX = InverseKinematics::getDeltaAngle(posX, posY);
                Serial.println("Ground Angle: " + String( newPosX));
                double angleOne = InverseKinematics::getBetaAngle(dist, posZ, InverseKinematics::lT, InverseKinematics::lB);
                Serial.println("Upper Angle: " + String( angleOne));
                double angleTwo = InverseKinematics::getAlphaAngle(dist, posZ, InverseKinematics::lT, InverseKinematics::lB, angleOne);
                Serial.println("Down Angle: " + String( angleTwo ));
                //AccelStepper Bibliothek setzt die Rotationen/Schritte als Ziel
                stepperBase.moveTo(map(newPosX, -180, 180, -1024, 1024));
                stepperBottom.moveTo(map(angleTwo, -180, 180, -256, 256));
                stepperTop.moveTo(map(angleOne, 0, 180, 512, -512));
            }
            break;
        }
        default:
            break;
    }

}

//--Setup--
void setup(){
    Serial.begin(9600); //Serial Ausgabe zu Testzwecken

    //WLAN im Access Point Mode initialisieren
    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAP(WIFI_NAME, WIFI_PASS);

    //Websocket Kommunikation vorbereiten
    webSocket.onEvent(webSocketEvent);
    webSocket.begin();

    //Schrittmotoren auf die richtigen Werte einstellen die wir bei unseren Messreihen herausgefunden haben
    stepperBase.setMaxSpeed(12);
    stepperBase.setAcceleration(128);
    stepperBottom.setMaxSpeed(12);
    stepperBottom.setAcceleration(128);
    stepperBottom.setCurrentPosition(map(90, -180, 180, -256, 256));
    stepperTop.setMaxSpeed(12);
    stepperTop.setAcceleration(128);
    stepperTop.setCurrentPosition(map(90, 0, 180, 512, -512));

}

//--Loop--
void loop(){
    //Websocket Kommunikation am laufen halten
    webSocket.loop();
    //Die Bibliothek guckt, ob die Schrittmotoren ihre Position erreicht haben und bewegt sie, wenn nötig
    stepperBase.run();
    stepperBottom.run();
    stepperTop.run();
}

