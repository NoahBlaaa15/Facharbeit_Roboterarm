//--Erforderliche Bibliotheken--
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>

//--WLAN-Netzwerk--
const char* WIFI_NAME =  "Noah";
const char* WIFI_PASS = "testtest01";

//--Oled-Display--
//SSD1306 Display über I2C mit den Hardware standard Ports für I2C
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

//--Websocket--
WebSocketsClient webSocket;
String msg = "";

//--Button Variablen--
//Werden als Button Debounce benutzt
bool joystickButtonLast = 0;
bool confirmButtonLast = 0;
bool resetButtonLast = 0;

//--Positions Variablen--
//Geschw. des Cursors
int X = 0;
int Y = 0;

//Position des Cursors
int posX = 500; //Min: 0; Max: 1000
int posY = 500;
int posZ = 500;

//--Automat Variablen--
int zustand = 0;
/* 1: Confirm press
 * 2: Confirm go
 * 3: Joystick move
 * 4: Joystick Button
 * 5: Reset press
 * 6: Reset go*/
int eingabe = 3;
/* 1: Pos send
 * 2: Pos reset
 * 3: show x/y
 * 4: change x/y
 * 5: show z
 * 6: change z */
int ausgabe = 3;

//--Funktionen--
//Hier deklariert da die richtige Deklaration erst unter den Stellen stattfindet, wo sie gebraucht werden
int changeJoyX();
int changeJoyY();
int sanitizeInput(int sanInt, int oldInt);
int g(int zustand, int eingabe);
int f(int zustand, int eingabe);

//--WebSocket Event--
//Sobald eine Verbindung aufgebaut ist, wird dem Arm mitgeteilt das der Controller verbunden ist
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_CONNECTED: {
            Serial.printf("[WSc] Connected to url: %s\n", payload);
            webSocket.sendTXT("Connected");
        }
            break;
        default:
            break;
    }

}


//--Setup--
void setup(void){
    Serial.begin(9600); //Serial Ausgabe zu Testzwecken
    u8g2.begin(); //Display initialisieren
    WiFi.begin(WIFI_NAME, WIFI_PASS); //WLAN im Station Mode initialisieren

    //Warten bis wir über WLAN mit dem Arm verbunden sind
    while (WiFi.status() != WL_CONNECTED) {
        //Display zeigt Connecting... an wenn noch nicht verbunden
        u8g2.setFont(u8g2_font_finderskeepers_tf); //Textart
        u8g2.clearBuffer(); //Display leer
        u8g2.drawBox(0,0, 128, 9); //Weiße Box
        u8g2.setDrawColor(0); //Farbe auf Schwarz setzen
        u8g2.drawStr(40,8, "Connecting..."); //Text in die Box schreiben
        Serial.println("Connection"); //Ausgabe zu Testzwecken
        u8g2.setDrawColor(1); //Farbe wieder auf Weiß setzen
        u8g2.sendBuffer(); //Frame an das Display senden
        delay(500);
    }

    //Websocket Kommunikation vorbereiten
    webSocket.begin(WiFi.gatewayIP().toString(), 81, "/");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);

    pinMode(13, INPUT); //Joystick Button initialisieren
    pinMode(0, INPUT); //Confirm Button initialisieren
    pinMode(2, INPUT); //Reset Button initialisieren
    X = map(changeJoyX(), 13, 236, 0, 50); //Joystick werte initialisieren
    Y =  map(changeJoyY(), 13, 236, 0, 50);
}

//--Loop--
void loop(void){
    //Zu Testzwecken Automat-Daten ausgeben
    Serial.println(zustand);
    Serial.println(eingabe);
    Serial.println(ausgabe);

    //Joystick Button Eingabe
    if(digitalRead(13) == 0){
        if(joystickButtonLast == 0){
            eingabe = 4;
        }
        joystickButtonLast = 1;
    }else if(joystickButtonLast == 1){
        joystickButtonLast = 0;
    }

    //ConfirmButton Eingabe
    if(digitalRead(0) == 0){
        eingabe = 1;
        confirmButtonLast = 1;
    }else if(confirmButtonLast == 1){
        eingabe = 2;
        confirmButtonLast = 0;
    }

    //ResetButton Eingabe
    if(digitalRead(2) == 0){
        eingabe = 5;
        resetButtonLast = 1;
    }else if(resetButtonLast == 1){
        eingabe = 6;
        resetButtonLast = 0;
    }

    //Joystick Eingabe
    X = sanitizeInput(changeJoyX(), X);
    Y = sanitizeInput(changeJoyY(), Y);
    if((X < 19 || X > 22) || (Y < 19 || Y > 22)){
        eingabe = 3;
    }


    //Ausgabe ausführen
    ausgabe = g(zustand, eingabe);
    switch (ausgabe) {
        case 1:

            //Werte werden über den WebSocket versendet
            msg = (String(posX) + "," + String(posY) + "," + String(posZ));
            webSocket.sendTXT(msg);

            break;
        case 2:

            //Cursor in die Mitte setzen
            posX = 500;
            posY = 500;
            posZ = 500;
            //Wenn der Arm "Connected" empfängt gehen die Schrittmotoren auf ihre anfangs Position zurück
            webSocket.sendTXT("Connected");

            break;
        case 3:

            //Auf dem Display wird die X/Y Auswahl dargestellt
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

            break;
        case 4:

            //Cursor X/Y Position wird je nach aktueller Joystick Position verändert

            posX = (posX + (X - 21)) > 1000 ? 1000 : (posX + (X - 21)) < 0 ? 0 : (posX + (X - 21) < posX - 1 ||
                                                                                  posX + (X - 21) > posX + 1) ? posX +
                                                                                                                (X - 21)
                                                                                                              : posX; //Von 0 zu 50 zu -2.5 bis +2.5
            posY = (posY + (Y - 21)) > 1000 ? 1000 : (posY + (Y - 21)) < 0 ? 0 : (posY + (Y - 21) < posY - 1 ||
                                                                                  posY + (Y - 21) > posY + 1) ? posY +
                                                                                                                (Y - 21): posY;

            //Aus Automaten Modell sicht sollte sich das Display hier nicht aktualisieren, tut man dies nicht hat man jedoch nur eine sehr geringe Bildwiederholrate von 1/4 FPS
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

            break;
        case 5:

            //Auf dem Display wird die Z Auswahl angezeigt
            u8g2.clearBuffer();
            u8g2.drawBox(0,0, 128, 9);
            u8g2.setDrawColor(0);
            u8g2.drawStr(40,8, "Select Z");
            u8g2.drawStr(0, 8, String(X).c_str());
            u8g2.drawStr(110, 8, String(Y).c_str());
            u8g2.setDrawColor(1);
            u8g2.drawFrame(58,10,12,54);
            u8g2.drawStr(61,map(posZ, 0, 1000, 65, 14), "+");
            u8g2.drawStr(0,26, String(posX).c_str());
            u8g2.drawStr(0,36, String(posY).c_str());
            u8g2.drawStr(108,26, String(posZ).c_str());
            u8g2.drawStr(0,52, "Con");
            u8g2.drawStr(4,60, "v");
            u8g2.drawStr(112,52, "Res");
            u8g2.drawStr(118,60, "v");
            u8g2.sendBuffer();

            break;
        case 6:

            //Cursor Z Position wird je nach aktueller Joystick Position verändert

            posZ = (posZ + (Y - 21)) > 1000 ? 1000 : (posZ + (Y - 21)) < 0 ? 0 : (posZ + (Y - 21) < posZ - 1 ||
                                                                                  posZ + (Y - 21) > posZ + 1) ? posZ +
                                                                                                                (Y - 21): posZ;


            //Aus Automaten Modell sicht sollte sich das Display hier nicht aktualisieren, tut man dies nicht hat man jedoch nur eine sehr geringe Bildwiederholrate von 1/4 FPS
            u8g2.clearBuffer();
            u8g2.drawBox(0,0, 128, 9);
            u8g2.setDrawColor(0);
            u8g2.drawStr(40,8, "Select Z");
            u8g2.drawStr(0, 8, String(X).c_str());
            u8g2.drawStr(110, 8, String(Y).c_str());
            u8g2.setDrawColor(1);
            u8g2.drawFrame(58,10,12,54);
            u8g2.drawStr(61,map(posZ, 0, 1000, 65, 14), "+");
            u8g2.drawStr(0,26, String(posX).c_str());
            u8g2.drawStr(0,36, String(posY).c_str());
            u8g2.drawStr(108,26, String(posZ).c_str());
            u8g2.drawStr(0,52, "Con");
            u8g2.drawStr(4,60, "v");
            u8g2.drawStr(112,52, "Res");
            u8g2.drawStr(118,60, "v");
            u8g2.sendBuffer();

            break;
        default:
            break;
    }

    zustand = f(zustand,eingabe);

    //Websocket Kommunikation am laufen halten
    webSocket.loop();

    delay(10);
}

//--Ausgabefunktion--
int g(int z, int e){
    switch (z) {
        case 0:
            switch (e) {
                case 1: return 1;
                case 3: return 4;
                case 4: return 5;
                case 5: return 2;
                default: return 3;
            }
            break;
        case 1:
            switch (e) {
                case 1: return 1;
                case 3: return 6;
                case 4: return 3;
                case 5: return 2;
                default: return 5;
            }
            break;
        case 2:
            switch (e) {
                case 2: return 3;
                default: return -1;
            }
            break;
        case 3:
            switch (e) {
                case 6: return 5;
                default: return -1;
            }
            break;
        case 4:
            switch (e) {
                case 6: return 3;
                default: return -1;
            }
            break;
        case 5:
            switch (e) {
                case 2: return 5;
                default: return -1;
            }
            break;
    }
    return -1;
}

//--Überführungsfunktion--
int f(int z, int e){
    switch (z) {
        case 0:
            switch (e) {
                case 1: return 2;
                case 3: return 0;
                case 4: return 1;
                case 5: return 4;
                default: return z;
            }
            break;
        case 1:
            switch (e) {
                case 1: return 5;
                case 3: return 1;
                case 4: return 0;
                case 5: return 3;
                default: return z;
            }
            break;
        case 2:
            switch (e) {
                case 2: return 0;
                default: return z;
            }
            break;
        case 3:
            switch (e) {
                case 6: return 1;
                default: return z;
            }
            break;
        case 4:
            switch (e) {
                case 6: return 0;
                default: return z;
            }
            break;
        case 5:
            switch (e) {
                case 2: return 1;
                default: return z;
            }
            break;
    }
    return z;
}

//--Joystick zu Geschwindigkeit konvertieren--
//angepasst nach den Werten die wir bei unserer Messreihe herausgefunden haben
int sanitizeInput(int sanInt, int oldInt){
    int newInt = map(sanInt, 13, 235, 0, 50);
    if(newInt > 50){
        newInt = 50;
    }
    if(newInt < 0){
        newInt = 0;
    }

    if(newInt < oldInt-2 || newInt > oldInt+2){ oldInt = newInt;}

    return oldInt;
}

//--x-Achse wird angeschaltet an A0--
int changeJoyX(){
    pinMode(14,INPUT);
    pinMode(12,OUTPUT);
    digitalWrite(12,LOW);
    return analogRead(A0);
}

//--y-Achse wird angeschaltet an A0--
int changeJoyY(){
    pinMode(12,INPUT);
    pinMode(14,OUTPUT);
    digitalWrite(14,LOW);
    return analogRead(A0);
}
