#include "ALIMENTATION.h"
#include <Arduino.h>

float pinControleTransistor = 27, pinTension = 36, tension, tensionSeuil = 6.0, MaxVoltage = 8.2;
float pinLedRouge = 2, pinLedVert = 4;
// IHM
int BPR = 5, valBPR = 0;

void setAlim(int x, int y, int z, int w, int p)
{
    Serial.printf("INIT PIN ALIM");
    Serial.println();
    pinLedRouge = x;
    pinLedVert = y;
    pinControleTransistor = z;
    BPR = w;
    pinTension = p;

    pinMode(BPR, INPUT_PULLUP);
    pinMode(pinControleTransistor, OUTPUT);
    pinMode(pinTension, INPUT);
    pinMode(pinLedRouge, OUTPUT);
    pinMode(pinLedVert, OUTPUT);
    digitalWrite(pinLedRouge, HIGH);
    digitalWrite(pinLedVert, HIGH);

    delay(1000);
    Serial.printf("Ouverture du relais");
    Serial.println();
    openRelais();
}

void openRelais()
{

    digitalWrite(pinControleTransistor, HIGH);
    digitalWrite(pinLedRouge, LOW);
    digitalWrite(pinLedVert, HIGH);
}
void closeRelais()
{
    digitalWrite(pinControleTransistor, LOW);
    digitalWrite(pinLedRouge, HIGH);
    digitalWrite(pinLedVert, LOW);
}
void lectureTension(int x)
{
    tension = analogReadMilliVolts(pinTension) * 10.0 / 3.2 / 1000.0;
    if (x)
    {
        Serial.printf("%4.4f\n", tension);
    }
    // tension = tension  / 3.6 + 0;
    // if (tension < 9)
    // {
    //     tension = tension + 0;
    // }
    // if (tension >= 9)
    // {
    //     tension = tension - 0.5;
    // }

    // Serial.printf("%4.4f\n", tension);
    // delay(1000);
}
void santeAlim()
{
    lectureTension(0);
    valBPR = digitalRead(BPR);
    // Serial.println(valBPR);
    if (valBPR == 0)
    {
        closeRelais();
    }
    if (tension < tensionSeuil)
    {
        closeRelais();
    }
}