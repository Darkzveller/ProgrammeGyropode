#include <Arduino.h>
#include <MOTEUR.h>
#include "ALIMENTATION.h"
// #include "BluetoothTache.h"

#include <BluetoothSerial.h>
#include <String.h>
#include <Wire.h>

// Bibliotheque MPU 6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

BluetoothSerial SerialBT;

// Declaration de class
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
// Déclaration des variable en globale et en float pour ne pas avoir de problème
float thetaG, thetaGF;
float thetaW, thetaWF;
float theta, somme_non_filtrer;

float Co=3.245;
float Kd=0.0845;
float ErreurPosAng;
float cons;
float consE = -0.0328;
float commande;
float commandeTraiter;
float commandeComp = 0.1566;
float vitesse;
float Kalpha = 4095.0;
int PWM;
int valeur;
int etat = 0;

int FlagCalcul = 0;
float Te = 10;   // période d'échantillonage en ms Mettre 10ms
float Tau = 175; // constante de temps du filtre en ms 1750ms
// coefficient du filtre
float A, B;

void controle(void *parameters)
{
  // Demander a M.CLAMAINS toute information complémentaire sur la ligne suivante
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    santeAlim();
    // Acquissition des toutes les mesures
    mpu.getEvent(&a, &g, &temp);

    // Calcul de théta a l'aide de l'accélération mesurer
    thetaG = (-1) * atan2(a.acceleration.y, a.acceleration.x); // Permet de calculer l'angle Théta G avec un angle dans la valeur est entier relatif
    // Calcul du Théta filtrer
    thetaGF = A * thetaG + B * thetaGF;

    // Calcul de théta a l'aide de la valeur mesurer par le gyroscope
    thetaW = g.gyro.z * Tau * 1e-3;
    // Calcul du thétaW filtrer
    thetaWF = A * thetaW + B * thetaWF;

    // Calcule de la somme permettant d'avoir un passe bande filtrer
    theta = thetaGF + thetaWF;

    ErreurPosAng = (cons + consE) - theta;
    commande = ErreurPosAng * Co + Kd * (-1) * g.gyro.z;
    if (commande < 0)
    {
      commandeTraiter = commande - commandeComp;
    }
    if (commande > 0)
    {
      commandeTraiter = commande + commandeComp;
    }
    if (commande == 0)
    {
      Controle_Moteur_Droit(0);
      Controle_Moteur_Gauche(0);
    }

    if (commande == 0)
    {
      Controle_Moteur_Droit(0);
      Controle_Moteur_Gauche(0);
    }
    if (commandeTraiter < -0.45)
    {
      commandeTraiter = -0.45;
    }
    else
    {
      if (commandeTraiter > 0.45)
      {
        commandeTraiter = 0.45;
      }
      // Serial.printf("avance\n");
    }

    vitesse = 0.5 + commandeTraiter;
    PWM = vitesse * Kalpha;

    Controle_Moteur_Droit(PWM);
    Controle_Moteur_Gauche(PWM);
    // Mise a 1 du flag permettant ainsi dans le void loop de pouvoir effectuer l'affichage
    FlagCalcul = 1;
    // Demander a M.CLAMAINS toute information complémentaire sur la ligne suivante
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}
void traitementBlutooth();

void setup()
{
  setAlim(2, 4, 27, 5, 36);

  Init_Pin_Moteur_Droit(19, 18);
  Init_Pin_Moteur_Gauche(16, 17);
  Config_Esp_Cannaux(19500, 12);

  // Permet la communication avec le terminal concu par M.GUINAND ou celui du PC
  Serial.begin(115200);
  // Test la communication avec le MPU 6050
  if (!mpu.begin())
  {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }
  Serial.println("Found a MPU-6050 sensor");
  // On augmente la précision de la mesure effectuer avec le gyroscope
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);

  // Demander a M.CLAMAINS toute information complémentaire sur la ligne suivante
  xTaskCreate(
      controle,   // nom de la fonction
      "controle", // nom de la tache que nous venons de vréer
      10000,      // taille de la pile en octet
      NULL,       // parametre
      1,          // tres haut niveau de priorite
      NULL        // descripteur
  );

  delay(2000);
  // setBlutTache();
  SerialBT.begin("JSP NAME");

  // calcul coeff filtre
  A = 1 / (1 + Tau / Te);
  B = Tau / Te * A;
}

void loop()
{
  if (FlagCalcul == 1) // Affichage des données
  {
    traitementBlutooth();

    // On affiche sur le terminal les informations souhaité
    // Serial.printf("%4.4f %4.4f %4.4f %4.4f \n", theta, commande, vitesse);
    // Mise a 0 du flag
    FlagCalcul = 0;
  }
}

void reception(char ch)
{

  static int i = 0;
  static String chaine = "";
  String commande;
  String valeur;
  int index, length;

  if ((ch == 13) or (ch == 10))
  {
    index = chaine.indexOf(' ');
    length = chaine.length();
    if (index == -1)
    {
      commande = chaine;
      valeur = "";
    }
    else
    {
      commande = chaine.substring(0, index);
      valeur = chaine.substring(index + 1, length);
    }

    if (commande == "Tau")
    {
      Tau = valeur.toFloat();
      // calcul coeff filtre
      A = 1 / (1 + Tau / Te);
      B = Tau / Te * A;
    }
    if (commande == "Te")
    {
      Te = valeur.toInt();
    }
    if (commande == "C")
    {
      Co = valeur.toFloat();
    }
    if (commande == "Kd")
    {
      Kd = valeur.toFloat();
    }
    if (commande == "consE")
    {
      consE = valeur.toFloat();
    }
    chaine = "";
  }
  else
  {
    chaine += ch;
  }
}

void serialEvent()
{
  while (Serial.available() > 0) // tant qu'il y a des caractères à lire
  {
    reception(Serial.read());
  }
}
// Serial.printf("%4.2f %4.2f %4.2f %4.2f \n", thetaGF,thetaG, thetaWF, somme_filtrer);
void traitementBlutooth()
{
  if (SerialBT.available())
  {

    int caractere = SerialBT.read();
    // Serial.println(SerialBT.read());
      if (caractere == 'D')
      {
        Serial.printf("Cesam OUVRE toi");
        Serial.println();
        openRelais();
      }
      if (caractere == 'd')
      {
        Serial.printf("Cesam FERME toi");
        Serial.println();
        closeRelais();
      }
      if (caractere == '1')
      {
        Serial.printf("Up\n");
      }
      if (caractere == '2')
      {
        Serial.printf("Droit\n");
      }
      if (caractere == '3')
      {
        Serial.printf("Bas\n");
      }
      if (caractere == '6')
      {
        Serial.printf("Gauche\n");
      }
    switch (etat)
    {
    case 0:
      if (caractere == 'A') // Est ce que j'ai recu l'information
      {

        etat = 1;
        valeur = 0;
      }

      break;
    case 1: //
      if ((caractere >= '0') && (caractere <= '9'))
      {
        valeur = valeur * 10 + caractere - '0';
      }
      else if (caractere == 'Z')
      {
        Serial.printf("\nValeur %d\n", valeur);

        etat = 0;
      }
      else
      {
        Serial.printf("Message non compris\n");

        etat = 0;
      }
      break;
    }
  }
}