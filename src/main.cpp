#include <Arduino.h>
#include <MOTEUR.h>
#include "ENCODEUR.h"
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

float Co = 3.245;
float Coprec = Co;
float Kd = 0.0845;
float ErreurPosAng;
float cons;
float consE = -0.0328;
float commande;
float commandeTraiter;
float commandeComp = 0.1566;
float vitesse;
float Kalpha = 4095.0;
int PWM;
float valeur;
int etat = 0;

extern float tension;
bool FlagCalcul = 0;
int asseractif = 0;
float Te = 5;    // période d'échantillonage en ms Mettre 10ms
float Tau = 175; // constante de temps du filtre en ms 1750ms

float TauW = 100;
float Kw = 21.6;
float Kwd = 2.64;
float CONSv;
float CONSvMax = 0.006;
float CONSvMIN = -1.0 * CONSvMax;
float CONSvPAS = CONSvMax / 100.0;
int direction;
float gauchedroit;
float consGaucheDroit;
float consGaucheDroitMAX = 0.2;
float consGaucheDroitPAS = consGaucheDroitMAX / 10.0;
// coefficient du filtre
float A, B;
int x = 0, y = 0;
unsigned long Intervalle_T = 50000;
unsigned long Echantillon_ms_precedent = 0;
void controle(void *parameters)
{
  // Demander a M.CLAMAINS toute information complémentaire sur la ligne suivante
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    santeAlim(0, 1);
    if (asseractif == 1)
    {
      /* if (y == 0)
       {
         if (millis() >= Echantillon_ms_precedent + Intervalle_T)
         {

           Echantillon_ms_precedent = millis();
           if (x == 1)
           {
             x = 3;
             Co = Coprec;
             y = 5;
           }
           if (x == 0)
           {
             x = 1;
             Co = 0;
             Intervalle_T = 800;
           }
         }
       }*/

      cons = CaclulVitesseAngulaireFiltre(Te, TauW, Kw, Kwd, CONSv);

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
      if (commandeTraiter > 0.45)
      {
        commandeTraiter = 0.45;
      }
      else if (commandeTraiter < -0.45)
      {
        commandeTraiter = -0.45;
      }

      // Serial.printf("avance\n");

      vitesse = 0.5 + commandeTraiter;
      PWM = vitesse * Kalpha;
      Controle_Moteur_Droit(PWM + (gauchedroit * Kalpha));
      Controle_Moteur_Gauche(PWM - (gauchedroit * Kalpha));
    }
    if (asseractif == 0)
    {
      vitesse = 0.5;
      PWM = vitesse * Kalpha;
      Controle_Moteur_Droit(PWM);
      Controle_Moteur_Gauche(PWM);
    }
    // Mise a 1 du flag permettant ainsi dans le void loop de pouvoir effectuer l'affichage
    FlagCalcul = 1;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));

    // Demander a M.CLAMAINS toute information complémentaire sur la ligne suivante
  }
}
void traitementBlutooth();
void rampe();

void setup()
{
  Serial.begin(115200);

  setAlim(2, 4, 27, 5, 36);
  Init_Pin_Moteur_Droit(19, 18);
  Init_Pin_Moteur_Gauche(16, 17);
  Serial.printf("Init Pin Moteur\n");
  delay(250);

  Config_Esp_Cannaux(19500, 12);
  Serial.printf("Init Canaux Moteur\n");
  delay(250);

  Init_Encondeur(33, 32, 26, 25);
  Serial.printf("Init   Encodeur\n");
  delay(250);

  // Permet la communication avec le terminal concu par M.GUINAND ou celui du PC
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

  delay(250);
  // setBlutTache();
  SerialBT.begin("JSP NAME");

  // calcul coeff filtre
  A = 1 / (1 + Tau / Te);
  B = Tau / Te * A;
  Serial.printf("Finish");
}

void loop()
{
  if (FlagCalcul == 1) // Affichage des données
  {
    traitementBlutooth();
    rampe();
    /**/
    if (millis() >= Echantillon_ms_precedent + Intervalle_T)
    {

      Echantillon_ms_precedent = millis();
      santeAlim(0, 2);
      SerialBT.printf("*G%2.1f", (tension * 10.0));
      // Serial.printf("*G%2.1f\n", (tension * 10.0));
    }
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
      // Serial.println(consE);
    }
    if (commande == "Kw")
    {
      Kw = valeur.toFloat();
    }
    if (commande == "Kwd")
    {
      Kwd = valeur.toFloat();
    }

    if (commande == "consVit")
    {
      CONSv = valeur.toFloat();
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

void traitementBlutooth()
{
  int mouvement = 0;
  /*
  if (SerialBT.available())
  {
    static int p = 0;
    static String chaine = "";
    String commande;
    String valeur;
    int index, length;
    char recept = SerialBT.read();
    // Serial.println(recept);

    if ((recept == '#'))
    {
      // Serial.printf("J'ai recu le caractere de fin de chaine\n");
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

      if (commande == "S")
      {
        float test = valeur.toFloat();
        Serial.print("Test : ");
        Serial.println(test);
      }
      if (commande == "d")
      {
        Serial.printf("Cesam FERME toi");
        Serial.println();
        closeRelais();
      }
      chaine = "";
    }
    else
    {
      chaine += recept;
    }
  }
*/
  /**/
  if (SerialBT.available())
  {

    int caractere = SerialBT.read();
    // Serial.println(SerialBT.read());
    if (caractere == 'C')
    {
      asseractif = 1;
      // Serial.printf("actif\n");
      // Serial.println(asseractif);
    }
    if (caractere == 'c')
    {
      asseractif = 0;
      // Serial.printf("non actif\n");
      // Serial.println(asseractif);
    }

    if (caractere == 'D')
    {
      // santeAlim(0, 0);
      // Serial.printf("Cesam OUVRE toi");
      // Serial.println();
      openRelais();
      // delay(1000);
    }
    if (caractere == 'd')
    {
      // Serial.printf("Cesam FERME toi");
      // Serial.println();
      closeRelais();
    }

    switch (caractere)
    {

    case '0':
      direction = 0;
      break;
    case '1':
      // CONSv = 0.005;
      direction = 1;
      // Serial.printf("UP\n");

      break;
    case '2':
      // Serial.printf("Droit\n");
      direction = 2;
      break;
    case '3':
      direction = 3;
      // Serial.printf("DOWN\n\n");
      // CONSv = -0.0083;
      break;

    case '4':
      direction = 4;
      // Serial.printf("Gauche\n");

      break;

    default:
      break;
    }
    /*
        switch (etat)
        {
        case 0:
          if (caractere == 'T') // Est ce que j'ai recu l'information
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
          else if (caractere == 't')
          {
            consE = (-1.0)*valeur /10000.0;
            SerialBT.printf("*w %f",consE);

            // Serial.printf("\nValeur %d\n", valeur);

            // CONSv = valeur / 100.0 * (0.0083 +0.0083) - 0.0083;
            // Serial.printf("\nValeur %4.4f\n", CONSv);

            etat = 0;
          }
          else
          {
            Serial.printf("Message non compris\n");

            etat = 0;
          }
          break;
        }*/
  }
}

void rampe()
{
  if (direction == 0)
  {
    gauchedroit = 0;
    CONSv = 0;
  }
  if (direction == 1)
  {
    if (CONSv <= CONSvMax)
    {
      // Serial.printf("++++\n");
      CONSv += CONSvPAS;
    }
    // asservissementEncodeur(PWM);
  }
  if (direction == 3)
  {
    if (CONSv >= CONSvMIN)
    {
      // Serial.printf("----\n");
      CONSv -= CONSvPAS;
    }
    // asservissementEncodeur(PWM);
  }
  if (direction == 2)
  {
    // Serial.printf("Tour DROIT\n");
    /* if (consGaucheDroit > ((-1.0) * consGaucheDroitMAX))
        {
          consGaucheDroit -= consGaucheDroitPAS;
        }*/
    gauchedroit = -0.15;
  }

  if (direction == 4)
  {
    // Serial.printf("Tour GAUCHE\n");
    /* if (consGaucheDroit < consGaucheDroitMAX)
     {
       consGaucheDroit += consGaucheDroitPAS;
     }*/
    gauchedroit = 0.15;
  }
}