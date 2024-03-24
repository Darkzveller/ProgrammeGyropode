#include "ENCODEUR.h"
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <math.h>

ESP32Encoder encoderGauche;
ESP32Encoder encoderDroit;

unsigned long encoderDroitlastToggled;
bool encoderDroitPaused = false;
float valeurcodeurdroit, valeurcodeurgauche;
float position, positionAncienne;

float distancedroit, distancegauche, ecartEnc;
float perimetre, diametreroue = 65.0;
float TIC_UN_TOUR = 752.0;

float thetaEncodeur, thetaEncodeurPrec, thetaEncodeurDroit, thetaEncodeurGauche, vitesseAngulaire;
float vitesseAngulaireFiltre, vitesseAngulaireFiltrePrec;
float vitesseAngulaireCONS, ErreurVitAng, ErreurVitAngPrec;

float commandeVit;

void Init_Encondeur(int x, int y, int z, int u)
{
  // Config encodeur
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoderGauche.attachHalfQuad(z, u);
  encoderDroit.attachHalfQuad(x, y);
  encoderDroit.clearCount();
  perimetre = diametreroue * M_PI;
}

float lecture_Encodeur_Droit(void)
{

  valeurcodeurdroit = encoderDroit.getCount();
  // Je sais qu'en 1 tour je x tic, donc je convertis le nombre de tic pour avoir des tours
  distancedroit = valeurcodeurdroit * 1.0 / TIC_UN_TOUR;
  // Je convertir mon nombre de tour en distance parcourue
  // distancedroit = distancedroit * perimetre;
  // Conversion en radians
  distancedroit = 2.0 * M_PI * distancedroit / 1.0;

  return distancedroit;
}

float lecture_Encodeur_Gauche(void)
{

  valeurcodeurgauche = encoderGauche.getCount();
  // Je sais qu'en 1 tour je x tic, donc je convertis le nombre de tic pour avoir des tours
  distancegauche = valeurcodeurgauche * 1.0 / TIC_UN_TOUR;
  // Je convertir mon nombre de tour en distance parcourue
  // distancegauche = distancegauche * perimetre;
  // Conversion en radians
  distancegauche = 2.0 * M_PI * distancegauche / 1.0;

  return distancegauche;
}

float CaclulVitesseAngulaireFiltre(float Te, float Tau, float Kw, float Kdw, float CONSv)
{
  vitesseAngulaireCONS=CONSv;
  thetaEncodeur = (lecture_Encodeur_Droit() + lecture_Encodeur_Gauche()) / 2.0;
  // Serial.printf("%4.4f %4.4f %4.4f\n", thetaEncodeur, lecture_Encodeur_Droit(), lecture_Encodeur_Gauche());
// Deriver de l'angle pour obtenir l'accélaration angulaire
  vitesseAngulaire = (thetaEncodeur - thetaEncodeurPrec) / Te;
  // Serial.printf("%4.4f \n", vitesseAngulaire);
  thetaEncodeurPrec = thetaEncodeur;

  vitesseAngulaireFiltre = (vitesseAngulaire + vitesseAngulaireFiltrePrec * Tau / Te) / (1 + Tau / Te);
  vitesseAngulaireFiltrePrec = vitesseAngulaireFiltre;
  // Serial.printf("%4.4f %4.4f\n", vitesseAngulaire, vitesseAngulaireFiltre);

  ErreurVitAng = vitesseAngulaireCONS - vitesseAngulaireFiltre;
  
  // Serial.printf("%4.4f \n",ErreurVitAng);
  

  commandeVit = -1.0 * ErreurVitAng * Kw + -1.0  * Kdw * (ErreurVitAng - ErreurVitAngPrec) / Te;


  // Serial.printf("%4.4f %4.4f %4.4f %4.4f \n", testval, brancheK, brancheKd,Kdw);
  ErreurVitAngPrec = ErreurVitAng;
/**/
  return commandeVit;
}

// void asservissementEncodeur(float PWM)
// {
//   extern int canal_moteur_droit, canal_moteur_droit_bis;
//   extern int canal_moteur_gauche, canal_moteur_gauche_bis;
//   position = lecture_Encodeur_Droit() - lecture_Encodeur_Gauche();
//   float commande;

//   commande = 1 * position + 1 * (position - positionAncienne);
//   positionAncienne = position;

//   if (commande > 0)
//   {
//     if (commande > PWM)
//     {
//       commande = PWM;
//     }
//     // MotDroit((PWM - commande));
//     // MotGauche((PWM));
//     Controle_Moteur_Droit(PWM);
//     Controle_Moteur_Gauche((PWM - commande));
//   }
//   else
//   {
//     if (commande < -PWM)
//     {
//       commande = -PWM;
//     }
//     Controle_Moteur_Gauche(PWM);
//     Controle_Moteur_Droit((PWM - commande));
//     // MotDroit((PWM));
//     // MotGauche((PWM - commande));
//   }
//   // Serial.printf("OK");
// }
