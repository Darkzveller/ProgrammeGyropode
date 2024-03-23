#include "MOTEUR.h"
#include <Arduino.h>
// Config du pwm Moteur Droit
int pinPWM_A_MoteurDroit, pinPWM_B_MoteurDroit;
// Config du pwm Moteur Gauche
int pinPWM_A_MoteurGauche, pinPWM_B_MoteurGauche;
// Config canal Moteur
int canal_moteur_droit = 0, canal_moteur_droit_bis = 1;
int canal_moteur_gauche = 2, canal_moteur_gauche_bis = 3;

// Config parametre pwm
void Init_Pin_Moteur_Droit(int pwmA, int pwmB)
{

    pinPWM_A_MoteurDroit = pwmA;
    pinPWM_B_MoteurDroit = pwmB;
}

void Init_Pin_Moteur_Gauche(int pwmA, int pwmB)
{
    pinPWM_A_MoteurGauche = pwmA;
    pinPWM_B_MoteurGauche = pwmB;
}

void Config_Esp_Cannaux(float frequence, float resolution)
{

    // Config Moteur
    ledcSetup(canal_moteur_droit, frequence, resolution);
    // Connexion entre les pins et les canaux
    ledcAttachPin(pinPWM_A_MoteurDroit, canal_moteur_droit);
    ledcAttachPin(pinPWM_B_MoteurDroit, canal_moteur_droit_bis);

    ledcSetup(canal_moteur_gauche, frequence, resolution);
    // Connexion entre les pins et les canaux
    ledcAttachPin(pinPWM_A_MoteurGauche, canal_moteur_gauche);
    ledcAttachPin(pinPWM_B_MoteurGauche, canal_moteur_gauche_bis);
}

void Controle_Moteur_Droit(int pwm)
{
    int reverse_pwm;
    reverse_pwm = 4095 - pwm;
    ledcWrite(canal_moteur_droit, pwm);
    ledcWrite(canal_moteur_droit_bis, reverse_pwm);
}

void Controle_Moteur_Gauche(int pwm)
{
    int reverse_pwm;
    reverse_pwm = 4095 - pwm;
    ledcWrite(canal_moteur_gauche, pwm);
    ledcWrite(canal_moteur_gauche_bis, reverse_pwm);
}
void tourner(int direction, int pwm)
{

    if (direction)
    {
        Controle_Moteur_Droit(pwm);
        Controle_Moteur_Gauche(4095 - pwm);
    }
    else
    {
        Controle_Moteur_Droit(4095 - pwm);
        Controle_Moteur_Gauche(pwm);
    }
}