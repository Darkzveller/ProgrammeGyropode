#ifndef _MOTEUR_H
#define _MOTEUR_H


void Init_Pin_Moteur_Droit(int pwmA, int pwmB);
void Init_Pin_Moteur_Gauche(int pwmA, int pwmB);
void Config_Esp_Cannaux(float frequence, float resolution);
void Controle_Moteur_Droit(int pwm);
void Controle_Moteur_Gauche(int pwm);
void tourner(int direction, int pwm);

#endif