#ifndef _ENCODEUR_H
#define _ENCODEUR_H

void Init_Encondeur(int x, int y, int z, int u);
float lecture_Encodeur_Droit(void);
float lecture_Encodeur_Gauche(void);
float CaclulVitesseAngulaireFiltre(float Te, float Tau, float Kw, float Kdw, float CONSv);
// void asservissementEncodeur(float PWM);

#endif
