/*
Projet: parcours domino
Equipe: P20
Auteurs: Etienne, Victo, Kat
Description: Programmation des parcours suivies par le robot
Date: 03-11-18
*/


// librairies
#include <LibRobus.h> // Essentielle pour utiliser RobUS
#include <arduino.h>
#include <QTRSensors.h>
#include <ADJDS311.h>
#include <math.h>

#define domino_drop 1000
const float distance_dominos_reelle = 2;

const long pulses = 3200;
#define bouton_ligne 51
#define bouton_droite 49
#define bouton_gauche 47
#define bouton_spirale 45
#define bouton_Scarree 43
#define bouton_go 41
#define del_ligne 39
#define del_droite 37
#define del_gauche 35
#define del_spirale 33
#define del_carre 31
#define del_go 30

const float circonferenceRoue = 24.19;
const float circonferenceCercleUneRoue = 116.43; 
const float circonferenceCercleDeuxRoues = 58.119; 
const float Kp = 0.000003, Ki = 0.0000006; //facteurs de correction a modifier selon comportement
//augmenter si correction trop petite et vice-versa tout en respectant le plus possible la relation Kp = 5Ki


void PlacerDomino(){
  const int Servo1AngleMax = 170;//165
  const int Servo1AngleMin = 87;
  const int Servo0AngleMax = 180;
  const int Servo0AngleMin = 88;
  SERVO_SetAngle(0,Servo0AngleMax);
  delay(900);
  SERVO_SetAngle(1,Servo1AngleMax);
  SERVO_SetAngle(0,Servo0AngleMin);
  delay(550);
  SERVO_SetAngle(1,Servo1AngleMin);
  delay(100);
}

void asservissement(float vit_droite, float vit_gauche, float const DISTANCEd, float const DISTANCEg, int const nb_dominos){
  int nb_dominos_place = 0;
  float totale_ENCODER_read_right = 0;
  float totale_ENCODER_read_left = 0;
  float dist_roue_droite = 0, dist_roue_gauche = 0;
  float dist_roue_droite_totale = 0, dist_roue_gauche_totale = 0;
  float erreur_droite = 0;
  float erreur_gauche = 0;
  float erreur_droite_totale = 0;
  float erreur_gauche_totale = 0;
  const int DUREE_DOMINOS_DROP = 500;

  const float Kp = 0.000046875; float Ki = Kp/5; //facteurs de correction a modifier selon comportement
  //augmenter si correction trop petite et vice-versa tout en respectant le plus possible la relation Kp = 5Ki
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  
  while(nb_dominos_place < nb_dominos){

    delay(10);// Delais pour décharger le CPU
    int i = 0;

    while (i < 5){
      ENCODER_ReadReset(RIGHT); ENCODER_ReadReset(LEFT);

      MOTOR_SetSpeed(RIGHT, vit_droite); MOTOR_SetSpeed(LEFT, vit_gauche);
      delay(100);

      totale_ENCODER_read_right += ENCODER_Read(RIGHT); totale_ENCODER_read_left += ENCODER_Read(LEFT);
      
      dist_roue_droite = (circonferenceRoue/3200)*ENCODER_Read(RIGHT);
      dist_roue_gauche = (circonferenceRoue/3200)*ENCODER_Read(LEFT);

      dist_roue_droite_totale = (circonferenceRoue/3200)*totale_ENCODER_read_right;
      dist_roue_gauche_totale = (circonferenceRoue/3200)*totale_ENCODER_read_left;

      erreur_droite = DISTANCEd - dist_roue_droite;
      erreur_gauche = DISTANCEg - dist_roue_gauche;
      erreur_droite_totale = DISTANCEd*i - dist_roue_droite_totale;
      erreur_gauche_totale = DISTANCEg*i - dist_roue_gauche_totale;

      vit_droite *=(1 + (erreur_droite*Kp + erreur_droite_totale*Ki));
      vit_gauche *= (1 + (erreur_gauche*Kp + erreur_gauche_totale*Ki));
      
      i++;
    }

    MOTOR_SetSpeed(RIGHT, 0); MOTOR_SetSpeed(LEFT, 0);
    nb_dominos_place++;

    Serial.print(dist_roue_gauche); Serial.print('/'); Serial.println(dist_roue_droite);

    delay(DUREE_DOMINOS_DROP);
    PlacerDomino();
  }
}

void ligne_droite(){ 
 asservissement(0.1, 0.112, 0.40, 0.40, 27);
}

void tourner_90(int sens){
  if (sens == RIGHT)
  {
    asservissement(0.05, 0.1, 0.50, 2.00, 15);
  }

  if (sens == LEFT)
  {
    asservissement(0.1, 0.05, 2.00, 0.50, 15);
  }
}

void tourner90(){
  MOTOR_SetSpeed(0,0.1);
  MOTOR_SetSpeed(1,0.2);
  delay(500);
  MOTOR_SetSpeed(0,0);
  MOTOR_SetSpeed(1,0);
  PlacerDomino();
}

/*void parcours(int deplacement[6]){    // fonction qui fait le parcours inventé par l'utilisateur
  int x = 0;
  while (x<5){        // un maximum de 5 déplacements de suite
    if (deplacement[x] == 0)
      return 0;
    if (deplacement[x] == 1){
      ligne_droite(20);
    }
    if (deplacement[x] == 2){
      tourner_droite();
    }
    if (deplacement[x] == 3){
      tourner_gauche();
    }
    x++;
  }
}*/



void setup(){
  BoardInit();
  pinMode(del_ligne, OUTPUT);
  pinMode(del_droite, OUTPUT);
  pinMode(del_gauche, OUTPUT);
  pinMode(del_spirale, OUTPUT);
  pinMode(del_carre, OUTPUT);
  pinMode(del_go, OUTPUT);

  Serial.begin(9600);
  
  SERVO_Enable(0);
  SERVO_Enable(1);
  
}

void loop(){
  int x = 0;
  int deplacement[6]={0,0,0,0,0};
  if (digitalRead(bouton_go) == 1){
    delay(100);                            // Debouncing
    if (digitalRead(bouton_go) == 1){
      Serial.println("bouton go");
      while (digitalRead(bouton_go) == 1){
        digitalWrite(del_go, HIGH);
      }
      while (x<5){          // un maximum de 5 déplacements de suite
        if (x == 0){
          if (digitalRead(bouton_spirale) == 1) {
            delay(100);                           // Debouncing
            if (digitalRead(bouton_spirale) == 1){
              deplacement[x] = 3;
              Serial.println("SPIRALE");
              x += 1;
              while (digitalRead(bouton_spirale) == 1){       //del pour confirmer
                digitalWrite(del_spirale, HIGH);      //allumer del
              }
            }
          }
          if (digitalRead(bouton_Scarree) == 1) {
            delay(100);                           // Debouncing
            if (digitalRead(bouton_Scarree) == 1){
              deplacement[x] = 3;
              Serial.println("CARRE");
              x += 1;
              while (digitalRead(bouton_Scarree) == 1){       //del pour confirmer
                digitalWrite(del_carre, HIGH);      //allumer del
              }
            }
          }
        }
        if (digitalRead(bouton_ligne) == 1) {
            delay(100);                            // Debouncing
            if (digitalRead(bouton_ligne) == 1){
              deplacement[x] = 1;
              Serial.println("LIGNE");
              x += 1;
              while (digitalRead(bouton_ligne) == 1){
                digitalWrite(del_ligne, HIGH);      //allumer del
              }
            }
          }
        if (digitalRead(bouton_droite) == 1) {
          delay(100);                            // Debouncing
          if (digitalRead(bouton_droite) == 1){
            deplacement[x] = 2;
            Serial.println("DROITE");
            x += 1;
            while (digitalRead(bouton_droite) == 1){
              digitalWrite(del_droite, HIGH);     //allumer del
            }
          }
        }
        if (digitalRead(bouton_gauche) == 1) {
          delay(100);                           // Debouncing
          if (digitalRead(bouton_gauche) == 1){
            deplacement[x] = 3;
            Serial.println("GAUCHE");
            x += 1;
            while (digitalRead(bouton_gauche) == 1){       //del pour confirmer
              digitalWrite(del_gauche, HIGH);     //allumer del
            }
          }
        }
        if (digitalRead(bouton_go) == 1){       // pour commencer le parcours avec moins de 5 déplacements
          delay(100);
          if (digitalRead(bouton_go) == 1){
            Serial.println("FIN");
            int y = 0;
            while (y<5){              // écrit les données dans le tableau
              Serial.println(deplacement[y]);
              y += 1;
            }
            while (digitalRead(bouton_go) == 1){
              digitalWrite(del_go, HIGH);     //allumer del
            }
            delay(2000);
            //parcours
            Serial.println("GO");
          }
        }
      }
    }
  }
}
