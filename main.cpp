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
#include <QTRSensors.h>   //sonar
#include <ADJDS311.h>     //servomoteurs
#include <math.h>

#define domino_drop 1000
const float distance_dominos_reelle = 2;

const long pulses = 3200;
// les numéros des boutons et des dels sont facilement modifiables selon leur emplacement sur le robot
#define bouton_go 22
#define bouton_droite 24
#define bouton_gauche 26
#define bouton_spirale 45
#define bouton_Scarree 47
#define bouton_ligne 49
#define del_go 28
#define del_droite 30
#define del_gauche 32
#define del_spirale 39
#define del_carre 41
#define del_ligne 43

const float circonferenceRoue = 24.19;
const float circonferenceCercleUneRoue = 116.43; 
const float circonferenceCercleDeuxRoues = 58.119; 
const float Kp = 0.000003, Ki = 0.0000006; //facteurs de correction a modifier selon comportement
//augmenter si correction trop petite et vice-versa tout en respectant le plus possible la relation Kp = 5Ki

// fonctions

void PlacerDomino(){
  delay(100);           //donne le temps au robot de se stabiliser
  const int Servo1AngleMax = 170;//165
  const int Servo1AngleMin = 87;
  const int Servo0AngleMax = 180;
  const int Servo0AngleMin = 88;
  SERVO_SetAngle(0,Servo0AngleMax);   //le pousseux pousse
  delay(900);         // donne le temps au domino de bien tomber
  SERVO_SetAngle(1,Servo1AngleMax);   //le pousseux se replace
  SERVO_SetAngle(0,Servo0AngleMin);   //le placeux place le domino
  delay(350);                         //donne un peu de temps au domino de rester droit
  SERVO_SetAngle(1,Servo1AngleMin);   //le placeux se replace
  delay(100);               //petit delay avant que le robot recommence à bouger
}

void dels_on(){
  digitalWrite(del_go, HIGH);
  digitalWrite(del_ligne, HIGH);
  digitalWrite(del_droite, HIGH);
  digitalWrite(del_gauche, HIGH);
  digitalWrite(del_spirale, HIGH);
  digitalWrite(del_carre, HIGH);
}

void dels_off(){
  digitalWrite(del_go, LOW);
  digitalWrite(del_ligne, LOW);
  digitalWrite(del_droite, LOW);
  digitalWrite(del_gauche, LOW);
  digitalWrite(del_spirale, LOW);
  digitalWrite(del_carre, LOW);
}
int bouton_parcours(int bouton, int del){     //fonction avec debouncing et dels
  if (digitalRead(bouton)== 1){
    delay(50);         //debouncing
    if (digitalRead(bouton)== 1){
      Serial.println(bouton);
      digitalWrite(del, HIGH);      //allume la del associée pour indiquer la sauvegarde du choix
      while (digitalRead(bouton) == 1){   //pour que ca le prenne juste une fois le choix même si on tiens le bouton
      }
      delay(200);                         // si la del s'allume pas, le choix n'est pas enregistré
      digitalWrite(del, LOW);
      return del;                   //// le choix est enregistré et l'utilisateur comprend
    }
    if (digitalRead(bouton)==0){
      return 0;
    }
  }
}

void arret_chargeur(){
  MOTOR_SetSpeed(0,0);
  MOTOR_SetSpeed(1,0);
  dels_on();
  while (bouton_go == 0){
  }
  if (bouton_go == 1){
    dels_off();
    bouton_parcours(bouton_go, del_go);
  }
}

int ligne_droite(int nb_dominos, int chargeur){
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
  float vit_droite = 0.1;
  float vit_gauche = 0.1;
  float DISTANCEd = 0.40;
  float DISTANCEg = 0.40;

  const float Kp = 0.000046875; float Ki = Kp/5; //facteurs de correction a modifier selon comportement
  //augmenter si correction trop petite et vice-versa tout en respectant le plus possible la relation Kp = 5Ki
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  
  while(nb_dominos_place < nb_dominos){

    delay(10);// Delais pour décharger le CPU
    int i = 0;
    if (chargeur == 0){
      arret_chargeur();
      chargeur = 27;
    }

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
    chargeur -= 1;

    Serial.print(dist_roue_gauche); Serial.print('/'); Serial.println(dist_roue_droite);

    delay(DUREE_DOMINOS_DROP);
    PlacerDomino();
  }
  return chargeur;
}

void tourner_droite(){
}

void tourner_gauche(){
}

void spirale(){
}

void spirale_carre(){
}

void tourner90(){
  MOTOR_SetSpeed(0,0.1);
  MOTOR_SetSpeed(1,0.2);
  delay(500);
  MOTOR_SetSpeed(0,0);
  MOTOR_SetSpeed(1,0);
  PlacerDomino();
}


void flash_go(int del){        //ca fait juste flasher la del pour avertir l'utilisateur que le robot va avancer
  digitalWrite(del, HIGH);
  delay(400);
  digitalWrite(del, LOW);
  delay(200);
}

void parcours(int deplacement[6]){    // fonction qui fait le parcours inventé par l'utilisateur
  int x = 0;
  int d = 27; // nombre de dominos dans le chargeur
  while (x<5){        // un maximum de 5 déplacements de suite
    if (deplacement[x] == 0)
      x=5;
    if (deplacement[x] == del_ligne){
      d = ligne_droite(2, d);
      Serial.println(d);
    }
    if (deplacement[x] == del_droite){
      tourner_droite();
    }
    if (deplacement[x] == del_gauche){
      tourner_gauche();
    }
    if (deplacement[x] == del_spirale){
      spirale();
    }
    if (deplacement[x] == del_carre){
      spirale_carre();
    }
    x++;
  }
}



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
  int x = 0;      //pour le maximum de 5 entrées
  int deplacement[6]={0,0,0,0,0};   //pour garder en note le parcours choisi
  int v = 0;
  int actif = 0;    //est-ce que la del s'est allumé et le choix du bouton bien enregistré?
  if (digitalRead(bouton_go) == 1) {
    actif = bouton_parcours(bouton_go, del_go);   //debouncing + del
    if (actif == del_go){      //debouncing
    actif = 0;            //pour etre certain que ca saute pas d'étape plus tard
    while (v == 0){       //pour reinitialiser le parcours et le tableau
      while (x<5){          // un maximum de 5 déplacements de suite
        if (x == 0){        // on ne peut pas rajouter des parcours lorsqu'on choisi les spirales
          if (digitalRead(bouton_spirale) == 1) {
          actif = bouton_parcours(bouton_spirale, del_spirale);
          if (actif ==del_spirale){
            actif = 0;
            deplacement[x] = del_spirale;
            x = 5;    //sortir de la boucle
            }
          }
          if (digitalRead(bouton_Scarree) == 1) {
            actif =bouton_parcours(bouton_Scarree, del_carre);
            if (actif == del_carre){
              actif = 0;
              deplacement[x] = del_carre;
              x = 5;    //sortir de la boucle
            }
          }
        }
        if (digitalRead(bouton_ligne) == 1) {
          actif = bouton_parcours(bouton_ligne, del_ligne);
          if (actif == del_ligne){
            Serial.println("enregistré");
            actif = 0;
            deplacement[x] = del_ligne;
            x += 1;
          }
        }
        if (digitalRead(bouton_droite) == 1) {
          actif = bouton_parcours(bouton_droite, del_droite);
          if (actif == del_droite){  
            actif = 0;
            deplacement[x] = del_droite;
            x += 1;
          }
        }
        if (digitalRead(bouton_gauche) == 1) {
          actif = bouton_parcours(bouton_gauche, del_gauche);
          if (actif == del_gauche){  
            actif = 0;
            deplacement[x] = del_gauche;
            x += 1;
          }
        }
        if (digitalRead(bouton_go) == 1){       // pour commencer le parcours avec moins de 5 déplacements
          actif = bouton_parcours(bouton_go, del_go);
          if (actif == del_go){
            actif = 0;
            int y = 0;
            delay(500);
            Serial.println("parcours");
            while (y<5){              // écrit les données dans le tableau
              Serial.println(deplacement[y]);
              if (deplacement[y] !=0){
                flash_go(deplacement[y]);    //allume les dels en ordre de leurs choix
              }
              y += 1;
            }
            delay(1000);

            parcours(deplacement);

            Serial.println("parcours fini");
            x = 5;      /// sors de la boucle
            v = 1;      //pour recommencer au complet
          }
        }
      }
      if (deplacement[0] != 0 && digitalRead(bouton_go) == 1){ // pour commencer le parcours avec 5 déplacements ou 1 complexe
        actif = bouton_parcours(bouton_go, del_go);
        if (actif == del_go){
          actif = 0;
          int y = 0;
          while (y<5){              // écrit les données dans le tableau
            Serial.println(deplacement[y]);
            if (deplacement[y] != 0){
              flash_go(deplacement[y]);
            }
            y += 1;
          }
          delay(1000);

          parcours(deplacement);

          Serial.println("parcours fini");
          v = 1; //pour recommencer au complet
        }
      }
    }
    }
  }
}

