/*
Projet: parcours domino
Equipe: P20
Auteurs: Etienne, Victo, Kat
Description: Programmation des parcours suivies par le robot
Date: 03-11-18
*/

/* ****************************************************************************
Inclure les librairies de functions que vous voulez utiliser
**************************************************************************** */

#include <LibRobus.h> // Essentielle pour utiliser RobUS
#include <arduino.h>


/* ****************************************************************************
Variables globales et defines
**************************************************************************** */

#define domino_drop 1000
const float distance_dominos_reelle = 2;

const long pulses = 3200;
#define buttonPin1 53
#define buttonPin2 54
#define buttonPin3 55
#define GoPin 56
const float circonferenceRoue = 24.19;
const float circonferenceCercleUneRoue = 116.43; 
const float circonferenceCercleDeuxRoues = 58.119; 
const float Kp = 0.000003, Ki = 0.0000006; //facteurs de correction a modifier selon comportement
//augmenter si correction trop petite et vice-versa tout en respectant le plus possible la relation Kp = 5Ki

/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */


void asservissement(float nbDominos){
  const float DISTANCE = 2.00; //changer la valeur pour distance entre deux dominos
  const int DUREE = 1000; //temps/dominos place, ici 1 sec, changer selon tests
  int i = 0; //compteur pour integrer la derniere erreur lue
  float dist_roue_droite = 0;
  float dist_roue_gauche = 0; //distance parcourue par les roues par cycle
  float dist_roue_droite_totale = 0;
  float dist_roue_gauche_totale = 0; //distance totale parcourue par les roues
  float vit_droite = 1.1;
  float vit_gauche = 1.15; //initialisation des vitesse des moteurs
  //A remarquer ici que ces vitesses seront ajustes jusqu'a ce qu'elles atteignent distance/duree (cm/us)

  float totale_ENCODER_read_right = 0;
  float totale_ENCODER_read_left = 0;
  float erreur_droite = 0;
  float erreur_gauche = 0;
  float erreur_droite_totale = 0;
  float erreur_gauche_totale = 0;
  
  while (i < nbDominos){
  ENCODER_ReadReset(RIGHT); ENCODER_ReadReset(LEFT);

  MOTOR_SetSpeed(1, vit_droite); 
  MOTOR_SetSpeed(0, vit_gauche);
  delay(DUREE);

  dist_roue_droite = (circonferenceRoue/3200)*ENCODER_Read(RIGHT);
  dist_roue_gauche = (circonferenceRoue/3200)*ENCODER_Read(LEFT);

  dist_roue_droite_totale = (circonferenceRoue/3200)*totale_ENCODER_read_right;
  dist_roue_gauche_totale = (circonferenceRoue/3200)*totale_ENCODER_read_left;
  
  totale_ENCODER_read_right += ENCODER_Read(RIGHT);
  totale_ENCODER_read_left += ENCODER_Read(LEFT);

  erreur_droite = DISTANCE - dist_roue_droite;
  erreur_gauche = DISTANCE - dist_roue_gauche;
  erreur_droite_totale = DISTANCE*i - dist_roue_droite_totale;
  erreur_gauche_totale = DISTANCE*i - dist_roue_gauche_totale;

  vit_droite *= (erreur_droite*Ki + erreur_droite_totale*Kp);
  vit_gauche *= (erreur_gauche*Ki + erreur_gauche_totale*Kp);

  i++;
  }
}

void ligne_droite(float dominos){ // nombre de dominos a placer

  long distance_dominos_encodeur = (distance_dominos_reelle)/circonferenceRoue * pulses; // distance entre 2 dominos
  long distance_parcours = distance_dominos_reelle; // compteur pour la ligne
  while (distance_parcours <= (dominos*5)){ //boucle pour compteur de dominos -> distance totale
    ENCODER_ReadReset(0);
    ENCODER_ReadReset(1);
    while (ENCODER_Read(0) < distance_dominos_encodeur){  //boucle entre 2 dominos (asservissement)
      asservissement(dominos);
    }
    MOTOR_SetSpeed(1, 0);
    MOTOR_SetSpeed(0, 0);
    delay(domino_drop);    // temps pour placer les dominos
    distance_parcours += distance_dominos_reelle;
  }
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
      tourner_droite(******);
    }
    if (deplacement[x] == 3){
      tourner_gauche(******);
    }
    x++;
  }
}*/


/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
// -> Se fait appeler au debut du programme
// -> Se fait appeler seulement un fois
// -> Generalement on y initilise les varibbles globales

void setup(){
  BoardInit();
}


/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop(){
  int x = 0;
  int deplacement[6]={0,0,0,0,0};
  int button2 = digitalRead(buttonPin2);    //bouton tourner droite
  if (button2 == 1){
    int go = 0;
    for (int i=0; i<10; i++){
      go = i;
    }
    if (go == 10){
      while (x<5){          // un maximum de 5 déplacements de suite
        int button1 = digitalRead(buttonPin1);    //bouton ligne droite
        int button2 = digitalRead(buttonPin2);    //bouton tourner droite
        int button3 = digitalRead(buttonPin3);    //bouton tourner gauche

        if (button1 == 1) {
          int go = 0;
          for (int i=0; i<10; i++){
            go = i;
          }
          if (go == 10){
            deplacement[x] = 1;
            Serial.println("LIGNE");
            x += 1;
          }
        }
        if (button2 == 1) {
          int go = 0;
          for (int i=0; i<10; i++){
            go = i;
          }
          if (go == 10){
            deplacement[x] = 2;
            Serial.println("DROITE");
            x += 1;
          }
        }
        if (button3 == 1) {
          int go = 0;
          for (int i=0; i<10; i++){
            go = i;
          }
          if (go == 10){
            deplacement[x] = 3;
            Serial.println("GAUCHE");
            x += 1;
          }
        }
        if (GoPin == 1){       // pour commencer le parcours avec moins de 5 déplacements
          int go = 0;
          for (int i=0; i<10; i++){
            go = i;
          }
          if (go == 10){
            //parcours(deplacement);
            Serial.println("GOOOOOO");
          }
        }
      }
    }
  Serial.println("//////////////////");
  int y = 0;
  while (y<5){
    Serial.println(deplacement[y]);
    y += 1;
  }
  if (GoPin == 1){       // pour commencer le parcours avec 5 déplacements
      //parcours(deplacement);
      Serial.println("GOOOOOO");
    }
    ligne_droite(10);
}