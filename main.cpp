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

const float distance_dominos_reelle = 2;

const long pulses = 3200;
// les numéros des boutons et des dels sont facilement modifiables selon leur emplacement sur le robot
#define bouton_gauche 22
#define bouton_ligne 24
#define bouton_droite 26
#define bouton_spirale 45
#define bouton_Scarree 47
#define bouton_go 49
#define del_gauche 28       //droite, ligne, gauche, spirale, carre
#define del_ligne 30
#define del_droite 32
#define del_spirale 39
#define del_carre 41
#define del_go 43

const float circonferenceRoue = 24.19;
const float circonferenceCercleUneRoue = 116.43; 
const float circonferenceCercleDeuxRoues = 58.119; 
const float Kp = 0.000003, Ki = 0.0000006; //facteurs de correction a modifier selon comportement
//augmenter si correction trop petite et vice-versa tout en respectant le plus possible la relation Kp = 5Ki

#define domino_drop 200
#define circonference_cercle 116.43//cm virage a une roue
#define pulse_roue 3200 //''coche''
#define distance_domino_reel 3.5

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
  delay(550);                         //donne un peu de temps au domino de rester droit
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
  while (digitalRead(bouton_go)==0){
    MOTOR_SetSpeed(0,0);
    MOTOR_SetSpeed(1,0);
    dels_on();
  }
  bouton_parcours(bouton_go, del_go);
  dels_off();
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
    if (digitalRead(bouton_go) ==1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }
    delay(10);// Delais pour décharger le CPU
    int i = 0;
    if (chargeur == 0){
      arret_chargeur();
      chargeur = 27;
    }
    while (i < 5){
      if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
        arret_chargeur();
      }
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
      if (digitalRead(bouton_go) ==1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
        arret_chargeur();
      }
    }

    MOTOR_SetSpeed(RIGHT, 0); 
    MOTOR_SetSpeed(LEFT, 0);
    nb_dominos_place++;
    chargeur -= 1;

    Serial.print(dist_roue_gauche); 
    Serial.print('/'); 
    Serial.println(dist_roue_droite);

    delay(DUREE_DOMINOS_DROP);
    PlacerDomino();
    if (digitalRead(bouton_go) ==1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }
  }
  return chargeur;
}

/*!!!! NE JAMAIS COMMANDER PLUS QUE 15 CM DANS LA FONCTION ASSERVISSEMENTDISTANCE SINON QUOI CA DEVIENT TRES IMPRECIS!!!!*/
void asservissementDistanceE(float distance_a_parcourir){
  float totale_ENCODER_read_right = 0;
  float totale_ENCODER_read_left = 0;
  float dist_roue_droite = 0, dist_roue_gauche = 0;
  float dist_roue_droite_totale = 0, dist_roue_gauche_totale = 0;
  float erreur_droite = 0;
  float erreur_gauche = 0;
  float erreur_droite_totale = 0;
  float erreur_gauche_totale = 0;
  float vit_droite = 0.1;
  float vit_gauche = 0.102;
  const float DISTANCE = (808/10/3200)*circonferenceRoue;
  const int DUREE_DOMINOS_DROP = 500;
  int i = 0;

  const float Kp = 0.000046875; float Ki = Kp/5; //facteurs de correction a modifier selon comportement
  //augmenter si correction trop petite et vice-versa tout en respectant le plus possible la relation Kp = 5Ki
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  
  distance_a_parcourir = distance_a_parcourir;//facteur correctionnel de la distance commandé


  while(dist_roue_droite_totale < distance_a_parcourir){
    ENCODER_Reset(RIGHT); ENCODER_Reset(LEFT);

    MOTOR_SetSpeed(RIGHT, vit_droite); MOTOR_SetSpeed(LEFT, vit_gauche);
    delay(10);

    totale_ENCODER_read_right += ENCODER_Read(RIGHT); totale_ENCODER_read_left += ENCODER_Read(LEFT);
      
    dist_roue_droite = (circonferenceRoue/3200)*ENCODER_Read(RIGHT);
    dist_roue_gauche = (circonferenceRoue/3200)*ENCODER_Read(LEFT);

    dist_roue_droite_totale = (circonferenceRoue/3200)*totale_ENCODER_read_right;
    dist_roue_gauche_totale = (circonferenceRoue/3200)*totale_ENCODER_read_left;

    erreur_droite = DISTANCE - dist_roue_droite;
    erreur_gauche = DISTANCE - dist_roue_gauche;
    erreur_droite_totale = DISTANCE*i - dist_roue_droite_totale;
    erreur_gauche_totale = DISTANCE*i - dist_roue_gauche_totale;

    vit_droite *=(1 + (erreur_droite*Kp + erreur_droite_totale*Ki));
    vit_gauche *= (1 + (erreur_gauche*Kp + erreur_gauche_totale*Ki));

    i++;

    Serial.println(dist_roue_droite_totale);
    if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }
  }
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
  totale_ENCODER_read_right = 0;
  totale_ENCODER_read_left = 0;
  dist_roue_droite = 0, dist_roue_gauche = 0;
  dist_roue_droite_totale = 0, dist_roue_gauche_totale = 0;
  erreur_droite = 0;
  erreur_gauche = 0;
  erreur_droite_totale = 0;
  erreur_gauche_totale = 0;
  vit_droite = 0.1;
  vit_gauche = 0.102;
  i = 0;
  ENCODER_Reset(RIGHT); 
  ENCODER_Reset(LEFT);

  if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
    arret_chargeur();
  }

  vit_gauche = 0.101595; 
  vit_droite = 0.1;

}

void rotation_horaireRDrecule(float angle){
  float vd = -0.15;
  float vg = 0;
  float angle_encodeur;
  ENCODER_Reset(0); ENCODER_Reset(1);
  angle= angle*31/30; // correction par pratique
  angle_encodeur = -((angle*circonference_cercle/360)*pulse_roue/circonferenceRoue);//conversion de la valeur de l'angle recu en format encodeur

  while(ENCODER_Read(1)>angle_encodeur )
  {
    ENCODER_Read(0);
    ENCODER_Read(1);
    MOTOR_SetSpeed(0, vg);
    MOTOR_SetSpeed(1, vd);
    if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }
  }
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
}

void rotation_horaireRGavance(float angle){
  float vd = 0;
  float vg = 0.15;
  float angle_encodeur;
  ENCODER_Reset(0); ENCODER_Reset(1);
  angle= angle; // correction par pratique
  angle_encodeur = ((angle*circonference_cercle/360)*pulse_roue/circonferenceRoue);//conversion de la valeur de l'angle recu en format encodeur

  while(ENCODER_Read(0)<angle_encodeur )
  {
    ENCODER_Read(0);
    ENCODER_Read(1);
    MOTOR_SetSpeed(0, vg);
    MOTOR_SetSpeed(1, vd);
    if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }
  }
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
}

void rotation_antihoraireRGrecule(float angle){
  float vd = 0;
  float vg = -0.15;
  float angle_encodeur;
  ENCODER_Reset(0); ENCODER_Reset(1);
  angle= angle; // correction par pratique
  angle_encodeur = -((angle*circonference_cercle/360)*pulse_roue/circonferenceRoue);//conversion de la valeur de l'angle recu en format encodeur

  while(ENCODER_Read(0)>angle_encodeur )
  {
    ENCODER_Read(0);
    ENCODER_Read(1);
    MOTOR_SetSpeed(0, vg);
    MOTOR_SetSpeed(1, vd);
    if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }
  }
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);

  vg = 0.101595; 
  vd = 0.1;
}

void rotation_antihoraireRDavance(float angle){
  float vd = 0.15;
  float vg = 0;
  float angle_encodeur;
  ENCODER_Reset(0); ENCODER_Reset(1);
  angle= angle*31/30; // correction par pratique
  angle_encodeur = ((angle*circonference_cercle/360)*pulse_roue/circonferenceRoue);//conversion de la valeur de l'angle recu en format encodeur

  while(ENCODER_Read(1)<angle_encodeur )
  {
    ENCODER_Read(0);
    ENCODER_Read(1);
    MOTOR_SetSpeed(0, vg);
    MOTOR_SetSpeed(1, vd);
    if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }
  }
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
}

void rotation_horaire2R(const double angle){ // est un virage a deux roues
  float vd = -0.1;
  float vg = 0.101959;
  double angle_encodeur, angleC;
  ENCODER_Reset(RIGHT); ENCODER_Reset(LEFT);
  angleC = angle*19/18; // correction par pratique
  angle_encodeur = (angleC*circonference_cercle/360/2)*pulse_roue/circonferenceRoue;//conversion de la valeur de l'angle recu en format encodeur

  while(ENCODER_Read(0)<angle_encodeur)
  {
    ENCODER_Read(0);
    ENCODER_Read(1);
    MOTOR_SetSpeed(0, vg);
    MOTOR_SetSpeed(1, vd);
    if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }
  }
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);

  
  ENCODER_Reset(RIGHT); 
  ENCODER_Reset(LEFT);

  vg = 0.101595; 
  vd = 0.1;

}

void rotation_antihoraire2R(const double angle){ // est un virage a deux roues
  float vd = 0.1;
  float vg = -0.101959;
  double angle_encodeur, angleC;
  ENCODER_Reset(RIGHT); ENCODER_Reset(LEFT);
  angleC = angle*43/45; // correction par pratique

  angle_encodeur = (angleC*circonference_cercle/360/2)*pulse_roue/circonferenceRoue;//conversion de la valeur de l'angle recu en format encodeur

  while(ENCODER_Read(1)<angle_encodeur)
  {
    ENCODER_Read(0);
    ENCODER_Read(1);
    MOTOR_SetSpeed(0, vg);
    MOTOR_SetSpeed(1, vd);
    if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }
  }
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
  vd = 0.1;
  vg = -0.101959;
  if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
    arret_chargeur();
  }
  ENCODER_Reset(RIGHT); ENCODER_Reset(LEFT);
}

////////TOURNER À DROITE
int virage90Dlarge(int chargeur){ // appeler la fonction fait faire un virage de 90 a droite 
  int x = 0;
  while (x<5){
    if (chargeur>0){
      if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
        arret_chargeur();
      }
      asservissementDistanceE(10);
      rotation_horaire2R(82);
      asservissementDistanceE(6.4);
      rotation_horaire2R(82);
      asservissementDistanceE(12);
      rotation_antihoraire2R(85);
      rotation_antihoraire2R(85);
      PlacerDomino();
      x += 1;
      chargeur -= 1;
    }
    if (chargeur ==0){
      arret_chargeur();
      chargeur = 27;
    }
    if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }
  }
  return chargeur;
}

void virage90Glarge(){ // appeler la fonction fait faire un virage de 90 a droite en tournant avec les deux roues
  float vg = 0.08; //vitesse gauche
  float vd = 0.2; //vitesse droite
  float distance_dominos_encodeur=(distance_domino_reel*pulse_roue/circonferenceRoue);
  float nouvelle_distance_dominos_encodeur = distance_dominos_encodeur;
  int encodeurG = 0;
  ENCODER_Reset(0);
  ENCODER_Reset(1);

  while(ENCODER_Read(1)-ENCODER_Read(0)< 3896)
  {
   MOTOR_SetSpeed(0, vg);
   MOTOR_SetSpeed(1, vd);
   ENCODER_Read(0);
   ENCODER_Read(1);
   
   if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }

   do{
     encodeurG=ENCODER_Read(0)-nouvelle_distance_dominos_encodeur;
    }
   while(encodeurG<0);
      if(true){
        MOTOR_SetSpeed(0, 0.1);
        MOTOR_SetSpeed(1, -0.1);
        delay(300);

        MOTOR_SetSpeed(0, 0);
        MOTOR_SetSpeed(1, 0);
        //PlacerDomino();
        delay(100);

        MOTOR_SetSpeed(1, 0.1);
        MOTOR_SetSpeed(0, 0.1);
        delay(100);

        if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
          arret_chargeur();
        }

        Serial.print("encodeur gauche totale : ");
        Serial.println(ENCODER_Read(0));
        Serial.print("encodeur droite totale : ");
        Serial.println(ENCODER_Read(1));
        
        Serial.print("difference encodeur : ");
        Serial.println(ENCODER_Read(1)-ENCODER_Read(0));
        Serial.println(" ");
        nouvelle_distance_dominos_encodeur += distance_dominos_encodeur;
      }
  }
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
}

//////tourner à gauche
int virage90Gserre(int chargeur){
  int x = 1;
  if (x == 1){
    if (chargeur>0){
      ENCODER_ReadReset(RIGHT);
      while (ENCODER_Read(RIGHT) < 808){
        MOTOR_SetSpeed(LEFT, 0.102); MOTOR_SetSpeed(RIGHT, 0.1);
        ENCODER_Read(RIGHT);
        if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
          arret_chargeur();
        }
      }

      rotation_antihoraire2R(90);

      ENCODER_ReadReset(RIGHT);
      while (ENCODER_Read(RIGHT) < 808){
        MOTOR_SetSpeed(LEFT, 0.102); MOTOR_SetSpeed(RIGHT, 0.1);
        ENCODER_Read(RIGHT);
        if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
          arret_chargeur();
        }
      }
      rotation_horaire2R(60);

      PlacerDomino();         ///////////
      chargeur -= 1;
      x  += 1;
    }
    else if (chargeur==0){
      arret_chargeur();
      chargeur = 27;
    }
  }
  if (x == 2){
    if (chargeur>0){
      ENCODER_ReadReset(RIGHT);
      while (ENCODER_Read(RIGHT) < 808){
        MOTOR_SetSpeed(LEFT, 0.102); MOTOR_SetSpeed(RIGHT, 0.1);
        ENCODER_Read(RIGHT);
      }
      rotation_antihoraire2R(90);
      if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
        arret_chargeur();
      }
      ENCODER_ReadReset(RIGHT);
      while (ENCODER_Read(RIGHT) < 808){
        MOTOR_SetSpeed(LEFT, 0.102); MOTOR_SetSpeed(RIGHT, 0.1);
        ENCODER_Read(RIGHT);
        if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
          arret_chargeur();
        }
      }
      rotation_horaire2R(60);

      PlacerDomino();           //////////////
      chargeur -= 1;
      x += 1;
    }
    else if (chargeur==0){
      arret_chargeur();
      chargeur = 27;
    }
  }

  if (x == 3){
    if (chargeur>0){
      ENCODER_ReadReset(RIGHT);
      while (ENCODER_Read(RIGHT) < 808){
        MOTOR_SetSpeed(LEFT, 0.102); MOTOR_SetSpeed(RIGHT, 0.1);
        ENCODER_Read(RIGHT);
        if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
          arret_chargeur();
        }
      }
      rotation_antihoraire2R(90);

      ENCODER_ReadReset(RIGHT);
      while (ENCODER_Read(RIGHT) < 780){
        MOTOR_SetSpeed(LEFT, 0.102); MOTOR_SetSpeed(RIGHT, 0.1);
        ENCODER_Read(RIGHT);
        if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
          arret_chargeur();
        }
      }
      rotation_horaire2R(60);

      PlacerDomino();
      chargeur -= 1;
    }
    else if (chargeur==0){
      arret_chargeur();
      chargeur = 27;
    }
  }
  return chargeur;
}

void spirale(int chargeur){
  PlacerDomino();

  int i = 0;
  int j = 0;
  int k = 0;

  while (k < 27){
    virage90Gserre(chargeur);
    ligne_droite(i, chargeur);
    i++;
    j += i;
    k = 3*i + j;
    if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }
  }

  arret_chargeur();

  PlacerDomino();

  while (k < 53){
    virage90Gserre(chargeur);
    ligne_droite(i, chargeur);
    i++;
    j += i;
    k = 3*i + j;
    if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }
  }
  MOTOR_SetSpeed(RIGHT, 0); 
  MOTOR_SetSpeed(LEFT, 0);

  delay(10000);
}

void asservissementDistance(float distance_a_parcourir){
  float totale_ENCODER_read_right = 0;
  float totale_ENCODER_read_left = 0;
  float dist_roue_droite = 0, dist_roue_gauche = 0;
  float dist_roue_droite_totale = 0, dist_roue_gauche_totale = 0;
  float erreur_droite = 0;
  float erreur_gauche = 0;
  float erreur_droite_totale = 0;
  float erreur_gauche_totale = 0;
  float vit_droite = 0.1;
  float vit_gauche = 0.102;
  const float DISTANCE = (808/20/3200)*circonferenceRoue;
  const int DUREE_DOMINOS_DROP = 500;
  int i = 0;

  const float Kp = 0.000046875; float Ki = Kp/5; //facteurs de correction a modifier selon comportement
  //augmenter si correction trop petite et vice-versa tout en respectant le plus possible la relation Kp = 5Ki
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  
  distance_a_parcourir = ((distance_a_parcourir - 0.867)/1.24);//facteur correctionnel de la distance commandé


  while(dist_roue_droite_totale < distance_a_parcourir)
  {
    delay(40);// Delais pour décharger le CPU

    ENCODER_Reset(RIGHT); ENCODER_Reset(LEFT);

    MOTOR_SetSpeed(RIGHT, vit_droite); MOTOR_SetSpeed(LEFT, vit_gauche);
    delay(25);

    totale_ENCODER_read_right += ENCODER_Read(RIGHT); totale_ENCODER_read_left += ENCODER_Read(LEFT);
      
    dist_roue_droite = (circonferenceRoue/3200)*ENCODER_Read(RIGHT);
    dist_roue_gauche = (circonferenceRoue/3200)*ENCODER_Read(LEFT);

    dist_roue_droite_totale = (circonferenceRoue/3200)*totale_ENCODER_read_right;
    dist_roue_gauche_totale = (circonferenceRoue/3200)*totale_ENCODER_read_left;

    erreur_droite = DISTANCE - dist_roue_droite;
    erreur_gauche = DISTANCE - dist_roue_gauche;
    erreur_droite_totale = DISTANCE*i - dist_roue_droite_totale;
    erreur_gauche_totale = DISTANCE*i - dist_roue_gauche_totale;

    vit_droite *=(1 + (erreur_droite*Kp + erreur_droite_totale*Ki));
    vit_gauche *= (1 + (erreur_gauche*Kp + erreur_gauche_totale*Ki));

    i++;
    if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }
  }
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);

  ENCODER_Reset(RIGHT); 
  ENCODER_Reset(LEFT);
}


int spirale_carre(int chargeur){
  delay(1000);
  int nombre_dominos=8;
  while (nombre_dominos<=300 || nombre_dominos*distance_dominos_reelle<100){
    Serial.println(nombre_dominos);
    chargeur = ligne_droite(nombre_dominos,chargeur);
    chargeur = virage90Gserre(chargeur);  
    chargeur = ligne_droite(nombre_dominos,chargeur);
    chargeur = virage90Gserre(chargeur);
    nombre_dominos= nombre_dominos+3;
    if (digitalRead(bouton_go) == 1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
    }
  }
  return chargeur;
}

void flash_go(int del){        //ca fait juste flasher la del pour avertir l'utilisateur que le robot va avancer
  digitalWrite(del, HIGH);
  delay(400);
  digitalWrite(del, LOW);
  delay(200);
}

void parcours(int deplacement[5]){    // fonction qui fait le parcours inventé par l'utilisateur
  int x = 0;
  int chargeur = 27; // nombre de dominos dans le chargeur
  while (x<5){        // un maximum de 5 déplacements de suite
    if (deplacement[x] == 0)
      x=5;
    if (deplacement[x] == del_ligne){
      chargeur = ligne_droite(10, chargeur);
      Serial.println(chargeur);
    }
    if (deplacement[x] == del_droite){
      chargeur = virage90Dlarge(chargeur);
    }
    if (deplacement[x] == del_gauche){
      chargeur = virage90Gserre(chargeur);
    }
    if (deplacement[x] == del_spirale && chargeur == 27){
      spirale(chargeur);
    }
    if (deplacement[x] == del_carre){
      chargeur = spirale_carre(chargeur);
    }
    if (digitalRead(bouton_go) ==1 || digitalRead(bouton_ligne) ==1 || digitalRead(bouton_droite) ==1 || digitalRead(bouton_gauche) ==1 || digitalRead(bouton_spirale) ==1 || digitalRead(bouton_Scarree) ==1){
      arret_chargeur();
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
  int deplacement[5]={0,0,0,0,0};   //pour garder en note le parcours choisi
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
              flash_go(deplacement[y]);    //allume les dels en ordre de leurs choix
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

