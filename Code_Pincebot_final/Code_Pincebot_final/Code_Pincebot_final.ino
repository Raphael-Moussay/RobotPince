/*/
/****** AUTOMATIC CONTROL - POLYTECH SORBONNE - JAN 2025 ****/
/******** AUTHORS : L. BRIETZKE & G. MOREL *******/
/********* Sequence 1 - Problem 1 **********/
/********** YOU'RE IN CONTROL **********/


// Libraries for CAN communications
#include <Servo.h>
#include <can-serial.h>
#include <mcp2515_can.h>
#include <mcp2515_can_dfs.h>
#include <mcp_can.h>
#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
const int SPI_CS_PIN = BCM8;
const int CAN_INT_PIN = BCM25;
#else
const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;
#endif

#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN);  // Set CS pin
#define MAX_DATA_SIZE 8

#include <SPI.h>
#include <math.h>

#define MY_PI 3.14159265359

#define PERIOD_IN_MICROS 25000 // 5 ms

// Identifiants des moteurs (en hexadécimal)
#define MOTOR_1_ID 0x01
#define MOTOR_2_ID 0x02  //moteur droite, vitesse negative pour avancer
#define MOTOR_3_ID 0x03  //  moteur gauche, vitesse positive pour avancer. 


/********* Variables globales. *********/
double relativeMotorPosEncoder[3]={0.0,0.0,0.0};

int offsetMotorPosEncoder[3] = {0,0,0}; // Offset de la position de l'encodeur
int currentNumOfMotorRevol[3]= {0,0,0}; // Nombre actuel de révolutions du moteur
double previousMotorPosDeg[3] ={0.0,0.0,0.0}; // Position précédente du moteur en degrés

// Tableaux pour stocker la vitesse et la position des moteurs
double currentMotorVelDegPerSec[3] = {0.0, 0.0, 0.0}; // Vitesse en degrés par seconde
double currentMotorPosDeg[3] = {0.0, 0.0, 0.0}; // Position en degrés
double currentMotorPosEncoder[3]= {0.0, 0.0, 0.0};
double currentMotorVelRaw[3] = {0.0, 0.0, 0.0};

// ==== Variables d'odométrie ====
double x = 0.0, y = 0.0, theta = 0.0;                // pose en mm et rad
const double Diametre_Roue_MM = 49.5;               // diamètre / mm
const double Dist_Roue_MM     = 177.0;               // entraxe roues / mm

// mémorisation des positions précédentes (en degrés) des deux roues
double prevMotorPosL = 0.0;  // pour MOTOR_3_ID (roue gauche)
double prevMotorPosR = 0.0;  // pour MOTOR_2_ID (roue droite)

// ——— En haut du fichier, avec tes autres constantes ———
const int SIGNE_MOTOR2 = -1;  // Moteur 2 = roue droite, signe – pour avancer
const int SIGNE_MOTOR3 = +1;  // Moteur 3 = roue gauche, signe + pour avancer

const int pin_trigavant = 4;
const int pin_echoavant = 3;

const int pin_led = 13;
const int pin_bouton = 2;
int run=0;
bool last_bouton = HIGH;

long duration=0;
float distanceCM=0;
float distancecoteCM=0;

// balise trouve
float prev_distanceCM=0;
float prev_distancecoteCM=0;
int chrono=0;
int chronoMIN=0;
int min=30000;


const int pin_trigcote = 7;
const int pin_echocote = 6;


const int pin_pince = 5;

int etat=0;
int temp_distancecote=0;

int gripperPos = 0;     
int feedbackVal = 0;   

int i=0;

float distance_cible = 0;


Servo servopince;

int vitesse=20000;
/********* Commandes moteur *********/

// Allumer un moteur
void motorON(uint8_t motor_id) {
  unsigned char msg[MAX_DATA_SIZE] = {0x88, 0, 0, 0, 0, 0, 0, 0};
  CAN.sendMsgBuf(0x140 + motor_id, 0, 8, msg);
}

// Éteindre un moteur
void motorOFF(uint8_t motor_id) {
  unsigned char msg[MAX_DATA_SIZE] = {0x80, 0, 0, 0, 0, 0, 0, 0};
  CAN.sendMsgBuf(0x140 + motor_id, 0, 8, msg);
}

void sendVelocityCommand(uint8_t motor_id, long int vel) {

  long int local_velocity;
  local_velocity = vel;

  unsigned char *adresse_low = (unsigned char *)(&local_velocity);

  unsigned char msg[MAX_DATA_SIZE] = {
    0xA2,
    0x00,
    0x00,
    0x00,
    *(adresse_low),
    *(adresse_low + 1),
    *(adresse_low + 2),
    *(adresse_low + 3)
  };

  CAN.sendMsgBuf(0x140 + motor_id, 0, 8, msg); 
}

int readMotorState(int motorID) {
    /*
    Récupère l'état du moteur identifié via les mesures de l'encodeur, et met à jour la valeur des variables globales suivantes :
    - currentNumOfMotorRevol : Nombre de révolutions du moteur (valeur précédente +/- 1 selon si une révolution à été observé par l'encodeur)
    - currentArmMotorPosDeg : Position du moteur en degrés, calculée à partir des valeurs brutes de l'encodeur
    - current(Left/Right/Arm)MotorVel : Vitesse du moteur, en degrés par secondes
    - previousArmMotorPosDeg : Position actuelle, en degrés (car la position actuelle deviendra la précédente au prochain appel)
    */
  
    uint32_t id;
    uint8_t type;
    uint8_t len;
    byte cdata[MAX_DATA_SIZE] = {0}; // Déclare et remplit le tableau de 0, buffer utilisé pour receptionner les données
    int data2_3, data4_5, data6_7;
    int rawMotorVel;
    int absoluteMotorPosEncoder;

    int timeout=0;

    // Attend de réceptionner des données
    while (CAN_MSGAVAIL != CAN.checkReceive() && timeout < 5000) {
      delayMicroseconds(1000);
      timeout++;
    }
    if (timeout > 5000) return -1;
    // Lis les données, len: data length, buf: data buf
    CAN.readMsgBuf(&len, cdata); // Écrit les valeurs du message transmis par le bus (données) CAN dans le buffer cdata
    id = CAN.getCanId(); // Récupère la valeur de l'ID du bus CAN depuis lequel les données sont reçues
    type = (CAN.isExtendedFrame() << 0) | (CAN.isRemoteRequest() << 1);


    if ((id - 0x140) == motorID) { // Si l'ID reçu correspond à celui du moteur
      data4_5 = cdata[4] + 256 * cdata[5];
      rawMotorVel = (int)data4_5; // Calcul la vitesse brute
      data6_7 = cdata[6] + 256 * cdata[7];
      absoluteMotorPosEncoder = (int)data6_7;
    }
  
    // Convertit la vitesse brute en degrés par secondes
    currentMotorVelDegPerSec[motorID - 1] = (double)rawMotorVel;
    
    relativeMotorPosEncoder[motorID - 1] = (double)absoluteMotorPosEncoder;
  
    // Déduction de la position en degré à partir de l'offset, du nombre de révolutions, et de la valeur brute en unité encodeur
    relativeMotorPosEncoder[motorID - 1] -= offsetMotorPosEncoder[motorID - 1]; // On adapte la position en fonction du décalage introduit initialement (position de départ)
    currentMotorPosDeg[motorID - 1] = 180.0 + ((double)relativeMotorPosEncoder[motorID - 1]) * (180.0 / 32768.0);  // Met à jour la variable globale
  
    double delta = currentMotorPosDeg[motorID - 1] - previousMotorPosDeg[motorID - 1];
    if (delta > 180.0) {
        currentNumOfMotorRevol[motorID - 1]--;
    } else if (delta < -180.0) {
        currentNumOfMotorRevol[motorID - 1]++;
    }
  
    // Affecte à la position précédente la valeur de la position courante pour le prochain appel
    previousMotorPosDeg[motorID - 1] = currentMotorPosDeg[motorID - 1]; // writing in the global variable for next call

    return 0;
}

/********* Initialisation *********/

void setup() {



  pinMode(pin_trigavant, OUTPUT); 
  pinMode(pin_echoavant, INPUT);
  pinMode(pin_trigcote, OUTPUT); 
  pinMode(pin_echocote, INPUT);
  
  //Bouton poussoir
  pinMode(pin_bouton,INPUT);
  pinMode(pin_led,OUTPUT);

  Serial.begin(115200);
  delay(1000);
  servopince.attach(pin_pince);  
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {
    Serial.println("CAN init fail, retry...");
    delay(500);
  }

  Serial.println("\n=== Initialisation CAN OK ===");

 
  // Initialisation des trois moteurs
  motorOFF(MOTOR_1_ID);  // Éteindre moteur 1
  delay(500);
  motorON(MOTOR_1_ID);   // Allumer moteur 1
  delay(500);

  motorOFF(MOTOR_2_ID);  // Éteindre moteur 2
  delay(500);
  motorON(MOTOR_2_ID);   // Allumer moteur 2
  delay(500);

  motorOFF(MOTOR_3_ID);  // Éteindre moteur 3
  delay(500);
  motorON(MOTOR_3_ID);   // Allumer moteur 3
  delay(500);

  //Démarrage à vitesse nulle pour sécurité
  velocity(MOTOR_1_ID, 0);

  velocity(MOTOR_2_ID, 0);

  velocity(MOTOR_3_ID, 0);


   // 2) Stocker ces positions comme références « prev »
  prevMotorPosL = currentMotorPosDeg[MOTOR_3_ID - 1];
  prevMotorPosR = currentMotorPosDeg[MOTOR_2_ID - 1];


  Serial.println("=== Odometrie initialisé ===");

  Serial.println("=== Tous les moteurs prêts ===");


}

void velocity(uint8_t motor_id, long int vel){
    sendVelocityCommand(motor_id,vel);
    readMotorState(motor_id);
}

void capteur_avant(){
  digitalWrite(pin_trigavant, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_trigavant, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trigavant, LOW);

  // Lecture de la durée de l'impulsion ECHO
  duration = pulseIn(pin_echoavant, HIGH, 10000);

  // Calcul de la distance (vitesse du son ≈ 343 m/s)
  distanceCM = (duration * 0.0343) / 2;

  // Affichage de la distance
  // Serial.print("Distance: ");
  // Serial.print(distanceCM);
  // Serial.println(" cm");


}

void capteur_cote(){
  digitalWrite(pin_trigcote, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_trigcote, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trigcote, LOW);


  duration = pulseIn(pin_echocote, HIGH, 10000);


  distancecoteCM = (duration * 0.0343) / 2;

  // Serial.print("Distance cote: ");
  // Serial.print(distancecoteCM);
  // Serial.println(" cm"); 
}

void ouvrir_la_pince(){
  servopince.write(0);  
  // Serial.print("0 degrées: "); //Pince ouverte
  delay(1000);
  // Serial.println(analogRead(A0));
}

void fermer_la_pince(){
  servopince.write(120);  
  // Serial.print("180 degrées: "); //Pince ouverte
  delay(1000);
  // Serial.println(analogRead(A0));
}

void avancer(){
  velocity(MOTOR_2_ID, SIGNE_MOTOR2*vitesse);
  velocity(MOTOR_3_ID, SIGNE_MOTOR3*vitesse);
}

void reculer(){
  velocity(MOTOR_2_ID, -SIGNE_MOTOR2*vitesse);
  velocity(MOTOR_3_ID, -SIGNE_MOTOR3*vitesse);
}

void stop1(){
  velocity(MOTOR_1_ID,0);
}

void stop(){
  velocity(MOTOR_3_ID,0);
  velocity(MOTOR_2_ID,0);
}

void tourner_90_gauche(){

  velocity(MOTOR_2_ID, SIGNE_MOTOR2*vitesse);
  velocity(MOTOR_3_ID, -SIGNE_MOTOR3*vitesse);
  delay(1400);// delay pour tourner de 90
  // delay(1400);// delay pour tourner de 70
  stop();
}

void attraper(){
  stop();
  stop1();
  ouvrir_la_pince();
  //reculer
  reculer();
  //delay et stop
  delay(1000);
  stop();
  stop1();
  velocity(MOTOR_1_ID,vitesse);
  delay(2000);
  fermer_la_pince();
  delay(50);
  velocity(MOTOR_1_ID,-vitesse);
  delay(2000);
}

void descendre_la_pince(){
  velocity(MOTOR_1_ID,-vitesse);
  delay(1000);
}

bool balise_trouve_cote (){
  capteur_cote();
  if (abs(prev_distancecoteCM-distancecoteCM) > 10) {
    Serial.println("Balise trouve cote");
    return true;
  } else {
    return false;
  }
}

void avancer_mur() {
  prev_distancecoteCM = distancecoteCM;  // sauvegarde de la distance précédente
  capteur_cote();  // met à jour distancecoteCM
  
  float erreur = distancecoteCM - distance_cible;

  // Gain proportionnel — ajuste-le pour des résultats plus ou moins agressifs
  float Kp = 100.0;

  // Calcul correction
  int correction = Kp * erreur;

  // Saturation pour ne pas dépasser les valeurs admissibles
  correction = constrain(correction, -20000, 20000);

  // Vitesse de base
  int base_speed = 20000;

  // Ajustement moteur (en supposant que M2 est gauche, M3 est droite)
  velocity(MOTOR_2_ID, SIGNE_MOTOR2 * (base_speed - correction));
  velocity(MOTOR_3_ID, SIGNE_MOTOR3 * (base_speed + correction));

  if(prev_distancecoteCM!=0 && distancecoteCM!=0){
    float ecart = distancecoteCM - prev_distancecoteCM;
    if (ecart > 7 && (etat==1 || etat==8 || etat==10)) {
      etat=etat+1;
      Serial.println("Balise détectée !");
    }
  }

}


bool balise_trouve_avant (){
  prev_distanceCM = distanceCM;
  capteur_avant();
  if (distanceCM < 10) {
    Serial.println("Balise trouve avant");
    return true;
  } else {
    return false;
  }
}

void bras_position(float consigne_pos_bras){
  // readMotorState(MOTOR_1_ID);
  float epsilon = 3;
  float erreur = currentMotorPosDeg[MOTOR_1_ID-1] - consigne_pos_bras;
  if(erreur>epsilon) 
  {
    velocity(MOTOR_1_ID,-vitesse);
  }else if(erreur<(-epsilon)){
    velocity(MOTOR_1_ID,vitesse);
  }
}

int balayage() {
    velocity(MOTOR_2_ID, SIGNE_MOTOR2*1000);
    delay(5);
    prev_distanceCM = distanceCM;
    capteur_avant();
    // Serial.print("prev_distanceCM");
    // Serial.println(prev_distanceCM);
    // Serial.print("distanceCM");
    // Serial.println(distanceCM);
    if(prev_distanceCM!=0 && distanceCM!=0){
      float ecart = abs(prev_distanceCM-distanceCM);
      if(ecart>20){
        return true;
      }else{
        return false;
      }
    }else{
      return false;
    }
    // chrono+=5;
    // capteur_avant();
    // if (distanceCM<min && distanceCM>1)
    // {
    //   min=distanceCM;
    //   chronoMIN=chrono;
    // }
  }
  // velocity(MOTOR_2_ID, -SIGNE_MOTOR2*5000);
  // delay(3000-chronoMIN);
  // stop();


int capteur_1fois=0;

void attrape_objet_petit(){
  Serial.println("attrape");
  stop();

  //remonte et ouvre la pince
  ouvrir_la_pince();
  sendVelocityCommand(MOTOR_1_ID, -vitesse-10000);
  delay(385);//remonte

  //avance attrape et remonte
  stop1();
  avancer();
  delay(1400); 
  stop();
  fermer_la_pince();
  Serial.print("Pince ferme");
  sendVelocityCommand(MOTOR_1_ID, vitesse+20000);
  delay(5000);
}

void ungrab(){
  sendVelocityCommand(MOTOR_1_ID, vitesse);
  delay(100);
  stop();
  ouvrir_la_pince();
  reculer();
  delay(1000);
  stop();
}

void loop() {
  int etat_bouton = digitalRead(pin_bouton);
  // Serial.println(etat_bouton);
  
  if ((etat_bouton == HIGH) && (last_bouton == LOW)) {
    etat = (etat == 0) ? 1 : 0;
    Serial.println("Changement d'état !");
    delay(50); // anti-rebond
    digitalWrite(pin_led,HIGH);
    capteur_cote();
    distance_cible = distancecoteCM;
    Serial.println(distanceCM);
  }else{
    digitalWrite(pin_led,LOW);
  }
  
  last_bouton = etat_bouton;
  // if(etat==1){
  //   if(balayage()){
  //       delay(100);
  //         stop();
  //         etat=0;
  //   }
  // }
  switch (etat) {
    case 0 :
      stop();
      stop1();
      prev_distanceCM = distanceCM;
      prev_distancecoteCM = distancecoteCM;
      capteur_avant();
      capteur_cote();
      Serial.println(distancecoteCM);
      break;
    case 1 :

  //   attrape_objet_petit();
  //     stop();
  //     stop1();
  //     tourner_90_gauche();
  //     etat = etat+1;
  //     break;
  // }

      avancer_mur();
      break;
    case 2 :
      avancer();
      delay(1300);
      tourner_90_gauche();
      stop();
      avancer();
      delay(800);
      stop();
      etat = 3;
      break;
    case 3 :
      velocity(MOTOR_1_ID,20000);
      delay(3000);
      etat=4;
      break;
    case 4 : 
      capteur_avant();
      prev_distanceCM = distanceCM;
      if(balayage()){
        velocity(MOTOR_2_ID, SIGNE_MOTOR2*1000);
        delay(1450);
        stop();
        etat=5;
      }
      break;
    case 5 :  
      avancer();
      if (balise_trouve_avant())
      {
        stop();
        delay(2000);
        etat=6;
      }
      break;
    case 6 :  
      attrape_objet_petit();
      stop();
      stop1();
      etat=etat+1;
      break;
    case 7 :
      avancer();
      delay(1200);
      stop();
      velocity(MOTOR_2_ID, SIGNE_MOTOR2*vitesse);
      velocity(MOTOR_3_ID, -SIGNE_MOTOR3*vitesse);
      delay(1600);// delay pour tourner de 90
      // delay(1400);// delay pour tourner de 70
      stop();
      delay(500);
      capteur_cote();
      distance_cible = distancecoteCM;
      etat= etat+1;
      break;
    case 8 :
      avancer_mur();
      break;
    case 9 :
      stop();
      avancer();
      delay(1200);
      stop();
      velocity(MOTOR_2_ID, SIGNE_MOTOR2*vitesse);
      velocity(MOTOR_3_ID, -SIGNE_MOTOR3*vitesse);
      delay(1600);// delay pour tourner de 90
      stop();
      delay(500);
      capteur_cote();
      distance_cible = distancecoteCM;
      etat=etat+1;
      break;
    case 10 : 
      avancer_mur();
      break;
    case 11 : 
      stop();
      reculer();
      delay(2000);
      ungrab();
      etat=etat+1;
      break;
    case 12:
      stop();
      break;
  }

  
}