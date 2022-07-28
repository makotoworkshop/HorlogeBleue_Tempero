#include <Wire.h>       // Communication avec le RTC
#include "Chronodot.h"  // Fonctions du Chronodot
#include "DHT.h"  // librairie Adafruit

/****************/
/* Déclarations */
/****************/
#define CHRONODOT_ADDRESS 0x68

 //——— DHT Sensors ———//
#define DHTinPIN A2     // what digital pin the DHT22 is conected to
#define DHTTYPE DHT11   // there are multiple kinds of DHT sensors
DHT dht_in(DHTinPIN, DHTTYPE);

/****** Signaux ******/
#define datapin 10   // pin 10 pour les données vers les registres à décallages
#define clockpin 11  // pin 11 pour l'horloge qui coordonne les registres à décallages
#define latchpin 8   // pin 8 pour le déclencheur des registres à décallages


int PIR = 5;              // Define the Sensor Connected to Pin 5
int ValeurPIR = 0;                 // Store the value of sensor
unsigned long temps;


/****** Boutons poussoir ******/
#define BoutonMinutes 12    // pin 12 pour le bouton poussoir d'incrément des minutes
#define BoutonHeures 13     // pin 13 pour le bouton poussoir d'incrément des heures
#define BoutonTempHum 2     // pin 6 pour le bouton poussoir Alarme1
#define BoutonCaroussel 6     // pin 2 alias aussi interruption 0 pour le bouton poussoir Alarme2
#define BoutonHorloge 7     // pin 7 pour le bouton poussoir Horloge
#define Radio A3            // 

int EtatBoutonMinutes = 1;  // variable pour stocker l'état du bouton aprés sa lecture
int EtatBoutonHeures = 1;   // variable pour stocker l'état du bouton aprés sa lecture
int EtatBoutonTempHum = 1;
int EtatBoutonCaroussel = 1;
int EtatBoutonHorloge = 1;
int EtatBoutonSupp = 1;

/****** Alimentation des 7 segments en PWM ******/
#define ledPin 9                    // pin 9 pour le signal PWM de cadencement des afficheurs et leds
#define dotledPin 3
#define photodiodepin A0             // pin A0 pour la photodiode
unsigned int ValeurPhotoDiode = 0;  // variable pour stocker l'état de la photodiode aprés sa lecture

/****** Variables temporelles ******/
unsigned int heuresHorloge;    // pour stocker les heures
unsigned int heuresHorlogeINC; // pour stocker l'incrément des heures
unsigned int minutesHorloge;   // pour stocker les minutes
unsigned int minutesHorlogeINC;// pour stocker l'incrément des minutes
unsigned int secondes;  // pour stocker les secondes
unsigned int annee;     // pour stocker l'annee
unsigned int mois;      // pour stocker le mois
unsigned int jour;      // pour stocker le jour

Chronodot HORLOGE;      // HORLOGE, nom choisis arbitrairement, référence à la librairie Chronodot

/****** Variables d'affichage ******/ 
int dizH = 0;          // pour stocker les dizaine des heures
int unitH = 0;         // pour stocker les unités des heures
int dizM = 0;          // pour stocker les dizaine des minutes
int unitM = 0;         // pour stocker les unités des minutes
byte AfficheurUN;      // données binaire pour l'afficheur UN, situé le plus à droite
byte AfficheurDEUX;    // données binaire pour l'afficheur DEUX
byte AfficheurTROIS;   // données binaire pour l'afficheur TROIS
byte AfficheurQUATRE;  // données binaire pour l'afficheur QUATRE, situé le plus à gauche
byte dataArray[12];    // Tableau de données


float temperature_in;
float humidite_in;
const long intervalA = 12000;  // 1000 = 1 seconde
const long intervalB = 1000;
const long intervalC = 2000;
unsigned long previousMillis = 0;   // Stores last time temperature was published
unsigned long previousMillis2 = 0;   // Stores last time temperature was published
int mode = 0;
int breaker = 0;
int lum;
int marker = 0;

/*******************/
/* Initialisations */
/*******************/
void setup ()      // Fonction d'initialisation obligatoire
{
//  Serial.begin(9600);           // uncomment to debug
  Serial.println("Initializing Chronodot.");           // uncomment to debug
  
//——— DHT Sensors ———//
  dht_in.begin();

  dataArray[0] = B00000011;  // Case du tableau qui contient la valeur binaire pour afficher zero sur un afficheur 7 segments
  dataArray[1] = B10011111;  // Case du tableau qui contient la valeur binaire pour afficher un sur un afficheur 7 segments
  dataArray[2] = B00100101;  // Case du tableau qui contient la valeur binaire pour afficher deux sur un afficheur 7 segments
  dataArray[3] = B00001101;  // Case du tableau qui contient la valeur binaire pour afficher trois sur un afficheur 7 segments
  dataArray[4] = B10011001;  // Case du tableau qui contient la valeur binaire pour afficher quatre sur un afficheur 7 segments
  dataArray[5] = B01001001;  // Case du tableau qui contient la valeur binaire pour afficher cinq sur un afficheur 7 segments
  dataArray[6] = B01000001;  // Case du tableau qui contient la valeur binaire pour afficher six sur un afficheur 7 segments
  dataArray[7] = B00011111;  // Case du tableau qui contient la valeur binaire pour afficher sept sur un afficheur 7 segments
  dataArray[8] = B00000001;  // Case du tableau qui contient la valeur binaire pour afficher huit sur un afficheur 7 segments 
  dataArray[9] = B00001001;  // Case du tableau qui contient la valeur binaire pour afficher neuf sur un afficheur 7 segments
  dataArray[10] = B00111001;  // Case du tableau qui contient la valeur binaire pour afficher neuf sur un afficheur ° segments
  dataArray[11] = B10010001;  // H
  
// afficheurs inversés, DP et f valeurs échangés
  
//  dataArray[0] = B00000011;  // Case du tableau qui contient la valeur binaire pour afficher zero sur un afficheur 7 segments
//  dataArray[1] = B10011111;  // Case du tableau qui contient la valeur binaire pour afficher un sur un afficheur 7 segments
//  dataArray[2] = B00100110;  // Case du tableau qui contient la valeur binaire pour afficher deux sur un afficheur 7 segments
//  dataArray[3] = B00001110;  // Case du tableau qui contient la valeur binaire pour afficher trois sur un afficheur 7 segments
//  dataArray[4] = B10011010;  // Case du tableau qui contient la valeur binaire pour afficher quatre sur un afficheur 7 segments
//  dataArray[5] = B01001010;  // Case du tableau qui contient la valeur binaire pour afficher cinq sur un afficheur 7 segments
//  dataArray[6] = B01000010;  // Case du tableau qui contient la valeur binaire pour afficher six sur un afficheur 7 segments
//  dataArray[7] = B00011111;  // Case du tableau qui contient la valeur binaire pour afficher sept sur un afficheur 7 segments
//  dataArray[8] = B00000010;  // Case du tableau qui contient la valeur binaire pour afficher huit sur un afficheur 7 segments 
//  dataArray[9] = B00001010;  // Case du tableau qui contient la valeur binaire pour afficher neuf sur un afficheur 7 segments

  pinMode(clockpin, OUTPUT);     // pin correspondant à "clockpin" initialisée en sortie
  pinMode(datapin, OUTPUT);      // pin correspondant à "datakpin" initialisée en sortie
  pinMode(latchpin, OUTPUT);     // pin correspondant à "latchpin" initialisée en sortie
  pinMode(BoutonMinutes, INPUT); // pin correspondant à "BoutonMinutes" initialisée en entrée 
  pinMode(BoutonHeures, INPUT);  // pin correspondant à "BoutonHeures" initialisée en entrée  
  pinMode(BoutonCaroussel, INPUT); // pin correspondant à "BoutonHeures" initialisée en entrée  
  pinMode(BoutonTempHum, INPUT); // pin correspondant à "BoutonHeures" initialisée en entrée   
  pinMode(BoutonHorloge, INPUT); // pin correspondant à "BoutonHeures" initialisée en entrée
   
  pinMode(PIR, INPUT);    // initialize the sensor as the input
//  attachInterrupt(0, Fonction_Breaker, FALLING); // attache l'interruption externe n°0 (pin2 soit bouton BoutonTempHum à la fonction stop2
   
  Wire.begin();     // initialisation du chronodot, référence à la librairie wire
  HORLOGE.begin();  // initialisation du chronodot, référence à la librairie Chronodot

//************ MISE à L'heure manuelle du module RTC DS3231 ************
//  HORLOGE.adjust(DateTime(2014,12,25,10,30,12));  // années, mois, jour, heures, minutes, secondes
//  RADIO.adjust(AlarmTime1(6,30,0));
//  REVEIL.adjust(AlarmTime2(7,30));  
}

int x=0;

/*************/
/* Programme */
/*************/
void loop ()      // boucle du programme !
{
  SurveilleBouton_TempHum();
  SurveilleBouton_Caroussel();
  SurveilleBouton_Horloge_Heures_Minutes();
  sensors();
  horloge();                // appelle la fct "horloge", récupération des données temporelles
  
  ValeurPIR = digitalRead(PIR);
  if (ValeurPIR == HIGH) {
    marker = 1;
  }

  if (marker == 1) { // si on a mémorisé que le PIR a été activé
    Mode_par_defaut();
    AjusteLuminosite();
    while ( ((millis() - temps) > 10800000) ) {  // 3h (180 min x 1000 X 60)
      temps = millis(); //on stocke la nouvelle heure
      digitalWrite(ledPin, LOW);      // Turning off the LEDs
      digitalWrite(dotledPin, LOW);   // Turning off the LEDs
      Serial.println("dodo !");
      Serial.println("dodo !");
      Serial.println("dodo !");
      Serial.println("dodo !");
      marker = 0;
    }  
  }
  
  Serial.print(ValeurPIR);
  Serial.print("     "); 
  Serial.print(mode);
  Serial.print("     "); 
  Serial.print(marker);
  Serial.print("     "); 
  Serial.print(temps);
  Serial.print("     "); 
  Serial.println(x++);
}


void Mode_par_defaut(void) {
  switch (mode) {
    case 0:
      Mode_caroussel();
      break;
    case 1:
      Mode_horloge_permanent();
      break;   
    case 2:
      Mode_tempero_permanent();
      break;
    case 3:
      Mode_humido_permanent();
      break;
//    case 4:
//      Mode_Tempero_Humido();
//      break;
  }
}

void Mode_horloge_permanent(void) {
   AfficheHorloge();
}

void Mode_tempero_permanent(void) {
  AfficheTempero();
}

void Mode_humido_permanent(void) {
  AfficheHumido();
}

void Mode_Tempero_Humido(void) {
  do {
    for(int i = 0; i <= 4000; i++) {  // ne sort pas de la boucle pour maintenir l'affichage tant qu'on décompte
      AfficheTempero();  // affiche la T° durant n secondes, temps de comptage pris sur l'intervalle
    }
    for(int i = 0; i <= 4000; i++) {  // ne sort pas de la boucle pour maintenir l'affichage tant qu'on décompte
      AfficheHumido();  // affiche la T° durant n secondes, temps de comptage pris sur l'intervalle
    }
  }  while (breaker == 1);
}

void Mode_caroussel(void) {
//  Serial.println(mode);
  unsigned long currentMillis = millis();
  // Every X number of seconds (interval = n seconds) 
  if (currentMillis - previousMillis >= intervalA) {
      // Save the last time a new reading was published
    previousMillis = currentMillis;
    analogWrite(dotledPin, LOW); 
    for(int i = 0; i <= 3500; i++) {  // ne sort pas de la boucle pour maintenir l'affichage tant qu'on décompte
      AfficheTempero();  // affiche la T° durant n secondes, temps de comptage pris sur l'intervalle
    }
    for(int i = 0; i <= 2000; i++) {  // ne sort pas de la boucle pour maintenir l'affichage tant qu'on décompte
      AfficheHumido();  // affiche la T° durant n secondes, temps de comptage pris sur l'intervalle
    }
    analogWrite(dotledPin, ValeurPhotoDiode);    
  }
  else {
    AfficheHorloge();
  }
}

void sensors() {
  unsigned long currentMillis2 = millis();
    // Every X number of seconds (interval = 2 seconds) 
  if (currentMillis2 - previousMillis2 >= intervalC) {
      // Save the last time a new reading was published
    previousMillis2 = currentMillis2;
    
      // New DHT sensor readings
    humidite_in = dht_in.readHumidity();
      // Read temperature as Celsius (the default)
    temperature_in = dht_in.readTemperature();
  
//    // Check if any reads failed and exit early (to try again).
//    if (isnan(humidite_in) || isnan(temperature_in)) {
//      Serial.println(F("Failed to read from DHT sensor IN !"));
//   //   return;
//    }
//    else {
//       // Printing the results on the serial monitor
//      Serial.print("Temperature IN = ");
//      Serial.print(temperature_in);
//      Serial.print(" °C ");
//      Serial.print("    Humidity IN = ");
//      Serial.print(humidite_in);
//      Serial.println(" % ");
//    }
  }
}

void AfficheTempero() {
//  AjusteLuminosite2();
  
  int temperature = temperature_in*100; // pour passer d’un flotant à un entier
  int millieme = temperature / 1000;
  int centieme = (temperature % 1000) /100;
  int dixieme = (temperature % 100) /10;
//  Serial.print(millieme); //
//  Serial.print(" millieme "); //
//  Serial.print(centieme); //
//  Serial.print(" centieme "); //
//  Serial.print(dixieme); //
//  Serial.print(" dixieme ");   //
//  Serial.println();  //

  AfficheurQUATRE = dataArray[millieme];    // stocke la valeur binaire 7 segments de l'unité de minutes dans "AfficheurUN"
  AfficheurTROIS = dataArray[centieme];   // stocke la valeur binaire 7 segments de la dizaine de minutes dans "AfficheurDEUX"
  AfficheurDEUX = dataArray[dixieme]; // stocke la valeur binaire 7 segments de l'unité d'heures dans "AfficheurTROIS"
  AfficheurUN = dataArray[10]; // stocke la valeur binaire 7 segments de la dizaine d'heures dans "AfficheurQUATRE"

  digitalWrite(latchpin, 1);                              // latch à l'état HAUT pour autoriser le transfert des données série   
  shiftOut(datapin, clockpin, LSBFIRST, AfficheurUN);     // envoi l'unités minute au registre à décallage
  shiftOut(datapin, clockpin, LSBFIRST, AfficheurDEUX);   // envoi la dizaine minute au registre à décallage
  shiftOut(datapin, clockpin, LSBFIRST, AfficheurTROIS);  // envoi l'unités heure au registre à décallage
  shiftOut(datapin, clockpin, LSBFIRST, AfficheurQUATRE); // envoi la dizaine heure au registre à décallage  
  digitalWrite(latchpin, 0);                              // latch à l'état BAS pour arreter le transfert des données série  
}

void AfficheHumido() {
//  AjusteLuminosite2();
  
  int humidite = humidite_in*100; // pour passer d’un flotant à un entier
  int millieme = humidite / 1000;
  int centieme = (humidite % 1000) /100;
  int dixieme = (humidite % 100) /10;
//  Serial.print(millieme); //
//  Serial.print(" millieme "); //
//  Serial.print(centieme); //
//  Serial.print(" centieme "); //
//  Serial.print(dixieme); //
//  Serial.print(" dixieme ");   //
//  Serial.println();  //

  AfficheurQUATRE = dataArray[millieme];    // stocke la valeur binaire 7 segments de l'unité de minutes dans "AfficheurUN"
  AfficheurTROIS = dataArray[centieme];   // stocke la valeur binaire 7 segments de la dizaine de minutes dans "AfficheurDEUX"
  AfficheurDEUX = dataArray[dixieme]; // stocke la valeur binaire 7 segments de l'unité d'heures dans "AfficheurTROIS"
  AfficheurUN = dataArray[11]; // stocke la valeur binaire 7 segments de la dizaine d'heures dans "AfficheurQUATRE"

  digitalWrite(latchpin, 1);                              // latch à l'état HAUT pour autoriser le transfert des données série   
  shiftOut(datapin, clockpin, LSBFIRST, AfficheurUN);     // envoi l'unités minute au registre à décallage
  shiftOut(datapin, clockpin, LSBFIRST, AfficheurDEUX);   // envoi la dizaine minute au registre à décallage
  shiftOut(datapin, clockpin, LSBFIRST, AfficheurTROIS);  // envoi l'unités heure au registre à décallage
  shiftOut(datapin, clockpin, LSBFIRST, AfficheurQUATRE); // envoi la dizaine heure au registre à décallage  
  digitalWrite(latchpin, 0);                              // latch à l'état BAS pour arreter le transfert des données série  
}


/***** Fonction d'horloge *******/   
void horloge(void)
{
  DateTime now = HORLOGE.now();  // lecture de l'heure en cours dans la puce DS3231, référence à la librairie Chronodot
  heuresHorloge = now.hour(), DEC;      // stocke l'heure en décimale dans la variable "heures" grace à la fct "hour" de la lib chronodot
  minutesHorloge = now.minute(), DEC;   // stocke les minutes en décimale dans la variable "minutes" grace à la fct "minute" de la lib chronodot
  secondes = now.second(), DEC;  // stocke les secondes en décimale dans la variable "secondes" grace à la fct "second" de la lib chronodot
  annee = now.year(), DEC;       // stocke l'année en décimale dans la variable "annee" grace à la fct "year" de la lib chronodot
  mois = now.month(), DEC;       // stocke le mois en décimale dans la variable "mois" grace à la fct "month" de la lib chronodot 
  jour = now.day(), DEC;         // stocke le jour en décimale dans la variable "jour" grace à la fct "day" de la lib chronodot
//  Serial.print(annee); //
//  Serial.print(':'); //
//  Serial.print(mois); //
//  Serial.print(':'); //
//  Serial.print(jour); //
//  Serial.print(':');   //
//  Serial.print(heures);   //décommenter pour débug
//  Serial.print(':');   //
//  Serial.print(minutes); //
//  Serial.println();  //
}

/***** Fonction d'affichage horloge sur les 7 segments *******/  
void AfficheHorloge(void)
{
//  AjusteLuminosite();
  
  dizH = heuresHorloge / 10;   // par calcul, extrait la dizaine de "heures" et stocke le résultat dans "dizH" 
  unitH = heuresHorloge % 10;  // par calcul, extrait l'unités de "heures" et stocke le résultat dans "unitH" 
  dizM = minutesHorloge / 10;  // par calcul, extrait la dizaine de "minutes" et stocke le résultat dans "dizM"   
  unitM = minutesHorloge % 10; // par calcul, extrait l'unité de "minutes" et stocke le résultat dans "dizM" 
        
  AfficheurUN = dataArray[unitM];    // stocke la valeur binaire 7 segments de l'unité de minutes dans "AfficheurUN"
//  AfficheurUN &= ~(1<<1);
  AfficheurDEUX = dataArray[dizM];   // stocke la valeur binaire 7 segments de la dizaine de minutes dans "AfficheurDEUX"
  AfficheurTROIS = dataArray[unitH]; // stocke la valeur binaire 7 segments de l'unité d'heures dans "AfficheurTROIS"
  AfficheurQUATRE = dataArray[dizH]; // stocke la valeur binaire 7 segments de la dizaine d'heures dans "AfficheurQUATRE"
//  AfficheurQUATRE &= ~(1<<1);  
  digitalWrite(latchpin, 1);                              // latch à l'état HAUT pour autoriser le transfert des données série   
  shiftOut(datapin, clockpin, LSBFIRST, AfficheurUN);     // envoi l'unités minute au registre à décallage
  shiftOut(datapin, clockpin, LSBFIRST, AfficheurDEUX);   // envoi la dizaine minute au registre à décallage
  shiftOut(datapin, clockpin, LSBFIRST, AfficheurTROIS);  // envoi l'unités heure au registre à décallage
  shiftOut(datapin, clockpin, LSBFIRST, AfficheurQUATRE); // envoi la dizaine heure au registre à décallage  
  digitalWrite(latchpin, 0);                              // latch à l'état BAS pour arreter le transfert des données série  
}


/***** Régler l'Horloge : Fonction Surveillance des BoutonHorloge, BoutonMinutes et BoutonHeures *******/    
void SurveilleBouton_Horloge_Heures_Minutes ()
{ 
  EtatBoutonHorloge = digitalRead(BoutonHorloge);  // Lit l'état du bouton (appuyé ou relaché) et stocke la valeur dans la variable "EtatBoutonHorloge"
  EtatBoutonHeures = digitalRead(BoutonHeures);  // Lit l'état du bouton (appuyé ou relaché) et stocke la valeur dans la variable "EtatBoutonHeures"
  EtatBoutonMinutes = digitalRead(BoutonMinutes);  // Lit l'état du bouton (appuyé ou relaché) et stocke la valeur dans la variable "EtatBoutonMinutes"
  if (EtatBoutonHorloge == LOW)
    {
    mode = 1; 
//    Serial.println(mode);
    }
  if ((EtatBoutonHorloge == LOW) && (EtatBoutonHeures == LOW))  // si le bouton est appuyé
    {  
    delay(170);                  // alors, attendre 170 ms, permet un appuie bref pour incrémenter de 1, et un appuie long pour incrémenter rapidement d'autant qu'on veut
    RegleHorlogeHeures ();              // et appeller la fonction "RegleHeures"
    } 
  if ((EtatBoutonHorloge == LOW) && (EtatBoutonMinutes == LOW))  // si le bouton est appuyé 
    {  
    delay(170);                    // alors,  attendre 170 ms, permet un appuie bref pour incrémenter de 1, et un appuie long pour incrémenter rapidement d'autant qu'on veut
    RegleHorlogeMinutes ();               // et appeller la fonction "RegleMinutes"
    } 
}

/***** Fonction d'incrément Heures *******/ 
void RegleHorlogeHeures ()
{ 
  heuresHorlogeINC = heuresHorloge + 1;  // additionne 1 à la valeur contenue dans la variable "heures" et stocke le résultat dans la variable "heuresHorlogeINC"
  if (heuresHorlogeINC > 23)      // si la valeur de "heuresHorlogeINC" dépasse 23 (23h)
  {
    heuresHorlogeINC = 0;         // alors, et la variable "heuresHorlogeINC" à zero (minuit)
  }
//    Serial.print(heuresHorlogeINC);
//    Serial.println(); 
    HORLOGE.adjust(DateTime(annee,mois,jour,heuresHorlogeINC,minutesHorloge,0));  // récup des données temporelles depuis les différentes variables pour mise à l'heure la puce DS3231, secondes à zero
}

/***** Fonction d'incrément minutes *******/ 
void RegleHorlogeMinutes ()
{ 
  minutesHorlogeINC = minutesHorloge + 1;  // additionne 1 à la valeur contenue dans la variable "minutes" et stocke le résultat dans la variable "minutesHorlogeINC"
  if (minutesHorlogeINC > 59)      // si la valeur de "minutesHorlogeINC" dépasse 59 (59min)
  {
    minutesHorlogeINC = 0;         // alors, met la variable "minutesHorlogeINC" à zero
  }
//    Serial.print(minutesHorlogeINC);
//    Serial.println(); 
    HORLOGE.adjust(DateTime(annee,mois,jour,heuresHorloge,minutesHorlogeINC,0));  // récup des données temporelles depuis les différentes variables pour mise à l'heure la puce DS3231, secondes à zero
 }


void SurveilleBouton_TempHum()
{ 
  EtatBoutonTempHum = digitalRead(BoutonTempHum);  // Lit l'état du bouton (appuyé ou relaché) et stocke la valeur dans la variable "EtatBoutonHorloge"
  if (EtatBoutonTempHum == LOW) // si le bouton est appuyé
    {  
    delay(170);                  // alors, attendre 170 ms, permet un appuie bref pour incrémenter de 1, et un appuie long pour incrémenter rapidement d'autant qu'on veut
  // passer en mode affichage température ou humidité permanent
    if (mode != 2) {
      mode = 2;
    }
    else if (mode == 2){
      mode = 3;
    }
    Serial.println("BoutonTempHum appuyé");
//    Serial.println(mode);
    } 
}

void Fonction_Breaker() {
  breaker = 1;
  Serial.println(breaker);
}

void SurveilleBouton_Caroussel()
{ 
  EtatBoutonCaroussel = digitalRead(BoutonCaroussel);  // Lit l'état du bouton (appuyé ou relaché) et stocke la valeur dans la variable "EtatBoutonHorloge"
  if (EtatBoutonCaroussel == LOW) // si le bouton est appuyé
    {  
    delay(170);                  // alors, attendre 170 ms, permet un appuie bref pour incrémenter de 1, et un appuie long pour incrémenter rapidement d'autant qu'on veut
  // passer en mode affichage Caroussel
    mode = 0;
    Serial.println("BoutonCaroussel appuyé");
    } 
}

/***** Fonction d'ajustement de la luminosité via la photodiode *******/ 
//Faire correspondre ces valeurs… avec map !
//PhotoDiode ledPin
//> 495      128
//> 462      120
//> 429      112
//> 396      104
//> 363      96
//> 330      88
//> 297      80
//> 264      72
//> 231      64
//> 198      56
//> 165      48
//> 132      40
//> 99       32
//> 66       24
//> 33       16
//> 0        5
// saut de 33    saut de 8
void AjusteLuminosite() {
  ValeurPhotoDiode = analogRead(photodiodepin);  // Lit la valeur renvoyée par la photodiode, et stocke la valeur dans la variable "ValeurPhotoDiode"
  ValeurPhotoDiode = map(ValeurPhotoDiode, 0, 495, 5, 128);  // Entre 0 et 4 secondes
  if ((mode == 2) || (mode == 3)) {
    analogWrite(ledPin, ValeurPhotoDiode);
    analogWrite(dotledPin, 0);
  }
  else {
    analogWrite(ledPin, ValeurPhotoDiode);
    analogWrite(dotledPin, ValeurPhotoDiode);    
  }
}


/*void SurveilleBouton_Supp ()
{ 
  EtatBoutonSupp = digitalRead(BoutonSupp);  // Lit l'état du bouton (appuyé ou relaché) et stocke la valeur dans la variable "EtatBoutonHorloge"
  if (EtatBoutonSupp == LOW) // si le bouton est appuyé
    {  
    delay(170);                  // alors, attendre 170 ms, permet un appuie bref pour incrémenter de 1, et un appuie long pour incrémenter rapidement d'autant qu'on veut
    //Sonnerie();              // et appeller la fonction "RegleHeures"
    REVEIL.Alarm2Stop();
    } 
}*/

