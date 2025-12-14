# MK2-PV-WITH-TEMP
Modification du code MK2 triphasé pour installation sonde température, communication avec ESP32, la gestion du forçage des triacs ainsi que la gestion d'un second ballon


/* Mk2_3phase_RFdatalog_4_with_Temperature.ino
 *
 * Le premier numéro est paru en janvier 2015.
 *
 * Ce schéma permet une surveillance continue de la puissance active sur trois phases.
 * La puissance excédentaire est distribuée séquentiellement à plusieurs charges. 
 * Un étage de sortie adapté est nécessaire pour chaque charge ; il peut s'agir d'un triac ou d'un relais statique.  
 *
 * L'enregistrement des données de puissance active et de tension efficace (Vrms) est assuré pour chaque phase.
 * La présence ou l'absence du module RFM12B doit être définie lors de la compilation.
 *
 * Janvier 2016, renommé Mk2_3phase_RFdatalog_2 avec les modifications suivantes :
 * - Amélioration du contrôle des charges multiples grâce à l'importation du
 * programme monophasé équivalent, Mk2_multiLoad_wired_6.ino
 * - Mise à jour de la routine d'interruption (ISR) pour corriger une possible anomalie de synchronisation
 * - Les variables de stockage des échantillons ADC sont désormais déclarées comme « volatiles »
 * - Prise en charge du module RF69
 * - Ajout d'un contrôle de performance avec envoi du résultat au port série
 * - Les signaux de contrôle des charges sont désormais actifs à l'état haut pour s'adapter aux circuits imprimés triphasés les plus récents
 *
 * Février 2016, renommé Mk2_3phase_RFdatalog_3 avec les modifications suivantes :
 * - Améliorations de la logique de démarrage. Le démarrage du fonctionnement normal est désormais
 * synchronisé avec le début d'un nouveau cycle secteur.
 * - Réduction du niveau de rétroaction dans le filtre passe-bas pour la suppression de la composante continue
 * du flux Vsample. Ceci résout une anomalie présente depuis
 * le début de ce projet. Bien que le niveau de rétroaction ait été auparavant
 * excessif, cette anomalie n'avait qu'un impact minime sur le comportement global du système.
 * - La puissance mesurée sur chaque phase a été inversée. Ces valeurs sont désormais
 * conformes à la convention Open Energy Monitor, où l'importation est positive et
 * l'exportation est négative.
 *   
 * Février 2020 : mise à jour vers Mk2_3phase_RFdatalog_3a avec les modifications suivantes :
 * - suppression de code redondant dans la logique de détermination du prochain état de charge.
 * 
 * Juillet 2022 : mise à jour vers Mk2_3phase_RFdatalog_4, avec la modification suivante :
 * - l'accumulateur d'acquisition de données pour Vsquared a été réduit à 1/16 de sa valeur précédente
 * afin d'éviter tout risque de débordement lors d'une période d'acquisition de données de 20 secondes.  
 *
 *      Robin Emley (code original)
 *      www.Mk2PVrouter.co.uk
 *
 * Modification du code original de Robin Emley pour ajouter :
 * - Sonde de température DS18B20 pour surveillance ballon d'eau chaude
 * - Forçage automatique du chauffage si température < seuil minimum
 * - Communication série avec ESP32 pour supervision
 *
 * Version modifiée : Novembre 2025
 *
 * NOUVELLES FONCTIONNALITÉS :
 * - Pin D3 : Sonde de température DS18B20 (ballon 1)
 * - Mode forçage automatique si T° < TEMP_MIN_FORCAGE
 * - Envoi données série vers ESP32 toutes les 5 secondes
 * - Hystérésis pour éviter oscillations mode forçage/routage
 *
 * Version modifiée : Décembre 2025
 
 * - Remplacement de Serial.readStringUntil() par un parseur série non-bloquant basé sur un buffer char fixe.
 * - Debounce non-bloquant pour la lecture heures creuses.
 * - Ajout de la gestion non-bloquante pour la conversion DS18B20 (setWaitForConversion(false) + poll).
 * - Conserve le reste du sketch intact (ISR ADC, logique de routage, datalogging, etc.)
*/

#include <Arduino.h> // Ce n'est peut-être pas nécessaire, mais il est probablement judicieux de l'inclure.

// Bibliothèques pour sonde de température DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

//#define RF_PRESENT // <- Cette ligne doit être commentée si le module RFM12B n'est pas présent.

#ifdef RF_PRESENT
//#define RF69_COMPAT 0 // pour le RFM12B
#define RF69_COMPAT 1 // pour le RF69
#include <JeeLib.h>     
#endif

// Dans ce schéma, le CAN fonctionne en mode libre avec un temps de cycle d'environ 104 µs.

//  La fonction WORKLOAD_CHECK permet de déterminer le temps de traitement disponible.
// Pour activer ce mode, la ligne #define ci-dessous doit être incluse : 
//#define WORKLOAD_CHECK  

#define CYCLES_PER_SECOND 50
//#define JOULES_PER_WATT_HOUR 3600 // peut être nécessaire pour l'enregistrement des données
#define WORKING_ZONE_IN_JOULES 3600
#define REQUIRED_EXPORT_IN_WATTS 0 // Lorsqu'il est réglé sur une valeur négative, il agit comme un générateur photovoltaïque.
#define NO_OF_PHASES 3
#define DATALOG_PERIOD_IN_SECONDS 10

// ============== NOUVEAUX PARAMETRES TEMPERATURE ==============
// Seuils de température (en °C)
const float TEMP_MIN_FORCAGE_HC = 48.0;   // Forçage en heures creuses
const float TEMP_MIN_FORCAGE = 40.0;      // Forçage hors heures creuses
const float TEMP_ARRET_FORCAGE = 60.0;    // Arrêt forçage
const float TEMP_MAX_SECURITE = 75.0;     // Sécurité
const float TEMP_SWITCH_BALLON2 = 68.0;   // Switch Ballon 2

// Configuration sonde DS18B20
#define ONE_WIRE_BUS 3                     // Pin D3
#define TEMP_READ_INTERVAL 5000
#define SERIAL_SEND_INTERVAL 5000

// Objets pour gestion sonde température
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress tempSensorAddress_B1;  // Adresse sonde ballon 1

// Variables globales température
float temperatureBallon1 = -127.0;        // Température actuelle ballon 1 (°C)
boolean modeForceActif = false;           // État mode forçage
boolean sondePresente = false;            // Détection présence sonde ballon 1
boolean sondeDefaillante = false;         // Sonde déconnectée/défaillante
byte erreursConsecutives = 0;             // Compteur erreurs lecture
unsigned long derniereLectureTemp = 0;    // Timer lecture (dernier démarrage de conversion)
unsigned long dernierEnvoiSerial = 0;     // Timer envoi série

// Variables pour conversion non-bloquante DS18B20
bool tempConversionPending = false;
unsigned long tempConversionRequestedAt = 0;
// Pour résolution 12-bit -> conversion ~750 ms. Si vous changez la résolution,
// adaptez TEMP_CONVERSION_MS en conséquence (12-bit = 750ms, 11-bit = 375ms, 10-bit = 188ms, 9-bit = 94ms).
const unsigned long TEMP_CONVERSION_MS = 750;

// Détection température figée (court-circuit probable)
float derniereTemperatureValide = -127.0; // Dernière température lue
byte compteurValeurIdentique = 0;         // Compteur lectures identiques
#define TEMP_DEFAULT_DS18B20 85.0         // Valeur par défaut DS18B20 (court-circuit)

// Paramètres gestion erreurs sonde
#define MAX_ERREURS_CONSECUTIVES 3        // 3 erreurs = sonde considérée défaillante
#define MAX_LECTURES_IDENTIQUES 20        // 20 lectures identiques = sonde figée (court-circuit probable)

// Paramètres gestion commutation ballon 2 (mode dégradé)
#define TIMEOUT_ESP32_MS 60000             // 60 secondes sans réponse ESP32 = mode dégradé
unsigned long dernierMessageESP32 = 0;     // Timestamp dernier message reçu de l'ESP32
boolean modeDegrade = false;               // Mode dégradé actif (ESP32 ne répond plus)
boolean contacteurBallon2Actif = false;    // État contacteur (D7) commutation ballon 2

// Paramètres gestion heures creuses
boolean heuresCreusesActives = false;      // État actuel heures creuses (lecture contact sec)
boolean forcageHeuresCreusesActif = false; // Forçage activé par heures creuses

// =============================================================

const byte noOfDumploads = 3; 

enum polarities {NEGATIVE, POSITIVE};
enum outputModes {ANTI_FLICKER, NORMAL};

// enum loadStates {LOAD_ON, LOAD_OFF}; // à utiliser si les charges sont actives à l'état bas (PCB d'origine)
enum loadStates {LOAD_OFF, LOAD_ON}; // à utiliser si les charges sont actives à l'état haut (PCB Rév. 2)
enum loadStates logicalLoadState[noOfDumploads]; 
enum loadStates physicalLoadState[noOfDumploads]; 

// Pour cette version à chargement multiple, le même mécanisme a été conservé, 
//mais le mode de sortie est codé en dur comme ci-dessous :
enum outputModes outputMode = NORMAL;    

#ifdef RF_PRESENT
#define freq RF12_433MHZ // Utilisez la fréquence correspondant au module dont vous disposez.

const int nodeID = 10;                                          
const int networkGroup = 210;                        
const int UNO = 1;
#endif
                            
typedef struct { 
  int power_L1;
  int power_L2;
  int power_L3; 
  int Vrms_L1;
  int Vrms_L2;
  int Vrms_L3;} Tx_struct;    // données révisées pour les communications RF
Tx_struct tx_data;


// ----------- Affectations de brochage  -----------
//
// digital pins:
// D3 utilisé pour sonde température (OneWire)
// D4 n'est pas utilisé
const byte physicalLoad_0_pin = 5; // pour PCB 3-phases, Load #1 (Rev 2 PCB)
const byte physicalLoad_1_pin = 6; // pour PCB 3-phases, Load #2 (Rev 2 PCB)
const byte physicalLoad_2_pin = 7; // pour PCB 3-phases, Load #3 (Rev 2 PCB) / Contacteur Ballon 2
const byte heuresCreusesPin = 8;   // Contact sec horloge heures creuses (NOUVEAU)
// D9 n'est pas utilisé

// analogue input pins 
const byte sensorV[NO_OF_PHASES] = {0,2,4}; // pour PCB 3-phases
const byte sensorI[NO_OF_PHASES] = {1,3,5}; // pour PCB 3-phases 


// --------------  variables globales générales -----------------
//
// Certaines de ces variables sont utilisées dans plusieurs blocs et ne peuvent donc pas être statiques.
// Pour les calculs sur les entiers, certaines variables doivent être de type « long ».
//
boolean beyondStartUpPeriod = false;    // Délai de démarrage, pour permettre aux choses de se stabiliser
byte initialDelay = 3;  // en secondes, pour laisser le temps d'ouvrir le moniteur série
byte startUpPeriod = 3;  // en secondes, pour permettre au filtre LP de se stabiliser

long DCoffset_V_long[NO_OF_PHASES];              // <--- for LPF
long DCoffset_V_min;               // <--- for LPF (min limit)
long DCoffset_V_max;               // <--- for LPF (max limit)
int DCoffset_I_nom;               // valeur médiane nominale de ADC @ x1 scale

// Pour une utilisation triphasée, avec des unités de Joules * CYCLES_PAR_SECONDE
float capacityOfEnergyBucket_main; 
float energyInBucket_main; 
float midPointOfEnergyBucket_main;
float lowerThreshold_default;  
float lowerEnergyThreshold;
float upperThreshold_default;
float upperEnergyThreshold;
float offsetOfEnergyThresholdsInAFmode = 0.1; // <-- ne doit pas dépasser 0.4

// pour un meilleur contrôle des charges multiples
boolean recentTransition = false;
byte postTransitionCount;
#define POST_TRANSITION_MAX_COUNT 3 // <-- permet à chaque transition de prendre effet
//#define POST_TRANSITION_MAX_COUNT 50 // <-- pour les tests uniquement
byte activeLoad = 0;

// pour datalogging
int datalogCountInMainsCycles;
const int maxDatalogCountInMainsCycles = DATALOG_PERIOD_IN_SECONDS * CYCLES_PER_SECOND;
float energyStateOfPhase[NO_OF_PHASES]; // utilisé seulement pour datalogging

// pour l'interaction entre le processeur principal et l'ISR
volatile boolean dataReady = false;
volatile int sampleV[NO_OF_PHASES];
volatile int sampleI[NO_OF_PHASES];

// Pour un mécanisme permettant de vérifier la continuité de la séquence d'échantillonnage
#define CONTINUITY_CHECK_MAXCOUNT 250 // cycles secteur
int mainsCycles_forContinuityChecker;
int lowestNoOfSampleSetsPerMainsCycle;

// Valeurs d'étalonnage
//-------------------
// Ce programme utilise trois valeurs d'étalonnage : powerCal, phaseCal et voltageCal.
// Avec la plupart des matériels, les valeurs par défaut devraient convenir.
// Il est donc nécessaire de les modifier. Voici une brève explication de chacune de ces valeurs :

// Lors du calcul de la puissance réelle, ce que fait ce code,
// les taux de conversion individuels de la tension et du courant n'ont pas d'importance.
// Seul le taux de conversion de la puissance est important.
// Il s'agit du produit des taux de conversion individuels de la tension et du courant.
// Il est donc exprimé en pas de convertisseur analogique-numérique au carré par watt.
// La plupart des systèmes auront un taux de conversion de puissance d'environ 20 (pas de convertisseur analogique-numérique au carré par watt).
// 
// powerCal est l'inverse du taux de conversion de puissance. Une bonne valeur 
// La valeur de départ est donc de 1/20 = 0,05 (watts par étape ADC au carré).
//
const float powerCal[NO_OF_PHASES] = {0.043, 0.043, 0.043};

// phaseCal sert à modifier la phase de la forme d'onde de tension par rapport à la forme d'onde de courant.
// L'algorithme interpole entre la paire la plus récente d'échantillons de tension en fonction de la valeur de phaseCal. 
//
//    Avec phaseCal = 1, L'échantillon le plus récent est utilisé.  
//    Avec phaseCal = 0, l'échantillon précédent est utilisé
//    Avec phaseCal = 0.5, la valeur médiane (moyenne) utilisée
//
// NB. Tout outil qui détermine la valeur optimale de phaseCal doit avoir un schéma similar pour la prise de valeurs d'échantillonnage à celui de ce schéma.
//
const float  phaseCal[NO_OF_PHASES] = {0.5, 0.5, 0.5}; // <- valeur nominal seulemnt
int phaseCal_int[NO_OF_PHASES];           // pour éviter le recours aux calculs en virgule flottante

// Pour l'enregistrement des données, voltageCal a également été ajouté. Étant donné que la plage de valeurs de l'ADC est
// similaire à la plage réelle de volts, la valeur optimale de ce facteur d'étalonnage est probablement
// proche de l'unité.
const float voltageCal[NO_OF_PHASES] = {1.03, 1.03, 1.03}; 


// ============== NOUVELLES FONCTIONS TEMPERATURE ==============

/**
 * Initialisation de la sonde de température DS18B20 (Ballon 1)
 */
void initTemperatureSensor() {
  Serial.println();
  Serial.println("Initialisation sonde de temperature DS18B20...");
  
  sensors.begin();
  
  byte nombreSondes = sensors.getDeviceCount();
  Serial.print("Nombre de sondes detectees: ");
  Serial.println(nombreSondes);
  
  if (nombreSondes == 0) {
    Serial.println("SONDE:NONE");
    sondePresente = false;
    return;
  }
  
  if (sensors.getAddress(tempSensorAddress_B1, 0)) {
    sondePresente = true;
    // Résolution 12-bit (par défaut). Pour permettre la conversion non-bloquante, on désactive l'attente.
    sensors.setResolution(tempSensorAddress_B1, 12);
    // Activer mode non-bloquant : requestTemperatures() retournera immédiatement et la conversion se fera en arrière-plan.
    sensors.setWaitForConversion(false);
    Serial.println("SONDE:OK (non-blocking conversions)");
  } else {
    Serial.println("SONDE:ERR");
    sondePresente = false;
    return;
  }
}


/**
 * Lecture de la température du ballon 1 (mode non-bloquant)
 * - On démarre une conversion toutes les TEMP_READ_INTERVAL ms (requestTemperatures())
 * - On attend TEMP_CONVERSION_MS (approx. 750ms pour 12-bit) avant de lire les données
 * - La fonction est non bloquante : elle démarre la conversion et la récupère plus tard
 */
void lireTemperature() {
  if (!sondePresente) return;
  
  unsigned long maintenant = millis();

  // Si une conversion est en attente, vérifier si elle est terminée (via délai)
  if (tempConversionPending) {
    if (maintenant - tempConversionRequestedAt >= TEMP_CONVERSION_MS) {
      // Conversion considérée terminée -> lecture
      tempConversionPending = false;
      float temp = sensors.getTempC(tempSensorAddress_B1);
      
      if (temp != DEVICE_DISCONNECTED_C && temp > -50.0 && temp < 120.0) {
        if (abs(temp - derniereTemperatureValide) < 0.1) {
          compteurValeurIdentique++;
          if (abs(temp - TEMP_DEFAULT_DS18B20) < 0.5) {
            Serial.println("ERR:85C");
            compteurValeurIdentique = MAX_LECTURES_IDENTIQUES;
          }
          if (compteurValeurIdentique >= MAX_LECTURES_IDENTIQUES && !sondeDefaillante) {
            sondeDefaillante = true;
            temperatureBallon1 = -127.0;
            Serial.println("SONDE:FIGEE");
          }
        } else {
          temperatureBallon1 = temp;
          derniereTemperatureValide = temp;
          compteurValeurIdentique = 0;
          erreursConsecutives = 0;
          if (sondeDefaillante) {
            sondeDefaillante = false;
            Serial.println("SONDE:OK");
          }
        }
      } else {
        erreursConsecutives++;
        if (erreursConsecutives >= MAX_ERREURS_CONSECUTIVES && !sondeDefaillante) {
          sondeDefaillante = true;
          temperatureBallon1 = -127.0;
          Serial.println("SONDE:OFF");
        }
      }
      // On conserve derniereLectureTemp comme timestamp du dernier cycle complet
      // (inutile ici de le mettre à maintenant car il sert aussi pour cadence des conversions)
    }
    // si conversion pas encore prête, on sort (non-bloquant)
    return;
  }

  // Si aucune conversion en cours et que l'intervalle est écoulé, démarrer une nouvelle conversion
  if (maintenant - derniereLectureTemp >= TEMP_READ_INTERVAL) {
    // Demander conversion non-bloquante
    sensors.requestTemperatures(); // avec setWaitForConversion(false) c'est non-bloquant
    tempConversionRequestedAt = maintenant;
    tempConversionPending = true;
    // marquer le temps du lancement de conversion pour respecter l'intervalle entre conversions
    derniereLectureTemp = maintenant;
    // On ne lit pas la valeur immédiatement (lecture différée)
  }
}


/**
 * Gestion du mode forçage en fonction de la température
 * SÉCURITÉ: Si sonde défaillante, pas de forçage automatique
 */
void gererModeForceTemperature() {
  if (!sondePresente || sondeDefaillante) {
    if (modeForceActif) {
      modeForceActif = false;
      forcageHeuresCreusesActif = false;
    }
    return;
  }
  
  if (temperatureBallon1 <= -50.0) return;
  
  // SÉCURITÉ
  if (temperatureBallon1 >= TEMP_MAX_SECURITE) {
    if (modeForceActif) {
      Serial.println("SECU: Temp haute");
      modeForceActif = false;
      forcageHeuresCreusesActif = false;
    }
    for (byte i = 0; i < noOfDumploads; i++) {
      logicalLoadState[i] = LOAD_OFF;
    }
    return;
  }
  
  lireEtatHeuresCreuses();
  
  // PRIORITÉ 1 : HC avec seuil 50°C
  if (heuresCreusesActives && !forcageHeuresCreusesActif) {
    if (temperatureBallon1 < TEMP_MIN_FORCAGE_HC) {
      Serial.print("HC ON T:");
      Serial.println(temperatureBallon1);
      modeForceActif = true;
      forcageHeuresCreusesActif = true;
    }
  }
  
  if (forcageHeuresCreusesActif) {
    if (!heuresCreusesActives || temperatureBallon1 >= TEMP_ARRET_FORCAGE) {
      Serial.println("HC OFF");
      modeForceActif = false;
      forcageHeuresCreusesActif = false;
    }
  }
  
  // PRIORITÉ 2 : Temp basse hors HC avec seuil 40°C
  if (!forcageHeuresCreusesActif) {
    if (!modeForceActif && temperatureBallon1 < TEMP_MIN_FORCAGE) {
      Serial.print("TEMP ON T:");
      Serial.println(temperatureBallon1);
      modeForceActif = true;
    }
    else if (modeForceActif && temperatureBallon1 > TEMP_ARRET_FORCAGE) {
      Serial.println("TEMP OFF");
      modeForceActif = false;
    }
  }
}


/**
 * Lecture de l'état du contact sec heures creuses (avec debounce non bloquant)
 * Contact fermé (LOW) = Heures creuses
 * Contact ouvert (HIGH) = Heures pleines
 *
 * Debounce réalisé par exigence de stabilité pendant debounceMs (default 50 ms).
 */
void lireEtatHeuresCreuses() {
  static boolean dernierEtatStable = false;
  static boolean dernierEtatLu = false;
  static unsigned long dernierBounce = 0;
  const unsigned long debounceMs = 50;
  
  int pinState = digitalRead(heuresCreusesPin);
  boolean nouvelEtat = (pinState == LOW);
  
  if (nouvelEtat != dernierEtatLu) {
    // changement de lecture => marquer le temps
    dernierBounce = millis();
    dernierEtatLu = nouvelEtat;
  } else {
    // si le nouvel état est stable depuis debounceMs, l'accepter
    if ((millis() - dernierBounce) >= debounceMs && nouvelEtat != dernierEtatStable) {
      dernierEtatStable = nouvelEtat;
      heuresCreusesActives = nouvelEtat;
      
      if (heuresCreusesActives) {
        Serial.println("HC START");
      } else {
        Serial.println("HC END");
      }
    }
  }
}


/**
 * Application du mode forçage sur les charges
 * En mode forçage : active charge 0 et 1 à 100% (ballon prioritaire)
 */
void appliquerModeForce() {
  if (modeForceActif) {
    // Forcer charges 0 et 1 à ON (ballon principal, 2 triacs pour triphasé)
    logicalLoadState[0] = LOAD_ON;
    logicalLoadState[1] = LOAD_ON;
    
    // Charge 2 (contacteur) reste géré par ESP32 ou mode dégradé
  }
}


/**
 * Traitement d'une commande complète reçue via série (ligne terminée)
 * Commandes prises en charge:
 *  - "D7:1" ou "D7:0" -> commande contacteur Ballon2 (pin physicalLoad_2_pin)
 *  - "PING" -> réponse "PONG"
 *
 * Fonction non bloquante, sans usage de String.
 */
void processSerialCommand(const char * commande) {
  // Mettre à jour timestamp de dernier message reçu
  dernierMessageESP32 = millis();
  
  if (modeDegrade) {
    modeDegrade = false;
    Serial.println("ESP32:OK");
  }
  
  if (strncmp(commande, "D7:", 3) == 0) {
    int valeur = atoi(commande + 3);
    if (valeur == 1) {
      contacteurBallon2Actif = true;
      digitalWrite(physicalLoad_2_pin, HIGH);
      Serial.println("D7:ON");
    } else {
      contacteurBallon2Actif = false;
      digitalWrite(physicalLoad_2_pin, LOW);
      Serial.println("D7:OFF");
    }
  } else if (strcmp(commande, "PING") == 0) {
    Serial.println("PONG");
  } else {
    // commande inconnue: on peut ajouter un retour optionnel si utile
    // Serial.print("UNK:");
    // Serial.println(commande);
  }
}

/**
 * Réception commandes depuis ESP32 via Serial (non bloquant, sans String)
 * On accumule les octets jusqu'à '\n' et on traite ligne complète.
 * Limite de longueur pour éviter overflow.
 */
void recevoirCommandesESP32() {
  static char cmdBuf[64];
  static uint8_t idx = 0;
  
  while (Serial.available()) {
    int r = Serial.read();
    if (r < 0) break;
    char c = (char)r;
    // ignorer carriage return
    if (c == '\r') continue;
    if (c == '\n') {
      if (idx > 0) {
        cmdBuf[idx] = '\0';
        processSerialCommand(cmdBuf);
        idx = 0;
      }
    } else {
      // n'accepter que caractères imprimables
      if (isPrintable((unsigned char)c)) {
        if (idx < (sizeof(cmdBuf) - 1)) {
          cmdBuf[idx++] = c;
        } else {
          // buffer overflow -> reset pour éviter corruption
          idx = 0;
        }
      }
    }
  }
}


void gererModeDegrade() {
  unsigned long maintenant = millis();
  
  if (!modeDegrade && (maintenant - dernierMessageESP32 > TIMEOUT_ESP32_MS)) {
    modeDegrade = true;
    Serial.println("ESP32:TIMEOUT");
  }
  
  if (modeDegrade && sondePresente && !sondeDefaillante) {
    boolean routageActif = false;
    for (byte i = 0; i < 2; i++) {
      if (logicalLoadState[i] == LOAD_ON && !modeForceActif) {
        routageActif = true;
        break;
      }
    }
    
    if (routageActif && temperatureBallon1 > TEMP_SWITCH_BALLON2) {
      if (!contacteurBallon2Actif) {
        contacteurBallon2Actif = true;
        digitalWrite(physicalLoad_2_pin, HIGH);
        Serial.println("B2:ON");
      }
    } else {
      if (contacteurBallon2Actif) {
        contacteurBallon2Actif = false;
        digitalWrite(physicalLoad_2_pin, LOW);
        Serial.println("B2:OFF");
      }
    }
  }
  
  if (modeDegrade && (!sondePresente || sondeDefaillante)) {
    if (contacteurBallon2Actif) {
      contacteurBallon2Actif = false;
      digitalWrite(physicalLoad_2_pin, LOW);
    }
  }
}


/**
 * Envoi des données vers ESP32 via série
 * Format: T1:XX.X,P:XXXX,R:X,F:X
 */
void envoyerDonneesSerial() {
  unsigned long maintenant = millis();
  
  if (maintenant - dernierEnvoiSerial >= SERIAL_SEND_INTERVAL) {
    dernierEnvoiSerial = maintenant;
    
    // Calcul puissance routée réelle (somme des 3 phases)
    int puissanceRoutee = 0;
    for (byte phase = 0; phase < NO_OF_PHASES; phase++) {
      puissanceRoutee += (int)(energyStateOfPhase[phase] / DATALOG_PERIOD_IN_SECONDS);
    }
    // Inverser le signe pour convention : export = positif
    puissanceRoutee *= -1;
    
    // Détermination si routage actif
    byte routageActif = 0;
    for (byte i = 0; i < noOfDumploads; i++) {
      if (logicalLoadState[i] == LOAD_ON && !modeForceActif) {
        routageActif = 1;
        break;
      }
    }
    
    // Envoi données au format compact
    Serial.print("T1:");
    Serial.print(temperatureBallon1, 1);
    Serial.print(",P:");
    Serial.print(puissanceRoutee);
    Serial.print(",R:");
    Serial.print(routageActif);
    Serial.print(",F:");
    Serial.println(modeForceActif ? 1 : 0);
  }
}

// =============================================================


void setup()
{  
  delay(initialDelay * 1000); // permet d'ouvrir le moniteur série
  
  Serial.begin(9600);         // intitialiser l'interface série
  Serial.println();
  Serial.println("-------------------------------------");
  Serial.println("Sketch ID: Mk2_3phase_RFdatalog_4_with_Temperature.ino");
  Serial.println("Version modifiee avec sonde temperature");
  Serial.println();
  
  pinMode(physicalLoad_0_pin, OUTPUT); //Broche de commande pour la charge n°1
  pinMode(physicalLoad_1_pin, OUTPUT); //Broche de commande pour la charge n°2
  pinMode(physicalLoad_2_pin, OUTPUT); //Broche de commande pour la charge n°3

  // ========== INITIALISATION SONDE TEMPERATURE ==========
  initTemperatureSensor();
  Serial.println();
  // ======================================================
  
  // Initialisation timestamp communication ESP32
  dernierMessageESP32 = millis(); // Évite déclenchement mode dégradé au démarrage
   
  for (byte phase = 0; phase < NO_OF_PHASES; phase++)
  {
    phaseCal_int[phase] = phaseCal[phase] * 256; // for integer maths
  }
  
  for (byte load = 0; load < noOfDumploads; load++)
  {
    logicalLoadState[load] = LOAD_OFF;
  }
  updatePhysicalLoadStates(); // permet d'éteindre toutes les charges
  
  // Configuration ADC en mode free-running (code original Robin Emley)
  Serial.println ("ADC mode:       free-running");
  
  // Configurez le contrôleur de domaine analogique (ADC) pour qu'il fonctionne en mode libre. 
  ADCSRA  = (1<<ADPS0)+(1<<ADPS1)+(1<<ADPS2);  // Réglez l'horloge du CAN sur l'horloge système / 128
  ADCSRA |= (1 << ADEN);                 // Activez le convertisseur analogique-numérique (ADC).
  
  ADCSRA |= (1<<ADATE);  // Activez le bit d'activation du déclenchement automatique dans le registre ADCSRA parce que
                         // les bits ADTS0 à ADTS2 n'ont pas été définis (c'est-à-dire qu'ils sont tous à zéro), 
                         // La source de déclenchement de l'ADC est configurée en « mode de fonctionnement libre ».
                         
  ADCSRA |=(1<<ADIE);    // Configurez le bit d'activation d'interruption ADC. Lorsque ce bit est écrit 
                         // à un et le bit I dans SREG est défini,
                         // l'interruption de conversion ADC terminée est activée.

  ADCSRA |= (1<<ADSC);   // Démarrer l'ADC manuellement la première fois 
  sei();                 // Activer les interruptions globales
  
  capacityOfEnergyBucket_main = 
     (long)WORKING_ZONE_IN_JOULES * CYCLES_PER_SECOND * (1/powerCal[0]);
  energyInBucket_main = 0;   
  midPointOfEnergyBucket_main = capacityOfEnergyBucket_main * 0.5;

  
  pinMode(heuresCreusesPin, INPUT);  
  digitalWrite(heuresCreusesPin, HIGH); // activer le pull-up interne pour contact sec
  
  #ifdef WORKLOAD_CHECK
     Serial.println ("WELCOME TO WORKLOAD_CHECK");
  #endif

  configureParamsForSelectedOutputMode(); 
  
  #ifdef RF_PRESENT
  delay(100);
  rf12_initialize(nodeID, freq, networkGroup);                                       
  #endif

  Serial.print ( ">>free RAM = ");
  Serial.println (freeRam());
  
  DCoffset_V_long[0] = 512L * 256; // valeur nominale de départ (pour la phase 0 uniquement)
  DCoffset_V_min = (long)(512L - 100) * 256; // limite inférieure
  DCoffset_V_max = (long)(512L + 100) * 256; // limite supérieure
  DCoffset_I_nom = 512;
  
  for (byte phase = 0; phase < NO_OF_PHASES; phase++)
  {
    DCoffset_V_long[phase] = 512L * 256; // valeur nominale de départ (pour toutes les phases)
  }
}


// Un ISR est spécifié pour chaque compteur. Le Timer 1 (utilisé ici) est de 16 bits.
// Cette ISR gère l'ADC en mode free-running pour échantillonner V et I sur 3 phases
ISR(ADC_vect)  
{                                         
  static unsigned char sample_index = 0;
  static int sample_V0_raw;
  static int sample_V1_raw;
  static int sample_I0_raw;
  static int sample_I1_raw;
  static int sample_I2_raw;
  
  switch(sample_index)
  {
    case 0:
      sample_I0_raw = ADC; 
      ADMUX = 0x40 + sensorI[1]; // préparer l'avant-dernière conversion
      sample_index++; // avancer le drapeau de contrôle             
      break;
    case 1:
      sample_V0_raw = ADC; 
      ADMUX = 0x40 + sensorV[1]; // pour l'avant-dernière conversion
      sample_index++; // avancer le drapeau de contrôle               
      break;
    case 2:
      sample_I1_raw = ADC; 
      ADMUX = 0x40 + sensorI[2]; // pour l'avant-dernière conversion
      sample_index++; // avancer le drapeau de contrôle                
      break;
    case 3:
      sample_V1_raw = ADC; 
      ADMUX = 0x40 + sensorV[2]; // pour l'avant-dernière conversion
      sample_index++; // avancer le drapeau de contrôle                
      break;
    case 4:
      sample_I2_raw = ADC; 
      ADMUX = 0x40 + sensorI[0]; // pour l'avant-dernière conversion
      sample_index++; // avancer le drapeau de contrôle                
      break;
    case 5:
      sampleV[2] = ADC; 
      ADMUX = 0x40 + sensorV[0]; // pour l'avant-dernière conversion
      sample_index = 0; // réinitialiser le drapeau de contrôle                
      sampleV[0] = sample_V0_raw;
      sampleV[1] = sample_V1_raw;
      sampleI[0] = sample_I0_raw;
      sampleI[1] = sample_I1_raw;
      sampleI[2] = sample_I2_raw;
      dataReady = true; 
      break;
    default:
      sample_index = 0;                 // pour éviter le blocage (cela ne devrait jamais arriver)      
  }  
}


// Le processeur principal attend dans la boucle `loop()` jusqu'à ce que l'indicateur `DataReady` soit activé par le CAN..  
// Une fois cet indicateur activé, le processeur principal l'efface et poursuit son exécution.
// le traitement d'un ensemble complet de 3 paires d'échantillons V et I. Il revient ensuite à
// loop() pour attendre que le prochain ensemble soit disponible.
//   Si le prochain ensemble d'échantillons devient disponible avant le traitement de l'ensemble précédent
// Une fois l'opération terminée, des données pourraient être perdues. Cette situation peut être évitée en utilisant préalablement
// le mode WORKLOAD_CHECK. Grâce à cette fonctionnalité, la quantité de capacité de traitement disponible est mesurée. 
// Il est possible de déterminer pour chaque ensemble de 6 échantillons.  
//
void loop()
{ 
  #ifdef WORKLOAD_CHECK
    static int minSampleSetsDuringThisMainsCycle;
    static int lowestNoOfSampleSetsPerMainsCycle = 999;
  #endif
  
  if (dataReady)
  {
    dataReady = false; 
    
    processRawSamples(); 
  }
  
  if (beyondStartUpPeriod)
  {    
    // ========== GESTION TEMPERATURE (NOUVELLE SECTION) ==========
    // Lecture température ballon 1 (non-bloquante)
    lireTemperature();
    
    // Détermination mode forçage selon température
    gererModeForceTemperature();
    
    // Application mode forçage si nécessaire
    appliquerModeForce();
    
    // Réception commandes ESP32 (commutation ballon 2)
    recevoirCommandesESP32();
    
    // Gestion mode dégradé si ESP32 ne répond plus
    gererModeDegrade();
    
    // Envoi données série vers ESP32
    envoyerDonneesSerial();
    // ============================================================
  }
  else
  {  
    static byte startUpModeCycleCount = 0;
    if (startUpModeCycleCount >= startUpPeriod)
    {
      beyondStartUpPeriod = true;
      lowestNoOfSampleSetsPerMainsCycle = 999;
      Serial.println ("End of start-up period");
    }
    else
    {      
      startUpModeCycleCount++;
    }
  }
}


void processRawSamples()
{
  static int samplesDuringThisMainsCycle[NO_OF_PHASES];    
  static enum polarities polarityOfLastSampleV[NO_OF_PHASES];  
  
  static long sumP[NO_OF_PHASES];                              
  static long sum_Vsquared[NO_OF_PHASES];
  static long samplesDuringThisDatalogPeriod;
  static int mainsCyclesPerDatalogPeriod;
  
  static long cumVdeltasThisCycle_long[NO_OF_PHASES];    
  static long lastSampleV_minusDC_long[NO_OF_PHASES];     
  
  #ifdef WORKLOAD_CHECK
    static int sampleSetsDuringThisMainsCycle;
  #endif
  
  for (byte phase = 0; phase < NO_OF_PHASES; phase++)
  {
    long sampleV_minusDC_long = ((long)sampleV[phase]<<8) - DCoffset_V_long[phase]; 
    long sampleIminusDC_long = ((long)sampleI[phase]<<8) - ((long)DCoffset_I_nom<<8);
    
    enum polarities polarityNow;   
    if(sampleV_minusDC_long > 0) { 
      polarityNow = POSITIVE; }
    else { 
      polarityNow = NEGATIVE; }
    
    if (polarityNow == POSITIVE) 
    { 
      if (polarityOfLastSampleV[phase] != POSITIVE)
      {
        if (phase == 0)
        {
          #ifdef WORKLOAD_CHECK
            long processingDelayAtStartOfDatalogPeriod = millis() - timeAtStartOfDatalogPeriod;
          #endif
        }
        
        long latestDCoffset_V_long = cumVdeltasThisCycle_long[phase] / samplesDuringThisMainsCycle[phase];
        DCoffset_V_long[phase] = latestDCoffset_V_long;
        
        if (DCoffset_V_long[phase] < DCoffset_V_min) {
          DCoffset_V_long[phase] = DCoffset_V_min; }
        else  
        if (DCoffset_V_long[phase] > DCoffset_V_max) {
          DCoffset_V_long[phase] = DCoffset_V_max; }
        
        float energyContribution = (float)sumP[phase] / samplesDuringThisMainsCycle[phase]; 
        processLatestContribution(phase, energyContribution);
        
        if (phase == 0)
        {
          datalogCountInMainsCycles++; // Compteur pour périodes datalogging
          
          byte tempLoad;
          
          // ========== MODIFICATION: Ignorer gestion charges si mode forçage actif ==========
          if (!modeForceActif) 
          {
            if (energyInBucket_main > midPointOfEnergyBucket_main)
            {
              boolean OK_toAddLoad = true;
              if (recentTransition)
              {
                if (postTransitionCount < POST_TRANSITION_MAX_COUNT)
                {
                  postTransitionCount++;
                  OK_toAddLoad = false;
                }
                else
                {
                  recentTransition = false;
                }
              }
              
              if (OK_toAddLoad)
              {
                tempLoad = nextLogicalLoadToBeAdded(); 
                if (tempLoad < noOfDumploads)
                {
                  logicalLoadState[tempLoad] = LOAD_ON;
                  activeLoad = tempLoad;
                  postTransitionCount = 0;
                  recentTransition = true;
                }
              }
            }
            else
            {
              boolean OK_toRemoveLoad = true;
              if (recentTransition)
              {
                if (postTransitionCount < POST_TRANSITION_MAX_COUNT)
                {
                  postTransitionCount++;
                  OK_toRemoveLoad = false;
                }
                else
                {
                  recentTransition = false;
                }
              }
              
              if (OK_toRemoveLoad)
              {        
                tempLoad = nextlogicalLoadToBeRemoved();
                if (tempLoad < noOfDumploads)
                {
                  logicalLoadState[tempLoad] = LOAD_OFF;
                  activeLoad = tempLoad;
                  postTransitionCount = 0;
                  recentTransition = true;
                }
              }
            }
          }
          // ================================================================================
          
          updatePhysicalLoadStates(); 
          
          // Mettre à jour les ports de contrôle pour chacune des charges physiques
          digitalWrite(physicalLoad_0_pin, physicalLoadState[0]);
          digitalWrite(physicalLoad_1_pin, physicalLoadState[1]);
          digitalWrite(physicalLoad_2_pin, physicalLoadState[2]);
          
          if (energyInBucket_main > capacityOfEnergyBucket_main) { 
            energyInBucket_main = capacityOfEnergyBucket_main; } 
          else         
          if (energyInBucket_main < 0) {
            energyInBucket_main = 0; }  
          
          // Datalogging - envoi RF si période écoulée
          if (datalogCountInMainsCycles >= maxDatalogCountInMainsCycles)
          {
            datalogCountInMainsCycles = 0;
            
            tx_data.power_L1 = energyStateOfPhase[0] / maxDatalogCountInMainsCycles;
            tx_data.power_L1 *= -1; // to match the OEM convention (import is =ve; export is -ve)
            tx_data.power_L2 = energyStateOfPhase[1] / maxDatalogCountInMainsCycles;
            tx_data.power_L2 *= -1;
            tx_data.power_L3 = energyStateOfPhase[2] / maxDatalogCountInMainsCycles;
            tx_data.power_L3 *= -1;
            tx_data.Vrms_L1 = (int)(voltageCal[0] * sqrt(sum_Vsquared[0] / samplesDuringThisDatalogPeriod) * 4);
            tx_data.Vrms_L2 = (int)(voltageCal[1] * sqrt(sum_Vsquared[1] / samplesDuringThisDatalogPeriod) * 4);
            tx_data.Vrms_L3 = (int)(voltageCal[2] * sqrt(sum_Vsquared[2] / samplesDuringThisDatalogPeriod) * 4);
            
            #ifdef RF_PRESENT
              send_rf_data();
            #endif
            
            energyStateOfPhase[0] = 0;
            energyStateOfPhase[1] = 0;
            energyStateOfPhase[2] = 0;
            sum_Vsquared[0] = 0;
            sum_Vsquared[1] = 0;
            sum_Vsquared[2] = 0;
            samplesDuringThisDatalogPeriod = 0;
          }
        }
        
        sumP[phase] = 0;
        samplesDuringThisMainsCycle[phase] = 0;
        cumVdeltasThisCycle_long[phase] = 0;
        
        #ifdef WORKLOAD_CHECK
          if (phase == 0)
          {
            if (sampleSetsDuringThisMainsCycle < lowestNoOfSampleSetsPerMainsCycle) {
              lowestNoOfSampleSetsPerMainsCycle = sampleSetsDuringThisMainsCycle; }
          }
        #endif
      }
    }
    
    // Déphaser la forme d'onde de la tension pour qu'elle s'aligne avec le courant lorsqu'une
    // charge résistive est utilisée
    long  phaseShiftedSampleV_minusDC_long = lastSampleV_minusDC_long[phase]
         + (((sampleV_minusDC_long - lastSampleV_minusDC_long[phase])*phaseCal_int[phase])>>8); 

     // calculer la « puissance réelle » de cette paire d'exemples et l'ajouter à la somme cumulée
    long filtV_div4 = phaseShiftedSampleV_minusDC_long>>2;  // reduce to 16-bits (x64, or 2^6)
    long filtI_div4 = sampleIminusDC_long>>2; // reduce to 16-bits (x64, or 2^6)
    long instP = filtV_div4 * filtI_div4;  // 32-bits (x4096, or 2^12)
    instP = instP>>12;    // réduit à 20 bits (x1) 
    sumP[phase] +=instP; // l'échelle est x1
 
    // pour le calcul de la Vrms (pour l'enregistrement des données uniquement)
    long inst_Vsquared = filtV_div4 * filtV_div4; // 32 bits (x4096, ou 2^12)
    // inst_Vsquared = inst_Vsquared>>12; // 20 bits (x1), plage insuffisante :-(
    inst_Vsquared = inst_Vsquared>>16;    // 16 bits (x1/16, ou 2^-4), pour une plage d'enregistrement de données plus étendue :-)    
    sum_Vsquared[phase] += inst_Vsquared; // l'échelle est de x1/16
    if (phase == 0) {
      samplesDuringThisDatalogPeriod ++; } // Il n'est pas nécessaire de tenir des comptes séparés pour chaque phase
   
    // ménage général
    cumVdeltasThisCycle_long[phase] += sampleV_minusDC_long; // à utiliser avec un filtre passe-bas
    samplesDuringThisMainsCycle[phase] ++;
    
    // Stocker les éléments pour une utilisation lors de la prochaine boucle
    lastSampleV_minusDC_long[phase] = sampleV_minusDC_long;  // requis pour l'algorithme phaseCal
    polarityOfLastSampleV[phase] = polarityNow;  // pour l'identification des limites de demi-cycle
  }
}
// fin du processRawSamples()

void processLatestContribution(byte phase, float power)
{
  float latestEnergyContribution = power; // Par souci d'efficacité, l'échelle d'énergie est exprimée en joules * cycles par seconde.

// ajouter la dernière contribution énergétique à l'accumulateur par phase concerné
  // (utilisé uniquement pour l'enregistrement des données de puissance)
  energyStateOfPhase[phase] += latestEnergyContribution;  
  
   // ajouter la dernière contribution énergétique à l'accumulateur d'énergie principal
  energyInBucket_main += latestEnergyContribution;
  
  // appliquer les ajustements nécessaires.
  if (phase == 0)
  {
    energyInBucket_main -= REQUIRED_EXPORT_IN_WATTS; // L'échelle d'énergie est de 50 joules.
  }

  // Application de limites maximales et minimales au niveau de l'accumulateur principal
  // est reportée jusqu'à ce que les décisions relatives à l'énergie aient été prises.
  //
}

byte nextLogicalLoadToBeAdded()
{ 
  byte retVal = noOfDumploads; 
  boolean success = false;
  
  // MODIFICATION: Pour ballon triphasé, activer charge 0 et 1 ENSEMBLE
  // Vérifier si charge 0 ou 1 est OFF
    if (logicalLoadState[0] == LOAD_OFF || logicalLoadState[1] == LOAD_OFF) 
    {
      // Activer les DEUX charges ensemble (ballon triphasé)
    logicalLoadState[0] = LOAD_ON;
    logicalLoadState[1] = LOAD_ON;
    success = true;
    retVal = 0; // Retourner 0 pour indiquer qu'on a ajouté une charge
  }
  // Charge 2 (contacteur ballon 2) gérée séparément
  else if (logicalLoadState[2] == LOAD_OFF)
  {
    if (sondePresente && ! sondeDefaillante && temperatureBallon1 >= TEMP_SWITCH_BALLON2)
    {
      success = true; 
      retVal = 2;
  }
  }
  return(retVal);
}


byte nextlogicalLoadToBeRemoved()
{
  byte retVal = noOfDumploads;
  boolean success = false;
  
  // D'abord retirer charge 2 (contacteur)
    if (logicalLoadState[2] == LOAD_ON)
    {
      success = true;
      retVal =2;
    }
  //puis retirer charges 0 et 1 ENSEMBLE
  else if (logicalLoadState[0] == LOAD_ON || logicalLoadState[1] == LOAD_ON)
  {
  logicalLoadState[0] = LOAD_OFF;
  logicalLoadState[1] = LOAD_OFF;

  retVal = 0;
  }
  
   return(retVal) ;
}


void updatePhysicalLoadStates()
/*
 * Cette fonction assure la liaison entre les charges logiques et physiques.
 * Le tableau `logicalLoadState[]` contient l'état activé/désactivé de toutes les charges logiques,
 * l'élément 0 correspondant à la charge ayant la priorité la plus élevée.
 * Le tableau `physicalLoadState[]` contient l'état activé/désactivé de toutes les charges physiques.
 * 
 * L'association entre les charges physiques et logiques est de 1:1. Par défaut, l'équivalence numérique est maintenue,
 * de sorte que logique(N) correspond à physique(N). Si la charge physique 1 est définie comme prioritaire,
 * plutôt que la charge physique 0, l'association logique-physique des charges 0 et 1 est inversée.
 *
 * Toute autre relation de mappage pourrait être configurée ici.
 */
{
  for (int i = 0; i < noOfDumploads; i++)
  {
    physicalLoadState[i] = logicalLoadState[i]; 
  }
   
  
  // Application des états physiques aux pins
  digitalWrite(physicalLoad_0_pin, physicalLoadState[0]);
  digitalWrite(physicalLoad_1_pin, physicalLoadState[1]);
  digitalWrite(physicalLoad_2_pin, physicalLoadState[2]);
}



// Bien que ce programme fonctionne toujours en mode ANTI_FLICKER, il était pratique
// de laisser ce mécanisme en place.
//
void configureParamsForSelectedOutputMode()
{  
  if (outputMode == ANTI_FLICKER)
  {
    lowerThreshold_default = 
       capacityOfEnergyBucket_main * (0.5 - offsetOfEnergyThresholdsInAFmode); 
    upperThreshold_default = 
       capacityOfEnergyBucket_main * (0.5 + offsetOfEnergyThresholdsInAFmode);   
  }
  else
  { 
    // Paramètres pour le mode normal
    lowerThreshold_default = capacityOfEnergyBucket_main * 0.5; 
    upperThreshold_default = capacityOfEnergyBucket_main * 0.5;   
  }
  
  // afficher les paramètres pertinents pour le mode de sortie sélectionné
  Serial.print("  capacityOfEnergyBucket_main = ");
  Serial.println(capacityOfEnergyBucket_main);
  Serial.print("  lowerEnergyThreshold   = ");
  Serial.println(lowerThreshold_default);
  Serial.print("  upperEnergyThreshold   = ");
  Serial.println(upperThreshold_default);
  
  Serial.print(">>free RAM = ");
  Serial.println(freeRam());  // une valeur utile à surveiller
}
 
 
#ifdef RF_PRESENT
void send_rf_data()
{
  int i = 0; 
  while (!rf12_canSend() && i<10)
  { 
    rf12_recvDone(); 
    i++;
  }
  rf12_sendStart(0, &tx_data, sizeof tx_data);
}
#endif


int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}