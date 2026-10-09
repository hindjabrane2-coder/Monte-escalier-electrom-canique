/*
 * Monte-escalier electromecanique
 * Arduino Mega 2560 -- Commande moteur DC 24V
 * Pignon-cremaillere, frein electromagnetique, capteurs multiples
 *
 * Stage EMPEGEC
 */

#include <EEPROM.h>
#include <avr/wdt.h>

// Configuration des broches

// Pont en H / relais de puissance
#define PIN_MOTOR_EN    9     // sortie PWM vers driver (signal ENABLE)
#define PIN_MOTOR_DIR1  22    // relais sens montee
#define PIN_MOTOR_DIR2  23    // relais sens descente
#define PIN_BRAKE       24    // relais frein electromagnetique (LOW = frein serre)

// Capteurs de fin de course (contacts NO, actifs a LOW grace au pullup)
#define PIN_FC_HAUT     30
#define PIN_FC_BAS      31

// Commandes utilisateur (homme-mort, actifs a LOW)
#define PIN_BTN_MONTE   32
#define PIN_BTN_DESC    33
#define PIN_ARU         34    // arret d'urgence (NF : ouvert = urgence)

// Capteur de courant moteur (ACS712-20A sur shunt)
#define PIN_COURANT     A0

// Sonde de temperature NTC sur le carter moteur
#define PIN_TEMP_MOTEUR A1

// Tension batterie via pont diviseur (ratio 1:4)
#define PIN_V_BATT      A2

// Encodeur incremental sur l'arbre de sortie du reducteur
#define PIN_ENC_A       2     // interruption INT0
#define PIN_ENC_B       3     // interruption INT1

// Indicateur LED RGB (anode commune : LOW = allume)
#define PIN_LED_R       5
#define PIN_LED_G       6
#define PIN_LED_B       7

// Buzzer piezo
#define PIN_BUZZER      8

// Parametres de fonctionnement

// Vitesse
#define PWM_MAX         200   // PWM max en regime normal (sur 255)
#define PWM_MIN         60    // PWM minimum pour vaincre l'inertie
#define PWM_LENT        100   // vitesse reduite pres des fins de course
#define ZONE_LENTE_MS   3000  // duree de la zone de ralentissement (ms)

// Rampe en S (acceleration douce)
#define RAMPE_PERIODE   15    // ms entre deux pas de rampe
#define RAMPE_PAS_INIT  1     // increment initial (demarrage doux)
#define RAMPE_PAS_MAX   4     // increment maximal (milieu de rampe)

// Courant (valeurs ADC, ACS712-20A : 512 = 0A, 25.6 unites/A)
#define COURANT_ZERO    512   // offset ADC pour 0A
#define SEUIL_AVERT     640   // seuil d'avertissement (~5A)
#define SEUIL_COUPURE   720   // seuil de coupure (~8A)
#define SEUIL_COURT_CIRC 850  // court-circuit (>13A)
#define FILTRE_COURANT  8     // nombre d'echantillons pour moyennage

// Temperature moteur (NTC 10k, beta 3950, avec pont diviseur 10k)
#define TEMP_AVERT      65    // seuil avertissement en degres C
#define TEMP_COUPURE    80    // seuil de coupure thermique

// Batterie (pont diviseur 1:4, reference 5V)
#define BATT_ADC_PLEINE 880   // ~26V
#define BATT_ADC_BASSE  800   // ~23.5V
#define BATT_ADC_VIDE   740   // ~21.7V (coupure)

// Encodeur
#define IMPULSIONS_PAR_TOUR 200
#define MM_PAR_IMPULSION    0.628  // pi * D_pignon / (Z * impulsions_par_tour * rapport)

// Communication
#define SERIAL_BAUD     115200
#define TIMEOUT_COMM_MS 10000  // timeout si le PC ne repond plus (0 = desactive)

// EEPROM
#define ADDR_COMPTEUR_TRAJETS 0    // uint32_t : nombre total de trajets
#define ADDR_DISTANCE_TOTALE  4    // uint32_t : distance totale en mm
#define ADDR_ERREURS          8    // 10 derniers codes d'erreur (10 octets)
#define ADDR_NB_ERREURS       18   // uint8_t
#define ADDR_CONFIG_PWM_MAX   20   // uint8_t
#define ADDR_CONFIG_SEUIL_I   21   // uint16_t

// Codes d'erreur
#define ERR_AUCUNE          0x00
#define ERR_SURINTENS       0x01   // E01 : surintensite
#define ERR_COURT_CIRCUIT   0x02   // E02 : court-circuit detecte
#define ERR_FC_INCOHERENT   0x03   // E03 : les deux fins de course actifs
#define ERR_SURCHAUFFE      0x04   // E04 : temperature moteur excessive
#define ERR_BATT_VIDE       0x05   // E05 : batterie trop basse
#define ERR_ENCODEUR        0x06   // E06 : encodeur bloque (pas d'impulsions)
#define ERR_TIMEOUT_COMM    0x07   // E07 : timeout communication PC
#define ERR_WATCHDOG        0x08   // E08 : reset par watchdog
#define ERR_FREIN            0x09   // E09 : frein ne s'est pas deserre
#define ERR_AUTOTEST        0x0A   // E10 : echec de l'autotest au demarrage

// Machine a etats
enum Etat {
    ARRET,
    MONTEE,
    DESCENTE,
    RALENTISSEMENT,   // freinage en approche de fin de course
    URGENCE,
    MAINTENANCE
};

// Variables globales
Etat etat = ARRET;
uint8_t derniere_erreur = ERR_AUCUNE;
int pwm_actuel = 0;
int pwm_cible = 0;
unsigned long dernier_pas_rampe = 0;
unsigned long debut_mouvement = 0;
unsigned long derniere_comm = 0;
bool mode_maintenance = false;

// compteur d'impulsions de l'encodeur (modifie dans l'ISR)
volatile long compteur_encodeur = 0;
volatile unsigned long derniere_impulsion = 0;

// position estimee du chariot en mm (0 = bas, max = haut)
long position_mm = 0;
bool position_calibree = false;

// statistiques du trajet en cours
unsigned long duree_trajet_ms = 0;
int courant_max_trajet = 0;

// filtrage du courant
int tampon_courant[FILTRE_COURANT];
int index_courant = 0;
int courant_filtre = COURANT_ZERO;

// compteurs persistants (charges depuis EEPROM)
uint32_t compteur_trajets = 0;
uint32_t distance_totale_mm = 0;

// version du firmware
const char VERSION[] = "2.4.1";

// Prototypes
void arreter_moteur();
void isr_encodeur_a();
int lire_courant_filtre();
int lire_temperature_moteur();
int lire_tension_batterie_mv();
void led_couleur(bool r, bool g, bool b);
void bip(int frequence, int duree_ms);
void enregistrer_erreur(uint8_t code);
void autotest();

// Interruption encodeur
void isr_encodeur_a() {
    // lecture de la voie B pour determiner le sens
    if (digitalRead(PIN_ENC_B) == HIGH) {
        compteur_encodeur++;
    } else {
        compteur_encodeur--;
    }
    derniere_impulsion = millis();
}

// Setup
void setup() {
    Serial.begin(SERIAL_BAUD);

    // sorties moteur
    pinMode(PIN_MOTOR_EN, OUTPUT);
    pinMode(PIN_MOTOR_DIR1, OUTPUT);
    pinMode(PIN_MOTOR_DIR2, OUTPUT);
    pinMode(PIN_BRAKE, OUTPUT);

    // capteurs
    pinMode(PIN_FC_HAUT, INPUT_PULLUP);
    pinMode(PIN_FC_BAS, INPUT_PULLUP);
    pinMode(PIN_BTN_MONTE, INPUT_PULLUP);
    pinMode(PIN_BTN_DESC, INPUT_PULLUP);
    pinMode(PIN_ARU, INPUT_PULLUP);
    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);

    // indicateurs
    pinMode(PIN_LED_R, OUTPUT);
    pinMode(PIN_LED_G, OUTPUT);
    pinMode(PIN_LED_B, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);

    // etat initial : tout coupe, frein serre
    arreter_moteur();

    // interruption encodeur
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isr_encodeur_a, RISING);

    // chargement des compteurs EEPROM
    EEPROM.get(ADDR_COMPTEUR_TRAJETS, compteur_trajets);
    EEPROM.get(ADDR_DISTANCE_TOTALE, distance_totale_mm);
    // valeurs non initialisees (0xFFFFFFFF) -> remise a zero
    if (compteur_trajets == 0xFFFFFFFF) compteur_trajets = 0;
    if (distance_totale_mm == 0xFFFFFFFF) distance_totale_mm = 0;

    // initialiser le tampon de courant
    for (int i = 0; i < FILTRE_COURANT; i++) {
        tampon_courant[i] = COURANT_ZERO;
    }

    // calibration de la position par fin de course
    if (digitalRead(PIN_FC_BAS) == LOW) {
        position_mm = 0;
        position_calibree = true;
    } else if (digitalRead(PIN_FC_HAUT) == LOW) {
        position_mm = 5000; // longueur du rail en mm
        position_calibree = true;
    }

    // bip de demarrage
    bip(1000, 100);
    delay(50);
    bip(1500, 100);

    // autotest au demarrage
    autotest();

    // activer le watchdog (timeout 2 secondes)
    wdt_enable(WDTO_2S);

    led_couleur(false, true, false); // vert = pret

    Serial.print("Monte-escalier v");
    Serial.print(VERSION);
    Serial.print(" pret. Trajets: ");
    Serial.println(compteur_trajets);

    derniere_comm = millis();
}

// Boucle principale
void loop() {
    wdt_reset();
    unsigned long maintenant = millis();

    // --- Arret d'urgence (priorite absolue) ---
    if (digitalRead(PIN_ARU) == HIGH) {
        if (etat != URGENCE) {
            arreter_moteur();
            etat = URGENCE;
            derniere_erreur = ERR_AUCUNE; // l'ARU n'est pas une erreur interne
            led_couleur(true, false, false);
            bip(500, 500);
            Serial.println("!ARU");
        }
        return;
    }

    // --- Acquittement apres ARU ---
    if (etat == URGENCE && derniere_erreur == ERR_AUCUNE) {
        // il faut relacher les boutons avant de pouvoir repartir
        if (digitalRead(PIN_BTN_MONTE) == HIGH && digitalRead(PIN_BTN_DESC) == HIGH) {
            etat = ARRET;
            led_couleur(false, true, false);
            Serial.println("ARU acquitte");
        }
        return;
    }

    // apres une erreur interne, seule la commande RESET par serie peut rearmer
    if (etat == URGENCE) {
        traiter_serie();
        led_clignotante_erreur(maintenant);
        return;
    }

    // --- Lecture des capteurs ---
    int courant_brut = lire_courant_filtre();
    int temperature = lire_temperature_moteur();
    int tension_batt = lire_tension_batterie_mv();
    bool fc_haut = (digitalRead(PIN_FC_HAUT) == LOW);
    bool fc_bas = (digitalRead(PIN_FC_BAS) == LOW);

    // --- Verification coherence fins de course ---
    if (fc_haut && fc_bas) {
        declarer_erreur(ERR_FC_INCOHERENT);
        return;
    }

    // --- Verification court-circuit ---
    if (courant_brut > SEUIL_COURT_CIRC && (etat == MONTEE || etat == DESCENTE)) {
        declarer_erreur(ERR_COURT_CIRCUIT);
        return;
    }

    // --- Verification surintensite ---
    if (courant_brut > SEUIL_COUPURE && (etat == MONTEE || etat == DESCENTE)) {
        declarer_erreur(ERR_SURINTENS);
        return;
    }

    // --- Avertissement courant ---
    if (courant_brut > SEUIL_AVERT && (etat == MONTEE || etat == DESCENTE)) {
        Serial.print("W:I_ELEVE=");
        Serial.println(courant_brut);
    }

    // --- Verification temperature ---
    if (temperature > TEMP_COUPURE) {
        declarer_erreur(ERR_SURCHAUFFE);
        return;
    }

    // --- Verification batterie ---
    if (tension_batt < BATT_ADC_VIDE && etat != ARRET) {
        declarer_erreur(ERR_BATT_VIDE);
        return;
    }

    // --- Timeout communication (si active) ---
    if (TIMEOUT_COMM_MS > 0 && (maintenant - derniere_comm > TIMEOUT_COMM_MS)) {
        if (etat == MONTEE || etat == DESCENTE) {
            // on ne coupe pas le mouvement mais on signale
            Serial.println("W:TIMEOUT_COMM");
        }
    }

    // --- Mise a jour de la position ---
    noInterrupts();
    long enc = compteur_encodeur;
    interrupts();
    position_mm = (long)(enc * MM_PAR_IMPULSION);

    // recalibration sur fin de course
    if (fc_bas) {
        compteur_encodeur = 0;
        position_mm = 0;
        position_calibree = true;
    }
    if (fc_haut) {
        position_calibree = true;
    }

    // --- Verification encodeur (blocage mecanique) ---
    if ((etat == MONTEE || etat == DESCENTE) && pwm_actuel > PWM_MIN + 30) {
        if (maintenant - derniere_impulsion > 2000) {
            // pas d'impulsion depuis 2 secondes alors qu'on roule
            declarer_erreur(ERR_ENCODEUR);
            return;
        }
    }

    // --- Lecture boutons ---
    bool btn_monte = (digitalRead(PIN_BTN_MONTE) == LOW);
    bool btn_desc = (digitalRead(PIN_BTN_DESC) == LOW);

    // appui simultane = arret
    if (btn_monte && btn_desc) {
        btn_monte = false;
        btn_desc = false;
    }

    // --- Suivi du courant max pendant le trajet ---
    if (etat == MONTEE || etat == DESCENTE) {
        if (courant_brut > courant_max_trajet) {
            courant_max_trajet = courant_brut;
        }
    }

    // --- Machine a etats ---
    switch (etat) {
        case ARRET:
            if (btn_monte && !fc_haut) {
                demarrer_mouvement(true);
            } else if (btn_desc && !fc_bas) {
                demarrer_mouvement(false);
            }
            break;

        case MONTEE:
            if (!btn_monte || fc_haut) {
                terminer_mouvement(fc_haut ? "FC_HAUT" : "BTN_RELACHE");
            } else {
                // ralentir en approche de fin de course
                int cible = calculer_pwm_cible(true, maintenant);
                gerer_rampe_s(cible, maintenant);
            }
            break;

        case DESCENTE:
            if (!btn_desc || fc_bas) {
                terminer_mouvement(fc_bas ? "FC_BAS" : "BTN_RELACHE");
            } else {
                int cible = calculer_pwm_cible(false, maintenant);
                gerer_rampe_s(cible, maintenant);
            }
            break;

        case MAINTENANCE:
            // en maintenance, vitesse reduite et le bouton doit rester appuye
            if (btn_monte && !fc_haut) {
                if (pwm_actuel == 0) demarrer_mouvement(true);
                gerer_rampe_s(PWM_LENT, maintenant);
            } else if (btn_desc && !fc_bas) {
                if (pwm_actuel == 0) demarrer_mouvement(false);
                gerer_rampe_s(PWM_LENT, maintenant);
            } else if (pwm_actuel > 0) {
                rampe_arret(maintenant);
                if (pwm_actuel == 0) {
                    arreter_moteur();
                    etat = MAINTENANCE;
                }
            }
            break;

        default:
            arreter_moteur();
            etat = ARRET;
            break;
    }

    // --- Communication serie ---
    traiter_serie();

    // --- Indicateur LED selon tension batterie ---
    if (etat == ARRET && tension_batt < BATT_ADC_BASSE) {
        // batterie basse : orange fixe
        led_couleur(true, true, false);
    }
}

// Gestion du mouvement

void demarrer_mouvement(bool montee) {
    // desserrer le frein avant de mettre le moteur en route
    digitalWrite(PIN_BRAKE, HIGH);
    // petit delai pour que le frein s'ouvre mecaniquement
    // (on fait ca en non-bloquant via la rampe qui demarre a PWM_MIN)
    unsigned long t0 = millis();
    while (millis() - t0 < 80) {
        wdt_reset(); // le watchdog ne doit pas declencher ici
    }

    if (montee) {
        digitalWrite(PIN_MOTOR_DIR1, HIGH);
        digitalWrite(PIN_MOTOR_DIR2, LOW);
        etat = MONTEE;
        Serial.println(">>MONTEE");
    } else {
        digitalWrite(PIN_MOTOR_DIR1, LOW);
        digitalWrite(PIN_MOTOR_DIR2, HIGH);
        etat = DESCENTE;
        Serial.println(">>DESCENTE");
    }

    pwm_actuel = PWM_MIN;
    pwm_cible = PWM_MAX;
    analogWrite(PIN_MOTOR_EN, pwm_actuel);
    debut_mouvement = millis();
    courant_max_trajet = COURANT_ZERO;

    // bip court d'avertissement
    bip(800, 80);

    led_couleur(true, true, false); // orange = en mouvement
}

void terminer_mouvement(const char* raison) {
    // rampe d'arret rapide
    analogWrite(PIN_MOTOR_EN, 0);
    digitalWrite(PIN_MOTOR_DIR1, LOW);
    digitalWrite(PIN_MOTOR_DIR2, LOW);
    digitalWrite(PIN_BRAKE, LOW); // serrer le frein

    duree_trajet_ms = millis() - debut_mouvement;
    pwm_actuel = 0;
    pwm_cible = 0;

    // sauvegarder les stats
    compteur_trajets++;
    long distance_parcourue = abs(position_mm); // approximation
    distance_totale_mm += (uint32_t)abs(duree_trajet_ms); // en temps comme proxy

    // ecriture EEPROM tous les 5 trajets pour limiter l'usure
    if (compteur_trajets % 5 == 0) {
        EEPROM.put(ADDR_COMPTEUR_TRAJETS, compteur_trajets);
        EEPROM.put(ADDR_DISTANCE_TOTALE, distance_totale_mm);
    }

    etat = mode_maintenance ? MAINTENANCE : ARRET;
    led_couleur(false, true, false);

    Serial.print("<<ARRET:");
    Serial.print(raison);
    Serial.print(",T=");
    Serial.print(duree_trajet_ms);
    Serial.print("ms,Imax=");
    Serial.println(courant_max_trajet);
}

void arreter_moteur() {
    analogWrite(PIN_MOTOR_EN, 0);
    digitalWrite(PIN_MOTOR_DIR1, LOW);
    digitalWrite(PIN_MOTOR_DIR2, LOW);
    digitalWrite(PIN_BRAKE, LOW); // frein serre par defaut (anti-rollback)
    pwm_actuel = 0;
    pwm_cible = 0;
}

// Rampe en S (acceleration non lineaire)

void gerer_rampe_s(int cible, unsigned long maintenant) {
    if (maintenant - dernier_pas_rampe < RAMPE_PERIODE) return;
    dernier_pas_rampe = maintenant;

    if (pwm_actuel == cible) return;

    // calculer le pas variable (forme en S)
    // le pas est faible au debut et a la fin, maximal au milieu
    int distance_totale = abs(cible - PWM_MIN);
    int distance_parcourue = abs(pwm_actuel - PWM_MIN);
    float progression = 0;
    if (distance_totale > 0) {
        progression = (float)distance_parcourue / distance_totale;
    }

    // fonction en cloche pour le pas : sin(pi * progression)
    float facteur = sin(progression * 3.14159);
    if (facteur < 0.15) facteur = 0.15; // pas minimum pour ne pas stagner
    int pas = RAMPE_PAS_INIT + (int)((RAMPE_PAS_MAX - RAMPE_PAS_INIT) * facteur);

    if (pwm_actuel < cible) {
        pwm_actuel = min(pwm_actuel + pas, cible);
    } else {
        pwm_actuel = max(pwm_actuel - pas, cible);
    }

    analogWrite(PIN_MOTOR_EN, pwm_actuel);
}

void rampe_arret(unsigned long maintenant) {
    // deceleration rapide pour l'arret
    if (maintenant - dernier_pas_rampe < RAMPE_PERIODE) return;
    dernier_pas_rampe = maintenant;

    pwm_actuel = max(pwm_actuel - RAMPE_PAS_MAX, 0);
    analogWrite(PIN_MOTOR_EN, pwm_actuel);
}

// Calcul du PWM cible (ralentissement pres des extremites)

int calculer_pwm_cible(bool montee, unsigned long maintenant) {
    unsigned long duree = maintenant - debut_mouvement;
    int cible = mode_maintenance ? PWM_LENT : PWM_MAX;

    // ralentir au demarrage (les premieres ZONE_LENTE_MS millisecondes)
    if (duree < ZONE_LENTE_MS) {
        cible = PWM_LENT;
    }

    // si on est proche d'un fin de course (base sur le temps de trajet estime)
    // en l'absence de position calibree, on utilise le temps comme proxy
    // duree typique d'un trajet complet : ~63 secondes
    unsigned long duree_estimee_totale = 63000;
    if (duree > duree_estimee_totale - ZONE_LENTE_MS) {
        cible = PWM_LENT;
    }

    // si la position est calibree, on peut faire mieux
    if (position_calibree) {
        long pos = position_mm;
        if (montee && pos > 4500) cible = PWM_LENT; // 500mm avant le haut
        if (!montee && pos < 500) cible = PWM_LENT;  // 500mm avant le bas
    }

    return cible;
}

// Capteurs

int lire_courant_filtre() {
    tampon_courant[index_courant] = analogRead(PIN_COURANT);
    index_courant = (index_courant + 1) % FILTRE_COURANT;

    long somme = 0;
    for (int i = 0; i < FILTRE_COURANT; i++) {
        somme += tampon_courant[i];
    }
    courant_filtre = (int)(somme / FILTRE_COURANT);
    return courant_filtre;
}

int lire_temperature_moteur() {
    int adc = analogRead(PIN_TEMP_MOTEUR);
    // conversion NTC 10k (beta 3950) avec pont diviseur 10k
    // formule simplifiee calibree experimentalement
    float resistance = 10000.0 * adc / (1023.0 - adc);
    float temp_k = 1.0 / (1.0 / 298.15 + log(resistance / 10000.0) / 3950.0);
    return (int)(temp_k - 273.15);
}

int lire_tension_batterie_mv() {
    int adc = analogRead(PIN_V_BATT);
    // pont diviseur 1:4, reference 5V
    // tension_reelle = adc * 5.0 / 1023.0 * 4.0
    // en millivolts :
    return (int)((long)adc * 19570 / 1023);
}

// Indicateurs

void led_couleur(bool r, bool g, bool b) {
    // anode commune : LOW = allume
    digitalWrite(PIN_LED_R, r ? LOW : HIGH);
    digitalWrite(PIN_LED_G, g ? LOW : HIGH);
    digitalWrite(PIN_LED_B, b ? LOW : HIGH);
}

void led_clignotante_erreur(unsigned long maintenant) {
    // clignotement rouge a 2Hz en cas d'erreur
    bool allume = ((maintenant / 250) % 2 == 0);
    led_couleur(allume, false, false);
}

void bip(int frequence, int duree_ms) {
    tone(PIN_BUZZER, frequence, duree_ms);
}

void bip_erreur() {
    // trois bips descendants
    bip(1200, 150);
    delay(80);
    bip(800, 150);
    delay(80);
    bip(500, 300);
}

// Gestion des erreurs

void declarer_erreur(uint8_t code) {
    arreter_moteur();
    etat = URGENCE;
    derniere_erreur = code;
    enregistrer_erreur(code);
    led_couleur(true, false, false);
    bip_erreur();

    Serial.print("!ERR:E");
    if (code < 10) Serial.print("0");
    Serial.println(code);
}

void enregistrer_erreur(uint8_t code) {
    // journal circulaire de 10 erreurs en EEPROM
    uint8_t nb = EEPROM.read(ADDR_NB_ERREURS);
    if (nb >= 10 || nb == 0xFF) nb = 0;
    EEPROM.write(ADDR_ERREURS + nb, code);
    nb++;
    EEPROM.write(ADDR_NB_ERREURS, nb);
}

// Autotest au demarrage

void autotest() {
    Serial.println("Autotest...");
    bool ok = true;

    // verifier que le courant est proche de zero au repos
    int i_repos = analogRead(PIN_COURANT);
    if (abs(i_repos - COURANT_ZERO) > 30) {
        Serial.println("  FAIL: offset courant anormal");
        ok = false;
    } else {
        Serial.println("  OK: capteur courant");
    }

    // verifier la tension batterie
    int v = lire_tension_batterie_mv();
    if (v < 18000) {
        Serial.println("  FAIL: tension batterie trop basse");
        ok = false;
    } else {
        Serial.print("  OK: batterie ");
        Serial.print(v);
        Serial.println(" mV");
    }

    // verifier la temperature
    int t = lire_temperature_moteur();
    if (t > 50 || t < -10) {
        Serial.print("  WARN: temperature inhabituelle: ");
        Serial.println(t);
    } else {
        Serial.print("  OK: temperature ");
        Serial.print(t);
        Serial.println(" C");
    }

    // verifier que les deux fins de course ne sont pas actifs ensemble
    if (digitalRead(PIN_FC_HAUT) == LOW && digitalRead(PIN_FC_BAS) == LOW) {
        Serial.println("  FAIL: deux FC actifs");
        ok = false;
    } else {
        Serial.println("  OK: fins de course");
    }

    // verifier que l'ARU n'est pas active
    if (digitalRead(PIN_ARU) == HIGH) {
        Serial.println("  WARN: ARU active au demarrage");
    }

    if (!ok) {
        Serial.println("Autotest ECHEC -- mode degrade");
        enregistrer_erreur(ERR_AUTOTEST);
        bip(300, 1000);
    } else {
        Serial.println("Autotest OK");
    }
}

// Communication serie

void traiter_serie() {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    derniere_comm = millis();

    if (cmd == "STATUS") {
        envoyer_statut();
    }
    else if (cmd == "STOP") {
        arreter_moteur();
        etat = mode_maintenance ? MAINTENANCE : ARRET;
        Serial.println("OK:STOP");
    }
    else if (cmd == "MOVE_UP") {
        if (etat == ARRET || etat == MAINTENANCE) {
            if (digitalRead(PIN_FC_HAUT) == HIGH) {
                demarrer_mouvement(true);
                Serial.println("OK:MOVE_UP");
            } else {
                Serial.println("ERR:FC_HAUT");
            }
        } else {
            Serial.println("ERR:BUSY");
        }
    }
    else if (cmd == "MOVE_DOWN") {
        if (etat == ARRET || etat == MAINTENANCE) {
            if (digitalRead(PIN_FC_BAS) == HIGH) {
                demarrer_mouvement(false);
                Serial.println("OK:MOVE_DOWN");
            } else {
                Serial.println("ERR:FC_BAS");
            }
        } else {
            Serial.println("ERR:BUSY");
        }
    }
    else if (cmd == "RESET") {
        if (etat == URGENCE) {
            derniere_erreur = ERR_AUCUNE;
            etat = mode_maintenance ? MAINTENANCE : ARRET;
            led_couleur(false, true, false);
            Serial.println("OK:RESET");
        } else {
            Serial.println("OK:RESET (pas en urgence)");
        }
    }
    else if (cmd == "VERSION") {
        Serial.print("V:");
        Serial.println(VERSION);
    }
    else if (cmd == "POSITION") {
        Serial.print("POS:");
        Serial.print(position_mm);
        Serial.print(",CAL:");
        Serial.println(position_calibree ? "1" : "0");
    }
    else if (cmd == "BATTERY") {
        int v = lire_tension_batterie_mv();
        Serial.print("BATT:");
        Serial.print(v);
        Serial.println("mV");
    }
    else if (cmd == "TEMP") {
        int t = lire_temperature_moteur();
        Serial.print("TEMP:");
        Serial.print(t);
        Serial.println("C");
    }
    else if (cmd == "STATS") {
        Serial.print("TRAJETS:");
        Serial.print(compteur_trajets);
        Serial.print(",DIST:");
        Serial.print(distance_totale_mm);
        Serial.println("mm");
    }
    else if (cmd == "LOG") {
        // afficher les dernieres erreurs enregistrees
        uint8_t nb = EEPROM.read(ADDR_NB_ERREURS);
        if (nb == 0xFF) nb = 0;
        Serial.print("ERREURS:");
        Serial.println(nb);
        for (uint8_t i = 0; i < min(nb, (uint8_t)10); i++) {
            uint8_t code = EEPROM.read(ADDR_ERREURS + i);
            Serial.print("  E");
            if (code < 10) Serial.print("0");
            Serial.println(code);
        }
    }
    else if (cmd == "MAINTENANCE") {
        mode_maintenance = !mode_maintenance;
        if (etat == ARRET || etat == MAINTENANCE) {
            etat = mode_maintenance ? MAINTENANCE : ARRET;
        }
        Serial.print("MAINTENANCE:");
        Serial.println(mode_maintenance ? "ON" : "OFF");
        if (mode_maintenance) {
            led_couleur(false, false, true); // bleu = maintenance
        } else {
            led_couleur(false, true, false);
        }
    }
    else if (cmd.startsWith("CONFIG")) {
        // format : CONFIG:PWM_MAX=180 ou CONFIG:SEUIL_I=700
        int sep = cmd.indexOf('=');
        if (sep > 0) {
            String param = cmd.substring(7, sep);
            int valeur = cmd.substring(sep + 1).toInt();
            if (param == "PWM_MAX" && valeur >= PWM_MIN && valeur <= 255) {
                EEPROM.write(ADDR_CONFIG_PWM_MAX, (uint8_t)valeur);
                Serial.print("OK:PWM_MAX=");
                Serial.println(valeur);
            } else if (param == "SEUIL_I" && valeur > COURANT_ZERO && valeur < 1023) {
                EEPROM.put(ADDR_CONFIG_SEUIL_I, (uint16_t)valeur);
                Serial.print("OK:SEUIL_I=");
                Serial.println(valeur);
            } else {
                Serial.println("ERR:PARAM");
            }
        } else {
            Serial.println("ERR:FORMAT");
        }
    }
    else {
        Serial.print("ERR:CMD_INCONNUE:");
        Serial.println(cmd);
    }
}

void envoyer_statut() {
    Serial.print("ETAT:");
    switch (etat) {
        case ARRET:         Serial.print("ARRET"); break;
        case MONTEE:        Serial.print("MONTEE"); break;
        case DESCENTE:      Serial.print("DESCENTE"); break;
        case RALENTISSEMENT: Serial.print("RALENTI"); break;
        case URGENCE:       Serial.print("URGENCE"); break;
        case MAINTENANCE:   Serial.print("MAINT"); break;
    }
    Serial.print(",FC_H:");
    Serial.print(digitalRead(PIN_FC_HAUT) == LOW ? "1" : "0");
    Serial.print(",FC_B:");
    Serial.print(digitalRead(PIN_FC_BAS) == LOW ? "1" : "0");
    Serial.print(",PWM:");
    Serial.print(pwm_actuel);
    Serial.print(",I:");
    Serial.print(courant_filtre);
    Serial.print(",T:");
    Serial.print(lire_temperature_moteur());
    Serial.print(",V:");
    Serial.print(lire_tension_batterie_mv());
    Serial.print(",POS:");
    Serial.print(position_mm);
    Serial.print(",ERR:E");
    if (derniere_erreur < 10) Serial.print("0");
    Serial.println(derniere_erreur);
}
