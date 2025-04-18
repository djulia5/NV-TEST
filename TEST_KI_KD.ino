#include <Arduino.h>
#include "driver/twai.h"
#include "xiaomi_cybergear_driver.h"
#include <math.h>

// Définitions des broches
#define RX_PIN D7
#define TX_PIN D6
#define BUTTON_PIN D5  // Broche du bouton poussoir

// IDs CAN
static const uint8_t MOTOR_ID_0 = 0x02;  // Moteur 0 0x7F
static const uint8_t MOTOR_ID_1 = 0x7F;  // Moteur 1 0x01
static const uint8_t MOTOR_ID_2 = 0x01;  // Moteur 2 0x02
static const uint8_t MASTER_ID = 0xFD;

// Paramètres ajustables
const float MAX_SPEED = 8.0f;  // Vitesse max en rad/s
//const float KP_VALUE = 2.0f;  // Vitesse max en rad/s
//const float KI_VALUE = 1f;   // Gain intégral (petit pour stabilité)
//const float KD_VALUE = 3f;   // Gain dérivé (amortit oscillations)
const float DEG2RAD = M_PI / 180.0f;  // Conversion degrés → radians
const float TRQ = 15.0f;  // Vitesse max en rad/s
const float CURR = 10.0f;  // Vitesse max en rad/s

// Gains pour les boucles de contrôle
const float KP_POSITION = 30.0f;     // Kp pour la boucle de position (déjà prévu)
const float KD_EFFECTIVE = 15.0f;     // Gain proportionnel de la boucle de vitesse (similaire à Kd)
const float KI_SPEED = 1.0f;       // Gain intégral de la boucle de vitesse (Ki)


// Dimensions du robot
const float e = 214.95f;  // Rayon de la base mobile (mm)
const float f = 424.7f;   // Rayon de la base fixe (mm)
const float re = 538.0f;  // Longueur des bras passifs (mm)
const float rf = 233.0f;  // Longueur des bras actifs (mm)

// Constantes trigonométriques
const float sqrt3 = sqrt(3.0f);
const float tan30 = 1.0f / sqrt3;

// Instances des drivers CyberGear
XiaomiCyberGearDriver motor0(MOTOR_ID_0, MASTER_ID);
XiaomiCyberGearDriver motor1(MOTOR_ID_1, MASTER_ID);
XiaomiCyberGearDriver motor2(MOTOR_ID_2, MASTER_ID);

// Variables pour la gestion du bouton
int etatBouton = 0;         // État actuel du bouton
int dernierEtatBouton = HIGH;  // Dernier état du bouton
unsigned long lastButtonPress = 0;  // Pour le débouncing

// Vérifier et réinitialiser le bus CAN si nécessaire
void check_and_reset_can_bus() {
    twai_status_info_t status;
    twai_get_status_info(&status);
    if (status.state != TWAI_STATE_RUNNING) {
        Serial.println("Bus CAN pas dans l’état RUNNING, réinitialisation...");
        twai_stop();
        delay(100);
        twai_start();
        delay(100);
        motor1.init_twai(RX_PIN, TX_PIN, true);
    }
}

// Définir la position actuelle comme zéro
void set_mechanical_position_to_zero(uint8_t motor_id, uint8_t master_id) {
    check_and_reset_can_bus();

    twai_message_t message;
    message.extd = 1;
    message.identifier = (6UL << 24) | ((uint32_t)master_id << 8) | motor_id;
    message.data_length_code = 8;
    message.data[0] = 1;
    for (uint8_t i = 1; i < 8; i++) {
        message.data[i] = 0;
    }

    esp_err_t ret = twai_transmit(&message, pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        Serial.print("Erreur lors de l’envoi de CMD_SET_MECH_POSITION_TO_ZERO : ");
        Serial.println(esp_err_to_name(ret));
    }
}

// Calcul de l'angle dans le plan YZ
float delta_calc_angle_yz(float x0, float y0, float z0) {
    float y1 = -0.5f * tan30 * f;
    y0 -= 0.5f * tan30 * e;

    float a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2.0f * z0);
    float b = (y1 - y0) / z0;
    float d = -(a + b * y1) * (a + b * y1) + rf * (1.0f + b * b) * rf;

    if (d < 0) return NAN;

    float yj = (y1 - a * b - sqrt(d)) / (b * b + 1.0f);
    float zj = a + b * yj;
    return atan2(-zj, y1 - yj) * (180.0f / M_PI);
}

// Cinématique inverse
bool delta_calc_inverse(float x0, float y0, float z0, float& theta1, float& theta2, float& theta3) {
    theta1 = delta_calc_angle_yz(x0, y0, z0);
    if (isnan(theta1)) return false;

    float x1 = x0 * cos(2.0f * M_PI / 3.0f) + y0 * sin(2.0f * M_PI / 3.0f);
    float y1 = y0 * cos(2.0f * M_PI / 3.0f) - x0 * sin(2.0f * M_PI / 3.0f);
    theta2 = delta_calc_angle_yz(x1, y1, z0);
    if (isnan(theta2)) return false;

    float x2 = x0 * cos(4.0f * M_PI / 3.0f) + y0 * sin(4.0f * M_PI / 3.0f);
    float y2 = y0 * cos(4.0f * M_PI / 3.0f) - x0 * sin(4.0f * M_PI / 3.0f);
    theta3 = delta_calc_angle_yz(x2, y2, z0);
    if (isnan(theta3)) return false;

    return true;
}

// Envoyer une position XYZ aux moteurs
void set_position_xyz(float x, float y, float z) {
    float theta1, theta2, theta3;
    if (!delta_calc_inverse(x, y, z, theta1, theta2, theta3)) {
        Serial.println("Position non réalisable");
        return;
    }

   Serial.println("Angles calculés :");

Serial.println("Position demandée :");
    Serial.print("  X: "); Serial.print(x); Serial.println(" mm");
    Serial.print("  Y: "); Serial.print(y); Serial.println(" mm");
    Serial.print("  Z: "); Serial.println(z); Serial.println(" mm");
    
Serial.println("Angles optenus :");
    Serial.print("  Motor0 : "); Serial.print(theta1); Serial.println(" °");
    Serial.print("  Motor1 : "); Serial.print(theta2); Serial.println(" °");
    Serial.print("  Motor2 : "); Serial.println(theta3); Serial.println(" °");
    
Serial.println("Envoi des consignes aux moteurs...");
    motor0.set_position_ref(theta1 * DEG2RAD);
    Serial.println("Consigne envoyée à Motor0");
    motor1.set_position_ref(theta2 * DEG2RAD);
    Serial.println("Consigne envoyée à Motor1");
    motor2.set_position_ref(theta3 * DEG2RAD);
    Serial.println("Consigne envoyée à Motor2");

Serial.println("Consignes envoyées :");
    Serial.print("  Motor0: "); Serial.print(theta1 * DEG2RAD); Serial.println(" rad");
    Serial.print("  Motor1: "); Serial.print(theta2 * DEG2RAD); Serial.println(" rad");
    Serial.print("  Motor2: "); Serial.print(theta3 * DEG2RAD); Serial.println(" rad");

    
}

// Positions prédéfinies
//void position_INIT_base() { set_position_xyz(0, 0, 400); delay(170); }
void position_INIT() { set_position_xyz(0, 0, 440); delay(180); }
void position_1() { set_position_xyz(-320, -120, 430); delay(180); }
void position_2() { set_position_xyz(-320, -120, 550); delay(180); }
void position_3() { set_position_xyz(0, 75, 430); delay(180); }
void position_4() { set_position_xyz(0, 75, 560); delay(180); }
void position_5() { set_position_xyz(300, -80, 430); delay(190); }
void position_6() { set_position_xyz(300, -80, 550); delay(190); }

// Séquence de mouvements
void run_sequence() {
    Serial.println("Début de la séquence...");
    position_INIT();

    position_1();
    position_2();
    position_1();

    position_3();
    position_4();
    position_3();

    position_5();
    position_6();
    position_5();


    //position_1();
    //position_2();
    //position_1();

    position_3();
    position_4();
    position_3();

    position_INIT();

    position_1();
    position_2();
    position_1();

    position_3();
    position_4();
    position_3();

    position_1();
    position_2();
    position_1();

    //position_5();
    //position_6();
    //position_5();

    //position_3();
    //position_4();
    //position_3();

    position_INIT();
    Serial.println("Séquence terminée.");
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    // Configuration du bouton sur D5
    pinMode(BUTTON_PIN, INPUT_PULLUP);  // Logique inversée : HIGH = non pressé, LOW = pressé

    // **Séquence d’initialisation**
    // 1. Initialisation du bus CAN
    Serial.println("Initialisation du bus CAN...");
    if (motor1.init_twai(RX_PIN, TX_PIN, true) != ESP_OK) {
        Serial.println("Erreur d’initialisation du bus CAN !");
        while (1) delay(1000);
    }
    delay(1000);

    
    // Configuration des gains de la boucle de vitesse (pour Kd et Ki)
    motor0.set_speed_kp(KD_EFFECTIVE);  // Ajuste Kd indirectement
    motor0.set_speed_ki(KI_SPEED);      // Ajuste Ki
    motor1.set_speed_kp(KD_EFFECTIVE);  // Ajuste Kd indirectement
    motor1.set_speed_ki(KI_SPEED);      // Ajuste Ki
    motor2.set_speed_kp(KD_EFFECTIVE);  // Ajuste Kd indirectement
    motor2.set_speed_ki(KI_SPEED);      // Ajuste Ki

    // Configuration du gain de la boucle de position (déjà prévu)
    //motor0.set_position_kp(KP_POSITION);
    //motor1.set_position_kp(KP_VALUE);
    //motor2.set_position_kp(KP_VALUE);

 
    
    
    // 2. Initialisation des moteurs
    Serial.println("Initialisation des moteurs...");
    motor0.init_motor(MODE_POSITION);
    //motor0.set_position_kp(KP_VALUE);
    //motor0.set_position_ki(KI_VALUE);
    //motor0.set_position_kd(KD_VALUE);
    motor0.set_limit_speed(MAX_SPEED);
    motor0.set_limit_current(CURR);
    motor0.set_limit_torque(TRQ);
    motor0.enable_motor();

    motor1.init_motor(MODE_POSITION);
    //motor1.set_position_kp(KP_VALUE);
    motor1.set_limit_speed(MAX_SPEED);
    motor1.set_limit_current(CURR);
    motor1.set_limit_torque(TRQ);
    motor1.enable_motor();

    motor2.init_motor(MODE_POSITION);
    //motor2.set_position_kp(KP_VALUE);
    motor2.set_limit_speed(MAX_SPEED);
    motor2.set_limit_current(CURR);
    motor2.set_limit_torque(TRQ);
    motor2.enable_motor();
    delay(1000);


    // 3. Définir la position actuelle comme zéro
    Serial.println("Définition de la position zéro...");
    set_mechanical_position_to_zero(MOTOR_ID_0, MASTER_ID);
    set_mechanical_position_to_zero(MOTOR_ID_1, MASTER_ID);
    set_mechanical_position_to_zero(MOTOR_ID_2, MASTER_ID);
    delay(1000);

    // Étape 10 : Envoyer une consigne de position (par exemple, 1.0 rad)
    Serial.println("Envoi d’une consigne de position (1.0 rad)...");
    motor0.set_position_ref(0.1);
    motor1.set_position_ref(0.1);
    motor2.set_position_ref(0.1);
    delay(2000);

    // Étape 11 : Ramener le moteur à la nouvelle position zéro (0.0 rad)
    Serial.println("Retour à la nouvelle position zéro (0.0 rad)...");
    motor0.set_position_ref(0.0f);
    motor1.set_position_ref(0.0f);
    motor2.set_position_ref(0.0f);
    delay(2000);
}

void loop() {
    // Lire l’état du bouton
    etatBouton = digitalRead(BUTTON_PIN);

    // Détecter un appui (logique inversée : HIGH → LOW)
    if (etatBouton == LOW && dernierEtatBouton == HIGH) {
        unsigned long currentTime = millis();
        if (currentTime - lastButtonPress >= 200) {  // Débouncing
            lastButtonPress = currentTime;
            run_sequence();  // Lancer la séquence de mouvements
        }
    }

    // Vérifier l’entrée série pour arrêter les moteurs
    //if (Serial.available() > 0) {
        //char input = Serial.read();
        //if (input == 's') {
           // Serial.println("Arrêt des moteurs...");
           // motor0.stop_motor();
           //motor1.stop_motor();
           // motor2.stop_motor();
           // Serial.println("Moteurs arrêtés.");
       // }
    //}



    dernierEtatBouton = etatBouton;
    delay(10);  // Réduire la charge CPU


}
