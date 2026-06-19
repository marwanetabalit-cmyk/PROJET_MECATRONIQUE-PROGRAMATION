#pragma once
#include <Arduino.h>

// =========================
// MODE DE TRAVAIL
// =========================
#define MODE_REAL       0
#define MODE_SIMULATION 1

constexpr int ROBOT_MODE = MODE_REAL;

// =========================
// BROCHES GENERALES
// =========================
constexpr uint8_t PIN_START_SWITCH = 27;   // microrupteur de départ
constexpr uint8_t PIN_ESTOP        = 14;   // arrêt d'urgence logique

// Mettre a false quand le bouton start sera cable.
constexpr bool AUTO_START_WITHOUT_BUTTON = true;

// =========================
// ULTRASONS
// Echo -> à adapter vers 3.3V avant entrée ESP32
// =========================
constexpr uint8_t US_FRONT_TRIG = 33;   // J11 - Trig
constexpr uint8_t US_FRONT_ECHO = 36;   // J11 - Echo, input only

constexpr uint8_t US_LEFT_TRIG  = 25;
constexpr uint8_t US_LEFT_ECHO  = 39;   // input only

constexpr uint8_t US_RIGHT_TRIG = 26;
constexpr uint8_t US_RIGHT_ECHO = 34;   // input only

constexpr unsigned long ULTRASON_TIMEOUT_US = 30000;
constexpr float SOUND_SPEED_CM_PER_US = 0.0343f;

// =========================
// DYNAMIXEL
// =========================
constexpr uint8_t DXL_RX_PIN  = 16;
constexpr uint8_t DXL_TX_PIN  = 17;
constexpr uint8_t DXL_DIR_PIN = 23;

constexpr uint8_t DXL_LEFT_ID  = 1;
constexpr uint8_t DXL_RIGHT_ID = 2;

constexpr uint32_t DXL_BAUDRATE = 57600;
constexpr float DXL_PROTOCOL_VERSION = 2.0f;

constexpr bool LEFT_MOTOR_INVERTED  = true;
constexpr bool RIGHT_MOTOR_INVERTED = false;

constexpr float DRIVE_FORWARD_RPM = 57.0f;
constexpr float DRIVE_TURN_RPM    = 57.0f;

// =========================
// ODOMETRIE CHENILLES
// =========================
// Donnees mecaniques fournies:
// - barbotin 18 dents
// - pas de chenille 6 mm
// Donc 1 tour de barbotin deplace la chenille de 18 * 6 mm = 108 mm.
constexpr float TRACK_PITCH_CM = 0.6f;
constexpr float TRACK_SPROCKET_TEETH = 18.0f;
constexpr float TRACK_CM_PER_REV = TRACK_PITCH_CM * TRACK_SPROCKET_TEETH;

// Distance entre les axes moyens des deux chenilles.
// A mesurer sur le robot, puis a calibrer en rotation.
constexpr float TRACK_BASE_CM = 21.0f;

// Les chenilles glissent en rotation: ajuster apres essais reels.
// Si le robot tourne trop peu, augmenter. S'il tourne trop, diminuer.
constexpr float TRACK_TURN_CORRECTION = 1.0f;

constexpr unsigned long ODOMETRY_PERIOD_MS = 50;

// Trajectoire issue de init_robot_demo.m / params_simscape_robot.m.
constexpr float STRATEGY_START_X_CM = 22.5f;
constexpr float STRATEGY_START_Y_CM = 170.0f;
constexpr float STRATEGY_START_THETA_RAD = -1.57079632679f;

constexpr float STRATEGY_LINEAR_SPEED_CM_S = 10.0f;
constexpr unsigned long STRATEGY_ROTATION_90_MS = 1500;
constexpr unsigned long STRATEGY_ROTATION_PAUSE_MS = 500;
constexpr unsigned long STRATEGY_MOVE_TIMEOUT_MARGIN_MS = 2000;
constexpr unsigned long STRATEGY_ROTATION_TIMEOUT_MARGIN_MS = 1000;

// =========================
// SERVOS CLASSIQUES
// =========================
constexpr uint8_t SERVO_LIFT_PIN      = 18;
constexpr uint8_t SERVO_GRIP_PIN      = 19;
constexpr uint8_t SERVO_SPLIT_PIN     = 21;
constexpr uint8_t SERVO_CURSOR_PIN    = 22;

constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;

constexpr int LIFT_DOWN_ANGLE   = 20;
constexpr int LIFT_UP_ANGLE     = 110;

constexpr int GRIP_OPEN_ANGLE   = 30;
constexpr int GRIP_CLOSE_ANGLE  = 100;

constexpr int SPLIT_OPEN_ANGLE  = 25;
constexpr int SPLIT_CLOSE_ANGLE = 95;

constexpr int CURSOR_HOME_ANGLE = 90;
constexpr int CURSOR_PUSH_ANGLE = 140;

// =========================
// SECURITE / LOGIQUE
// =========================
// Distance de détection d'obstacle (déclenchement évitement)
constexpr float OBSTACLE_STOP_CM = 20.0f;

// Période d'affichage debug (réduction des appels Serial)
constexpr unsigned long DEBUG_PERIOD_MS = 250;

// Période de lecture des capteurs (équilibre précision/performance)
constexpr unsigned long SENSOR_PERIOD_MS = 80;

// Durée du match Eurobot 2026 = 90 secondes
constexpr unsigned long MATCH_DURATION_MS = 90000;

// Timing pour stratégie d'évitement (non-bloquant)
constexpr unsigned long AVOID_STOP_MS = 500;
constexpr unsigned long AVOID_TURN_MS = 700;
constexpr float AVOID_TURN_DEG = 20.0f;
