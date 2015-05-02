#ifndef SEGUIDOR_H
#define SEGUIDOR_H

#include "Arduino.h"

////////////////////////////////////////////////////////////////////
///////////////////////// MACROS ///////////////////////////////////
////////////////////////////////////////////////////////////////////

/***** Botao de acionamento *****/
#define BUTTON_PIN 7
#define BUTTON_INTERRUPT 0
/********************************/

/***** Pinagem da Ponte H **************/
// Motor direito
#define R_ENABLE		UNUSED
#define R_MOTOR_1		6
#define R_MOTOR_2		13
#define R_MOTOR_MAX_PWM	        255
// Motor esquerdo
#define L_ENABLE		UNUSED
#define L_MOTOR_1		9
#define L_MOTOR_2		10
#define L_MOTOR_MAX_PWM	        255
/***************************************/

/***** Bluetooth *****/
#define BLUETOOTH_TX 4
#define BLUETOOTH_RX 16
/*********************/

/***** Ultrassom *****/
// Esquerdo
#define TRIG_ESC  21
#define ECHO_ESC  20
// Direito
#define TRIG_DIR  19
#define ECHO_DIR  18

/*********************/
/***** SHARP *****/
#define SHARP_ESQ  A7
#define SHARP_DIR  A8
#define DISTANCIA_MAXIMA_OBSTACULO 150
/*********************/
/********** Sensores de Reflectancia **********/
// Sensor de Linha
#define EMITTER_PIN  QTR_NO_EMITTER_PIN
#define NUM_SENSORS     4
#define CENTER_POSITION 1500
#define WHITE_LINE 1

// Sensor de Parada
#define SENSOR_PARADA     12
#define LIMIAR_PARADA     300

// Calibracao
#define VEL_CALIBRACAO    180
#define TEMPO_CALIBRACAO  1600
/**********************************************/

/********** Sensores de Tensão da Bateria *****/
#define SENSOR_BATERIA_PIN  A6
/**********************************************/

/********** LED ****************************/
#define LED_PIN              11
/**********************************************/

/* TODO: create class follower */
#endif