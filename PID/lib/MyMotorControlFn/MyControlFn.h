///////////////////
//
// Archivo de cabecera con funciones para lectura de ENCODER Y generación de PWM
//
//////////////////
#include <iostream>

/// Rango de trabajo encoders en grados
#define LIM_SUP_E0 230
#define LIM_INF_E0 110 

#define LIM_SUP_E1 110
#define LIM_INF_E1 45

#define LIM_SUP_E2 230
#define LIM_INF_E2 170 

/// Tiempo de muestreo en segundos

#define SAMPLE_TIME 0.02

/// Parámetros del control PID

#define KP_M0 6
#define KI_M0 2

#define KP_M2 1.5
#define KI_M2 1.2

#define KP_M1 0.5
#define KI_M1 0
#define KD_M1 0.001

#define KP_M1_SUBIDA 0.28
#define KP_M1_BAJADA 0.07

#define KD_M1_SUBIDA 0.002
#define KD_M1_BAJADA 0.0015

#define SATPOS 245
#define SATNEG -245

#define N 1
#define C 2.5

/// Funciones
float linPWM(float valPWM, int numMot);



