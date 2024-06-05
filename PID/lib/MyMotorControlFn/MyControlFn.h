///////////////////
//
// Archivo de cabecera con funciones para lectura de ENCODER Y generación de PWM
//
//////////////////
#include <iostream>

/// Rango de trabajo encoders en grados
#define LIM_SUP_E0 230
#define LIM_INF_E0 110 

#define LIM_SUP_E1 115
#define LIM_INF_E1 50

#define LIM_SUP_E2 230
#define LIM_INF_E2 170

/// Tiempo de muestreo en segundos

#define SAMPLE_TIME 0.02

/// Funciones
float linPWM(float valPWM, int numMot);



