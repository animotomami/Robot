///////////////////
//
// Archivo de cabecera con funciones para lectura de ENCODER Y generación de PWM
//
//////////////////
#include <iostream>

/// Rango de trabajo encoders en grados
#define LIM_SUP_E0 230
#define LIM_INF_E0 110

#define LIM_SUP_E1 130
#define LIM_INF_E1 45

#define LIM_SUP_E2 100
#define LIM_INF_E2 30

/// Tiempo de muestreo en milisegundos

#define SAMPLE_TIME 200

/// Funciones
float linPWM(float valPWM, int numMot);



