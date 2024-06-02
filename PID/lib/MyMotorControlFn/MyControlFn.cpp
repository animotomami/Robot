#include "MyControlFn.h"
#include <iostream>
#include <Arduino.h>


/// @brief Linealizacion de PWM en zonas de trabajo para los 3 motores
/// @param valPWM 
/// @param numMot Motor 0, 1 y 2
/// @return duty escalado
float linPWM(float valPWM, int numMot)
{
    float min_fwd = 0.1, min_bck=-0.1;
    // {base,hombro,codo}
    float upLimit_fwd[3] = {0.8,0.9, 0.8};
    float lowLimit_fwd[3] = {0.5,0.6, 0.5};

    float upLimit_bck[3] = {-0.8,-0.3, -0.8};
    float lowLimit_bck[3] = {-0.5,-0.1,-0.5};

if (valPWM>min_fwd)
    valPWM = lowLimit_fwd[numMot] + valPWM * (upLimit_fwd[numMot]-lowLimit_fwd[numMot]);

else if (valPWM<min_bck)
    valPWM = lowLimit_bck[numMot] - valPWM * (upLimit_bck[numMot]-lowLimit_bck[numMot]);     
    
else
    valPWM = 0;

return valPWM;
}
