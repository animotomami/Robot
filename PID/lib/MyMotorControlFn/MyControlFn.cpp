#include "MyControlFn.h"
#include <iostream>
#include <Arduino.h>

// Llamada a variables globales externas 

extern float Ts; //Periodo de muestreo
extern float pos_ref_previa;
extern float pos_ref_previa_sb;
extern float pos_encoder_previa;
extern float pos_encoder_previa_sb;

// Variables para parte integral
extern float Iprevio_m1;
extern float satur_m1; //Saturacion para integrar el anti-windup m1
extern float _error_ant; // *para D

extern uint32_t _cont_serial;



// Funciones

float linPWM(float valPWM, int numMot)
{
    float min_fwd = 0.01, min_bck=-0.01;
    // {base,hombro,codo}
    float upLimit_fwd[3] = {0.8,1, 0.8};
    float lowLimit_fwd[3] = {0.4,0.75, 0.45};

    float upLimit_bck[3] = {-0.8,-0.35, -0.8};
    float lowLimit_bck[3] = {-0.4,-0.35,-0.5};

if (valPWM>min_fwd)
    valPWM = lowLimit_fwd[numMot] + valPWM * (upLimit_fwd[numMot]-lowLimit_fwd[numMot]);

else if (valPWM<min_bck)
    valPWM = lowLimit_bck[numMot] - valPWM * (upLimit_bck[numMot]-lowLimit_bck[numMot]);     
    
else
    valPWM = 0;

return valPWM;
}


float Control_motor(float pos_ref,float pos_encoder,float rangoError, float kp, float ki)
{
    //Declaración de la variable de control

    float u;
    float u_pwm;

    //Declaración de las variables temporales del P

    float P;
    float error = pos_ref-pos_encoder;

 
  if (error>=rangoError || error<=(rangoError*(-1))) // Si el error es menor
  {
    
    /////// Acción proporcional 
    P=kp*(pos_ref-pos_encoder);


    ////// Consigna de control

    u=P;
    
    if (u>SATPOS) //Si la consigna es mayor que la saturación se pone a 1.
    {  
      u=SATPOS; 
    }
    else if (u<SATNEG) //Si la consigna es menor que la saturación se pone a -1
    {
      u=SATNEG; 
    }

    
    //Actualización de variables 
    
    pos_ref_previa=pos_ref; //Actualizar posicion de referencia previ del paso anterior
    pos_encoder_previa=pos_encoder; //Actualizar posición del encoder previa del paso anterior
    u_pwm=u/245;
  }
  else
  {
    u_pwm=0;
  }
    
  return u_pwm;
}

float Control_motor_sb(float pos_ref,float pos_encoder,float rangoError) // Control que distingue entre desplazamiento de subida y bajada
{
    //Declaración de la variable de control

    float u;
    float u_pwm;

    //Declaración de las variables temporales del PID

    float P;
    float I;
    float D;  
    float incrI; 

    float error = pos_ref-pos_encoder;
 
  if (error>=rangoError || error<=(rangoError*(-1))) // Si el error es menor
  { 
    //Acción integral 
    incrI=KI_M1*Ts*(pos_ref_previa_sb-pos_encoder_previa_sb); // Convertir a almacenamiento de error (IMPORTANTE)
    //Antiwind‐up 
    if (satur_m1*incrI>0)
    {
      I=Iprevio_m1;
    } 
    else
    { 
      I=Iprevio_m1+incrI; 
    }

    //Acción proporcional 

    if (pos_encoder>pos_ref)  // bajada
    {
      P=KP_M1_BAJADA*(pos_ref-pos_encoder);
      if (P<-0.35)
      {
        P=-0.35;
      }
      D=KD_M1_BAJADA*((error-_error_ant)/Ts);
    }
    else if (pos_encoder<=pos_ref) //subida
    {
      P=KP_M1_SUBIDA*(pos_ref-pos_encoder);
      if (P>=0.9)
      {
        P=0.9;
      }
      D=KD_M1_SUBIDA*((error-_error_ant)/Ts);
    }
    

  //Consigna de control

  // if (((P+D)<0 && P>0) || ((P+D)>0 && P<0))
  // {
  //   u=0;
  // } 
  // else 
  // {
  //   u=P+I;
  // }
    
    u=P+I+D;
    Serial.printf(">P: %f\n", P);  
    Serial.printf(">I: %f\n", I);   
    Serial.printf(">D: %f\n", D);    
    
    if (u>0.9) //Si la consigna es mayor que la saturación se pone a 1.
    {  
      u=0.9; 
      satur_m1=1;
    }
    else if (u<-0.4) //Si la consigna es menor que la saturación se pone a -1
    {
      u=-0.4; 
      satur_m1=-1;
    }
    else //Si no se cumple ninguna condición anterior la consigna no se modifica
    {
      Iprevio_m1=I; 
      satur_m1=0; 
    }

    if (u>0.9) //Si la consigna es mayor que la saturación se pone a 1.
    {  
      u=0.9; 
    }
    else if (u<-0.4) //Si la consigna es menor que la saturación se pone a -1
    {
      u=-0.4; 
    }

    if (pos_ref!=pos_ref_previa_sb || _cont_serial==0)
    {
      Iprevio_m1=0;
    }
  //Actualización de variables 
  pos_ref_previa_sb=pos_ref; //Actualizar posicion de referencia previ del paso anterior
  pos_encoder_previa_sb=pos_encoder; //Actualizar posición del encoder previa del paso anterior
  _error_ant=error;

  u_pwm=u;
  }
  else
  {
    u_pwm=0;
  }
  

return u_pwm;
}
