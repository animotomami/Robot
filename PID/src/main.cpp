#include <Arduino.h> 
#include "MyControlFn.h"
#include "MyEncoderFn.h"
#include "MyPwmFn.h"

using namespace std;

//Variables globales
float duty_vec[3]={0,0,0}; //Array de duties. Pueden tomar valor -1 a 1.
float _Pos_ref[3]={170,70,200}; //Posición de referencia en grados
uint32_t _lastTimeSample=0;
uint32_t _cont_serial=0,_stop=0;


/////////////////////Declaración de parámetros del PID ////////////////

float N=1; //Coeficiente del filtro para la acción diferencial 
float b=1; //Peso del punto de funcionamiento en el término proporcional
float c=2.5; //Peso del punto de funcionamiento en el término de la acción diferencial
float Ts=SAMPLE_TIME; //Periodo de muestreo

float satpos=245; //Valor de la saturación positiva del control. 
float satneg=-245; //Valor de la saturación negativa del control. 

float Q1;
float Q2;
//Declaracion de variables

float Dprevio_m1=0; //Valor de la parte derivativa previo
float Iprevio_m0=0;  //Valor de la parte integral previa m0
float Iprevio_m1=0;  //Valor de la parte integral previa m1
float Iprevio_m2=0;  //Valor de la parte integral previa m1
float satur_m0=0; //Saturacion para integrar el anti-windup m0
float satur_m1=0; //Saturacion para integrar el anti-windup m1
float satur_m2=0;  //Valor de la parte integral previa m1

//Constantes del PID
float kp_m0=6; //Constante Proporcional
float ki_m0=2; //Constante Integral
float kp_m2=1.5; //Constante Proporcional
float ki_m2=1.2; //Constante Integral
float kp_m1=1.5; //Constante Proporcional
float ki_m1=1.2; //Constante Integral
float kd_m1=0.4447; //Constante Derivativa

//Variables de memoria

float pos_ref_previa_m0;
float pos_ref_previa_m1;
float pos_ref_previa_m2;
float pos_encoder_previa_m0;
float pos_encoder_previa_m1;
float pos_encoder_previa_m2;


//////////////////////////////////////////////////////

//********************** FN PID **********************

float Robot_PID_m0(float pos_ref,float pos_encoder,float rangoError)
{
    //Declaración de la variable de control

    float u;
    float u_pwm;

    //Declaración de las variables temporales del PID

    float P;
    float I;
    // float D;   

    float incrI;
    float error = pos_ref-pos_encoder;
    
    //Definición de factores 
    // Q1=1-N*Ts; //Cálculo del parámetro Q1
    // Q2=kd*N;  //Cálculo del parámetro Q2
 
  if (error>=rangoError || error<=(rangoError*(-1))) // Si el error es menor
  {
    //Acción integral 
    incrI=ki_m0*Ts*(pos_ref_previa_m0-pos_encoder_previa_m0); // Convertir a almacenamiento de error (IMPORTANTE)
  //Antiwind‐up 
    if (satur_m0*incrI>0)
    {
      I=Iprevio_m0;
    } 
    else
    { 
      I=Iprevio_m0+incrI; 
    }

   
  //Acción proporcional 
    P=kp_m0*(b*pos_ref-pos_encoder);

  //Acción derivativa 
    // D=Q1*Dprevio+Q2*c*(pos_ref-pos_ref_previa_m0)-Q2*(pos_encoder-pos_encoder_previa_m0);
    // Dprevio=D;

  //Consigna de control

    u=P+I;
    
    if (u>satpos) //Si la consigna es mayor que la saturación se pone a 1.
    {  
      u=satpos; 
      satur_m0=1;
    }
    else if (u<satneg) //Si la consigna es menor que la saturación se pone a -1
    {
      u=satneg; 
      satur_m0=-1;
    }
    else //Si no se cumple ninguna condición anterior la consigna no se modifica
    {
      Iprevio_m0=I; 
      satur_m0=0; 
    }
    
    if (pos_ref!=pos_ref_previa_m0 || _cont_serial==0)
    {
      Iprevio_m0=0;
    }

  //Actualización de variables 
  pos_ref_previa_m0=pos_ref; //Actualizar posicion de referencia previ del paso anterior
  pos_encoder_previa_m0=pos_encoder; //Actualizar posición del encoder previa del paso anterior
  u_pwm=u/245;
  }
  else
  {
    u_pwm=0;
  }
  

return u_pwm;
}

float Robot_PID_m1(float pos_ref,float pos_encoder,float rangoError)
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
    
    //Definición de factores 
    Q1=1-N*Ts; //Cálculo del parámetro Q1
    Q2=kd_m1*N;  //Cálculo del parámetro Q2
 
  if (error>=rangoError || error<=(rangoError*(-1))) // Si el error es menor
  {
    //Acción integral 
    incrI=ki_m1*Ts*(pos_ref_previa_m1-pos_encoder_previa_m1); // Convertir a almacenamiento de error (IMPORTANTE)
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
    P=kp_m1*(b*pos_ref-pos_encoder);

  //Acción derivativa 
    D=Q1*Dprevio_m1+Q2*c*(pos_ref-pos_ref_previa_m0)-Q2*(pos_encoder-pos_encoder_previa_m0);
    Dprevio_m1=D;

  //Consigna de control

    u=P+I+D;
    

    if (u>satpos) //Si la consigna es mayor que la saturación se pone a 1.
    {  
      u=satpos; 
      satur_m1=1;
    }
    else if (u<satneg) //Si la consigna es menor que la saturación se pone a -1
    {
      u=satneg; 
      satur_m1=-1;
    }
    else //Si no se cumple ninguna condición anterior la consigna no se modifica
    {
      Iprevio_m1=I; 
      satur_m1=0; 
    }

    if (pos_ref!=pos_ref_previa_m1 || _cont_serial==0)
    {
      Iprevio_m1=0;
    }

  //Actualización de variables 
  pos_ref_previa_m1=pos_ref; //Actualizar posicion de referencia previ del paso anterior
  pos_encoder_previa_m1=pos_encoder; //Actualizar posición del encoder previa del paso anterior
  u_pwm=u/245;
  }
  else
  {
    u_pwm=0;
  }
  

return u_pwm;
}

float Robot_PID_m2(float pos_ref,float pos_encoder,float rangoError)
{
    //Declaración de la variable de control

    float u;
    float u_pwm;

    //Declaración de las variables temporales del PID

    float P;
    float I;
    // float D;   

    float incrI;
    float error = pos_ref-pos_encoder;
    
    //Definición de factores 
    // Q1=1-N*Ts; //Cálculo del parámetro Q1
    // Q2=kd*N;  //Cálculo del parámetro Q2
 
  if (error>=rangoError || error<=(rangoError*(-1))) // Si el error es menor
  {
    //Acción integral 
    incrI=ki_m2*Ts*(pos_ref_previa_m2-pos_encoder_previa_m2); // Convertir a almacenamiento de error (IMPORTANTE)
  //Antiwind‐up 
    if (satur_m2*incrI>0)
    {
      I=Iprevio_m2;
    } 
    else
    { 
      I=Iprevio_m2+incrI; 
    }

    
  //Acción proporcional 
    P=kp_m2*(b*pos_ref-pos_encoder);

  //Acción derivativa 
    // D=Q1*Dprevio+Q2*c*(pos_ref-pos_ref_previa_m0)-Q2*(pos_encoder-pos_encoder_previa_m0);
    // Dprevio=D;

  if (pos_ref!=pos_ref_previa_m2)
    {
      Iprevio_m2=0;
      I=0;
    }
  //Consigna de control

    u=P+I;
    

    if (u>satpos) //Si la consigna es mayor que la saturación se pone a 1.
    {  
      u=satpos; 
      satur_m2=1;
    }
    else if (u<satneg) //Si la consigna es menor que la saturación se pone a -1
    {
      u=satneg; 
      satur_m2=-1;
    }
    else //Si no se cumple ninguna condición anterior la consigna no se modifica
    {
      Iprevio_m2=I; 
      satur_m2=0; 
    }

    

  //Actualización de variables 
  pos_ref_previa_m2=pos_ref; //Actualizar posicion de referencia previ del paso anterior
  pos_encoder_previa_m2=pos_encoder; //Actualizar posición del encoder previa del paso anterior
  u_pwm=u/245;
  }
  else
  {
    u_pwm=0;
  }
  

return u_pwm;
}


hw_timer_t * timer = NULL; // Puntero para configurar timer

// *************** FN INTERRUPCION TIMER *********************

volatile bool has_expired = false;
void ARDUINO_ISR_ATTR InterrrupTimer()
{
  has_expired=true;
}

//********************** MAIN **********************
void setup()
{ 
  // Timer para tiempo de muestreo
  timer = timerBegin(1000000); // 1 Mhz de resolución (us)
  timerAttachInterrupt(timer, &InterrrupTimer);
  timerAlarm(timer, SAMPLE_TIME*1000000,true,0); // Set time en us

  encoder_init();
  Serial.begin(115200);  
  Serial.println("Comienzo de programa"); 
  Serial.flush();  //Espera a que termina la transmisión serie anterior 
  delay(200);

  pwm_init();
  delay(500); 
}


void loop() {

  float ang_0=0, ang_1=0, ang_2=0;
  float u_PID_M0=0, u_PID_M1=0, u_PID_M2=0; //Variable de control del PWM

// Cambiar set point por serial

String refPosition;

while (Serial.available()>0)    // Si hay datos disponibles por el puerto serie:
      {
        _cont_serial++;
        String input=Serial.readString();
        input.replace(" ", "");

        if (input.startsWith("M0="))
        {
          input=input.substring(3);
          int Pos_ref_prelim=input.toFloat();
          if (Pos_ref_prelim>= LIM_INF_E0 && Pos_ref_prelim <= LIM_SUP_E0)
          {
              _Pos_ref[0]=Pos_ref_prelim;
          }
        }
        else if (input.startsWith("M1="))
        {
           input=input.substring(3);
          int Pos_ref_prelim=input.toFloat();
          if (Pos_ref_prelim>= LIM_INF_E1 && Pos_ref_prelim <= LIM_SUP_E1)
          {
              _Pos_ref[1]=Pos_ref_prelim;
          }
        }
        else if (input.startsWith("M2="))
        {
           input=input.substring(3);
          int Pos_ref_prelim=input.toFloat();
          if (Pos_ref_prelim>= LIM_INF_E2 && Pos_ref_prelim <= LIM_SUP_E2)
          {
              _Pos_ref[2]=Pos_ref_prelim;
          }
        }
        else if(input.startsWith("STOP"))
        {
          _cont_serial=0;
          _stop=1;
        }
        else if(input.startsWith("GO"))
        {
          _stop=0;
        }
      }

// while (Serial.available() > 0)
//   {
//     refPosition = Serial.readString();
//     refPosition.trim();
//     int j=0;
//     int coma_pos_ant=0;
//     for(int i=0;i<3;i++)
//     {
//       Serial.printf(">J: ",j);
      
//       int coma_pos=refPosition.indexOf(",",j);
//       refPosition=refPosition.substring(j, coma_pos);

//       float Pos_ref_prelim = refPosition.toFloat();      // pass string to float value for control
//       int lim_sup;
//       int lim_inf;
//       if(i==0)
//       {
//         lim_sup=LIM_SUP_E0;
//         lim_inf=LIM_INF_E0;
//       }else if(i==1)
//       {
//         lim_sup=LIM_SUP_E1;
//         lim_inf=LIM_INF_E1;
//       }else if (i==2)
//       {
//         lim_sup=LIM_SUP_E2;
//         lim_inf=LIM_INF_E2;
//       }

//       if (Pos_ref_prelim>= lim_inf && Pos_ref_prelim <= lim_sup)
//       {
//           _Pos_ref[i]=Pos_ref_prelim;
//       }
//     j=coma_pos+1;
//     }

    
//   }

  // Se ejecuta cada entrada a interrupción

  if(has_expired)
  {
    Serial.print("interrupcion");
    ang_0=encoder0_read(); //Introducir valor en el código si no hay encoder para hacer pruebas
    ang_1=encoder1_read();
    ang_2=encoder2_read();

    if ((ang_0>=LIM_SUP_E0 || ang_0<=LIM_INF_E0) || (ang_1>=LIM_SUP_E1 || ang_1<=LIM_INF_E1) || (ang_2>=LIM_SUP_E2 || ang_2<=LIM_INF_E2) || _cont_serial==0 || _stop==1)
    {
        u_PID_M0=0; //Primer parámetro Posición de referencia; Segundo parámetro posición actual del encoder; Tercer parámetro: grados de margen para error
        u_PID_M1=0;
        u_PID_M2=0;
        Iprevio_m2=0;
    }
    else
    {
        // Cálculo de variable de control de motores

      u_PID_M0=Robot_PID_m0(_Pos_ref[0],ang_0,3); //Primer parámetro Posición de referencia; Segundo parámetro posición actual del encoder; Tercer parámetro: grados de margen para error
      u_PID_M1=Robot_PID_m1(_Pos_ref[1],ang_1,3);
      u_PID_M2=Robot_PID_m2(_Pos_ref[2],ang_2,2);

    }    
    // Aplica pwm a motores

    duty_vec[0]=linPWM(u_PID_M0, 0);
    duty_vec[1]=linPWM(u_PID_M1, 1);
    duty_vec[2]=linPWM(u_PID_M2, 2);
    set_comp_tres_motores(duty_vec);

    // Imprime info

    // Serial.print("Set point: ");
    Serial.printf(">Set point M0: %f\n", _Pos_ref[0]);     //envía la posición en grados al terminal serie
    Serial.printf(">Set point M1: %f\n", _Pos_ref[1]);     //envía la posición en grados al terminal serie
    Serial.printf(">Set point M2: %f\n", _Pos_ref[2]);     //envía la posición en grados al terminal serie
    //Serial.print("Ángulo encoder M0: "); 
    Serial.printf(">Ángulo encoder M0: %f\n", ang_0);     //envía la posición en grados al terminal serie
    // Serial.println(ang_0);     //envía la posición en grados al terminal serie

    Serial.print(">Variable de control M0: ");
    Serial.println(duty_vec[0]);

    // Serial.print("Ángulo encoder M1: "); 
    Serial.printf(">Ángulo encoder M1: %f\n", ang_1);     //envía la posición en grados al terminal serie
    // Serial.println(ang_1);     //envía la posición en grados al terminal serie

    Serial.print(">Variable de control M1: ");
    Serial.println(duty_vec[1]);

    // Serial.print("Ángulo encoder M2: "); 
    Serial.printf(">Ángulo encoder M2: %f\n", ang_2);     //envía la posición en grados al terminal serie
    // Serial.println(ang_2);     //envía la posición en grados al terminal serie

    Serial.print(">Variable de control M2: ");
    Serial.println(duty_vec[2]);

    
    // Serial.println(_Pos_ref);  
    
    has_expired=false;
  }
  
}
  
