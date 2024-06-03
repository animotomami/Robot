#include <Arduino.h> 
#include "MyControlFn.h"
#include "MyEncoderFn.h"
#include "MyPwmFn.h"

using namespace std;

//Variables globales
float duty_vec[3]={0,-0.5,0.5}; //Array de duties. Pueden tomar valor -1 a 1.
float _Pos_ref=140; //Posición de referencia en grados
uint32_t _lastTimeSample=0;


/////////////////////Declaración de parámetros del PID ////////////////

float N=100.104; //Coeficiente del filtro para la acción diferencial 
float b=1; //Peso del punto de funcionamiento en el término proporcional
float c=2.547; //Peso del punto de funcionamiento en el término de la acción diferencial
float Ts=0.001; //Periodo de muestreo

float satpos=245; //Valor de la saturación positiva del control. 
float satneg=-245; //Valor de la saturación negativa del control. 

float Q1;
float Q2;
//Declaracion de variables

float Dprevio=0; //Valor de la parte derivativa previo
float Iprevio=0;  //Valor de la parte integral previa
float satur; //Saturacion para integrar el anti-windup

//Constantes del PID
float kp=4.5; //Constante Proporcional
float ki=200; //Constante Integral
float kd=-0.4447; //Constante Derivativa

//Variables de memoria

float pos_ref_previa;
float pos_encoder_previa;

//////////////////////////////////////////////////////

//********************** FN PID **********************

float Robot_PID(float pos_ref,float pos_encoder,float rangoError)
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
    Q2=kd*N;  //Cálculo del parámetro Q2
 
  if (error>=rangoError || error<=(rangoError*(-1))) // Si el error es menor
  {
    //Acción integral 
    incrI=ki*Ts*(pos_ref_previa-pos_encoder_previa); 
  //Antiwind‐up 
    if (satur*incrI>0)
    {
      I=Iprevio;
    } 
    else
    { 
      I=Iprevio+ki*Ts*(pos_ref_previa-pos_encoder_previa); 
    }
  //Acción proporcional 
    P=kp*(b*pos_ref-pos_encoder);

  //Acción derivativa 
    D=Q1*Dprevio+Q2*c*(pos_ref-pos_ref_previa)-Q2*(pos_encoder-pos_encoder_previa);
    Dprevio=D;

  //Consigna de control

    u=P+I;

    if (u>satpos) //Si la consigna es mayor que la saturación se pone a 1.
    {  
      u=satpos; 
      satur=1;
    }
    else if (u<satneg) //Si la consigna es menor que la saturación se pone a -1
    {
      u=satneg; 
      satur=-1;
    }
    else //Si no se cumple ninguna condición anterior la consigna no se modifica
    {
      Iprevio=I; 
      satur=0; 
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
while (Serial.available() > 0)
  {
    refPosition = Serial.readString();
    refPosition.trim();
    Serial.printf("ref set to: %f \n", refPosition.toFloat());
    float Pos_ref_prelim = refPosition.toFloat();      // pass string to float value for control
    if (Pos_ref_prelim >= LIM_INF_E2 && Pos_ref_prelim <= LIM_SUP_E2)
    {
        _Pos_ref=Pos_ref_prelim;
    }
  }

  // Se ejecuta cada entrada a interrupción

  if(has_expired)
  {
    Serial.print("interrupcion");
    ang_0=encoder0_read(); //Introducir valor en el código si no hay encoder para hacer pruebas
    ang_1=encoder1_read();
    ang_2=encoder2_read();
    
    // Cálculo de variable de control de motores

    u_PID_M0=Robot_PID(_Pos_ref,ang_0,3); //Primer parámetro Posición de referencia; Segundo parámetro posición actual del encoder; Tercer parámetro: grados de margen para error
    u_PID_M1=Robot_PID(_Pos_ref,ang_1,3);
    u_PID_M2=Robot_PID(_Pos_ref,ang_2,3);

    // Aplica pwm a motores

    duty_vec[0]=linPWM(u_PID_M0, 0);
    duty_vec[1]=linPWM(u_PID_M1, 1);
    duty_vec[2]=linPWM(u_PID_M2, 2);
    set_comp_tres_motores(duty_vec);

    // Imprime info

    Serial.print("Ángulo encoder M0: "); 
    Serial.printf(">Ángulo encoder: %f\n", ang_0);     //envía la posición en grados al terminal serie
    Serial.println(ang_0);     //envía la posición en grados al terminal serie

    Serial.print("Variable de control M0: ");
    Serial.println(duty_vec[0]);

    Serial.print("Ángulo encoder M1: "); 
    Serial.printf(">Ángulo encoder: %f\n", ang_1);     //envía la posición en grados al terminal serie
    Serial.println(ang_1);     //envía la posición en grados al terminal serie

    Serial.print("Variable de control M1: ");
    Serial.println(duty_vec[1]);

    Serial.print("Ángulo encoder M2: "); 
    Serial.printf(">Ángulo encoder: %f\n", ang_2);     //envía la posición en grados al terminal serie
    Serial.println(ang_2);     //envía la posición en grados al terminal serie

    Serial.print("Variable de control M2: ");
    Serial.println(duty_vec[2]);

    Serial.print("Set point: ");
    Serial.printf(">Set point: %f\n", _Pos_ref);     //envía la posición en grados al terminal serie
    Serial.println(_Pos_ref);  
    
    has_expired=false;
  }
  
}
  
