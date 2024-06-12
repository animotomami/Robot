#include <Arduino.h> 
#include "MyControlFn.h"
#include "MyEncoderFn.h"
#include "MyPwmFn.h"
#include "PinzaFn.h"
#include "MyTrayec.h"
#include <ESP32Servo.h>


using namespace std;

//Variables globales
float duty_vec[3]={0,0,0}; //Array de duties. Pueden tomar valor -1 a 1.
float _Pos_ref[3]={170,81,200}; //Posición de referencia en grados

float Ts=SAMPLE_TIME; //Periodo de muestreo

float Q1;
float Q2;

float Dprevio_m1=0; //Valor de la parte derivativa previo

float pos_ref_previa;
float pos_ref_previa_sb;
float pos_encoder_previa;
float pos_encoder_previa_sb;

// Variables para parte integral
float Iprevio_m1;
float satur_m1=0; //Saturacion para integrar el anti-windup m1

uint32_t _cont_serial=0,_stop=0, _linea=0;
float _error_ant=0; // *para D

float _pos_actual_xyz[3]={0,0,0}, ang_bhcm[4]={0, 0, 0, 0};
uint32_t _t_wait_lin=0, _cont_lin=0, _cont_pasos=0;
uint32_t _cont_interr_pasos=0; 


void FnLinea(float pos0_xyz[],float pos1_xyz[]) // Posicion inicial_0, Posicion final_1
{
  // Declara variables locales
  int pasos=0;
  float delta_x=0,delta_y=0, delta_z=0; // mm
  
  if (_cont_lin==0) // SOLO ENTRA LA PRIMERA VEZ
  {
    cin_Inversa(pos0_xyz[0],pos0_xyz[1],pos0_xyz[2]);
    _Pos_ref[0]=ang_bhcm[0];
    _Pos_ref[1]=ang_bhcm[1];
    _Pos_ref[2]=ang_bhcm[2];

    Serial.printf(">Ang0 hombro: %f\n", _Pos_ref[1]);     //envía la posición en grados al terminal serie
    Serial.printf(">Ang0 codo: %f\n", _Pos_ref[2]);     //envía la posición en grados al terminal serie
     
    _pos_actual_xyz[0]=pos0_xyz[0];
    _pos_actual_xyz[1]=pos0_xyz[1];
    _pos_actual_xyz[2]=pos0_xyz[2];
    _t_wait_lin=millis();
    _cont_lin++;
  }

  if (millis()-_t_wait_lin > 2000) {

    pasos=abs((pos1_xyz[0]-pos0_xyz[0])/STEP_SIZE);    // Cálculo del número de pasos (basado en eje x para garantizar la fiabilidad de rectas diagonales).

    if (pasos==0) {                // Si el movimiento se produce exclusivamente en el eje y, el paso se basa en el eje y.
        pasos=abs((pos1_xyz[1]-pos0_xyz[1])/STEP_SIZE);
    } 
    if (pasos==0) {                // Si el movimiento se produce exclusivamente en el eje y, el paso se basa en el eje y.
        pasos=abs((pos1_xyz[2]-pos0_xyz[2])/STEP_SIZE);
    }

    delta_x=((pos1_xyz[0]-pos0_xyz[0]))/pasos;  // Cálculo de la distancia a recorrer por paso en el eje x.
    delta_y=(pos1_xyz[1]-pos0_xyz[1])/pasos;  // Cálculo de la distancia a recorrer por paso en el eje y.
    delta_z=(pos1_xyz[2]-pos0_xyz[2])/pasos;  // Cálculo de la distancia a recorrer por paso en el eje y.
    
    _cont_interr_pasos++;
    
   if (_cont_interr_pasos>(TIEMPO_ENTRE_PASOS/SAMPLE_TIME))
   {
    if(_cont_pasos<pasos)
        {

          _cont_pasos++;
          _pos_actual_xyz[0]=_pos_actual_xyz[0]+delta_x;  // Suma de la distancia objetivo a la posición x actual.
          _pos_actual_xyz[1]=_pos_actual_xyz[1]+delta_y;  // Suma de la distancia objetivo a la posición y actual.
          _pos_actual_xyz[2]=_pos_actual_xyz[2]+delta_z;  // Suma de la distancia objetivo a la posición y actual.
          cin_Inversa(_pos_actual_xyz[0],_pos_actual_xyz[1],_pos_actual_xyz[2]);  // Llamada a función del cálculo de la cinemática inversa.

            _Pos_ref[0]=ang_bhcm[0];
            _Pos_ref[1]=ang_bhcm[1];
            _Pos_ref[2]=ang_bhcm[2];  

            _cont_interr_pasos=0; // reset contador
        }
        else
        {
          _cont_pasos=0;
          _linea=0;
          _cont_lin=0;
        }
   }
    

    Serial.printf(">Pos ref base: %f\n", _Pos_ref[0]);     //envía la posición en grados al terminal serie
    Serial.printf(">Pos ref hombro: %f\n", _Pos_ref[1]);     //envía la posición en grados al terminal serie
    Serial.printf(">Pos ref codo: %f\n", _Pos_ref[2]);     //envía la posición en grados al terminal serie
  }

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


hw_timer_t * timer = NULL; // Puntero para configurar timer

// *************** FN INTERRUPCION TIMER *********************

volatile bool has_expired = false;
void ARDUINO_ISR_ATTR InterrrupTimer()
{
  has_expired=true;
}



void setup()
{ 
  // Timer para tiempo de muestreo

  timer = timerBegin(1000000); // 1 Mhz de resolución (us)
  timerAttachInterrupt(timer, &InterrrupTimer);
  timerAlarm(timer, SAMPLE_TIME*1000000,true,0); // Set time en us

  // Inicializaciones

  Serial.begin(115200);  
  encoder_init();
  pwm_init();
  pinza_init(SERVO_PIN);
}


void loop() {
  // Declaracion de variables locales
  float ang_encoder[3]={0,0,0};  // Ángulos leidos de encoder
  float u_PID_M[3]={0,0,0};  // Variable de control de pwm

  // Cambiar set point por serial
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
        else if (input.startsWith("ABRIR"))
        {
          abrir_pinza();
        }
        else if(input.startsWith("CERRAR"))
        {
          cerrar_pinza();
        }
        else if(input.startsWith("LINEA"))
        {
          _linea=1;
        }
      }


  // Se ejecuta cada entrada a interrupción:
  if(has_expired)
  {

    // Lectura de encoders
    ang_encoder[0]=encoder0_read(); 
    ang_encoder[1]=encoder1_read();
    ang_encoder[2]=encoder2_read();

    // duty=0 si: Encoder lee fuera de rangos, se teclea STOP o si no se ha escrito nada por el serial desde el incio del programa
    if ((ang_encoder[0]>=LIM_SUP_E0 || ang_encoder[0]<=LIM_INF_E0) || (ang_encoder[1]>=LIM_SUP_E1 || ang_encoder[1]<=LIM_INF_E1) || (ang_encoder[2]>=LIM_SUP_E2 || ang_encoder[2]<=LIM_INF_E2) || _cont_serial==0 || _stop==1)
    {
        u_PID_M[0]=0; //Primer parámetro Posición de referencia; Segundo parámetro posición actual del encoder; Tercer parámetro: grados de margen para error
        u_PID_M[1]=0;
        u_PID_M[2]=0;
    }
    else
    {
      // Cálculo de variable de control de motores
      
      if(_linea==1) // si llamamos por serial el comando LINEA
      {
        float pos0_xyz[3]={0,350,-120}; //posicion inicial   ESTO SE REEMPLAZA POR COORDENADAS QUE DIGA LA TRAYECTORIA
        float pos1_xyz[3]={0,350,-160}; //posicion final

        FnLinea(pos0_xyz,pos1_xyz); // pos0_inicial, pos1_final 
      }

      // u_PID_M[0]=Robot_PID_m0(_Pos_ref[0],ang_encoder[0],3); //Primer parámetro Posición de referencia; Segundo parámetro posición actual del encoder; Tercer parámetro: grados de margen para error
      u_PID_M[0]=Control_motor(_Pos_ref[0],ang_encoder[0],3, KP_M0, KI_M0);
      u_PID_M[1]=Control_motor_sb(_Pos_ref[1],ang_encoder[1],0.5);
      //u_PID_M[2]=Robot_PID_m2(_Pos_ref[2],ang_encoder[2],1);
      u_PID_M[2]=Control_motor(_Pos_ref[2],ang_encoder[2],1, KP_M2, KI_M2);

    }    
    // Aplica pwm a motores

    duty_vec[0]=linPWM(u_PID_M[0], 0);
    duty_vec[1]=u_PID_M[1];
    duty_vec[2]=linPWM(u_PID_M[2], 2);
    set_comp_tres_motores(duty_vec);

    // Imprime info

    // Serial.print("Set point: ");
    Serial.printf(">Set point M0: %f\n", _Pos_ref[0]);     //envía la posición en grados al terminal serie
    Serial.printf(">Set point M1: %f\n", _Pos_ref[1]);     //envía la posición en grados al terminal serie
    Serial.printf(">Set point M2: %f\n", _Pos_ref[2]);     //envía la posición en grados al terminal serie
    //Serial.print("Ángulo encoder M0: "); 
    Serial.printf(">Ángulo encoder M0: %f\n", ang_encoder[0]);     //envía la posición en grados al terminal serie
    // Serial.println(ang_0);     //envía la posición en grados al terminal serie

    Serial.print(">Variable de control M0: ");
    Serial.println(duty_vec[0]);

    // Serial.print("Ángulo encoder M1: "); 
    Serial.printf(">Ángulo encoder M1: %f\n", ang_encoder[1]);     //envía la posición en grados al terminal serie
    // Serial.println(ang_1);     //envía la posición en grados al terminal serie

    Serial.print(">Variable de control M1: ");
    Serial.println(duty_vec[1]);

    // Serial.print("Ángulo encoder M2: "); 
    Serial.printf(">Ángulo encoder M2: %f\n", ang_encoder[2]);     //envía la posición en grados al terminal serie
    // Serial.println(ang_2);     //envía la posición en grados al terminal serie

    Serial.print(">Variable de control M2: ");
    Serial.println(duty_vec[2]);

    
    has_expired=false;
  }
  
}
  
