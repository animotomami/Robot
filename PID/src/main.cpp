#include <Arduino.h> 
#include "MyControlFn.h"
#include "MyEncoderFn.h"
#include "MyPwmFn.h"
#include "PinzaFn.h"
#include <ESP32Servo.h>


float L1=250, L2=250, L3=45, L_pen=70; // HACER DEFINES


#define TIEMPO_ENTRE_PASOS 5 // es TIEMPO ENTRE PASOS*TIEMPO DE MUESTREO en ms
#define STEP_SIZE 10 // Tamaño de paso en mm para trayectorias

using namespace std;

//Variables globales
float duty_vec[3]={0,0,0}; //Array de duties. Pueden tomar valor -1 a 1.
float _Pos_ref[3]={170,70,200}; //Posición de referencia en grados

uint32_t _cont_serial=0,_stop=0, _linea=0;
float _error_ant=0; // *para D

float _pos_actual_xyz[3]={0,0,0}, ang_bhcm[4]={0, 0, 0, 0};
uint32_t _t_wait_lin=0, _cont_lin=0, _cont_pasos=0;
uint32_t _cont_interr_pasos=0; 


float max_2Val (float val_1, float val_2)   
    //  Función de cálculo del máximo entre dos valores:
    //      - Entradas: (float) val_1, val_2: valores entre los que se calcula el máximo.
    //      - Salidas: (float) valor máximo obtenido (y aviso de error si ambos son iguales).
{
    float max=0;    // Declaración de variables.

    if (val_1>val_2)        // Si el valor 1 es mayor que el 2:
        max=val_1;          //      El valor 1 es máximo.

    else if (val_1<val_2)   // Si el valor 2 es mayor que el 1:
        max=val_2;          //      El valor 2 es máximo.

      else    // Si no se cumple ninguna condición:
      {
          std::cout << "Ambos valores son iguales." << std::endl;     // Envío de mensaje de igualdad.
          max=val_2;                                                  // Se establece el valor como máximo igualmente.
      }

    return max;     // Se devuelve el valor obtenido.
}

float convAng_RadToDeg (float angRad)   
    // Función de conversión de ángulos en radianes a decimales:
    //      - Entradas: (float) angRad: Valor angular en radianes a convertir.
    //      - Salidas: (float) angDeg: Valor angular convertido a Grados Decimales.
{
    float angDeg=0;                 // Declaración de variables.
    angDeg=angRad*360/(2*3.1416);   // Fórmula de conversión a decimales.
    return angDeg;                  // Devuelve el ángulo calculado en Grados Decimales.
}


void cin_Inversa (float x, float y, float z)    
    // Función de cálculo de la cinemática inversa de un Robot Tipo Braccio de TinkerKit.
    // Entra el valor de la posición de la pluma en ejes x, y y z y calcula los ángulos que deben adquirir
    // los servos del brazo para alcanzar la posición manteniendo la pluma perpendicular al plano de dibujo.
    //      - Entradas: (float) x, y, z: Posiciones en ejes x, y y z deseadas de la pluma.
    //      - Salidas: Modifica sobre la variable global de ángulos de servo "ang_bhcm[4]".
{
    float t, a, b, c, zP1_plus, zP1_minus, zP1;     // Declaración de variables.
    float P1[2]={0,0}, P2[2]={0,0}, P3[2]={0,0};

    // Nota: Se trabaja en el plano w-z, que contiene las articulaciones del hombro, codo y muñeca, ignorando el giro de la base.
    P3[0]=z+L_pen;
    P3[1]=sqrt(pow(x,2)+pow(y,2));

    P2[0]=P3[0]+L3;
    P2[1]=P3[1];

    // RESOLUCIÖN DE LA ECUACIÖN CUADRÁTICA PARA LA POSICIÓN DEL CODO:
    t=-pow(L2,2)+pow(L1,2)+pow(P2[1],2)+pow(P2[0],2);   // Cálculo de la constante t.
    a=4*pow(P2[0],2)+(4*pow(P2[1],2));                  // Cálculo del coeficiente a.
    b=-4*t*P2[0];                                       // Cálculo del coeficiente b.
    c=(-4*pow(P2[1],2)*pow(L1,2))+(pow(t,2));           // Cálculo del coeficiente c.

    zP1_plus=(-b+sqrt(pow(b,2)-(4*a*c)))/(2*a);     // Cálculo de la solución 01 de la ecuación.
    zP1_minus=(-b-sqrt(pow(b,2)-(4*a*c)))/(2*a);    // Cálculo de la solución 02 de la ecuación.

    zP1=max_2Val(zP1_minus,zP1_plus);                       // LLamada a función de selección de la mayor solución disponible.

    P1[0]=zP1;
    P1[1]=sqrt((L1*L1)-(zP1*zP1));

    // CÁLCULOS DE LOS ÁNGULOS (radianes, en Referencia Robot), (almacenamiento en variable global):
    ang_bhcm[0]=asin(x/P3[1]);                                                        // Cálculo del ángulo de la base.
    ang_bhcm[1]=atan2(P1[1],P1[0]);                                                   // Cálculo del ángulo del hombro.
    ang_bhcm[2]=-(atan2((P2[0]-P1[0]),(P2[1]-P1[1]))-((3.1416/2)-ang_bhcm[1]));       // Cálculo del ángulo del codo.
    ang_bhcm[3]=acos((P3[0]-P2[0])/L3)-ang_bhcm[1]-ang_bhcm[2];                       // Cálculo del ángulo de la base.

    // BUCLE DE COONVERSIÓN A GRADOS DECIMALES:
    for (int j=0;j<4;j++)
    {
        ang_bhcm[j]=convAng_RadToDeg(ang_bhcm[j]);   //  Llamada a función de conversión de radianes a grados, (sobreescritura de variable global).
    }

    // NORMALIZADO:
    ang_bhcm[0]=180+ang_bhcm[0];
    ang_bhcm[1]=135-ang_bhcm[1];
    ang_bhcm[2]=110+ang_bhcm[2];
}

void FnLinea(float pos0_xyz[], float pos1_xyz[]) // Posicion inicial_0, Posicion final_1
{
  // Declara variables locales
  int pasos=0;
  float delta_x=0,delta_y=0; // mm
  
  if (_cont_lin==0) // SOLO ENTRA LA PRIMERA VEZ
  {
    cin_Inversa(pos0_xyz[0],pos0_xyz[1],pos0_xyz[2]);
    _Pos_ref[0]=ang_bhcm[0];
    _Pos_ref[1]=ang_bhcm[1];
    _Pos_ref[2]=ang_bhcm[2];
     Serial.printf(">ANG0: %f\n", ang_bhcm[0]);     //envía la posición en grados al terminal serie
     Serial.printf(">ANG1: %f\n", ang_bhcm[1]);     //envía la posición en grados al terminal serie
     Serial.printf(">ANG2: %f\n", ang_bhcm[2]);     //envía la posición en grados al terminal serie
    
    _pos_actual_xyz[0]=pos0_xyz[0];
    _pos_actual_xyz[1]=pos0_xyz[1];
    _pos_actual_xyz[2]=pos0_xyz[2];
    _t_wait_lin=millis();
    _cont_lin++;
  }

  if (millis()-_t_wait_lin > 2000) {
    
    Serial.println("Se cumplio 2 seg");
        
    
    pasos=abs((pos1_xyz[0]-pos0_xyz[0])/STEP_SIZE);    // Cálculo del número de pasos (basado en eje x para garantizar la fiabilidad de rectas diagonales).

    if (pasos==0) {                // Si el movimiento se produce exclusivamente en el eje y, el paso se basa en el eje y.
        pasos=abs((pos1_xyz[1]-pos0_xyz[1])/STEP_SIZE);
    }

    delta_x=((pos1_xyz[0]-pos0_xyz[0]))/pasos;  // Cálculo de la distancia a recorrer por paso en el eje x.
    delta_y=(pos1_xyz[1]-pos0_xyz[1])/pasos;  // Cálculo de la distancia a recorrer por paso en el eje y.
    _cont_interr_pasos++;
    
   if (_cont_interr_pasos>TIEMPO_ENTRE_PASOS/SAMPLE_TIME)
   {
    if(_cont_pasos<pasos)
        {

          _cont_pasos++;
          _pos_actual_xyz[0]=_pos_actual_xyz[0]+delta_x;  // Suma de la distancia objetivo a la posición x actual.
          _pos_actual_xyz[1]=_pos_actual_xyz[1]+delta_y;  // Suma de la distancia objetivo a la posición y actual.
          cin_Inversa(_pos_actual_xyz[0],_pos_actual_xyz[1],0);  // Llamada a función del cálculo de la cinemática inversa.

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
//float kp_m1=1.8; //Constante Proporcional
float kp_m1=0.5; //Constante Proporcional
float ki_m1=0; //Constante Integral
float kd_m1=0.001; //Constante Derivativa

float kp_m1_subida=0.32;
float kp_m1_bajada=0.05;

float kd_m1_subida=0.002;
float kd_m1_bajada=0.0015;

//Variables de memoria

float pos_ref_previa_m0;
float pos_ref_previa_m1;
float pos_ref_previa_m2;
float pos_encoder_previa_m0;
float pos_encoder_previa_m1;
float pos_encoder_previa_m2;



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
    // Q1=1-N*Ts; //Cálculo del parámetro Q1
    // Q2=kd_m1*N;  //Cálculo del parámetro Q2
 
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

    if (pos_encoder>pos_ref)  // bajada
    {
      P=kp_m1_bajada*(pos_ref-pos_encoder);
      if (P<-0.35)
      {
        P=-0.35;
      }
      D=kd_m1*((error-_error_ant)/Ts);
    }
    else if (pos_encoder<=pos_ref) //subida
    {
      P=kp_m1_subida*(pos_ref-pos_encoder);
      if (P>=0.9)
      {
        P=0.9;
      }
      D=kd_m1*((error-_error_ant)/Ts);
    }
    

  //Acción derivativa 
    // D=Q1*Dprevio_m1+Q2*c*(pos_ref-pos_ref_previa_m0)-Q2*(pos_encoder-pos_encoder_previa_m0);
    // Dprevio_m1=D;


    

  //Consigna de control

  // if (((P+D)<0 && P>0) || ((P+D)>0 && P<0))
  // {
  //   u=0;
  // } 
  // else 
  // {
  //   u=P+I;
  // }
    u=P+D;
    Serial.printf(">P: %f\n", P);     //envía la posición en grados al terminal serie
    Serial.printf(">D: %f\n", D);     //envía la posición en grados al terminal serie
    

    if (u>1) //Si la consigna es mayor que la saturación se pone a 1.
    {  
      u=1; 
      satur_m1=1;
    }
    else if (u<-1) //Si la consigna es menor que la saturación se pone a -1
    {
      u=-1; 
      satur_m1=-1;
    }
    else //Si no se cumple ninguna condición anterior la consigna no se modifica
    {
      Iprevio_m1=I; 
      satur_m1=0; 
    }

    // Si se actualiza el set point o si se pasa de STOP a GO
    if (pos_ref!=pos_ref_previa_m1 || _cont_serial==0)
    {
      Iprevio_m1=0;
    }

  //Actualización de variables 
  pos_ref_previa_m1=pos_ref; //Actualizar posicion de referencia previ del paso anterior
  pos_encoder_previa_m1=pos_encoder; //Actualizar posición del encoder previa del paso anterior
  _error_ant=error;

  u_pwm=u;
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
  //float ang_0=0, ang_1=0, ang_2=0;
  float ang_encoder[3]={0,0,0};  // Ángulos leidos de encoder
  //float u_PID_M0=0, u_PID_M1=0, u_PID_M2=0; //Variable de control del PWM
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
    // Contador para tiempo entre pasos
    

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
        Iprevio_m2=0;
    }
    else
    {
        // Cálculo de variable de control de motores
      if(_linea==1) // si llamamos por serial el comando LINEA
      {
        float pos0_xyz[3]={0,350,0}; //posicion inicial   ESTO SE REEMPLAZA POR COORDENADAS QUE DIGA LA TRAYECTORIA
        float pos1_xyz[3]={0,400,0}; //posicion final

        FnLinea(pos0_xyz,pos1_xyz); // La funcion cambia las referencias 
      }

      u_PID_M[0]=Robot_PID_m0(_Pos_ref[0],ang_encoder[0],3); //Primer parámetro Posición de referencia; Segundo parámetro posición actual del encoder; Tercer parámetro: grados de margen para error
      u_PID_M[1]=Robot_PID_m1(_Pos_ref[1],ang_encoder[1],1);
      u_PID_M[2]=Robot_PID_m2(_Pos_ref[2],ang_encoder[2],1);

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

    
    // Serial.println(_Pos_ref);  
    
    has_expired=false;
  }
  
}
  
