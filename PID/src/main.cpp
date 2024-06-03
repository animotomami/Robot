#include <Arduino.h> 
// #include <SPI.h> 

// Para pwm
// #include "driver/mcpwm_timer.h"
// #include "driver/mcpwm_oper.h"
// #include "driver/mcpwm_cmpr.h"
// #include "driver/mcpwm_gen.h"
// #include "driver/mcpwm_sync.h"
// #include "driver/mcpwm_cap.h"
// #include "driver/mcpwm_types.h"
// #include "driver/gpio.h"
// #include "esp_private/esp_clk.h" //Para esp_clk_apb_freq()

#include "MyControlFn.h"
#include "MyEncoderFn.h"
#include "MyPwmFn.h"

using namespace std;

//////////// PARA PWM /////////////
//Variables globales
float duty_vec[3]={0,-0.5,0.5}; //Array de duties. Pueden tomar valor -1 a 1.
float _Pos_ref=140; //Posición de referencia en grados
uint32_t _lastTimeSample=0;

// uint32_t _ticks_comp[3];

// //Se crea el handle (identificador) del timer. Se crea un solo timer para los tres motores.
// mcpwm_timer_handle_t timer0;
// //Se crea la estructura para la configuración del timer.
// mcpwm_timer_config_t conf_timer;

// //*************OPERADOR*************
// //Se crea el handle (identificador) del operador. Se crea un solo operador para los tres motores.
// mcpwm_oper_handle_t operador[3];
// //Se crea la estructura del operador.
// mcpwm_operator_config_t conf_operador;

// //*************COMPARADORES*************
// //Comparador 0
// //Se crea el handle (identificador) del comparador.
// mcpwm_cmpr_handle_t comparador[3]; //Se crean 3 comparadores. Uno por motor
// //Se crea la estructura del comparador.
// mcpwm_comparator_config_t conf_comparador;

// //*************GENERADORES*************
// mcpwm_gen_handle_t generadorA[3], generadorB[3];; //Se crean 6 generadores. Dos por motor.
// mcpwm_generator_config_t conf_gen0A,conf_gen0B,conf_gen1A,conf_gen1B,conf_gen2A,conf_gen2B;

// //Declaración de función.
// static void set_comp_tres_motores(float duty[]);

///////////////////////////////////////////////////

// ////// PARA ENCODER /////////////////

// //Frecuencia de reloj.
// static const int spiClk = 500000; // 500 kHz

// //Se definene las variables.

// uint16_t ABSposition = 0; 
// uint16_t ABSposition_last = 0; 
// uint8_t temp[2];

// //Variables del encoder 0
// float deg_enc0 = 0.00;
// float deg_abs_enc0 = 0.00;

// //Variables del encoder 1
// float deg_enc1 = 0.00;
// float deg_abs_enc1 = 0.00;

// //Variables del encoder 2
// float deg_enc2 = 0.00;
// float deg_abs_enc2 = 0.00;

// //Crear objetos SPI
// SPIClass * hspi = NULL;

//////////////////////////////////

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

//********************** FN SECUENCIA DE TRANSMISIÓN DE BITS POR SPI **********************

// uint8_t SPI_T_CS0 (uint8_t msg)    //msg representa el byte (8 bits) que se envía en cada caso por MOSI 
// { 
//   uint8_t msg_temp = 0;  //variable que contendrá los datos recibidos por MISO 
//   digitalWrite(HSPI_CS0,LOW);     
// //Se baja la línea SS y comienza la comunicación 
//   msg_temp = SPI.transfer(msg);    //Se envía y se recibe el dato por el bus 
//   digitalWrite(HSPI_CS0,HIGH);    //Se active la línea SS y termina la comunicación 
//   delayMicroseconds(25); //Se esperan 25us para la siguiente comunicación 
 
//   return(msg_temp);      //La función retorna el byte recibido 
// }

// uint8_t SPI_T_CS1 (uint8_t msg)    // Secuencia de transmisión de 8 bits en el bus SPI 
// //msg representa el byte (8 bits) que se envía en cada caso por MOSI 
// { 
//   uint8_t msg_temp = 1;  //variable que contendrá los datos recibidos por MISO 
//   digitalWrite(HSPI_CS1,LOW);     
// //Se baja la línea SS y comienza la comunicación 
//   msg_temp = SPI.transfer(msg);    //Se envía y se recibe el dato por el bus 
//   digitalWrite(HSPI_CS1,HIGH);    //Se active la línea SS y termina la comunicación 
//   delayMicroseconds(25); //Se esperan 25us para la siguiente comunicación 
 
// return(msg_temp);      //La función retorna el byte recibido 
// }

// uint8_t SPI_T_CS2 (uint8_t msg)    // Secuencia de transmisión de 8 bits en el bus SPI 
// //msg representa el byte (8 bits) que se envía en cada caso por MOSI 
// { 
//   uint8_t msg_temp = 0;  //variable que contendrá los datos recibidos por MISO 
//   digitalWrite(HSPI_CS2,LOW); //Se baja la línea CS2 y comienza la comunicación      
//   msg_temp = SPI.transfer(msg);    //Se envía y se recibe el dato por el bus 
//   digitalWrite(HSPI_CS2,HIGH);    //Se activa la línea CS2 y termina la comunicación 
//   delayMicroseconds(25); //Se esperan 25us para la siguiente comunicación 
 
// return(msg_temp);      //La función retorna el byte recibido
// //set_zero_enc_0 ();

// }

// //********************** FN LEER ENCODER **********************

// float encoder0_read ()
// {
//   uint8_t recibido = 0xA5;    //Se declara una variable temporal 
 
//   ABSposition = 0;    //Se inicializa el valor de posición del encoder 
 
//   SPI_T_CS0(0x10);   //Se envía el commando de lectura de posición al encoder. Se 
// //ignora el dato que provenga del encoder porque no será válido. 
 
//   recibido = SPI_T_CS0(0x00);  //Se envía NOP para comprobar si el esclavo está ya listo 
 
//   while (recibido != 0x10)    //bucle de espera a que el encoder responda con el 
// //eco del valor original enviado 
//   { 
//     recibido = SPI_T_CS0(0x00);  //sigue comprobando si está listo  
//   } 
 
//   temp[0] = SPI_T_CS0(0x00);    //Se recibe el MSByte en temp[0] 
//   temp[1] = SPI_T_CS0(0x00);    // Se recibe el LSByte en temp[1] 
 
//   temp[0] &= 0x0F;    //elimina los primeros 4 bits del MSByte 

//   ABSposition = temp[0] << 8; //desplaza MSB 8 posiciones a la izquierda para 
// //que ocupe el byte alto en ABSposition 
//   ABSposition += temp[1];    // añade LSB a ABSposition para reproducir el 
// //mensaje enviado por el encoder.

//   if (ABSposition != ABSposition_last)    //si no hubo cambios no se refresca 
//   { 
//     ABSposition_last = ABSposition;    //Toma la última medida como la buena 
//     deg_abs_enc0 = ABSposition;
//     deg_enc0=map(deg_abs_enc0,0,4096,0,360);
// //deg = deg * 0.08789;    // aprox 360/4096 
//     Serial.println(deg_enc0);     //envía la posición en grados al terminal serie 
//   }
//   return deg_enc0;
// }

// float encoder1_read ()
// {
//   uint8_t recibido = 0xA5;    //Se declara una variable temporal 
 
//   ABSposition = 0;    //Se inicializa el valor de posición del encoder 
 
//   SPI_T_CS1(0x10);   //Se envía el commando de lectura de posición al encoder. Se 
// //ignora el dato que provenga del encoder porque no será válido. 
 
//   recibido = SPI_T_CS1(0x00);  //Se envía NOP para comprobar si el esclavo está ya listo 
 
//   while (recibido != 0x10)    //bucle de espera a que el encoder responda con el 
// //eco del valor original enviado 
//   { 
//     recibido = SPI_T_CS1(0x00);  //sigue comprobando si está listo  
//   } 
 
//   temp[0] = SPI_T_CS1(0x00);    //Se recibe el MSByte en temp[0] 
//   temp[1] = SPI_T_CS1(0x00);    // Se recibe el LSByte en temp[1] 
 
//   temp[0] &= 0x0F;    //elimina los primeros 4 bits del MSByte 

//   ABSposition = temp[0] << 8; //desplaza MSB 8 posiciones a la izquierda para 
// //que ocupe el byte alto en ABSposition 
//   ABSposition += temp[1];    // añade LSB a ABSposition para reproducir el 
// //mensaje enviado por el encoder.

//   if (ABSposition != ABSposition_last)    //si no hubo cambios no se refresca 
//   { 
//     ABSposition_last = ABSposition;    //Toma la última medida como la buena 
//     deg_abs_enc1 = ABSposition;
//     //deg_enc1=map(deg_abs_enc1,0,4096.00,0,360.00);
//     deg_enc1 = deg_abs_enc1 * 0.08789;    // aprox 360/4096 
//     Serial.println(deg_enc1);     //envía la posición en grados al terminal serie 
//   }
//   return deg_enc1;
// }

// float encoder2_read ()
// {
//   uint8_t recibido = 0xA5;    //Se declara una variable temporal 
 
//   ABSposition = 0;    //Se inicializa el valor de posición del encoder 
 
//   SPI_T_CS2(0x10);   //Se envía el commando de lectura de posición al encoder. Se 
// //ignora el dato que provenga del encoder porque no será válido. 
 
//   recibido = SPI_T_CS2(0x00);  //Se envía NOP para comprobar si el esclavo está ya listo 
 
//   while (recibido != 0x10)    //bucle de espera a que el encoder responda con el 
// //eco del valor original enviado 
//   { 
//     recibido = SPI_T_CS2(0x00);  //sigue comprobando si está listo  
//   } 
 
//   temp[0] = SPI_T_CS2(0x00);    //Se recibe el MSByte en temp[0] 
//   temp[1] = SPI_T_CS2(0x00);    // Se recibe el LSByte en temp[1] 
 
//   temp[0] &= 0x0F;    //elimina los primeros 4 bits del MSByte 

//   ABSposition = temp[0] << 8; //desplaza MSB 8 posiciones a la izquierda para 
// //que ocupe el byte alto en ABSposition 
//   ABSposition += temp[1];    // añade LSB a ABSposition para reproducir el 
// //mensaje enviado por el encoder.

//   if (ABSposition != ABSposition_last)    //si no hubo cambios no se refresca 
//   { 
//     ABSposition_last = ABSposition;    //Toma la última medida como la buena 
//     deg_abs_enc2 = ABSposition;
//     //deg_enc2=map(deg_abs_enc2,0,4096.00,0,360.00);
//     deg_enc2 = deg_abs_enc2 * 0.08789;    // aprox 360/4096 
//     Serial.println(deg_enc2);     //envía la posición en grados al terminal serie 
//   }
//   return deg_enc2;
// }

// //********************** FN ZERO SET POINT**********************

// void set_zero_enc_0 () //Funcion que pone a cero el encoder
// {
//   uint8_t recibido = 0xA5;    //Se declara una variable temporal 

//   SPI_T_CS0(0x70);   //Se envía el commando de zero al encoder
 
//   recibido = SPI_T_CS0(0x00);  //Se envía NOP_A5 para comprobar si el esclavo está ya listo 
 
//   while (recibido != 0x80)    //bucle de espera a que el encoder responda con el 
// //eco del valor original enviado 
//   { 
//     recibido = SPI_T_CS0(0x00);  //sigue comprobando si está listo  
//   } 
// }

// void set_zero_enc_1 () //Funcion que pone a cero el encoder
// {
//   uint8_t recibido = 0xA5;    //Se declara una variable temporal 

//   SPI_T_CS1(0x70);   //Se envía el commando de zero al encoder
 
//   recibido = SPI_T_CS1(0x00);  //Se envía NOP_A5 para comprobar si el esclavo está ya listo 
 
//   while (recibido != 0x80)    //bucle de espera a que el encoder responda con el 
// //eco del valor original enviado 
//   { 
//     recibido = SPI_T_CS1(0x00);  //sigue comprobando si está listo  
//   } 
// }

// void set_zero_enc_2 () //Funcion que pone a cero el encoder
// {
//   uint8_t recibido = 0xA5;    //Se declara una variable temporal 

//   SPI_T_CS2(0x70);   //Se envía el commando de zero al encoder
 
//   recibido = SPI_T_CS2(0x00);  //Se envía NOP_A5 para comprobar si el esclavo está ya listo 
 
//   while (recibido != 0x80)    //bucle de espera a que el encoder responda con el 
// //eco del valor original enviado 
//   { 
//     recibido = SPI_T_CS2(0x00);  //sigue comprobando si está listo  
//   } 
// }

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

//   //Se crea la clase del HSPI
//   hspi = new SPIClass(HSPI);

//   //vspi.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_CS);
//   hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS0);

//   pinMode(HSPI_CS0,OUTPUT); //Se define la patilla CS0 como salida de Arduino
//   pinMode(HSPI_CS1,OUTPUT); //Se define la patilla CS1 como salida de Arduino
//   pinMode(HSPI_CS2,OUTPUT); //Se define la patilla CS2 como salida de Arduino

//   digitalWrite(HSPI_CS0,HIGH); //Se coloca la señal CS0 en ‘1’ lógico: comunicación inactiva 
//   digitalWrite(HSPI_CS1,HIGH); //Se coloca la señal CS1 en ‘1’ lógico: comunicación inactiva 
//   digitalWrite(HSPI_CS2,HIGH); //Se coloca la señal CS2 en ‘1’ lógico: comunicación inactiva 

// //Configuración de la comunicación SPI 
//   SPI.begin();  //Se inicializa la comunicación, definiendo SCK y MOSI como salidas y colocándolas a nivel bajo (inactivo) 

//   SPI.setBitOrder(MSBFIRST);  // Se enviará primero el bit de mayor significado

//   SPI.setDataMode(SPI_MODE0); //Se selecciona el estandar de comunicación SPI 
// //conocido como Modo 0, porque así lo precisa el esclavo de este ejemplo

//   SPI.setClockDivider(SPI_CLOCK_DIV32); // Se divide el reloj de Arduino por 32 
// //para determinar la frecuencia de la señal de reloj del bus (16MHz/32 = 500 kHz) 

  encoder_init();
  Serial.begin(115200);  
  Serial.println("Comienzo de programa"); 
  Serial.flush();  //Espera a que termina la transmisión serie anterior 
  delay(200);

  //***************setup TIMER***************
// //Creación de la estructura del timer
// conf_timer.group_id=0;//Selección del módulo MCPWM0
// conf_timer.clk_src=MCPWM_TIMER_CLK_SRC_DEFAULT; //Frecuencia default de 160 MHZ
// conf_timer.resolution_hz=10000000; //10 MHz -> 1 tick= 0.1 us
// conf_timer.count_mode=MCPWM_TIMER_COUNT_MODE_UP;
// conf_timer.period_ticks=500; // 500*0.1 us = 20 kHz
// //Fin de creación de estructura del timer.

// //Se crea el timer con la estructura definida anteriormente.
// mcpwm_new_timer(&conf_timer,&timer0);

// //***************setup OPERADOR***************
// //Creación de la estructura del OPERADOR
// conf_operador.group_id=0; //Debe compartir el mismo módulo que el timer asociado.
// conf_operador.intr_priority=0; //Prioridad por defecto.

// //Se crea el operador 0 con la estructura previamente definida.

// //Se crea el operador i con la estructura previamente definida y se vincula el operador i al timer 0.
// for(int i=0;i<3;i++)
// {
//   mcpwm_new_operator(&conf_operador,&operador[i]);
//   mcpwm_operator_connect_timer(operador[i],timer0);
// }

// //***************setup COMPARADOR***************

// //Configuración de los tres comparadores
//  conf_comparador.flags.update_cmp_on_tez = true;

// //Se crean los 3 comparadores

// for(int i=0;i<3;i++)
// {
//   mcpwm_new_comparator(*(operador+i), &conf_comparador, comparador+i);
//   mcpwm_comparator_set_compare_value(comparador[i], 0); //Valor de comparación. Se inicializa a cero.
// }
//   //para obtener 3 duties distintos.

// //***************setup GENERADORES***************
// //Se crea la configuración de cada generador. Se asocia un GPIO a cada uno.
// conf_gen0A.gen_gpio_num = GPIO_PWM_0A;
// conf_gen0B.gen_gpio_num = GPIO_PWM_0B;

// conf_gen1A.gen_gpio_num = GPIO_PWM_1A;
// conf_gen1B.gen_gpio_num = GPIO_PWM_1B;

// conf_gen2A.gen_gpio_num = GPIO_PWM_2A;
// conf_gen2B.gen_gpio_num = GPIO_PWM_2B;

// //Se crean los generadores y se asocian al operador

// mcpwm_new_generator(operador[0], &conf_gen0A, &generadorA[0]);
// mcpwm_new_generator(operador[0], &conf_gen0B, &generadorB[0]);

// mcpwm_new_generator(operador[1], &conf_gen1A, &generadorA[1]);
// mcpwm_new_generator(operador[1], &conf_gen1B, &generadorB[1]);

// mcpwm_new_generator(operador[2], &conf_gen2A, &generadorA[2]);
// mcpwm_new_generator(operador[2], &conf_gen2B, &generadorB[2]);

// //Se tratan los eventos que forman el PWM del generador.

// for(int i=0;i<3;i++)
// {
//   //GENERADOR iA
//   mcpwm_generator_set_action_on_timer_event(generadorA[i], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH ) ) ;
//   mcpwm_generator_set_action_on_compare_event(generadorA[i], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparador[i], MCPWM_GEN_ACTION_LOW ) );

// //GENERADOR iB
//   mcpwm_generator_set_action_on_timer_event(generadorB[i], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH ) ) ;
//   mcpwm_generator_set_action_on_compare_event(generadorB[i], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparador[i], MCPWM_GEN_ACTION_LOW ) );
// }

// //Finalmente se habilita el Timer
// //Se habilita el timer.
// mcpwm_timer_enable(timer0);

// //Se arranca el timer.
// mcpwm_timer_start_stop(timer0,MCPWM_TIMER_START_NO_STOP); //Arranca el timer y no para hasta recibir la orden de parada.
  
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
  


// //Función para control de motores según el duty introducido.

// void set_comp_tres_motores(float set_duty[])
// {
//  for(int i=0;i<3;i++)
//   {
//     if(set_duty[i]<0)
//       {
//         _ticks_comp[i]=uint32_t((-1)*(set_duty[i]*500)); //Se adapta a valores del comparador
//         Serial.println(set_duty[i]);
//         Serial.println(_ticks_comp[i]);
       
//         mcpwm_comparator_set_compare_value(comparador[i], _ticks_comp[i]); //Valor de comparación.

//         //RETROCESO DEL MOTOR
//         mcpwm_generator_set_force_level(generadorA[i], 0,true);
//         mcpwm_generator_set_force_level(generadorB[i], -1,true);
        
//       }
//     else if(set_duty[i]>0)
//       {
//         _ticks_comp[i]=uint32_t(set_duty[i]*500); //Se adapta a valores del comparador
//         Serial.println(set_duty[i]);
//         Serial.println(_ticks_comp[i]);

//         mcpwm_comparator_set_compare_value(comparador[i], _ticks_comp[i]); //Valor de comparación.

//         //AVANCE DEL MOTOR
//         mcpwm_generator_set_force_level(generadorA[i], -1,true); //Con -1 se deshabilita el forzado del nivel. El bool true lo mantiene hasta que se indique el -1.
//         mcpwm_generator_set_force_level(generadorB[i], 0,true); //Fuerza el nivel de la salida pwm a 0. Lo apaga.
      
//       }
//     else
//       {
//         _ticks_comp[i]=0; //Se pone el duty a cero.
//         Serial.println(set_duty[i]);
//         Serial.println(_ticks_comp[i]);

        
//         mcpwm_comparator_set_compare_value(comparador[i], _ticks_comp[i]);  //Valor de comparación.

//         //PARADA DEL MOTOR
//         mcpwm_generator_set_force_level(generadorA[i], 0,true);
//         mcpwm_generator_set_force_level(generadorB[i], 0,true);
       
//       }  
//   }
// }


// void retroceso_motor(int motor)
// {
//     mcpwm_comparator_set_compare_value(comparador[motor], 200); //Valor de comparación.

// //RETROCESO DEL MOTOR
//     mcpwm_generator_set_force_level(generadorA[motor], 0,true);
//     mcpwm_generator_set_force_level(generadorB[motor], -1,true);
// }

// void avance_motor(int motor)
// {
//     mcpwm_comparator_set_compare_value(comparador[motor], 200); //Valor de comparación.

// //AVANCE DEL MOTOR
//     mcpwm_generator_set_force_level(generadorA[motor], -1,true);
//     mcpwm_generator_set_force_level(generadorB[motor], 0,true);
// }

// void parada_motor(int motor, float valor_comp)
// {
//     mcpwm_comparator_set_compare_value(comparador[motor], 0); //Valor de comparación.

// //PARADA DEL MOTOR
//     mcpwm_generator_set_force_level(generadorA[motor], 0,true);
//     mcpwm_generator_set_force_level(generadorB[motor], 0,true);
// }


