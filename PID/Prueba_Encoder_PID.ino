#include <SPI.h>

#define HSPI 2  // 2 para S2 y S3, 1 para S1.
#define VSPI 3

// Se reemplazan los pines del HSPI por los pines que se van a usar.
const int HSPI_MISO = 13;
const int HSPI_MOSI = 11;
const int HSPI_SCLK = 12;
const int HSPI_CS0 = 14;


//Frecuencia de reloj.
static const int spiClk = 500000; // 500 kHz

//Se definene las variables.

uint16_t ABSposition = 0; 
uint16_t ABSposition_last = 0; 
uint8_t temp[2];

//Variables del encoder 0
float deg_enc0 = 0.00;
float deg_abs_enc0 = 0.00;

//Crear objetos SPI
SPIClass * hspi = NULL;

//Declaración de parámetros del PID

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
float kp=5; //Constante Proporcional
float ki=10.315; //Constante Integral
float kd=-0.4447; //Constante Derivativa

//Variables de memoria

float pos_ref_previa;
float pos_encoder_previa;

void setup()
{ 
  //Se crea la clase del HSPI
  hspi = new SPIClass(HSPI);

//vspi.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_CS);
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS0);



  pinMode(HSPI_CS0,OUTPUT); //Se define la patilla CS0 como salida de Arduino

  digitalWrite(HSPI_CS0,HIGH); //Se coloca la señal CS0 en ‘1’ lógico: comunicación inactiva 

//Configuración de la comunicación SPI 
  SPI.begin();  //Se inicializa la comunicación, definiendo SCK y MOSI como salidas y colocándolas a nivel bajo (inactivo) 

  SPI.setBitOrder(MSBFIRST);  // Se enviará primero el bit de mayor significado

  SPI.setDataMode(SPI_MODE0); //Se selecciona el estandar de comunicación SPI 
//conocido como Modo 0, porque así lo precisa el esclavo de este ejemplo

  SPI.setClockDivider(SPI_CLOCK_DIV32); // Se divide el reloj de Arduino por 32 
//para determinar la frecuencia de la señal de reloj del bus (16MHz/32 = 500 kHz) 
//Se define una comunicación serie para enviar la lectura obtenida a un terminal 
  Serial.begin(9600);  
  Serial.println("Comienzo de programa"); 
  Serial.flush();  //Espera a que termina la transmisión serie anterior 
  delay(2000); //Espera 2 segundos 
}


uint8_t SPI_T_CS0 (uint8_t msg)    // Secuencia de transmisión de 8 bits en el bus SPI 
//msg representa el byte (8 bits) que se envía en cada caso por MOSI 
{ 
  uint8_t msg_temp = 0;  //variable que contendrá los datos recibidos por MISO 
  digitalWrite(HSPI_CS0,LOW);     
//Se baja la línea SS y comienza la comunicación 
  msg_temp = SPI.transfer(msg);    //Se envía y se recibe el dato por el bus 
  digitalWrite(HSPI_CS0,HIGH);    //Se active la línea SS y termina la comunicación 
  delayMicroseconds(25); //Se esperan 25us para la siguiente comunicación 
 
return(msg_temp);      //La función retorna el byte recibido 
}

void loop() {
  float ang_0=0;
  float u_PID=0; //Variable de control del PWM

  float Pos_ref=105; //Posición de referencia en grados

  ang_0=encoder0_read(); //Introducir valor en el código si no hay encoder para hacer pruebas

  //ang_0=Serial.parseFloat();// Esta linea es solo para pruebas
  
  u_PID=Robot_PID(Pos_ref,ang_0); //Primer parámetro Posición de referencia; Segundo parámetro posición actual del encoder

  Serial.println("Ángulo encoder"); 
  Serial.println(ang_0);     //envía la posición en grados al terminal serie

  Serial.println("Variable de control");
  Serial.println(u_PID);  
  delay(500);

}

float Robot_PID(float pos_ref,float pos_encoder)
{
    //Declaración de la variable de control

    float u;
    float u_pwm;

    //Declaración de las variables temporales del PID

    float P;
    float I;
    float D;   

    float incrI;
    
    //Definición de factores 
    Q1=1-N*Ts; //Cálculo del parámetro Q1
    Q2=kd*N;  //Cálculo del parámetro Q2
 
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

  Serial.println("Proporcional");
  Serial.println(P);
  Serial.println("Integral");
  Serial.println(I);
  Serial.println("Derivativa");
  Serial.println(D);
    u=P+I+D;

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

return u_pwm;
}

float encoder0_read ()
{
  uint8_t recibido = 0xA5;    //Se declara una variable temporal 
 
  ABSposition = 0;    //Se inicializa el valor de posición del encoder 
 
  SPI_T_CS0(0x10);   //Se envía el commando de lectura de posición al encoder. Se 
//ignora el dato que provenga del encoder porque no será válido. 
 
  recibido = SPI_T_CS0(0x00);  //Se envía NOP para comprobar si el esclavo está ya listo 
 
  while (recibido != 0x10)    //bucle de espera a que el encoder responda con el 
//eco del valor original enviado 
  { 
    recibido = SPI_T_CS0(0x00);  //sigue comprobando si está listo  
  } 
 
  temp[0] = SPI_T_CS0(0x00);    //Se recibe el MSByte en temp[0] 
  temp[1] = SPI_T_CS0(0x00);    // Se recibe el LSByte en temp[1] 
 
  temp[0] &= 0x0F;    //elimina los primeros 4 bits del MSByte 

  ABSposition = temp[0] << 8; //desplaza MSB 8 posiciones a la izquierda para 
//que ocupe el byte alto en ABSposition 
  ABSposition += temp[1];    // añade LSB a ABSposition para reproducir el 
//mensaje enviado por el encoder.

  if (ABSposition != ABSposition_last)    //si no hubo cambios no se refresca 
  { 
    ABSposition_last = ABSposition;    //Toma la última medida como la buena 
    deg_abs_enc0 = ABSposition;
    deg_enc0=map(deg_abs_enc0,0,4096,0,360);
//deg = deg * 0.08789;    // aprox 360/4096 
    Serial.println(deg_enc0);     //envía la posición en grados al terminal serie 
  }
  return deg_enc0;
}

//**********************ZERO SET POINT**********************

void set_zero_enc_0 () //Funcion que pone a cero el encoder
{
  uint8_t recibido = 0xA5;    //Se declara una variable temporal 

  SPI_T_CS0(0x70);   //Se envía el commando de zero al encoder
 
  recibido = SPI_T_CS0(0x00);  //Se envía NOP_A5 para comprobar si el esclavo está ya listo 
 
  while (recibido != 0x80)    //bucle de espera a que el encoder responda con el 
//eco del valor original enviado 
  { 
    recibido = SPI_T_CS0(0x00);  //sigue comprobando si está listo  
  } 
}
