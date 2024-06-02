
#include <Arduino.h>
#include <SPI.h> 
#include "MyEncoderFn.h"

////// PARA ENCODER /////////////////

//Frecuencia de reloj.
static const int spiClk = 500000; // 500 kHz

//Se definene las variables.

uint16_t ABSposition = 0; 
uint16_t ABSposition_last = 0; 
uint8_t temp[2];

//Variables del encoder 0
float deg_enc0 = 0.00;
float deg_abs_enc0 = 0.00;

//Variables del encoder 1
float deg_enc1 = 0.00;
float deg_abs_enc1 = 0.00;

//Variables del encoder 2
float deg_enc2 = 0.00;
float deg_abs_enc2 = 0.00;

//Crear objetos SPI
SPIClass * hspi = NULL;

uint8_t SPI_T_CS0 (uint8_t msg)    //msg representa el byte (8 bits) que se envía en cada caso por MOSI 
{ 
  uint8_t msg_temp = 0;  //variable que contendrá los datos recibidos por MISO 
  digitalWrite(HSPI_CS0,LOW);     
//Se baja la línea SS y comienza la comunicación 
  msg_temp = SPI.transfer(msg);    //Se envía y se recibe el dato por el bus 
  digitalWrite(HSPI_CS0,HIGH);    //Se active la línea SS y termina la comunicación 
  delayMicroseconds(25); //Se esperan 25us para la siguiente comunicación 
 
  return(msg_temp);      //La función retorna el byte recibido 
}

uint8_t SPI_T_CS1 (uint8_t msg)    // Secuencia de transmisión de 8 bits en el bus SPI 
//msg representa el byte (8 bits) que se envía en cada caso por MOSI 
{ 
  uint8_t msg_temp = 1;  //variable que contendrá los datos recibidos por MISO 
  digitalWrite(HSPI_CS1,LOW);     
//Se baja la línea SS y comienza la comunicación 
  msg_temp = SPI.transfer(msg);    //Se envía y se recibe el dato por el bus 
  digitalWrite(HSPI_CS1,HIGH);    //Se active la línea SS y termina la comunicación 
  delayMicroseconds(25); //Se esperan 25us para la siguiente comunicación 
 
return(msg_temp);      //La función retorna el byte recibido 
}

uint8_t SPI_T_CS2 (uint8_t msg)    // Secuencia de transmisión de 8 bits en el bus SPI 
//msg representa el byte (8 bits) que se envía en cada caso por MOSI 
{ 
  uint8_t msg_temp = 0;  //variable que contendrá los datos recibidos por MISO 
  digitalWrite(HSPI_CS2,LOW); //Se baja la línea CS2 y comienza la comunicación      
  msg_temp = SPI.transfer(msg);    //Se envía y se recibe el dato por el bus 
  digitalWrite(HSPI_CS2,HIGH);    //Se activa la línea CS2 y termina la comunicación 
  delayMicroseconds(25); //Se esperan 25us para la siguiente comunicación 
 
return(msg_temp);      //La función retorna el byte recibido
//set_zero_enc_0 ();

}

//********************** FN LEER ENCODER **********************

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

float encoder1_read ()
{
  uint8_t recibido = 0xA5;    //Se declara una variable temporal 
 
  ABSposition = 0;    //Se inicializa el valor de posición del encoder 
 
  SPI_T_CS1(0x10);   //Se envía el commando de lectura de posición al encoder. Se 
//ignora el dato que provenga del encoder porque no será válido. 
 
  recibido = SPI_T_CS1(0x00);  //Se envía NOP para comprobar si el esclavo está ya listo 
 
  while (recibido != 0x10)    //bucle de espera a que el encoder responda con el 
//eco del valor original enviado 
  { 
    recibido = SPI_T_CS1(0x00);  //sigue comprobando si está listo  
  } 
 
  temp[0] = SPI_T_CS1(0x00);    //Se recibe el MSByte en temp[0] 
  temp[1] = SPI_T_CS1(0x00);    // Se recibe el LSByte en temp[1] 
 
  temp[0] &= 0x0F;    //elimina los primeros 4 bits del MSByte 

  ABSposition = temp[0] << 8; //desplaza MSB 8 posiciones a la izquierda para 
//que ocupe el byte alto en ABSposition 
  ABSposition += temp[1];    // añade LSB a ABSposition para reproducir el 
//mensaje enviado por el encoder.

  if (ABSposition != ABSposition_last)    //si no hubo cambios no se refresca 
  { 
    ABSposition_last = ABSposition;    //Toma la última medida como la buena 
    deg_abs_enc1 = ABSposition;
    //deg_enc1=map(deg_abs_enc1,0,4096.00,0,360.00);
    deg_enc1 = deg_abs_enc1 * 0.08789;    // aprox 360/4096 
    Serial.println(deg_enc1);     //envía la posición en grados al terminal serie 
  }
  return deg_enc1;
}

float encoder2_read ()
{
  uint8_t recibido = 0xA5;    //Se declara una variable temporal 
 
  ABSposition = 0;    //Se inicializa el valor de posición del encoder 
 
  SPI_T_CS2(0x10);   //Se envía el commando de lectura de posición al encoder. Se 
//ignora el dato que provenga del encoder porque no será válido. 
 
  recibido = SPI_T_CS2(0x00);  //Se envía NOP para comprobar si el esclavo está ya listo 
 
  while (recibido != 0x10)    //bucle de espera a que el encoder responda con el 
//eco del valor original enviado 
  { 
    recibido = SPI_T_CS2(0x00);  //sigue comprobando si está listo  
  } 
 
  temp[0] = SPI_T_CS2(0x00);    //Se recibe el MSByte en temp[0] 
  temp[1] = SPI_T_CS2(0x00);    // Se recibe el LSByte en temp[1] 
 
  temp[0] &= 0x0F;    //elimina los primeros 4 bits del MSByte 

  ABSposition = temp[0] << 8; //desplaza MSB 8 posiciones a la izquierda para 
//que ocupe el byte alto en ABSposition 
  ABSposition += temp[1];    // añade LSB a ABSposition para reproducir el 
//mensaje enviado por el encoder.

  if (ABSposition != ABSposition_last)    //si no hubo cambios no se refresca 
  { 
    ABSposition_last = ABSposition;    //Toma la última medida como la buena 
    deg_abs_enc2 = ABSposition;
    //deg_enc2=map(deg_abs_enc2,0,4096.00,0,360.00);
    deg_enc2 = deg_abs_enc2 * 0.08789;    // aprox 360/4096 
    Serial.println(deg_enc2);     //envía la posición en grados al terminal serie 
  }
  return deg_enc2;
}

//********************** FN ZERO SET POINT**********************

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

void set_zero_enc_1 () //Funcion que pone a cero el encoder
{
  uint8_t recibido = 0xA5;    //Se declara una variable temporal 

  SPI_T_CS1(0x70);   //Se envía el commando de zero al encoder
 
  recibido = SPI_T_CS1(0x00);  //Se envía NOP_A5 para comprobar si el esclavo está ya listo 
 
  while (recibido != 0x80)    //bucle de espera a que el encoder responda con el 
//eco del valor original enviado 
  { 
    recibido = SPI_T_CS1(0x00);  //sigue comprobando si está listo  
  } 
}

void set_zero_enc_2 () //Funcion que pone a cero el encoder
{
  uint8_t recibido = 0xA5;    //Se declara una variable temporal 

  SPI_T_CS2(0x70);   //Se envía el commando de zero al encoder
 
  recibido = SPI_T_CS2(0x00);  //Se envía NOP_A5 para comprobar si el esclavo está ya listo 
 
  while (recibido != 0x80)    //bucle de espera a que el encoder responda con el 
//eco del valor original enviado 
  { 
    recibido = SPI_T_CS2(0x00);  //sigue comprobando si está listo  
  } 
}

void encoder_init()
{
//Se crea la clase del HSPI
  hspi = new SPIClass(HSPI);

  //vspi.begin(VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_CS);
  hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, HSPI_CS0);

  pinMode(HSPI_CS0,OUTPUT); //Se define la patilla CS0 como salida de Arduino
  pinMode(HSPI_CS1,OUTPUT); //Se define la patilla CS1 como salida de Arduino
  pinMode(HSPI_CS2,OUTPUT); //Se define la patilla CS2 como salida de Arduino

  digitalWrite(HSPI_CS0,HIGH); //Se coloca la señal CS0 en ‘1’ lógico: comunicación inactiva 
  digitalWrite(HSPI_CS1,HIGH); //Se coloca la señal CS1 en ‘1’ lógico: comunicación inactiva 
  digitalWrite(HSPI_CS2,HIGH); //Se coloca la señal CS2 en ‘1’ lógico: comunicación inactiva 

//Configuración de la comunicación SPI 
  SPI.begin();  //Se inicializa la comunicación, definiendo SCK y MOSI como salidas y colocándolas a nivel bajo (inactivo) 

  SPI.setBitOrder(MSBFIRST);  // Se enviará primero el bit de mayor significado

  SPI.setDataMode(SPI_MODE0); //Se selecciona el estandar de comunicación SPI 
//conocido como Modo 0, porque así lo precisa el esclavo de este ejemplo

  SPI.setClockDivider(SPI_CLOCK_DIV32); // Se divide el reloj de Arduino por 32 
//para determinar la frecuencia de la señal de reloj del bus (16MHz/32 = 500 kHz) 

}

 