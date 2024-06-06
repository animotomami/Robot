#include <ESP32Servo.h>
#include "PinzaFn.h"

//Se crea el objeto
Servo servo_pinza;

void abrir_pinza()
{
  servo_pinza.write(ANG_MAX);
}

void cerrar_pinza()
{
  servo_pinza.write(ANG_MIN);
}