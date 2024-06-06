
#include "PinzaFn.h"
//Se crea el objeto
Servo servo_pinza;
// inicialización de la pinza
void pinza_init(int servo_pin)
{
  servo_pinza.attach(servo_pin);
}

void abrir_pinza()
{
  servo_pinza.write(ANG_MAX);
}

void cerrar_pinza()
{
  servo_pinza.write(ANG_MIN);
}