#define ANG_MAX 90 // Ángulo máximo que puede girar el servo
#define ANG_MIN 0 // Ángulo mínimo que puede girar el servo

#define SERVO_PIN 17 // Se define el pin del servo

#include <ESP32Servo.h>

//Se crea el objeto
extern Servo servo_pinza;

void pinza_init(int servo_pin);

void abrir_pinza();

void cerrar_pinza();