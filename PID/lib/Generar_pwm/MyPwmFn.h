
/// PWM PINS
#define GPIO_PWM_0A        6
#define GPIO_PWM_0B        7
#define GPIO_PWM_1A        4
#define GPIO_PWM_1B        5
#define GPIO_PWM_2A        15
#define GPIO_PWM_2B        16

/// Fn inicialización de generadores para pwm
void pwm_init();

/// Fn para control de motores según el duty introducido.
void set_comp_tres_motores(float set_duty[]);

void retroceso_motor(int motor);
void avance_motor(int motor);
void parada_motor(int motor, float valor_comp);