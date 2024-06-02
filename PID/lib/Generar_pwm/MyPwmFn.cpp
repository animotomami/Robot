#include <Arduino.h> 
#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_sync.h"
#include "driver/mcpwm_cap.h"
#include "driver/mcpwm_types.h"
#include "driver/gpio.h"
#include "esp_private/esp_clk.h" //Para esp_clk_apb_freq()

#include "MyPwmFn.h"

uint32_t _ticks_comp[3];

//Se crea el handle (identificador) del timer. Se crea un solo timer para los tres motores.
mcpwm_timer_handle_t timer0;
//Se crea la estructura para la configuración del timer.
mcpwm_timer_config_t conf_timer;

//*************OPERADOR*************
//Se crea el handle (identificador) del operador. Se crea un solo operador para los tres motores.
mcpwm_oper_handle_t operador[3];
//Se crea la estructura del operador.
mcpwm_operator_config_t conf_operador;

//*************COMPARADORES*************
//Comparador 0
//Se crea el handle (identificador) del comparador.
mcpwm_cmpr_handle_t comparador[3]; //Se crean 3 comparadores. Uno por motor
//Se crea la estructura del comparador.
mcpwm_comparator_config_t conf_comparador;

//*************GENERADORES*************
mcpwm_gen_handle_t generadorA[3], generadorB[3];; //Se crean 6 generadores. Dos por motor.
mcpwm_generator_config_t conf_gen0A,conf_gen0B,conf_gen1A,conf_gen1B,conf_gen2A,conf_gen2B;

//Declaración de función.
static void set_comp_tres_motores(float duty[]);

void pwm_init()
{
    //Creación de la estructura del timer
    conf_timer.group_id=0;//Selección del módulo MCPWM0
    conf_timer.clk_src=MCPWM_TIMER_CLK_SRC_DEFAULT; //Frecuencia default de 160 MHZ
    conf_timer.resolution_hz=10000000; //10 MHz -> 1 tick= 0.1 us
    conf_timer.count_mode=MCPWM_TIMER_COUNT_MODE_UP;
    conf_timer.period_ticks=500; // 500*0.1 us = 20 kHz
    //Fin de creación de estructura del timer.

    //Se crea el timer con la estructura definida anteriormente.
    mcpwm_new_timer(&conf_timer,&timer0);

    //***************setup OPERADOR***************
    //Creación de la estructura del OPERADOR
    conf_operador.group_id=0; //Debe compartir el mismo módulo que el timer asociado.
    conf_operador.intr_priority=0; //Prioridad por defecto.

    //Se crea el operador 0 con la estructura previamente definida.

    //Se crea el operador i con la estructura previamente definida y se vincula el operador i al timer 0.
    for(int i=0;i<3;i++)
    {
    mcpwm_new_operator(&conf_operador,&operador[i]);
    mcpwm_operator_connect_timer(operador[i],timer0);
    }

    //***************setup COMPARADOR***************

    //Configuración de los tres comparadores
    conf_comparador.flags.update_cmp_on_tez = true;

    //Se crean los 3 comparadores

    for(int i=0;i<3;i++)
    {
    mcpwm_new_comparator(*(operador+i), &conf_comparador, comparador+i);
    mcpwm_comparator_set_compare_value(comparador[i], 0); //Valor de comparación. Se inicializa a cero.
    }
    //para obtener 3 duties distintos.

    //***************setup GENERADORES***************
    //Se crea la configuración de cada generador. Se asocia un GPIO a cada uno.
    conf_gen0A.gen_gpio_num = GPIO_PWM_0A;
    conf_gen0B.gen_gpio_num = GPIO_PWM_0B;

    conf_gen1A.gen_gpio_num = GPIO_PWM_1A;
    conf_gen1B.gen_gpio_num = GPIO_PWM_1B;

    conf_gen2A.gen_gpio_num = GPIO_PWM_2A;
    conf_gen2B.gen_gpio_num = GPIO_PWM_2B;

    //Se crean los generadores y se asocian al operador

    mcpwm_new_generator(operador[0], &conf_gen0A, &generadorA[0]);
    mcpwm_new_generator(operador[0], &conf_gen0B, &generadorB[0]);

    mcpwm_new_generator(operador[1], &conf_gen1A, &generadorA[1]);
    mcpwm_new_generator(operador[1], &conf_gen1B, &generadorB[1]);

    mcpwm_new_generator(operador[2], &conf_gen2A, &generadorA[2]);
    mcpwm_new_generator(operador[2], &conf_gen2B, &generadorB[2]);

    //Se tratan los eventos que forman el PWM del generador.

    for(int i=0;i<3;i++)
    {
    //GENERADOR iA
    mcpwm_generator_set_action_on_timer_event(generadorA[i], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH ) ) ;
    mcpwm_generator_set_action_on_compare_event(generadorA[i], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparador[i], MCPWM_GEN_ACTION_LOW ) );

    //GENERADOR iB
    mcpwm_generator_set_action_on_timer_event(generadorB[i], MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH ) ) ;
    mcpwm_generator_set_action_on_compare_event(generadorB[i], MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparador[i], MCPWM_GEN_ACTION_LOW ) );
    }

    //Finalmente se habilita el Timer
    //Se habilita el timer.
    mcpwm_timer_enable(timer0);

    //Se arranca el timer.
    mcpwm_timer_start_stop(timer0,MCPWM_TIMER_START_NO_STOP); //Arranca el timer y no para hasta recibir la orden de parada.
    
}

//Función para control de motores según el duty introducido.
void set_comp_tres_motores(float set_duty[])
{
 for(int i=0;i<3;i++)
  {
    if(set_duty[i]<0)
      {
        _ticks_comp[i]=uint32_t((-1)*(set_duty[i]*500)); //Se adapta a valores del comparador
        Serial.println(set_duty[i]);
        Serial.println(_ticks_comp[i]);
       
        mcpwm_comparator_set_compare_value(comparador[i], _ticks_comp[i]); //Valor de comparación.

        //RETROCESO DEL MOTOR
        mcpwm_generator_set_force_level(generadorA[i], 0,true);
        mcpwm_generator_set_force_level(generadorB[i], -1,true);
        
      }
    else if(set_duty[i]>0)
      {
        _ticks_comp[i]=uint32_t(set_duty[i]*500); //Se adapta a valores del comparador
        Serial.println(set_duty[i]);
        Serial.println(_ticks_comp[i]);

        mcpwm_comparator_set_compare_value(comparador[i], _ticks_comp[i]); //Valor de comparación.

        //AVANCE DEL MOTOR
        mcpwm_generator_set_force_level(generadorA[i], -1,true); //Con -1 se deshabilita el forzado del nivel. El bool true lo mantiene hasta que se indique el -1.
        mcpwm_generator_set_force_level(generadorB[i], 0,true); //Fuerza el nivel de la salida pwm a 0. Lo apaga.
      
      }
    else
      {
        _ticks_comp[i]=0; //Se pone el duty a cero.
        Serial.println(set_duty[i]);
        Serial.println(_ticks_comp[i]);

        
        mcpwm_comparator_set_compare_value(comparador[i], _ticks_comp[i]);  //Valor de comparación.

        //PARADA DEL MOTOR
        mcpwm_generator_set_force_level(generadorA[i], 0,true);
        mcpwm_generator_set_force_level(generadorB[i], 0,true);
       
      }  
  }
}


void retroceso_motor(int motor)
{
    mcpwm_comparator_set_compare_value(comparador[motor], 200); //Valor de comparación.

//RETROCESO DEL MOTOR
    mcpwm_generator_set_force_level(generadorA[motor], 0,true);
    mcpwm_generator_set_force_level(generadorB[motor], -1,true);
}

void avance_motor(int motor)
{
    mcpwm_comparator_set_compare_value(comparador[motor], 200); //Valor de comparación.

//AVANCE DEL MOTOR
    mcpwm_generator_set_force_level(generadorA[motor], -1,true);
    mcpwm_generator_set_force_level(generadorB[motor], 0,true);
}

void parada_motor(int motor, float valor_comp)
{
    mcpwm_comparator_set_compare_value(comparador[motor], 0); //Valor de comparación.

//PARADA DEL MOTOR
    mcpwm_generator_set_force_level(generadorA[motor], 0,true);
    mcpwm_generator_set_force_level(generadorB[motor], 0,true);
}