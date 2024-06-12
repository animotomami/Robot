#include <Arduino.h>
#include "MyTrayec.h"
#include "MyControlFn.h"
#include <iostream>
#include <cmath> 

// Llamada a variables globales externas 

extern float _pos_actual_xyz[3], ang_bhcm[4];
extern uint32_t _t_wait_lin, _cont_lin, _cont_pasos;
extern uint32_t _cont_interr_pasos; 
extern float _Pos_ref[3]; //Posición de referencia en grados
extern uint32_t _linea;

// Funciones

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
    P3[0]=z+L_PEN;
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