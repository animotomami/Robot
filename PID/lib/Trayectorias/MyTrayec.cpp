#include "MyTrayec.h"
#include <iostream>
#include <cmath> 

// Llamada a variables globales externas 

extern float _pos_actual_xyz[3], ang_bhcm[4];

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