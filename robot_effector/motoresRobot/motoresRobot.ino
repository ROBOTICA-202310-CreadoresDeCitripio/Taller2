#include <string.h>

// Velocidades
float velLineal;
float velAngular;

// Motor A
int enA = 2;
int in1 = 50;
int in2 = 48;
 
// Motor B
int enB = 3;
int in3 = 24;
int in4 = 26;

void setup()
{
  Serial.begin(9600);
  // Terminales de salida en el Arduino
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop()
{
  if (Serial.available()) {
    delay(10);
    // Read input from serial
    String data = Serial.readStringUntil('\n');
    int separator = data.indexOf(',');
    String value1String = data.substring(0, separator);
    String value2String = data.substring(separator + 1);
    // Extract linear and angular velocities from input string
    float velLineal = value1String.toFloat();
    float velAngular = value2String.toFloat();
  
  
    if (velLineal > 0.0 && velAngular == 0.0) 
    {
      adelante(velLineal);
    }
    else if (velLineal < 0.0 && velAngular == 0.0) 
    {
      atras(velLineal);
    }
    else if (velLineal == 0.0 && velAngular > 0.0)
    {
      izquierda(velAngular);
    }
    else if (velLineal == 0.0 && velAngular < 0.0)
    {
      derecha(velAngular);
    }
    else {
      quieto();
    }
  }
}

void adelante(float velocidadLineal)
{
  // Calcular la velocidad respecto al máximo (35cm/s)
  int activacion = int(abs(255*(velocidadLineal/35)));

  // Velocidad del Motor A y B
  analogWrite(enA, activacion);
  analogWrite(enB, activacion);

  // Motor A gira adelante
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Motor B gira adelante
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void atras(float velocidadLineal)
{
  // Calcular la velocidad respecto al máximo (35cm/s)
  int activacion = int(abs(255*(velocidadLineal/35)));

  // Velocidad del Motor A y B
  analogWrite(enA, activacion);
  analogWrite(enB, activacion);

  // Motor A gira adelante
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  // Motor B gira adelante
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void izquierda(float velocidadAngular)
{
  // Calcular la velocidad respecto al máximo (50deg/s)
  int activacion = int(abs(255*(velocidadAngular/50)));

  // Velocidad del Motor A y B
  analogWrite(enA, activacion);
  analogWrite(enB, 0);

  // Motor A gira adelante
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Motor B queda estático
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void derecha(float velocidadAngular)
{
  // Calcular la velocidad respecto al máximo (50deg/s)
  int activacion = int(abs(255*(velocidadAngular/50)));

  // Velocidad del Motor A y B
  analogWrite(enA, 0);
  analogWrite(enB, activacion);

  // Motor A queda estático
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  // Motor B gira adelante
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void quieto()
{
  // Velocidad del Motor A y B
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  // Motor A gira adelant0e
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  // Motor B queda estático
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
