#include <TimerOne.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>
#include <Stepper.h>

char serialData;
unsigned long auxtimer = 0;

int Inicio_Ciclo = 0;
unsigned long inicioHigh = 0;

float Velo_Inspiracion = 0.0;
float Velo_Expiracion = 0.0;
float PMAX = 0;
float PEEP = 0;
byte byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9, byte10, byte11, byte12, byte13, byte14, byte15, byte16, byte17, byte18;
unsigned int aux1, aux2, aux3, aux4;
int Cantidad_bytes_esperados = 18;
int Vtidal = 0;

int A = 2;                   //variable A del econder a pin digital 2 (DT en modulo)
int B = 4;                   //variable B del encoder a pin digital 4 (CLK en modulo)
int Anterior = 0;            // almacena valor anterior de la variable Posicion
volatile int Posicion = 100; // variable Posicion con valor inicial de 0 y definida como global al ser usada en loop e ISR (encoder)

//volatile unsigned long Motor = 0;

int Variable_Aux = 0;
unsigned long tiempoTranscurrido = 0;

long initial_homing = -1; // Used to Home Stepper at startup

#define Sensor_Home 10 // Se define el pin 9 como la conexion del sensor de Home (optoacoplador)
int Sensor_Home_VALUE = 0;
//Se definen las conexiones con arduino Pin 2 dirección, Pin 3 Pasos. Con el driver TB6600 se utiliza la interface tipo 1
#define dirPin 8
#define stepPin 9
#define motorInterfaceType 1
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
#define Led_Home 12
#define Led_Marcha 11
int Led_Home_Value = 0;
int posicion = 0;

int Inicio;

int avance = 0;

volatile unsigned long ultimaInterrupcion = 0;     // variable static con ultimo valor de tiempo de interrupcion

//***********************************************************************************************************************+
// Seteos de valores de la relacion y configuracion del sistema. Al variarlos aqui variaran uniformamente en la logica
//***********************************************************************************************************************

float Angulo_Brazos = 0;
float Relacion_Transmision = 5;
float Cantidad_Pulsos_Apriete = 900.0;
float Cantidad_Pulsos_Motor = 3200.0;
float Pasos_Avance = 0.0;
float Velo_Motor_Insp = 0.0;
float Velo_Motor_Exp = 0.0;
float Angulo_Pulso = 0.0;

void setup()
{
  Serial.begin(9600); // Se inicializa puerto serie
  //pinMode(Pulsador_Marcha, INPUT);                            //Se define como entrada
  //pinMode(Pulsador_Parada, INPUT);                            //Se define como entrada
  pinMode(A, INPUT);           // A donde se encuentra el encoder como entrada
  pinMode(B, INPUT);           // B donde se encuentra el encoder como entrada
  pinMode(Sensor_Home, INPUT); //Se define como entrada digital
  delay(5);
  pinMode(Led_Home, OUTPUT);
  pinMode(Led_Marcha, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(A), encoder, LOW); // interrupcion sobre pin A con funcion ISR encoder y modo LOW
                                                           // Serial.println("Listo");                                    // imprime en monitor serial Listo
  Wire.begin();                                            //Se inicializa la comunicación I2C

  // attachInterrupt( 1, Pulsador_Stop, RISING);

  Timer1.initialize(250000); // Dispara cada 250 ms
  //        Timer1.attachInterrupt(Motor); // Activa la interrupcion y la asocia a motor

  //************************************************************************************************//
  // Inicializacion de la posicion de los brazos. Al encender se irán a posición Home
  //************************************************************************************************//

  while (!digitalRead(Sensor_Home))
  { // Muevo el motor en CCW hasta que se active el optoacoplador
    digitalWrite(Led_Home, HIGH);
    digitalWrite(dirPin, LOW);

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(400);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(400);
    // avance++;

    //  stepper.moveTo(initial_homing);  // Set the position to move to
    //  initial_homing--;  // Decrease by 1 for next move if needed
    //  stepper.run();  // Start moving the stepper
    //  delay(5);
  }

  //  stepper.setCurrentPosition(0);  // Set the current position as zero for now
  //   stepper.setMaxSpeed(100);      // Set Max Speed of Stepper (Slower to get better accuracy)
  //  stepper.setAcceleration(100);  // Set Acceleration of Stepper
  //  initial_homing=1;

  while (digitalRead(Sensor_Home))
  { // Make the Stepper move CW until the switch is deactivated
    digitalWrite(Led_Home, HIGH);
    // stepper.moveTo(initial_homing);
    // stepper.run();
    // initial_homing++;
    // delay(5);
    digitalWrite(dirPin, HIGH);

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(400);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(400);
  }

  digitalWrite(Led_Home, LOW);
  stepper.setCurrentPosition(0);
  Serial.println("Homing Completed"); //Lo utilizo para visualizar el dato en el RS232

  digitalWrite(dirPin, HIGH);
  for (int d = 0; d < 1400; d++) //Forward 1600 steps
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(400);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(400);
  }
}

void loop()
{

  Serial.print("Posicion");
  Serial.println(Posicion); // imprime valor de POSICION
  //*****************************************************************************************************************//
  // Lo voy a utilizar para chequear en el serie si el encoder cuenta correctamente
  //*****************************************************************************************************************//

  if (Posicion != Anterior) //si el valor de Posicion es distinto de Anterior
  {
    Anterior = Posicion; // asigna a Anterior el valor actualizado de Posicion
  }

  while (Inicio_Ciclo) // Si inicio de ciclo es 1 el motor comienza su ciclo
  {
    //******************************************************************************************************************
    // Calculos de velocidades en Motor según pulsos de avance de apretado, relacion de transformacion y cantidad de pulsos por vuelta del motor
    //******************************************************************************************************************

    Pasos_Avance = (Vtidal * Cantidad_Pulsos_Apriete) / 100;
    Angulo_Pulso = 360.0 / Cantidad_Pulsos_Motor;
    Angulo_Brazos = ((Pasos_Avance * 360) / (Cantidad_Pulsos_Motor * Relacion_Transmision));
    Velo_Motor_Insp = (Angulo_Pulso / ((Angulo_Brazos * Relacion_Transmision) / Velo_Inspiracion)) * 1000000.0;
    Velo_Motor_Exp = (Angulo_Pulso / ((Angulo_Brazos * Relacion_Transmision) / Velo_Expiracion)) * 1000000.0;

    //Serial.print("Pasos Avance: ");
    //Serial.println(Pasos_Avance);
    //Serial.print("Angulo Brazos: ");
    //Serial.println(Angulo_Brazos);
    //Serial.print("Velocidad inspiracion: ");
    //Serial.println(Velo_Motor_Insp);
    //Serial.print("Velocidad Expiracion: ");
    //Serial.println(Velo_Motor_Exp);

    digitalWrite(Led_Marcha, HIGH);

    digitalWrite(dirPin, HIGH);
    for (int i = 0; i < Pasos_Avance; i++) //Forward 1600 steps
    {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(Velo_Motor_Insp / 2.0);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(Velo_Motor_Insp / 2.0);
    }
    delay(1000);
    digitalWrite(dirPin, LOW);

    for (int i = 0; i < Pasos_Avance; i++) //Backward 1600 steps
    {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(Velo_Motor_Exp / 2.0);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(Velo_Motor_Exp / 2.0);
    }
    //delay(1000);

    // Guardar el tiempo actual para calcular el tiempo que tarda la lectura de los valores seteados
    // y completar la espera de 1000 ms
    unsigned long currentMillis = millis();
    // Leer los valores configurados desde Mega
    getConfiguredValues();
    while (millis() < currentMillis + 1000)
    {
      /* wait */
    }
  }

  digitalWrite(Led_Marcha, LOW);

  // Cargar los valores obtenidos en las variables correspondientes

  //if( millis() - auxtimer > 10){
  //auxtimer = millis();
  //Serial.println((String(stepper.currentPosition()) + String(",") + String(stepper.speed())));
  // }
}

//**********************************************************************************************************************************************//
// Función de Lectura de datos enviados por el Maestro del bus I2C                                                                              //
//**********************************************************************************************************************************************//

void getConfiguredValues()
{
  Wire.requestFrom(2, 17);

  byte values[17];
  int i = 17;
  while (Wire.available())
  {
    values[i] = Wire.read();
    i--;
  }

  // Velocidad de Inspiracion
  aux1 = (values[2] << 8) | values[3];       // Ajusta a parte fracionáia (depois da vírgula)
  Velo_Inspiracion = (float)(aux1 * 0.0001); // Atribui a parte fracionária, depois da vírgula
  aux1 = (values[0] << 8) | values[1];       // Ajusta a parte inteira (antes da vírgula)
  Velo_Inspiracion += aux1;                  // Atribui a parte iteira
                                             //  Serial.println("Velocidad de Inspiracion:");
                                             //  Serial.println(Velo_Inspiracion);

  // Velocidad de Expiracion
  aux2 = (values[6] << 8) | values[7];      // Ajusta a parte fracionáia (depois da vírgula)
  Velo_Expiracion = (float)(aux2 * 0.0001); // Atribui a parte fracionária, depois da vírgula
  aux2 = (values[4] << 8) | values[5];      // Ajusta a parte inteira (antes da vírgula)
  Velo_Expiracion += aux2;                  // Atribui a parte iteira
                                            //    Serial.println("Velocidad de Expiracion:");
                                            //   Serial.println(Velo_Expiracion);

  //Presion Maxima
  aux3 = (values[10] << 8) | values[11]; // Ajusta a parte fracionáia (depois da vírgula)
  PMAX = (float)(aux3 * 0.0001);         // Atribui a parte fracionária, depois da vírgula
  aux3 = (values[8] << 8) | values[9];   // Ajusta a parte inteira (antes da vírgula)
  PMAX += aux3;                          // Atribui a parte iteira
                                         //   Serial.println("Presion Maxima:");
                                         //   Serial.println(PMAX);

  // Presion PEEP
  aux4 = (values[14] << 8) | values[15]; // Ajusta a parte fracionáia (depois da vírgula)
  PEEP = (float)(aux4 * 0.0001);         // Atribui a parte fracionária, depois da vírgula
  aux4 = (values[12] << 8) | values[13]; // Ajusta a parte inteira (antes da vírgula)
  PEEP += aux4;                          // Atribui a parte iteira
                                         //   Serial.println("Presion PEEP:");
                                         //   Serial.println(PEEP);

  //Inicio de Ciclo
  Inicio_Ciclo = values[16];
  //   Serial.println("Inicio_Ciclo:");
  //  Serial.println(Inicio_Ciclo);

  //Porcentaje Volumen Tidal
  Vtidal = values[17];
  //    Serial.println("Volumen Tidal:");
  //    Serial.println(Vtidal);
}
//**********************************************************************************************************************************************//
// Función de Lectura de datos del encoder                                                                            //
//**********************************************************************************************************************************************//

void encoder()
{
  unsigned long tiempoInterrupcion = millis();     // variable almacena valor de func. millis
  if (tiempoInterrupcion - ultimaInterrupcion > 5) // rutina antirebote desestima pulsos menores a 5 mseg.
  {
    if (digitalRead(B) == HIGH) // si B es HIGH, sentido horario
    {
      Posicion++; // incrementa POSICION en 1
    }
    else // si B es LOW, senti anti horario
    {
      Posicion--; // decrementa POSICION en 1
    }
    Posicion = min(100, max(0, Posicion));   // establece limite inferior de 0 y superior de 100 para POSICION
    ultimaInterrupcion = tiempoInterrupcion; // guarda valor actualizado del tiempo de la interrupcion en variable static
  }
}
