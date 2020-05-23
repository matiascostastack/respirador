#include <TimerOne.h>
#include <AccelStepper.h>
#include <Wire.h>

typedef enum TipoDeCiclo
{ 
    INSPIRACION,
    EXPIRACION
};

bool Modo_ON = false;      // Inicializacion en ciclo de REPOSO
TipoDeCiclo CicloActual = INSPIRACION; 

float Velo_Inspiracion = 0.0;
float Velo_Expiracion = 0.0;
float PMAX = 0;
float PEEP = 0;

int Vtidal = 0;

//Enconder
#define A 2                                          //variable A del econder a pin digital 2 (DT en modulo)
#define B 4                                          //variable B del encoder a pin digital 4 (CLK en modulo)
volatile int Posicion = 100;                          // variable Posicion con valor inicial de 0 y definida como global al ser usada en loop e ISR (encoder)
volatile unsigned long encoderUltimaInterrupcion = 0; // variable static con ultimo valor de tiempo de interrupcion

// Sensor Home
#define Sensor_Home 10 // Se define el pin 9 como la conexion del sensor de Home (optoacoplador)

//Se definen las conexiones con arduino Pin 2 dirección, Pin 3 Pasos. Con el driver TB6600 se utiliza la interface tipo 1
#define dirPin 8
#define stepPin 9
#define motorInterfaceType 1
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// Indicadores LED
#define Led_Home 12
#define Led_Marcha 11

// Seteos de valores de la relacion y configuracion del sistema. Al variarlos aqui variaran uniformamente en la logica
float Angulo_Brazos = 0;
float Relacion_Transmision = 5;
float Cantidad_Pulsos_Apriete = 900.0;
float Cantidad_Pulsos_Motor = 3200.0;
float Pasos_Avance = 0.0;
float Velo_Motor_Insp = 0.0;
float Velo_Motor_Exp = 0.0;
float Angulo_Pulso = 0.0;
int Pasos_Actuales = 0;

double Vout = 0.0;
double Vs = 5.0;
double PcmH2o = 0.0;
double Vout2 = 0.0;
double PcmH2o_Grafica = 0.0;

double Presion_PIP = 0.0;
double Presion_Plateau = 0.0;
double Presion_PEEP = 0.0;

// Flag de Interrupciones
bool Presion_Grafica_Interrupcion_Flag = false;

//*********************************************************************************************************//
// SETUP
//*********************************************************************************************************//
void setup()
{
    Serial.begin(9600); // Se inicializa puerto serie

    pinMode(A, INPUT);           // A donde se encuentra el encoder como entrada
    pinMode(B, INPUT);           // B donde se encuentra el encoder como entrada
    pinMode(Sensor_Home, INPUT); //Se define como entrada digital

    delay(5);
    pinMode(Led_Home, OUTPUT);
    pinMode(Led_Marcha, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(A), encoder, FALLING); // interrupcion sobre pin A con funcion ISR encoder y modo LOW
                                                                 //attachInterrupt(digitalPinToInterrupt(B), encoder, LOW);    // interrupcion sobre pin A con funcion ISR encoder y modo LOW
                                                                 // Serial.println("Listo");                                    // imprime en monitor serial Listo
    Wire.begin(4);                                               // Inicia como esclavo con dirección 4 en la comunicación I2C
    Wire.onReceive(receiveEvent);                                //Interrupción habilitada cuando el maestro envía bytes en la comunicación I2C

    //*********************************************************************************************************//
    // Tomamos valores de presion del sistema con una interrupcion para graficar dicha presion
    //*********************************************************************************************************//

    Timer1.initialize(250000); // Dispara cada 100 ms
    Timer1.attachInterrupt(Presion_Grafica);

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

//*********************************************************************************************************//
// LOOP
//*********************************************************************************************************//

void loop()
{
    if (Modo_ON)
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

        // Atender flags de Interrupciones
        if (Presion_Grafica_Interrupcion_Flag)
        {
            Presion_Grafica();
            Presion_Grafica_Interrupcion_Flag = false;
        }

        // Manejar los ciclos
        if (CicloActual == INSPIRACION)
        {
            digitalWrite(dirPin, HIGH);
            if (Pasos_Actuales < Pasos_Avance)
            {
                digitalWrite(stepPin, HIGH);
                delayMicroseconds(Velo_Motor_Insp / 2.0);
                digitalWrite(stepPin, LOW);
                delayMicroseconds(Velo_Motor_Insp / 2.0);
            }
            else
            {
                Presion_PIP = Presion();
                //  Serial.print("Valor Presion PIP:");
                //  Serial.println(Presion_PIP);
                delay(100);
                Presion_Plateau = Presion();
                // Serial.print("Valor Presion Plateau:");
                // Serial.println(Presion_Plateau);

                // Cambiar el estado del ciclo
                CicloActual = EXPIRACION;
                Pasos_Avance = 0;
                delay(1000);
            }
        }
        else if (CicloActual == EXPIRACION)
        {
            digitalWrite(dirPin, LOW);
            if (Pasos_Actuales < Pasos_Avance)
            {
                digitalWrite(stepPin, HIGH);
                delayMicroseconds(Velo_Motor_Exp / 2.0);
                digitalWrite(stepPin, LOW);
                delayMicroseconds(Velo_Motor_Exp / 2.0);
            }
            else
            {
                Presion_PEEP = Presion();
                //  Serial.print("Valor Presion PEEP:");
                //  Serial.println(Presion_PEEP);

                // Cambiar el estado del ciclo
                CicloActual = INSPIRACION;
                Pasos_Avance = 0;
                delay(1000);
            }
        }
    }
    else
    {
        digitalWrite(Led_Marcha, LOW);
    }
}

//**********************************************************************************************************************************************//
// Función de Lectura de datos enviados por el Maestro del bus I2C                                                                              //
//**********************************************************************************************************************************************//
void receiveEvent(int cantBytes)
{ // Está código é executado quando "quantidade_bytes_esperados" foi recebido via I2C
    byte byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9, byte10, byte11, byte12, byte13, byte14, byte15, byte16, byte17, byte18;

    byte1 = Wire.read(); // Lê os 4 bytes enviados pelo mestre
    byte2 = Wire.read();
    byte3 = Wire.read();
    byte4 = Wire.read();
    byte5 = Wire.read(); // Lê os 4 bytes enviados pelo mestre
    byte6 = Wire.read();
    byte7 = Wire.read();
    byte8 = Wire.read();
    byte9 = Wire.read(); // Lê os 4 bytes enviados pelo mestre
    byte10 = Wire.read();
    byte11 = Wire.read();
    byte12 = Wire.read();
    byte13 = Wire.read(); // Lê os 4 bytes enviados pelo mestre
    byte14 = Wire.read();
    byte15 = Wire.read();
    byte16 = Wire.read();
    byte17 = Wire.read();
    byte18 = Wire.read();

    byte aux;
    // Velocidad de Inspiracion
    aux = (byte3 << 8) | byte4;               // Ajusta a parte fracionáia (depois da vírgula)
    Velo_Inspiracion = (float)(aux * 0.0001); // Atribui a parte fracionária, depois da vírgula
    aux = (byte1 << 8) | byte2;               // Ajusta a parte inteira (antes da vírgula)
    Velo_Inspiracion += aux;                  // Atribui a parte iteira
                                              //  Serial.println("Velocidad de Inspiracion:");
                                              //  Serial.println(Velo_Inspiracion);

    // Velocidad de Expiracion
    aux = (byte7 << 8) | byte8;              // Ajusta a parte fracionáia (depois da vírgula)
    Velo_Expiracion = (float)(aux * 0.0001); // Atribui a parte fracionária, depois da vírgula
    aux = (byte5 << 8) | byte6;              // Ajusta a parte inteira (antes da vírgula)
    Velo_Expiracion += aux;                  // Atribui a parte iteira
                                             //    Serial.println("Velocidad de Expiracion:");
                                             //   Serial.println(Velo_Expiracion);

    //Presion Maxima
    aux = (byte11 << 8) | byte12; // Ajusta a parte fracionáia (depois da vírgula)
    PMAX = (float)(aux * 0.0001); // Atribui a parte fracionária, depois da vírgula
    aux = (byte9 << 8) | byte10;  // Ajusta a parte inteira (antes da vírgula)
    PMAX += aux;                 // Atribui a parte iteira
                                  //   Serial.println("Presion Maxima:");
                                  //   Serial.println(PMAX);

    // Presion PEEP
    aux = (byte15 << 8) | byte16; // Ajusta a parte fracionáia (depois da vírgula)
    PEEP = (float)(aux * 0.0001); // Atribui a parte fracionária, depois da vírgula
    aux = (byte13 << 8) | byte14; // Ajusta a parte inteira (antes da vírgula)
    PEEP += aux;                 // Atribui a parte iteira
                                  //   Serial.println("Presion PEEP:");
                                  //   Serial.println(PEEP);

    //Inicio de Ciclo
    Modo_ON = (bool)byte17;

    //Porcentaje Volumen Tidal
    Vtidal = byte18;
}

//**********************************************************************************************************************************************//
// Función de Lectura de datos del encoder                                                                            //
//**********************************************************************************************************************************************//
void encoder()
{
    unsigned long tiempoInterrupcion = millis();     // variable almacena valor de func. millis
    if (tiempoInterrupcion - encoderUltimaInterrupcion > 5) // rutina antirebote desestima pulsos menores a 5 mseg.
    {
        if (digitalRead(B) == HIGH) // si B es HIGH, sentido horario
        {
            Posicion++; // incrementa POSICION en 1
        }
        else // si B es LOW, senti anti horario
        {
            Posicion--; // decrementa POSICION en 1
        }
        Posicion = min(100, max(0, Posicion));          // establece limite inferior de 0 y superior de 100 para POSICION
        encoderUltimaInterrupcion = tiempoInterrupcion; // guarda valor actualizado del tiempo de la interrupcion en variable static
        Serial.print("Posicion");
        Serial.println(Posicion); // imprime valor de POSICION
    }
}

//**********************************************************************************************************************************************//
// Funcion para la obtencion de los valores de presion en los puntos de la curva necesarios.
//**********************************************************************************************************************************************//
double Presion()
{
    double Aux = 0;
    for (int p = 0; p < 10; p++)
    {
        Aux = Aux + (float(analogRead(A0) * 5.0 / 1023.0)); //Leo la entrada analogica que tiene conectada el sensor de presión
        delay(5);
    }
    Vout = Aux / 10.0;
    PcmH2o = ((Vout - 0.04 * Vs + 0.01) / (0.09 * Vs)) * 10.1972; //Se multiplica por el equivalente para la conversion a cmH20
    return PcmH2o;
}

//**********************************************************************************************************************************************//
// Funcion para la obtencion de una grafica de presion. Se desactiva luego de las pruebas
//**********************************************************************************************************************************************//
void PresionGraficaInterrupcion()
{
    Presion_Grafica_Interrupcion_Flag = true;
}

void Presion_Grafica()
{
    double Aux2 = 0;
    for (int g = 0; g < 10; g++)
    {
        Aux2 = Aux2 + (float(analogRead(A0) * 5.0 / 1023.0)); //Leo la entrada analogica que tiene conectada el sensor de presión
        delay(5);
    }
    Vout2 = Aux2 / 10.0;
    PcmH2o_Grafica = ((Vout2 - 0.04 * Vs + 0.01) / (0.09 * Vs)) * 10.1972; //Se multiplica por el equivalente para la conversion a cmH20
    Serial.print("Presion del sistema:");
    Serial.println(PcmH2o);
}
