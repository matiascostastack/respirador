#include <TimerOne.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Encoder.h>
#include "MegunoLink.h"

typedef enum ModoOperacion
{
    VCV,
    PSV,
    NIL
};

typedef enum TipoDeCiclo
{
    INSPIRACION,
    EXPIRACION,
    INICIO
};

typedef enum TipoDeAlarma
{
    ALARMA_PIP,
    ALARMA_PEEP,
    ALARMA_MECANICA,
    PACIENTE_DESCONECTADO,
    SIN_ALARMA
};

bool Modo_ON = false; // Inicializacion en ciclo de REPOSO
ModoOperacion ModoSeleccionado = VCV;
ModoOperacion ModoActual = VCV;
TipoDeCiclo CicloActual = INSPIRACION;
TipoDeAlarma AlarmaActual = SIN_ALARMA;

float Velo_Inspiracion = 0.0;
float Velo_Expiracion = 0.0;
float PMAX = 0;
float PEEP = 0;

int Vtidal = 0;
int PTrigger = 0;
#define PsvTiempoEspera 15 // Intervalo entre los ciclos espresado en segundos

//Enconder
#define A 2                                           //variable A del econder a pin digital 2 (DT en modulo)
#define B 4                                           //variable B del encoder a pin digital 4 (CLK en modulo)
volatile int Posicion = 100;                          // variable Posicion con valor inicial de 0 y definida como global al ser usada en loop e ISR (encoder)
volatile unsigned long encoderUltimaInterrupcion = 0; // variable static con ultimo valor de tiempo de interrupcion
Encoder myEncoder(2, 4);

// Sensor Home
#define Sensor_Home 10 // Se define el pin 10 como la conexion del sensor de Home (optoacoplador)

//Se definen las conexiones con arduino Pin 2 dirección, Pin 3 Pasos. Con el driver TB6600 se utiliza la interface tipo 1
#define dirPin 8
#define stepPin 9
#define motorInterfaceType 1
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);

// Indicadores LED
#define Led_Home 12
#define Led_Marcha 11
#define Led_Falla 12

#define Pulsador_Reset 6
int Pulsador_Reset_VALUE = 0;

// Alarmas
#define Pmin_Paciente_Desconectado 3        // Setea la presion minima para disparar alarma de paciente desconectado
#define Buzzer_Pin 5
#define Tono_Alarma 800
#define Tiempo_Alarma 1000
bool Alarma_ON = false;
unsigned long Tiempo_Alarma_Transcurrido = 0;

// Seteos de valores de la relacion y configuracion del sistema. Al variarlos aqui variaran uniformamente en la logica
float Angulo_Brazos = 0;
float Relacion_Transmision = 26.4705882;
float Cantidad_Pulsos_Apriete = 5000.0;
float Cantidad_Pulsos_Motor = 3200.0;
float Pasos_Avance = 0.0;
float Velo_Motor_Insp = 0.0;
float Velo_Motor_Exp = 0.0;
float Angulo_Pulso = 0.0;
int Pasos_Actuales = 0;

double Vout = 0.0;
double Vs = 4.5;
double Vs2 = 4.6;
double PcmH2o = 0.0;
double Vout2 = 0.0;
double Vout3 = 0.0;
double P1 = 0.0;
double P1_1 = 0.0;
double P2 = 0.0;

double Presion_PIP = 0.0;
double Presion_PIP_Anterior = 0.0;
double Presion_Plateau = 0.0;
double Presion_PEEP = 0.0;

// Intervalo para Graficos
//unsigned long Presion_Grafica_Intervalo = 50; // Intervalo en ms para generar el grafico
//unsigned long Volumen_Grafica_Intervalo = 1500; // Intervalo en ms para generar el grafico

// Auxiliares Delay
unsigned long Aux_Tiempo_Ciclo = 0;
unsigned long Aux_Presion_Grafica = 0;
unsigned long Aux_Volumen_Grafica = 0;

TimePlot MyPlot;

//*********************************************************************************************************//
// SETUP
//*********************************************************************************************************//
void setup()
{
    Serial.begin(9600); // Se inicializa puerto serie

    pinMode(Buzzer_Pin, OUTPUT); // Buzzer

    pinMode(A, INPUT);           // A donde se encuentra el encoder como entrada
    pinMode(B, INPUT);           // B donde se encuentra el encoder como entrada
    pinMode(Sensor_Home, INPUT); //Se define como entrada digital
    pinMode(Pulsador_Reset, INPUT);

    delay(5);
    pinMode(Led_Home, OUTPUT);
    pinMode(Led_Marcha, OUTPUT);
    pinMode(Led_Falla, OUTPUT);

    // attachInterrupt(digitalPinToInterrupt(A), encoder, FALLING); // interrupcion sobre pin A con funcion ISR encoder y modo LOW
    //attachInterrupt(digitalPinToInterrupt(B), encoder, LOW);    // interrupcion sobre pin A con funcion ISR encoder y modo LOW
    // Serial.println("Listo");                                    // imprime en monitor serial Listo
    Wire.begin(4);                // Inicia como esclavo con dirección 4 en la comunicación I2C
    Wire.onReceive(receiveEvent); //Interrupción habilitada cuando el maestro envía bytes en la comunicación I2C
    Wire.onRequest(sendEvent);

    //*********************************************************************************************************//
    // Tomamos valores de presion del sistema con una interrupcion para graficar dicha presion
    //*********************************************************************************************************//

    //Timer1.initialize(250000); // Dispara cada 100 ms
    //Timer1.attachInterrupt(Presion_Grafica);
    // Timer1.initialize(250000); // Dispara cada 100 ms
    // Timer1.attachInterrupt(Volumen_Grafica);

    //************************************************************************************************//
    // Inicializacion de la posicion de los brazos. Al encender se irán a posición Home
    //************************************************************************************************//

    while (digitalRead(Sensor_Home))
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

    while (!digitalRead(Sensor_Home))
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
    myEncoder.write(0);                 // set the encoder position to 0
    Serial.println("Homing Completed"); //Lo utilizo para visualizar el dato en el RS232

    digitalWrite(dirPin, HIGH);
    for (int d = 0; d < 2000; d++) //Forward 1600 steps
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

    Posicion = myEncoder.read(); // read position

    // Serial.print("Posicion:"); // print the position
    //Serial.println(Posicion);

    // Alarmas
    manejoAlarmas();

    if (Modo_ON)
    {
        checkDelay();

        digitalWrite(Led_Marcha, HIGH);

        if (Pasos_Avance == 0)
        {
            CalcularParametros();
        }

        if ((ModoActual != ModoSeleccionado) && (Pasos_Actuales == 0))
        {
          ModoActual = ModoSeleccionado;
          CalcularParametros();
          if(ModoActual==PSV)
          {
           delayMillis(PsvTiempoEspera * 1000);
          }
        }
        
        // Alarmas
        manejoAlarmas();

        // Graficos
        manejoGraficos();

         if (ModoActual == PSV && estaEnDelay() && Pasos_Actuales == 0 && CicloActual == INSPIRACION)
         { // Modo Control Presion de Soporte PSV
             if (ChequeoSoporteVolumenPresion())
             {
              //   CalcularParametros();
                 cancelarDelay(); // corta el delay de expera entre los ciclo en el modo PSV
             }
             if (ModoSeleccionado != ModoActual && estaEnDelay()) // Cambia de modo de funcionamiento mientras esta en el intervalo de espera
            {
                ModoActual = ModoSeleccionado;
                CalcularParametros();
                cancelarDelay();
            }
         }
   
        // Ciclo de funcionamiento
        manejoCiclo();
    }
    else
    {

        if (AlarmaActual != ALARMA_PIP)
        {
            SilenciarAlarma();

            if (digitalRead(Led_Marcha) == HIGH)
                IrAlInicio();

            ModoActual = NIL;
            digitalWrite(Led_Marcha, LOW);
        }
    }
}

void manejoCiclo()
{
    if (estaEnDelay())
        return;

    if (Pasos_Avance == 0)
        return;

    // Manejar los ciclos
    switch (CicloActual)
    {
    case INSPIRACION:
        digitalWrite(dirPin, HIGH);
        if (Pasos_Actuales < Pasos_Avance)
        {
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(Velo_Motor_Insp / 2.0);
            digitalWrite(stepPin, LOW);
            delayMicroseconds(Velo_Motor_Insp / 2.0);
            Pasos_Actuales++;
        }
        else
        {
            // Chequear presión PIP al final del ciclo de inspiración
            if (ChequeoPIP())
            {
                digitalWrite(Led_Marcha, LOW);
                return;
            }

            // Cambiar el estado del ciclo
            CicloActual = EXPIRACION;
            Pasos_Actuales = 0;

            // Imprimer el tiempo que duro el ciclo de inspiracion
            Serial.print("Tiempo Expiracion: ");
            Serial.println((millis() - Aux_Tiempo_Ciclo) / 1000.0);
            Aux_Tiempo_Ciclo = millis();

            delayMillis(1000);
        }
        break;
    case EXPIRACION:
        digitalWrite(dirPin, LOW);
        if (Pasos_Actuales < Pasos_Avance)
        {
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(Velo_Motor_Exp / 2.0);
            digitalWrite(stepPin, LOW);
            delayMicroseconds(Velo_Motor_Exp / 2.0);
            Pasos_Actuales++;
        }
        else
        {
            // Chequear presión PEEP al final del ciclo de inspiración
            ChequeoPEEP();

            // Cambiar el estado del ciclo
            CicloActual = INSPIRACION;
            Pasos_Actuales = 0;

            // Imprimer el tiempo que duro el ciclo de expiracion
            Serial.print("Tiempo Expiracion: ");
            Serial.println((millis() - Aux_Tiempo_Ciclo) / 1000.0);
            Aux_Tiempo_Ciclo = millis();
            CalcularParametros();
            ModoActual = ModoSeleccionado; // solo se cambia el modo de funcionamiento al final del ciclo
            if (ModoActual == PSV)
            {
                delayMillis(PsvTiempoEspera * 1000);
                return;
            }

            // Calcula los parametro para el siguiente ciclo
            
            delayMillis(1000);
        }
        break;
    default:
        break;
    }
}

void manejoAlarmas()
{
    switch (AlarmaActual)
    {
    case ALARMA_PIP: // No debería entrar en esta opcion pero por las dudas se contempla
        AlarmaContinua();
        Modo_ON = false;
        break;
    case ALARMA_PEEP:
        AlarmaIntermitente();
        break;
    case ALARMA_MECANICA:
        AlarmaContinua();
        break;
    case PACIENTE_DESCONECTADO:
        break;
    case SIN_ALARMA:
        SilenciarAlarma();
        break;
    default:
        break;
    }
}

void manejoGraficos()
{
    // Chequear Intervalos para graficos
    //    if ((unsigned long)millis() - Aux_Presion_Grafica > Presion_Grafica_Intervalo)
    //    {
    //       if (Presion_Grafica())
    //            Aux_Presion_Grafica = millis();
    //    }

    //    if ((unsigned long)millis() - Aux_Volumen_Grafica > Volumen_Grafica_Intervalo)
    //    {
    //      if (Volumen_Grafica())
    //           Aux_Volumen_Grafica = millis();
    //   }
}

//**********************************************************************************************************************************************//
// Funcion de delay propia para no bloquear el loop principal y poder realizar operaciones secundarias                                                                              //
//**********************************************************************************************************************************************//
volatile bool Aux_En_Delay = false;
unsigned long Aux_Delay_Time = 0;
unsigned long Aux_Delay_Begin = 0;

void cancelarDelay()
{
    Aux_En_Delay = false;
    Aux_Delay_Time = 0;
    Aux_Delay_Begin = 0;
}

bool estaEnDelay()
{
    return Aux_En_Delay;
}

void checkDelay()
{
    if (!Aux_En_Delay)
        return;

    if ((unsigned long)millis() - Aux_Delay_Begin >= Aux_Delay_Time)
    {
    cancelarDelay();
    }
}

void delayMillis(unsigned long time)
{
    if (Aux_En_Delay)
        return;

    Aux_Delay_Begin = millis();
    Aux_Delay_Time = time;
    Aux_En_Delay = true;
}

//**********************************************************************************************************************************************//
// Función de Lectura de datos enviados por el Maestro del bus I2C                                                                              //
//**********************************************************************************************************************************************//
void receiveEvent(int cantBytes)
{ // Está código é executado quando "quantidade_bytes_esperados" foi recebido via I2C
    byte byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9, byte10, byte11, byte12, byte13, byte14, byte15, byte16, byte17, byte18, byte19, byte20, byte21;

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
    byte19 = Wire.read();
    byte20 = Wire.read();
    byte21 = Wire.read();

    unsigned int aux;
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
    PMAX += aux;                  // Atribui a parte iteira
                                  //   Serial.println("Presion Maxima:");
                                  //  Serial.println(PMAX);

    // Presion PEEP
    aux = (byte15 << 8) | byte16; // Ajusta a parte fracionáia (depois da vírgula)
    PEEP = (float)(aux * 0.0001); // Atribui a parte fracionária, depois da vírgula
    aux = (byte13 << 8) | byte14; // Ajusta a parte inteira (antes da vírgula)
    PEEP += aux;                  // Atribui a parte iteira
                                  //   Serial.println("Presion PEEP:");
                                  //   Serial.println(PEEP);

    //Inicio de Ciclo
    Modo_ON = (bool)byte17;
    // Serial.print("Modo");
    // Serial.println(Modo_ON);

    //Porcentaje Volumen Tidal
    Vtidal = byte18;

    //Presión Trigger
    PTrigger = byte19 * -1; //Se multiplica´por -1 para obtener el valor de presión de trigger original ya que no se puede transmitir en negativo
                            // Serial.print("Presion Trigger:");
                            // Serial.println(PTrigger);

    // Modo Seleccionado. 0= VCV, 1=PSV
    if (byte20 == 0)
    {
        ModoSeleccionado = VCV;
    }
    else
    {
        ModoSeleccionado = PSV;
      //  delayMillis(PsvTiempoEspera * 1000);
      //  CalcularParametros();
    }

    // Alarmas
    AlarmaActual = byte21;

    // Serial.print("Modo:");
    // Serial.println(Modo_Seleccionado);
}

void sendEvent()
{
    //if (Presion_PIP != Presion_PIP_Anterior )
    // {
    byte byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9, byte10, byte11, byte12, byte13;
    unsigned int auxi;

    float Presion_PIP_Tx = Presion_PIP;
    auxi = (unsigned int)Presion_PIP_Tx; // aux = 46689, Pega somente a parte inteira da variável float (0 - 65536)
    byte2 = auxi;                        // byte2 = 0B01100001, pega apenas os primeros 8 bits
    byte1 = (auxi >> 8);                 // byte1 = 0B10110110, pega os 8 ultimos bits
    // Ajustando o número depois da vírgula
    Presion_PIP_Tx -= auxi;              // Deixa apenas o número depois da vírgula
    Presion_PIP_Tx *= 10000;             // Multiplica por 10k para pegar 4 dígitos após a vírgula
    auxi = (unsigned int)Presion_PIP_Tx; // Pega somente o valor antes da vírgula
    byte4 = auxi;                        // byte2 = 0B00101110, pega apenas os primeros 8 bits
    byte3 = (auxi >> 8);                 // byte1 = 0B00100010, pega os 8 ultimos bits

    float Presion_Plateau_Tx = Presion_Plateau;
    auxi = (unsigned int)Presion_Plateau_Tx; // aux = 46689, Pega somente a parte inteira da variável float (0 - 65536)
    byte6 = auxi;                            // byte2 = 0B01100001, pega apenas os primeros 8 bits
    byte5 = (auxi >> 8);                     // byte1 = 0B10110110, pega os 8 ultimos bits
    // Ajustando o número depois da vírgula
    Presion_Plateau_Tx -= auxi;              // Deixa apenas o número depois da vírgula
    Presion_Plateau_Tx *= 10000;             // Multiplica por 10k para pegar 4 dígitos após a vírgula
    auxi = (unsigned int)Presion_Plateau_Tx; // Pega somente o valor antes da vírgula
    byte8 = auxi;                            // byte2 = 0B00101110, pega apenas os primeros 8 bits
    byte7 = (auxi >> 8);                     // byte1 = 0B00100010, pega os 8 ultimos bits

    float Presion_PEEP_Tx = Presion_PEEP;
    auxi = (unsigned int)Presion_PEEP_Tx; // aux = 46689, Pega somente a parte inteira da variável float (0 - 65536)
    byte10 = auxi;                        // byte2 = 0B01100001, pega apenas os primeros 8 bits
    byte9 = (auxi >> 8);                  // byte1 = 0B10110110, pega os 8 ultimos bits
    // Ajustando o número depois da vírgula
    Presion_PEEP_Tx -= auxi;              // Deixa apenas o número depois da vírgula
    Presion_PEEP_Tx *= 10000;             // Multiplica por 10k para pegar 4 dígitos após a vírgula
    auxi = (unsigned int)Presion_PEEP_Tx; // Pega somente o valor antes da vírgula
    byte12 = auxi;                        // byte2 = 0B00101110, pega apenas os primeros 8 bits
    byte11 = (auxi >> 8);                 // byte1 = 0B00100010, pega os 8 ultimos bits

    // Informar el estado de las alarmas
    byte13 = AlarmaActual;

    Wire.write(byte1);
    Wire.write(byte2);
    Wire.write(byte3);
    Wire.write(byte4);
    Wire.write(byte5);
    Wire.write(byte6);
    Wire.write(byte7);
    Wire.write(byte8);
    Wire.write(byte9);
    Wire.write(byte10);
    Wire.write(byte11);
    Wire.write(byte12);

    // }

    //Presion_PIP_Anterior = Presion_PIP;
}

//**********************************************************************************************************************************************//
// Función para calcular los valores de funcionamiento
// Calculos de velocidades en Motor según pulsos de avance de apretado, relacion de transformacion y cantidad de pulsos por vuelta del motor
//******************************************************************************************************************
void CalcularParametros()
{
    Pasos_Avance = (Vtidal * Cantidad_Pulsos_Apriete) / 100;
    Angulo_Pulso = 360.0 / Cantidad_Pulsos_Motor;
    Angulo_Brazos = ((Pasos_Avance * 360) / (Cantidad_Pulsos_Motor * Relacion_Transmision));
    Velo_Motor_Insp = (Angulo_Pulso / ((Angulo_Brazos * Relacion_Transmision) / Velo_Inspiracion)) * 1000000.0;
    Velo_Motor_Exp = (Angulo_Pulso / ((Angulo_Brazos * Relacion_Transmision) / Velo_Expiracion)) * 1000000.0;
}

//**********************************************************************************************************************************************//
// Función de Lectura de datos del encoder                                                                            //
//**********************************************************************************************************************************************//
void encoder()
{
    unsigned long tiempoInterrupcion = millis();            // variable almacena valor de func. millis
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
// Funcion para la obtencion de los valores de presion en los puntos de la curva necesarios. Se toma el valor y se compara con los datos cargados para alarmas
//**********************************************************************************************************************************************//
double Presion()
{
    float Aux = 0.0;
    int p = 0;
    float AP = 0.01; //Error en la presión
    for (p = 0; p < 10; p++)
    {
        Aux = Aux + (float(analogRead(A0) * 5.0 / 1023.0)); //Leo la entrada analogica que tiene conectada el sensor de presión
        delay(5);
    }
    Vout = Aux / 10.0;
    P1 = ((Vout - 0.04 * Vs) / (0.09 * Vs) + AP) * 10.1972; // 10.1972Se multiplica por el equivalente para la conversion a cmH20
    P1 = 1.002 * P1 + 0.182;                                // Regresion lineal obtenida de los valores experimentales. Ver tabla en excel
    return P1;
}

bool ChequeoPIP()
{
    Presion_PIP = Presion();
    delay(1000);
    Presion_Plateau = Presion();
    if (Presion_PIP > PMAX)
    {
        InformarAlarma(ALARMA_PIP);
        AlarmaActual = ALARMA_PIP;
        IrAlInicio();
        Modo_ON = false;
        return true;
    }
    return false;
}

bool ChequeoPEEP()
{
    Presion_PEEP = Presion();
    if (Presion_PIP > PMAX)
    {
        InformarAlarma(ALARMA_PEEP);
        IrAlInicio();
        AlarmaActual = ALARMA_PEEP;
        return true;
    }
    return false;
}

bool ChequeoSoporteVolumenPresion()
{
   return (Presion() < (float)PTrigger);
}

bool ChequeoPacienteDesconectado(double presion)
{
    if (presion < Pmin_Paciente_Desconectado) {
        AlarmaActual = PACIENTE_DESCONECTADO;
        return true;
    }
    return false;
}

//**********************************************************************************************************************************************//
// Funcion para la obtencion de una grafica de presion. Se desactiva luego de las pruebas
//**********************************************************************************************************************************************//
float Presion_Grafica_Valores = 0;
int Presion_Grafica_Valor_Actual = 0;
unsigned long Presion_Grafica_Valores_Timer = 0;
const float AP = 0.01; //Error en la presión
bool Presion_Grafica()
{
    if ((unsigned long)millis() - Presion_Grafica_Valores_Timer < 5)
    {
        return false;
    }

    Presion_Grafica_Valores = Presion_Grafica_Valores + (float(analogRead(A0) * 5.0 / 1023.0)); //Leo la entrada analogica que tiene conectada el sensor de presión
    Presion_Grafica_Valor_Actual++;
    if (Presion_Grafica_Valor_Actual < 10)
    {
        Presion_Grafica_Valores_Timer = millis();
        return false;
    }

    Vout2 = Presion_Grafica_Valores / 10.0;
    P1 = ((Vout2 - 0.04 * Vs) / (0.09 * Vs) + AP) * 10.1972; // 10.1972Se multiplica por el equivalente para la conversion a cmH20
    P1 = 1.002 * P1 + 0.182;                                 // Regresion lineal obtenida de los valores experimentales. Ver tabla en excel
    P1_1 = (1.002 * P1 + 0.182) * (1000.0 / 10.1972);        // Presion en Pasacales para los calculos de caudal en bernoulli

    // Serial.print("Presion del sistema en CmH2O:");
    // Serial.println(P1);
    // Serial.print("Presion P1 [Pascales]:");
    // Serial.println(P1_1);
    MyPlot.SendData("Presion", P1);
    Presion_Grafica_Valor_Actual = 0;
    Presion_Grafica_Valores_Timer = 0;
    Presion_Grafica_Valores = 0;
    return true;
}

float Volumen_Grafica_Valores = 0;
int Volumen_Grafica_Valor_Actual = 0;
unsigned long Volumen_Grafica_Valores_Timer = 0;
const float AP2 = 0.01; //Error en la presión
bool Volumen_Grafica()
{
    if ((unsigned long)millis() - Volumen_Grafica_Valores_Timer < 5)
    {
        return false;
    }

    Volumen_Grafica_Valores = Volumen_Grafica_Valores + (float(analogRead(A1) * 5.0 / 1023.0)); //Leo la entrada analogica que tiene conectada el sensor de presión
    Volumen_Grafica_Valor_Actual++;

    if (Volumen_Grafica_Valor_Actual < 10)
    {
        Volumen_Grafica_Valores_Timer = millis();
        return false;
    }

    Vout3 = Volumen_Grafica_Valores / 10.0;
    P2 = ((Vout3 - 0.04 * Vs2) / (0.09 * Vs2) + AP2) * 1000.0; // Se multuplica x 1000 para obtener el valor en Pascales, unidad que necesitamos en la ecuacion de bernoulli
                                                               //  En volumen vamos a trabajar las presiones en Pascales (El sensor da el valor en KPA hay que convertir o multiplicar por 1000 para tener el mismo en pascales)
    P2 = 1.017 * P2 - 0.474;                                   // Regresion lineal obtenida de los valores experimentales. Ver tabla en excel
    //   Serial.print("Presion P2:[Pasacales]");
    //   Serial.println(P2);

    Volumen_Grafica_Valor_Actual = 0;
    Volumen_Grafica_Valores_Timer = 0;
    Volumen_Grafica_Valores = 0;
    return true;
}

void Caudal()
{
    float r1 = 0.15;   // Radio r1 en metros
    float r2 = 0.10;   // Radio r2 en metros
    float rho = 1.225; // Densidad del aire 1.225 Kg/m3
    float num = 0.0;
    float den = 0.0;
    float V1 = 0.0; // en m/s
    float Q = 0.0;  // l/s o m3/s
    float pi = 3.14159265;
    float delta_P = 0.0;
    float A1 = 0.0;
    float A2 = 0.0;

    // P1_1 y P2 los sensores la dan en KPascales. ejemplo si P1_1-P2 = 20Kpa=> 20Kpa = 20KN/m2 *1000 N/ KN = 20000 N/m2 = 20000 Kg m/s2 Ose aque el valor lo multiplico x 1000 para tenerlo en Kg m/s2
    //P1_1 = 100.0;
    //P2 = 80.0;
    A1 = pi * pow(r1, 2);
    A2 = pi * pow(r2, 2);
    delta_P = (P1_1 - P2) * 1000;          // Tengo el valor en Kg m/s2
    num = 2 * delta_P;                     // Valor de numerador para obtner la velocidad,
    den = rho * (pow(A1, 2) - pow(A2, 2)); // Valor del denominador para obtener la velocidad. , queda sin unidad
    V1 = A2 * pow((num / den), 0.5);       // Calculo la velocidad 1 en m/s
    Q = A1 * V1;                           // Calculo el caudal
    // Serial.print("Caudal:");
    // Serial.println(Q);
}

//**********************************************************************************************************************************************//
// Funciones de posicionamiento de los brazos
//**********************************************************************************************************************************************//
void IrAlInicio()
{
    int pasos;
    if (CicloActual == INSPIRACION)
        pasos = Pasos_Actuales;
    if (CicloActual == EXPIRACION)
        pasos = Pasos_Avance - Pasos_Actuales;
    digitalWrite(dirPin, LOW);
    for (int i = 0; i < pasos; i++) //Backward 1600 steps
    {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(Velo_Motor_Exp / 2.0);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(Velo_Motor_Exp / 2.0);
    }
    Pasos_Avance = 0;
    Pasos_Actuales = 0;
    CicloActual = INSPIRACION;
}

//**********************************************************************************************************************************************//
// Alarmas
//**********************************************************************************************************************************************//
void AlarmaIntermitente()
{
    if ((unsigned long)millis() - Tiempo_Alarma_Transcurrido > 1000)
    {
        if (Alarma_ON)
        {
            SilenciarAlarma();
            Alarma_ON = false;
        }
        else
        {
            analogWrite(Buzzer_Pin, Tono_Alarma);
            Alarma_ON = true;
        }
        Tiempo_Alarma_Transcurrido = millis();
    }
}

void AlarmaContinua()
{
    analogWrite(Buzzer_Pin, Tono_Alarma);
}

void SilenciarAlarma()
{
    analogWrite(Buzzer_Pin, 0);
}

void InformarAlarma(TipoDeAlarma alarma)
{
    // Comunicar al controlador de pantalla el tipo de alarma
    switch (alarma)
    {
    case ALARMA_PEEP:
        break;
    case ALARMA_PIP:
        break;
    case ALARMA_MECANICA:
        break;
    case SIN_ALARMA:
        break;
    default:
        break;
    }
}
