#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#include <Encoder.h>

//#include <LiquidCrystal.h>
typedef enum TipoDeAlarma
{
  ALARMA_PIP,
  ALARMA_PEEP,
  ALARMA_MECANICA,
  PACIENTE_DESCONECTADO,
  SIN_ALARMA
};
TipoDeAlarma AlarmaActual = SIN_ALARMA;

// Encoder
Encoder myEnc(2, 4);

// Pantalla
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // Inicializa el LCD con DIR, E, RW, RS, D4, D5, D6, D7)

// Pulsador Pantalla
#define Pulsador_Pantalla 5               // Seleccion de entrada para Pulsador navegar en pantallas
int Pulsador_Pantalla_VALUE = 0;          // Variable que guarda el estado de la entrada
int Pulsador_Pantalla_Anterior_VALUE = 0; // Se guarda el valor anetrior para leer flancos ascendentes en la señal de entrada

// Pulsador Seleccion
#define Pulsador_Seleccion 6               // Seleccion de entrada para Pulsador navegar en pantallas
int Pulsador_Seleccion_VALUE = 0;          // Variable que guarda el estado de la entrada
int Pulsador_Seleccion_Anterior_VALUE = 0; // Se guarda el valor anetrior para leer flancos ascendentes en la señal de entrada

// Pulsador de Marcha
#define Pulsador_Marcha 7
int Pulsador_Marcha_VALUE = 0;
int Pulsador_Marcha_VALUE_Anterior = 0;

//Pulsador Modo Funcionamiento

int Contador_Pantalla = 0; //Se define la variable contador par navegar entre las pantallas disponibles
int Contador_IE = 0;
bool Cambio_Opcion_Pantalla = true;
int Valor_Actual_Pantalla = 0;

int BPM_Seleccionado = 26;
int IE_Seleccionado = 0;
int PMAX_Seleccionado = 40;
int PMAX_Seleccionado_Anterior = 40;
int PEEP_Seleccionado = 5;
int PEEP_Seleccionado_Anterior = 5;
int Vtidal_Seleccionado = 40; //(En porcentaje. 40 %. Caudal total del ambu 1500ml)
int Vtidal_Seleccionado_Anterior = 40;
int Vtidal_Seleccionado_LCD = 700; //500ml
int Ptrigger_Seleccionado = -3;
int Ptrigger_Seleccionado_Anterior = -3;

float Tciclo = 0.0;
float Tinspiracion = 0.0;
float Texpiracion = 0.0;

float Velo_Inspiracion = 0.0;
float Velo_Inspiracion_Anterior = 0.0;
float Velo_Expiracion = 0.0;
float Velo_Expiracion_Anterior = 0.0;

float LCD_Presion_PIP = 0.0;
float LCD_Presion_Plateau = 0.0;
float LCD_Presion_PEEP = 0.0;

volatile bool Modo_ON = false;     // Inicializacion en ciclo de REPOSO
volatile bool Cambio_Modo = false; // Indica si hubo un cambio de estado que necesita ser transmitido

int Modo_Seleccionado = 0; //0- Modo Control por Volumen VCV, 1= Modo Control Presion de Soporte PSV
int Modo_Seleccionado_Anterior = 0;

float Sensibilidad_Sensor_Corriente = 0.100;
float ruido = 0.0;
float ValorReposo = 2.5;
float IntensidadPico_A = 0.0;
float IntensidadPico_B = 0.0;
float Intervalo_Lectura_Sensor_Corriente = 1000;
#define Alarma_SensorCorriente_Aviso 2.5
#define Alarma_SensorCorriente_Parada 3
int sensorIntensidad_A = A0;
int sensorIntensidad_B = A1;

unsigned long Aux_Delay_Lectura_Sensor_Corriente = 0;

//*********************************************************************************************************//
// SETUP
//*********************************************************************************************************//
void setup()
{

  Serial.begin(9600);               //iniciamos el puerto serie
  Wire.begin();                     //Se inicializa la comunicación I2C
  lcd.setBacklightPin(3, POSITIVE); //Se enciende la luz del LCD
  lcd.setBacklight(HIGH);
  lcd.begin(16, 2);
  pinMode(Pulsador_Pantalla, INPUT);  //Le digo que el pin valor, 2 es una entrada digital
  pinMode(Pulsador_Seleccion, INPUT); //Le digo que el pin valor, 3 es una entrada digital
  pinMode(Pulsador_Marcha, INPUT);

  //attachInterrupt( 5, Pulsador_Marcha, RISING); //Defino interrupcion 5 Pin 18 para el pulsador de marcha
  attachInterrupt(1, Pulsador_Parada, CHANGE); //Defino interrupcion 6 Pin 19 para pulsador de parada
}

//*********************************************************************************************************//
// LOOP
//*********************************************************************************************************//
void loop()
{
  checkDelay();

  int newValor = myEnc.read();
  int addEndoderValue = 0;
  if (newValor != 0 && newValor % 4 == 0)
  {
    if (newValor < 0)
      addEndoderValue = 1;
    else
      addEndoderValue = -1;
    myEnc.write(0);
  }

  // Leer valores de Input
  Pulsador_Pantalla_VALUE = digitalRead(Pulsador_Pantalla);   // Lectura del estado del pulsador Pantalla
  Pulsador_Seleccion_VALUE = digitalRead(Pulsador_Seleccion); // Lectura del estado del pulsador Selección
  Pulsador_Marcha_VALUE = digitalRead(Pulsador_Marcha);

  if (Pulsador_Marcha_VALUE == HIGH && Pulsador_Marcha_VALUE_Anterior == LOW)
  {
    Modo_ON = true;
    Cambio_Modo = true;
    //  Serial.print("------ MARCHA ------");
  }
  Pulsador_Marcha_VALUE_Anterior = Pulsador_Marcha_VALUE;

  // Movimiento entre pantallas                                                                                            //
  if (Pulsador_Pantalla_VALUE == HIGH & Pulsador_Pantalla_Anterior_VALUE == LOW)
  {
    Contador_Pantalla++;
    lcd.clear();
    Cambio_Opcion_Pantalla = true;
  }
  Pulsador_Pantalla_Anterior_VALUE = Pulsador_Pantalla_VALUE;

  //**********************************************************************//
  // Leer los valors desde el motor
  //**********************************************************************//

  ObtenerValoresSensores();

  //**********************************************************************//
  // Si hay alarma la muestra en pantalla
  //**********************************************************************//
  if (AlarmaActual != SIN_ALARMA)
  {
    ManejoAlarmas();
    MostrarAlarmaEnDisplay();
  }
  else
  {
    //**********************************************************************//
    // Si NO hay alarmas se muestra la opcion seleccionada
    //**********************************************************************//

    switch (Contador_Pantalla)
    {
    //**********************************************************************//
    case 0:
      if (Cambio_Opcion_Pantalla)
      {
        Valor_Actual_Pantalla = Modo_Seleccionado;
        Cambio_Opcion_Pantalla = false;
      }
      NewModo(addEndoderValue);
      if (Pulsador_Seleccion_VALUE == HIGH)
      {
        SetModo(Valor_Actual_Pantalla);
      }
      break;
      //*******************************************************************//

    case 1:
      if (Cambio_Opcion_Pantalla)
      {
        Valor_Actual_Pantalla = BPM_Seleccionado;
        Cambio_Opcion_Pantalla = false;
      }
      NewBpm(addEndoderValue);
      if (Pulsador_Seleccion_VALUE == HIGH)
      {
        SetBpm(Valor_Actual_Pantalla);
      }
      break;

    case 2:
      if (Cambio_Opcion_Pantalla)
      {
        Valor_Actual_Pantalla = Contador_IE;
        Cambio_Opcion_Pantalla = false;
      }
      NewIE(addEndoderValue);
      if (Pulsador_Seleccion_VALUE == HIGH)
      {
        SetIE(Valor_Actual_Pantalla);
      }
      break;

    case 3:
      if (Cambio_Opcion_Pantalla)
      {
        Valor_Actual_Pantalla = PMAX_Seleccionado;
        Cambio_Opcion_Pantalla = false;
      }
      NewPmax(addEndoderValue);
      if (Pulsador_Seleccion_VALUE == HIGH)
      {
        SetPmax(Valor_Actual_Pantalla);
      }
      break;

    case 4:
      if (Cambio_Opcion_Pantalla)
      {
        Valor_Actual_Pantalla = PEEP_Seleccionado;
        Cambio_Opcion_Pantalla = false;
      }
      NewPeep(addEndoderValue);
      if (Pulsador_Seleccion_VALUE == HIGH)
      {
        SetPeep(Valor_Actual_Pantalla);
      }
      break;

    case 5:
      if (Cambio_Opcion_Pantalla)
      {
        Valor_Actual_Pantalla = Vtidal_Seleccionado;
        Cambio_Opcion_Pantalla = false;
      }
      NewVtidal(addEndoderValue);
      if (Pulsador_Seleccion_VALUE == HIGH)
      {
        SetVtidal(Valor_Actual_Pantalla);
      }
      break;

    case 6:
      if (Cambio_Opcion_Pantalla)
      {
        Valor_Actual_Pantalla = Ptrigger_Seleccionado;
        Cambio_Opcion_Pantalla = false;
      }
      NewPtrigger(addEndoderValue);
      if (Pulsador_Seleccion_VALUE == HIGH)
      {
        SetPtrigger(Valor_Actual_Pantalla);
      }
      break;

    case 7:
      // Lee los valores sensados solo si necesita mostrarlos en la pantalla

      lcd.setCursor(0, 0);
      lcd.print("PIP:"); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(4, 0);
      lcd.print(LCD_Presion_PIP); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(8, 0);
      lcd.print(" PL:"); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(12, 0);
      lcd.print(LCD_Presion_Plateau); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(0, 1);
      lcd.print("PEEP:"); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(5, 1);
      lcd.print(LCD_Presion_PEEP); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(9, 1);
      lcd.print("[cmH2O]"); // Escribimos el Mensaje en el LCD.
      break;

    case 8:
      // Pantalla realizada para visulaziar los valores de Tcliclo, Tinsp, Texp, Pmax y PEER !!!!!BORRAR!!!!!
      lcd.setCursor(0, 0);
      lcd.print(Tciclo); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(5, 0);
      lcd.print(Tinspiracion); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(10, 0);
      lcd.print(Texpiracion); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(0, 1);
      lcd.print(PMAX_Seleccionado); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(5, 1);
      lcd.print(PEEP_Seleccionado); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(8, 1);
      lcd.print(Pulsador_Seleccion_VALUE); // Escribimos el Mensaje en el LCD.
      break;

    case 9:
      Contador_Pantalla = 0;
      break;
    }
  }
  //********************************************************************************************************************************//
  // Calculo de los valores necesarios para el funiconamiento en el ciclo de respiración                                            //
  // int BPM_Seleccionado contiene el valor seleccionado para los ciclos por minuto                                                 //
  // Si la variable int Contador_IE=0 tomo IE:1:1, Si int Contador_IE=1 tomo IE:1:2, Si int Contador_IE=2 tomo IE:1:3               //
  // int PMAX_Seleccionado contiene el valor seleccionado para la maxima presion a la que trabaja el sistema                        //
  // int PEEP_Seleccionado Contiene el valor de presión PEEP Seleccionado                                                           //
  //********************************************************************************************************************************//

  Tciclo = 60.0 / BPM_Seleccionado; // Calculo del Tiempo de ciclo

  // Selección de I:E
  //I:E 1:1
  if (Contador_IE == 0)
  {
    Tinspiracion = Tciclo / 2;
    Texpiracion = Tciclo - Tinspiracion;
  }

  //I:E 1:2
  if (Contador_IE == 1)
  {
    Tinspiracion = Tciclo / 3;
    Texpiracion = Tciclo - Tinspiracion;
  }

  //I:E 1:3
  if (Contador_IE == 2)
  {
    Tinspiracion = Tciclo / 4;
    Texpiracion = Tciclo - Tinspiracion;
  }

  // Calculo de la velocidad de Inspiración y Expiración para el movimiento hacia adelante y hacia atrás de las paletas
  Velo_Inspiracion = Tinspiracion; //Paso el valor la cuenta la realizo en el otro arduino (UNO)
  Velo_Expiracion = Texpiracion;   // Paso el valor, la cuenta la realizo en el otro arduino (UNO)

  //*******************************************************************************************************************************
  // Toma lecturas de corriente bobinas del motor (Hacer una interrupcion para medir la corriente cada 1 seg)
  //*********************************************************************************************************************************
  /*
  if ((unsigned long)millis() - Aux_Delay_Lectura_Sensor_Corriente >= Intervalo_Lectura_Sensor_Corriente)
  {
    IntensidadPico_A = Corriente_Sensor_A(); //Funcion para llamar a la lectura de corriente en la Bobina A del motor paso a paso.
    IntensidadPico_B = Corriente_Sensor_B(); //Funcion para llamar a la lectura de corriente en la Bobina A del motor paso a paso.
    Aux_Delay_Lectura_Sensor_Corriente = millis();
    if (IntensidadPico_A > Alarma_SensorCorriente_Aviso || IntensidadPico_B > Alarma_SensorCorriente_Aviso)
    {
      AlarmaActual = ALARMA_MECANICA;
      if (IntensidadPico_A > Alarma_SensorCorriente_Parada || IntensidadPico_B > Alarma_SensorCorriente_Parada)
      {
        Modo_ON = false;
      }
    }
  }
  */

  //************************************************************************************************************************//
  // Comunicacion I2c entre Mega (Maestro) y Uno (Esclavo)                                                                  //
  // Datos Transmitidos: Velocidad de Inspiracion, Velocidad de Expiracion, Presion maxima y PEEP                           //
  //************************************************************************************************************************//
  //Solo se transmiten datos cuando hay algun cambio en algunos de los parametros.

  //

  if ((Velo_Inspiracion != Velo_Inspiracion_Anterior) || (Velo_Expiracion != Velo_Expiracion_Anterior) || (PMAX_Seleccionado != PMAX_Seleccionado_Anterior) || (PEEP_Seleccionado != PEEP_Seleccionado_Anterior) || (Vtidal_Seleccionado != Vtidal_Seleccionado_Anterior) || Cambio_Modo || Ptrigger_Seleccionado != Ptrigger_Seleccionado_Anterior || Modo_Seleccionado != Modo_Seleccionado_Anterior)
  // (Pulsador_Marcha_VALUE == digitalRead(19))
  {
    byte byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9, byte10, byte11, byte12, byte13, byte14, byte15, byte16, byte17, byte18, byte19, byte20;
    unsigned int aux;

    // Se envia Velocidad de Inspiracion
    // Ajustando o número antes da vírgula
    float Velo_Inspiracion_Tx = Velo_Inspiracion;
    aux = (unsigned int)Velo_Inspiracion_Tx; // aux = 46689, Pega somente a parte inteira da variável float (0 - 65536)
    byte2 = aux;                             // byte2 = 0B01100001, pega apenas os primeros 8 bits
    byte1 = (aux >> 8);                      // byte1 = 0B10110110, pega os 8 ultimos bits
    // Ajustando o número depois da vírgula
    Velo_Inspiracion_Tx -= aux;              // Deixa apenas o número depois da vírgula
    Velo_Inspiracion_Tx *= 10000;            // Multiplica por 10k para pegar 4 dígitos após a vírgula
    aux = (unsigned int)Velo_Inspiracion_Tx; // Pega somente o valor antes da vírgula
    byte4 = aux;                             // byte2 = 0B00101110, pega apenas os primeros 8 bits
    byte3 = (aux >> 8);                      // byte1 = 0B00100010, pega os 8 ultimos bits

    //Se envia velocidad de Expiracion
    float Velo_Expiracion_Tx = Velo_Expiracion;
    aux = (unsigned int)Velo_Expiracion_Tx; // aux = 46689, Pega somente a parte inteira da variável float (0 - 65536)
    byte6 = aux;                            // byte2 = 0B01100001, pega apenas os primeros 8 bits
    byte5 = (aux >> 8);                     // byte1 = 0B10110110, pega os 8 ultimos bits
    // Ajustando o número depois da vírgula
    Velo_Expiracion_Tx -= aux;              // Deixa apenas o número depois da vírgula
    Velo_Expiracion_Tx *= 10000;            // Multiplica por 10k para pegar 4 dígitos após a vírgula
    aux = (unsigned int)Velo_Expiracion_Tx; // Pega somente o valor antes da vírgula
    byte8 = aux;                            // byte2 = 0B00101110, pega apenas os primeros 8 bits
    byte7 = (aux >> 8);                     // byte1 = 0B00100010, pega os 8 ultimos bits

    //Se envia Presion MAxima de trabajo
    float PMAX_Tx = PMAX_Seleccionado;
    aux = (unsigned int)PMAX_Tx; // aux = 46689, Pega somente a parte inteira da variável float (0 - 65536)
    byte10 = aux;                // byte2 = 0B01100001, pega apenas os primeros 8 bits
    byte9 = (aux >> 8);          // byte1 = 0B10110110, pega os 8 ultimos bits
    // Ajustando o número depois da vírgula
    PMAX_Tx -= aux;              // Deixa apenas o número depois da vírgula
    PMAX_Tx *= 10000;            // Multiplica por 10k para pegar 4 dígitos após a vírgula
    aux = (unsigned int)PMAX_Tx; // Pega somente o valor antes da vírgula
    byte12 = aux;                // byte2 = 0B00101110, pega apenas os primeros 8 bits
    byte11 = (aux >> 8);         // byte1 = 0B00100010, pega os 8 ultimos bits

    //Se envia Presion PEEP
    float PEEP_Tx = PEEP_Seleccionado;
    aux = (unsigned int)PEEP_Tx; // aux = 46689, Pega somente a parte inteira da variável float (0 - 65536)
    byte14 = aux;                // byte2 = 0B01100001, pega apenas os primeros 8 bits
    byte13 = (aux >> 8);         // byte1 = 0B10110110, pega os 8 ultimos bits
    // Ajustando o número depois da vírgula
    PEEP_Tx -= aux;              // Deixa apenas o número depois da vírgula
    PEEP_Tx *= 10000;            // Multiplica por 10k para pegar 4 dígitos após a vírgula
    aux = (unsigned int)PEEP_Tx; // Pega somente o valor antes da vírgula
    byte16 = aux;                // byte2 = 0B00101110, pega apenas os primeros 8 bits
    byte15 = (aux >> 8);         // byte1 = 0B00100010, pega os 8 ultimos bits

    //Envio la variable Inicio de Ciclo
    byte17 = Modo_ON;

    //Envio el porcentaje de caudal de aire
    byte18 = Vtidal_Seleccionado;

    //Envío Presión Trigger
    byte19 = Ptrigger_Seleccionado * -1; //Al no poder transmitir en negativo se hace en positivo y se multiplica por -1 en el receptor

    //Envio el Modo de funcionamiento 0=VCV y 1=PSC
    byte20 = Modo_Seleccionado;

    //Inicia transmision de Bytes
    Wire.beginTransmission(4); // Começa transmissão para o escravo 0x2C
    Wire.write(byte1);         // Envia os bytes do número antes da vírgua e depois da vírgula
    Wire.write(byte2);
    Wire.write(byte3);
    Wire.write(byte4);
    Wire.write(byte5); // Envia os bytes do número antes da vírgua e depois da vírgula
    Wire.write(byte6);
    Wire.write(byte7);
    Wire.write(byte8);
    Wire.write(byte9); // Envia os bytes do número antes da vírgua e depois da vírgula
    Wire.write(byte10);
    Wire.write(byte11);
    Wire.write(byte12);
    Wire.write(byte13); // Envia os bytes do número antes da vírgua e depois da vírgula
    Wire.write(byte14);
    Wire.write(byte15);
    Wire.write(byte16);
    Wire.write(byte17);
    Wire.write(byte18);
    Wire.write(byte19);
    Wire.write(byte20);
    Wire.endTransmission(); // Termina a transmissão
  }

  Velo_Inspiracion_Anterior = Velo_Inspiracion;
  Velo_Expiracion_Anterior = Velo_Expiracion;
  PMAX_Seleccionado_Anterior = PMAX_Seleccionado;
  PEEP_Seleccionado_Anterior = PEEP_Seleccionado;
  Vtidal_Seleccionado_Anterior = Vtidal_Seleccionado;
  Ptrigger_Seleccionado_Anterior = Ptrigger_Seleccionado;
  Modo_Seleccionado_Anterior = Modo_Seleccionado;
  Cambio_Modo = false;
}

//*******************************************************************************//
// Funcion lectura de corriente Sensor Bobina A motor Paso a Paso
//*******************************************************************************

float Corriente_Sensor_A()
{
  float intensidadMaxima_A = 0.0;
  float intensidadMinima_A = 0.0;
  float Lectura_A = analogRead(sensorIntensidad_A) * (5.0 / 1023.0);
  float Corriente_Motor_A = 0.9 * Corriente_Motor_A + 0.1 * ((Lectura_A - ValorReposo) / Sensibilidad_Sensor_Corriente);
  if (Corriente_Motor_A > intensidadMaxima_A)
    intensidadMaxima_A = Corriente_Motor_A;
  if (Corriente_Motor_A < intensidadMinima_A)
    intensidadMinima_A = Corriente_Motor_A;
  return (((intensidadMaxima_A - intensidadMinima_A) / 2) - ruido);
}

float Corriente_Sensor_B()
{
  float intensidadMaxima_B = 0.0;
  float intensidadMinima_B = 0.0;
  float Lectura_B = analogRead(sensorIntensidad_B) * (5.0 / 1023.0);
  float Corriente_Motor_B = 0.9 * Corriente_Motor_B + 0.1 * ((Lectura_B - ValorReposo) / Sensibilidad_Sensor_Corriente);
  if (Corriente_Motor_B > intensidadMaxima_B)
    intensidadMaxima_B = Corriente_Motor_B;
  if (Corriente_Motor_B < intensidadMinima_B)
    intensidadMinima_B = Corriente_Motor_B;
  return (((intensidadMaxima_B - intensidadMinima_B) / 2) - ruido);
}

//void Mostrar_Corriente_A()
//{
// float irms = IntensidadPico*0.707;
//Serial.print("Intensidad Pico:");
//Serial.println(IntensidadPico,3);
//Serial.print("Irms:");
//Serial.println(irms,3);
//delay(100);
//}

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

//****************************************************************************************************************
// Funcion Pulsador de Marcha
//****************************************************************************************************************

//void Pulsador_Marcha ()
//{
// Inicio_Ciclo = 1;
//Serial.print("Inicio_Ciclo:");
//Serial.println(Inicio_Ciclo);
//}

void Pulsador_Parada()
{
  Modo_ON = false;
  Cambio_Modo = true;
  //  Serial.print("------ PARADA ------");
}

//****************************************************************************************************************
// Leer los valores sensados
//****************************************************************************************************************

void ObtenerValoresSensores()
{
  if (estaEnDelay())
    return;
  LeerValoresSensados();
  delayMillis(250);
}

void LeerValoresSensados()
{
  Wire.requestFrom(4, 13); //4,4 significa nodo 4, 4 bytes
  byte Rx_slave1, Rx_slave2, Rx_slave3, Rx_slave4, Rx_slave5, Rx_slave6, Rx_slave7, Rx_slave8, Rx_slave9, Rx_slave10, Rx_slave11, Rx_slave12, Rx_slave13;
  Rx_slave1 = Wire.read();
  Rx_slave2 = Wire.read();
  Rx_slave3 = Wire.read();
  Rx_slave4 = Wire.read();
  Rx_slave5 = Wire.read();
  Rx_slave6 = Wire.read();
  Rx_slave7 = Wire.read();
  Rx_slave8 = Wire.read();
  Rx_slave9 = Wire.read();
  Rx_slave10 = Wire.read();
  Rx_slave11 = Wire.read();
  Rx_slave12 = Wire.read();
  Rx_slave13 = Wire.read();

  unsigned int aux_rx;
  aux_rx = (Rx_slave3 << 8) | Rx_slave4;      // Ajusta a parte fracionáia (depois da vírgula)
  LCD_Presion_PIP = (float)(aux_rx * 0.0001); // Atribui a parte fracionária, depois da vírgula
  aux_rx = (Rx_slave1 << 8) | Rx_slave2;      // Ajusta a parte inteira (antes da vírgula)
  LCD_Presion_PIP += aux_rx;                  // Atribui a parte iteira

  aux_rx = (Rx_slave7 << 8) | Rx_slave8;          // Ajusta a parte fracionáia (depois da vírgula)
  LCD_Presion_Plateau = (float)(aux_rx * 0.0001); // Atribui a parte fracionária, depois da vírgula
  aux_rx = (Rx_slave5 << 8) | Rx_slave6;          // Ajusta a parte inteira (antes da vírgula)
  LCD_Presion_Plateau += aux_rx;                  // Atribui a parte iteira

  aux_rx = (Rx_slave11 << 8) | Rx_slave12;     // Ajusta a parte fracionáia (depois da vírgula)
  LCD_Presion_PEEP = (float)(aux_rx * 0.0001); // Atribui a parte fracionária, depois da vírgula
  aux_rx = (Rx_slave9 << 8) | Rx_slave10;      // Ajusta a parte inteira (antes da vírgula)
  LCD_Presion_PEEP += aux_rx;                  // Atribui a parte iteir

  char codigoAlarma = Rx_slave13;
  AlarmaActual = ObtenerAlarmaDesdeCodigo(codigoAlarma);

  //Serial.print("Presion PIP");
  //Serial.println(LCD_Presion_PIP);
  //   Serial.print("Presion Plateau");
  // Serial.println(LCD_Presion_Plateau);
  //  Serial.print("Presion PEEP");
  //Serial.println(LCD_Presion_PEEP);
}

//****************************************************************************************************************
// Funciones control valores
//****************************************************************************************************************
// Error de rango
void ImprimirErrorDeRango()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fuera de Rango"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(0, 1);
  lcd.print("Intentelo"); // Escribimos el Mensaje en el LCD.
  delay(1500);
  lcd.clear();
}

// Modo de Funcionamiento  0- Modo Control por Volumen VCV, 1= Modo Control Presion de Soporte PSV  //
void NewModo(int value)
{
  lcd.setCursor(0, 0);
  lcd.print("MODO [VCV/PSV]"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(0, 1);
  lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(4, 1);

  Valor_Actual_Pantalla = ((Valor_Actual_Pantalla + abs(value)) % 2);
  switch (Valor_Actual_Pantalla)
  {
  case 0:
    lcd.print("VCV"); // Escribimos el Mensaje en el LCD.
    break;
  case 1:
    lcd.print("PSV"); // Escribimos el Mensaje en el LCD.
    break;
  }
  lcd.setCursor(9, 1);
  lcd.print("SET:"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(13, 1);
  if (Modo_Seleccionado)
  {
    lcd.print("PSV"); // Escribimos el Mensaje en el LCD.
  }
  else
  {
    lcd.print("VCV"); // Escribimos el Mensaje en el LCD.
  }
}

void SetModo(int value)
{
  Modo_Seleccionado = value;
  switch (Modo_Seleccionado)
  {
  case 0:
    lcd.print("VCV"); // Escribimos el Mensaje en el LCD.
    break;
  case 1:
    lcd.print("PSV"); // Escribimos el Mensaje en el LCD.
    break;
  }
}

// BPM  ****************************************************************************************************************
#define BmpMinValue 8
#define BmpMaxValue 30
void NewBpm(int value)
{
  if (Valor_Actual_Pantalla + value > BmpMaxValue || Valor_Actual_Pantalla + value < BmpMinValue)
  {
    value = 0;
  }

  Valor_Actual_Pantalla += value;
  lcd.setCursor(0, 0);
  lcd.print("BPM [8-30]"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(0, 1);
  lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(4, 1);
  lcd.print("   ");
  lcd.setCursor(4, 1);
  lcd.print(Valor_Actual_Pantalla); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(9, 1);
  lcd.print("SET:");           // Escribimos el Mensaje en el LCD.
  lcd.print(BPM_Seleccionado); // Escribimos el Mensaje en el LCD.
}

void SetBpm(int value)
{
  if (value >= BmpMinValue && value <= BmpMaxValue)
  {
    lcd.setCursor(11, 1);
    lcd.clear();
    BPM_Seleccionado = value;
    lcd.print(BPM_Seleccionado); // Escribimos el Mensaje en el LCD.
  }
  else
  {
    ImprimirErrorDeRango();
  }
}

// IE ****************************************************************************************************************
void NewIE(int value)
{
  lcd.setCursor(0, 0);
  lcd.print("I:E [1:1/2/3]"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(0, 1);
  lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(4, 1);

  Valor_Actual_Pantalla = ((Valor_Actual_Pantalla + abs(value)) % 3);
  switch (Valor_Actual_Pantalla)
  {
  case 0:
    lcd.print("1:1"); // Escribimos el Mensaje en el LCD.
    break;
  case 1:
    lcd.print("1:2"); // Escribimos el Mensaje en el LCD.
    break;
  case 2:
    lcd.print("1:3"); // Escribimos el Mensaje en el LCD.
    break;
  }
  lcd.setCursor(8, 1);
  lcd.print("SET:"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(12, 1);
  lcd.print("1:" + String(Contador_IE + 1)); // Escribimos el Mensaje en el LCD.
}

void SetIE(int value)
{
  if (value >= 0 && value <= 3)
  {
    Contador_IE = value;
  }
}

// P Max  ****************************************************************************************************************
#define PmaxMinValue 3
#define PmaxMaxValue 60
void NewPmax(int value)
{
  if (Valor_Actual_Pantalla + value > PmaxMaxValue || Valor_Actual_Pantalla + value < PmaxMinValue)
  {
    value = 0;
  }

  Valor_Actual_Pantalla += value;

  lcd.setCursor(0, 0);
  lcd.print("PMax [" + String(PmaxMinValue) + "-" + PmaxMaxValue + "cmH2o]"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(0, 1);
  lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(4, 1);
  lcd.print("   ");
  lcd.setCursor(4, 1);
  lcd.print(Valor_Actual_Pantalla); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(9, 1);
  lcd.print("SET:");            // Escribimos el Mensaje en el LCD.
  lcd.print(PMAX_Seleccionado); // Escribimos el Mensaje en el LCD.
}

void SetPmax(int value)
{
  if (value >= PmaxMinValue && value <= PmaxMaxValue)
  {
    lcd.setCursor(11, 1);
    lcd.clear();
    PMAX_Seleccionado = value;
    lcd.print(PMAX_Seleccionado); // Escribimos el Mensaje en el LCD.
  }
  else
  {
    ImprimirErrorDeRango();
  }
}

// PEEP  ****************************************************************************************************************
#define PeepMinValue 0
#define PeepMaxValue 25
void NewPeep(int value)
{
  if (Valor_Actual_Pantalla + value > PeepMaxValue || Valor_Actual_Pantalla + value < PeepMinValue)
  {
    value = 0;
  }

  Valor_Actual_Pantalla += value;

  lcd.setCursor(0, 0);
  lcd.print("PEEP [" + String(PeepMinValue) + "-" + PeepMaxValue + "cmH2o]"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(0, 1);
  lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(4, 1);
  lcd.print("   ");
  lcd.setCursor(4, 1);
  lcd.print(Valor_Actual_Pantalla); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(9, 1);
  lcd.print("SET:");            // Escribimos el Mensaje en el LCD.
  lcd.print(PEEP_Seleccionado); // Escribimos el Mensaje en el LCD.
}

void SetPeep(int value)
{
  if (value >= PeepMinValue && value <= PeepMaxValue)
  {
    lcd.setCursor(11, 1);
    lcd.clear();
    PEEP_Seleccionado = value;
    lcd.print(PEEP_Seleccionado); // Escribimos el Mensaje en el LCD.
  }
  else
  {
    ImprimirErrorDeRango();
  }
}

// Vtidal  ****************************************************************************************************************
#define VtidalMinValue 0
#define VtidalMaxValue 100
void NewVtidal(int value)
{
  if (Valor_Actual_Pantalla + value > VtidalMaxValue || Valor_Actual_Pantalla + value < VtidalMinValue)
  {
    value = 0;
  }

  Valor_Actual_Pantalla += value;

  lcd.setCursor(0, 0);
  lcd.print("V.TIDAL [" + String(VtidalMinValue) + "-" + VtidalMaxValue + "%]"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(0, 1);
  lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(4, 1);
  lcd.print("   ");
  lcd.setCursor(4, 1);
  lcd.print(Valor_Actual_Pantalla); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(9, 1);
  lcd.print("SET:");              // Escribimos el Mensaje en el LCD.
  lcd.print(Vtidal_Seleccionado); // Escribimos el Mensaje en el LCD.
}

void SetVtidal(int value)
{
  if (value >= VtidalMinValue && value <= VtidalMaxValue)
  {
    lcd.setCursor(11, 1);
    lcd.clear();
    Vtidal_Seleccionado = value;
    lcd.print(Vtidal_Seleccionado); // Escribimos el Mensaje en el LCD.
  }
  else
  {
    ImprimirErrorDeRango();
  }
}

// Presion Trigger Modo Soporte de Presión  ****************************************************************************************************************
#define triggerMinValue -3
#define triggerMaxValue 0
void NewPtrigger(int value)
{
  if (Valor_Actual_Pantalla + value > triggerMaxValue || Valor_Actual_Pantalla + value < triggerMinValue)
  {
    value = 0;
  }

  Valor_Actual_Pantalla += value;

 // lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Trig[" + String(triggerMinValue) + "-" + triggerMaxValue + "cmH2O]"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(0, 1);
  lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(4, 1);
  lcd.print("   ");
  lcd.setCursor(4, 1);
  lcd.print(Valor_Actual_Pantalla); // Escribimos el Mensaje en el LCD.
  lcd.setCursor(9, 1);
  lcd.print("SET:");                // Escribimos el Mensaje en el LCD.
  lcd.print(Ptrigger_Seleccionado); // Escribimos el Mensaje en el LCD.
}

void SetPtrigger(int value)
{
  if (value >= triggerMinValue && value <= triggerMaxValue)
  {
    lcd.setCursor(11, 1);
    lcd.clear();
    Ptrigger_Seleccionado = value;
    lcd.print(Ptrigger_Seleccionado); // Escribimos el Mensaje en el LCD.
  }
  else
  {
    ImprimirErrorDeRango();
  }
}

// Alarmas  ****************************************************************************************************************
void ManejoAlarmas()
{
  switch (AlarmaActual)
  {
  case ALARMA_PEEP:
  case ALARMA_PIP:
  case ALARMA_MECANICA:
  case PACIENTE_DESCONECTADO:
    break;
  default:
    break;
  }
}

bool _AuxMostrarAlarma = false;
void MostrarAlarmaEnDisplay()
{
  //lcd.clear();

  lcd.setCursor(5, 0);
  lcd.print("ALARMA"); // Escribimos el Mensaje en el LCD.
  if (AlarmaActual == ALARMA_MECANICA)
  {
    lcd.setCursor(1, 1);
    lcd.print("FALLA MECANICA"); // Escribimos el Mensaje en el LCD.
  }
  if (AlarmaActual == ALARMA_PEEP)
  {
    lcd.setCursor(1, 1);
    lcd.print("P.INFERIOR.PEEP"); // Escribimos el Mensaje en el LCD.
  }
  if (AlarmaActual == ALARMA_PIP)
  {
    lcd.setCursor(1, 1);
    lcd.print("P.MAX.EXCEDIDA"); // Escribimos el Mensaje en el LCD.
  }
  if (AlarmaActual == PACIENTE_DESCONECTADO)
  {
    lcd.setCursor(0, 1);
    lcd.print("PAC.DESCONECTADO"); // Escribimos el Mensaje en el LCD.
  }
  delay(100);
  lcd.clear();
}

TipoDeAlarma ObtenerAlarmaDesdeCodigo(char codigo)
{
  switch (codigo)
  {
  case 0:
    return SIN_ALARMA;
  case 1:
    return ALARMA_PEEP;
  case 2:
    return ALARMA_PIP;
  case 3:
    return ALARMA_MECANICA;
  case 4:
    return PACIENTE_DESCONECTADO;
  }
}
