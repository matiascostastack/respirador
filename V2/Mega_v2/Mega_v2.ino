#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

//#include <LiquidCrystal.h>

// Potenciometro
#define Pote A0                                   // Se selecciona la entrada A0 para la lectura analogica del potenciometro
int Pote_Value;                                   // variable que almacena el valor raw de A0 (0 a 1023)
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // Inicializa el LCD con DIR, E, RW, RS, D4, D5, D6, D7)

// Pulsador Pantalla
#define Pulsador_Pantalla 2               // Seleccion de entrada para Pulsador navegar en pantallas
int Pulsador_Pantalla_VALUE = 0;          // Variable que guarda el estado de la entrada
int Pulsador_Pantalla_Anterior_VALUE = 0; // Se guarda el valor anetrior para leer flancos ascendentes en la señal de entrada

// Pulsador Seleccion
#define Pulsador_Seleccion 3               // Seleccion de entrada para Pulsador navegar en pantallas
int Pulsador_Seleccion_VALUE = 0;          // Variable que guarda el estado de la entrada
int Pulsador_Seleccion_Anterior_VALUE = 0; // Se guarda el valor anetrior para leer flancos ascendentes en la señal de entrada

// Pulsador de Marcha
#define Pulsador_Marcha 18
int Pulsador_Marcha_VALUE = 0;
int Pulsador_Marcha_VALUE_Anterior = 0;

int Contador_Pantalla = 0; //Se define la variable contador par navegar entre las pantallas disponibles
int Contador_IE = 0;
int Pote_Display = 0;
int BPM_Seleccionado = 15;
int IE_Seleccionado = 0;
int PMAX_Seleccionado = 30;
int PMAX_Seleccionado_Anterior = 30;
int PEEP_Seleccionado = 5;
int PEEP_Seleccionado_Anterior = 5;
int Vtidal_Seleccionado = 40; //(En porcentaje. 40 %. Caudal total del ambu 1500ml)
int Vtidal_Seleccionado_Anterior = 40;
float Tciclo = 0.0;
float Tinspiracion = 0.0;
float Texpiracion = 0.0;

float Velo_Inspiracion = 0.0;
float Velo_Inspiracion_Anterior = 0.0;
float Velo_Expiracion = 0.0;
float Velo_Expiracion_Anterior = 0.0;

volatile bool Modo_ON = false;     // Inicializacion en ciclo de REPOSO
volatile bool Cambio_Modo = false; // Indica si hubo un cambio de estado que necesita ser transmitido

//*********************************************************************************************************//
// SETUP
//*********************************************************************************************************//
void setup()
{

  Serial.begin(9600);               //iniciamos el puerto serie
  Wire.begin();                     //Se inicializa la comunicación I2C
  lcd.setBacklightPin(3, POSITIVE); //Se enciende la luz del LCD
  lcd.setBacklight(HIGH);
  lcd.begin(16, 2);                   // Inicializar el LCD con el número de  columnas y filas del LCD
  pinMode(Pulsador_Pantalla, INPUT);  //Le digo que el pin valor, 2 es una entrada digital
  pinMode(Pulsador_Seleccion, INPUT); //Le digo que el pin valor, 3 es una entrada digital
  pinMode(Pulsador_Marcha, INPUT);

  //attachInterrupt( 5, Pulsador_Marcha, RISING); //Defino interrupcion 5 Pin 18 para el pulsador de marcha
  attachInterrupt(4, Pulsador_Parada, CHANGE); //Defino interrupcion 6 Pin 19 para pulsador de parada
}

//*********************************************************************************************************//
// LOOP
//*********************************************************************************************************//
void loop()
{
  // Leer valores de Input
  Pote_Value = analogRead(Pote);                              // Se realiza la lectura del canal analogico A0
  Pote_Display = map(Pote_Value, 0, 1023, 0, 99);             // Mapeo de la señal analogica entre 0 y 100
  Pulsador_Pantalla_VALUE = digitalRead(Pulsador_Pantalla);   // Lectura del estado del pulsador Pantalla
  Pulsador_Seleccion_VALUE = digitalRead(Pulsador_Seleccion); // Lectura del estado del pulsador Selección
  Pulsador_Marcha_VALUE = digitalRead(Pulsador_Marcha);

  if (Pulsador_Marcha_VALUE == HIGH & Pulsador_Marcha_VALUE_Anterior == LOW)
  {
    Modo_ON = true;
    Cambio_Modo = true;
    Pulsador_Marcha_VALUE_Anterior = Pulsador_Marcha_VALUE;
    Serial.print("------ PARADA ------");
  }

  // Movimiento entre pantallas                                                                                            //
  if (Pulsador_Pantalla_VALUE == HIGH & Pulsador_Pantalla_Anterior_VALUE == LOW)
  {
    Contador_Pantalla++;
    lcd.clear();
    Pulsador_Pantalla_Anterior_VALUE = Pulsador_Pantalla_VALUE;
  }

  switch (Contador_Pantalla)
  {
  case 0:
    lcd.setCursor(0, 0);
    lcd.print("BPM [8-30]"); // Escribimos el Mensaje en el LCD.
    lcd.setCursor(0, 1);
    lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
    lcd.setCursor(4, 1);
    lcd.print(Pote_Display); // Escribimos el Mensaje en el LCD.
    lcd.setCursor(7, 1);
    lcd.print("SET:");           // Escribimos el Mensaje en el LCD.
    lcd.print(BPM_Seleccionado); // Escribimos el Mensaje en el LCD.

    if (Pulsador_Seleccion_VALUE == HIGH)
    {
      if (Pote_Display<31 & Pote_Display> 7)
      {
        lcd.setCursor(11, 1);
        lcd.clear();
        BPM_Seleccionado = Pote_Display;
        lcd.print(BPM_Seleccionado); // Escribimos el Mensaje en el LCD.
      }
      else
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Fuera de Rango"); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(0, 1);
        lcd.print("Intentelo"); // Escribimos el Mensaje en el LCD.
        delay(1500);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("BPM [3-60]"); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(0, 1);
        lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(4, 1);
        lcd.print(Pote_Display); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(7, 1);
        lcd.print("SET:");           // Escribimos el Mensaje en el LCD.
        lcd.print(BPM_Seleccionado); // Escribimos el Mensaje en el LCD.
      }
    }
    break;

  case 1:
    lcd.setCursor(0, 0);
    lcd.print("I:E [1:1/2/3]"); // Escribimos el Mensaje en el LCD.
    lcd.setCursor(0, 1);
    lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
    lcd.setCursor(4, 1);

    if (Pulsador_Seleccion_VALUE == HIGH & Pulsador_Seleccion_Anterior_VALUE == LOW)
    {
      Contador_IE++;
      lcd.clear();
    }
    Pulsador_Seleccion_Anterior_VALUE = Pulsador_Seleccion_VALUE;

    switch (Contador_IE)
    {
    case 0:
      lcd.print("1:1"); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(8, 1);
      lcd.print("SET:"); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(12, 1);
      lcd.print("1:1 "); // Escribimos el Mensaje en el LCD.

      break;

    case 1:
      lcd.print("1:2"); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(8, 1);
      lcd.print("SET:"); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(12, 1);
      lcd.print("1:2"); // Escribimos el Mensaje en el LCD.

      break;

    case 2:
      lcd.print("1:3"); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(8, 1);
      lcd.print("SET:"); // Escribimos el Mensaje en el LCD.
      lcd.setCursor(12, 1);
      lcd.print("1:3"); // Escribimos el Mensaje en el LCD.

      break;

    case 3:
      Contador_IE = 0;

      break;
    }

    break;

  case 2:
    lcd.setCursor(0, 0);
    lcd.print("PMax [3-40cmH2o]"); // Escribimos el Mensaje en el LCD.
    lcd.setCursor(0, 1);
    lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
    lcd.setCursor(4, 1);
    lcd.print(Pote_Display); // Escribimos el Mensaje en el LCD.
    lcd.setCursor(7, 1);
    lcd.print("SET:");            // Escribimos el Mensaje en el LCD.
    lcd.print(PMAX_Seleccionado); // Escribimos el Mensaje en el LCD.

    if (Pulsador_Seleccion_VALUE == HIGH)
    {

      if (Pote_Display<40 & Pote_Display> 0)
      {
        lcd.setCursor(11, 1);
        lcd.clear();
        PMAX_Seleccionado = Pote_Display;
        lcd.print(PMAX_Seleccionado); // Escribimos el Mensaje en el LCD.
      }
      else
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Fuera de Rango"); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(0, 1);
        lcd.print("Intentelo"); // Escribimos el Mensaje en el LCD.
        delay(1500);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("PMax [3-40cmH2o]"); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(0, 1);
        lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(4, 1);
        lcd.print(Pote_Display); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(7, 1);
        lcd.print("SET:");            // Escribimos el Mensaje en el LCD.
        lcd.print(PMAX_Seleccionado); // Escribimos el Mensaje en el LCD.
      }
    }
    break;

  case 3:
    lcd.setCursor(0, 0);
    lcd.print("PEEP [0-10cmH2o]"); // Escribimos el Mensaje en el LCD.
    lcd.setCursor(0, 1);
    lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
    lcd.setCursor(4, 1);
    lcd.print(Pote_Display); // Escribimos el Mensaje en el LCD.
    lcd.setCursor(7, 1);
    lcd.print("SET:");            // Escribimos el Mensaje en el LCD.
    lcd.print(PEEP_Seleccionado); // Escribimos el Mensaje en el LCD.

    if (Pulsador_Seleccion_VALUE == HIGH)
    {
      if (Pote_Display<10 & Pote_Display> 0)
      {
        lcd.setCursor(11, 1);
        lcd.clear();
        PEEP_Seleccionado = Pote_Display;
        lcd.print(PEEP_Seleccionado); // Escribimos el Mensaje en el LCD.
      }
      else
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Fuera de Rango"); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(0, 1);
        lcd.print("Intentelo"); // Escribimos el Mensaje en el LCD.
        delay(1500);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("PEEP [0-10cmH2o]"); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(0, 1);
        lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(4, 1);
        lcd.print(Pote_Display); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(7, 1);
        lcd.print("SET:");            // Escribimos el Mensaje en el LCD.
        lcd.print(PEEP_Seleccionado); // Escribimos el Mensaje en el LCD.
      }
    }
    break;

  case 4:
    lcd.setCursor(0, 0);
    lcd.print("Vtidal [0-100%]"); // Escribimos el Mensaje en el LCD.
    lcd.setCursor(0, 1);
    lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
    lcd.setCursor(4, 1);
    lcd.print(Pote_Display); // Escribimos el Mensaje en el LCD.
    lcd.setCursor(7, 1);
    lcd.print("SET:");              // Escribimos el Mensaje en el LCD.
    lcd.print(Vtidal_Seleccionado); // Escribimos el Mensaje en el LCD.

    if (Pulsador_Seleccion_VALUE == HIGH)
    {
      if (Pote_Display<101 & Pote_Display> 0)
      {
        lcd.setCursor(11, 1);
        lcd.clear();
        Vtidal_Seleccionado = Pote_Display;
        lcd.print(Vtidal_Seleccionado); // Escribimos el Mensaje en el LCD.
      }
      else
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Fuera de Rango"); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(0, 1);
        lcd.print("Intentelo"); // Escribimos el Mensaje en el LCD.
        delay(1500);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Volumen TIDAL [0-1500ml]"); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(0, 1);
        lcd.print("SEL:"); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(4, 1);
        lcd.print(Pote_Display); // Escribimos el Mensaje en el LCD.
        lcd.setCursor(7, 1);
        lcd.print("SET:");              // Escribimos el Mensaje en el LCD.
        lcd.print(Vtidal_Seleccionado); // Escribimos el Mensaje en el LCD.
      }
    }
    break;

  case 5:
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
  case 6:
    Contador_Pantalla = 0;

    break;
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

  //************************************************************************************************************************//
  // Comunicacion I2c entre Mega (Maestro) y Uno (Esclavo)                                                                  //
  // Datos Transmitidos: Velocidad de Inspiracion, Velocidad de Expiracion, Presion maxima y PEEP                           //
  //************************************************************************************************************************//
  //Solo se transmiten datos cuando hay algun cambio en algunos de los parametros.

  //

  if ((Velo_Inspiracion != Velo_Inspiracion_Anterior) || (Velo_Expiracion != Velo_Expiracion_Anterior) || (PMAX_Seleccionado != PMAX_Seleccionado_Anterior) || (PEEP_Seleccionado != PEEP_Seleccionado_Anterior) || (Vtidal_Seleccionado != Vtidal_Seleccionado_Anterior) || Cambio_Modo)
  // (Pulsador_Marcha_VALUE == digitalRead(19))
  {
    byte byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9, byte10, byte11, byte12, byte13, byte14, byte15, byte16, byte17, byte18;
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
    Wire.endTransmission(); // Termina a transmissão
  }

  Velo_Inspiracion_Anterior = Velo_Inspiracion;
  Velo_Expiracion_Anterior = Velo_Expiracion;
  PMAX_Seleccionado_Anterior = PMAX_Seleccionado;
  PEEP_Seleccionado_Anterior = PEEP_Seleccionado;
  Vtidal_Seleccionado_Anterior = Vtidal_Seleccionado;
  Cambio_Modo = false;
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
  Serial.print("------ PARADA ------");
}
