/* Código para mecanismo R2-D2 Dome
  // servocontrolador controlado a través de i2c
  // Te sugiero que te concentres en conectar un mecanismo primero, el Lifeform Scanner o Periscope, para aprender
  // cómo interactuará el código y los errores del interruptor de búsqueda de fallas
  // Cablee los componentes necesarios y mantenga la tensión de la correa en las poleas firme, pero no demasiado apretada,
  // asegúrese de que los elevadores se deslicen fácilmente antes de agregar la tensión de la correa
  // Ejecute este código en el monitor serial de su computadora mientras está conectado al Arduino Mega 2560,
  // Tasa de baudios 57600, para mostrar los finales de carrera ZBOT, ZTOP por ejemplo.
  // Activar los interruptores a mano para comprobar que están conectados al Arduino correctamente y el valor será
  // cambio de 1 a 0, o viceversa
  // Los activadores de botón tienen tres estados, recuento de botones=1, mover hacia arriba, recuento de botones=2, mover hacia abajo, recuento de botones=3, restablecer.
  // Agregue energía a la placa del controlador del motor y vea si cuando se presiona el botón, PIN 34 en el Arduino para
  // tierra,levanta la correa del motor hacia arriba o hacia abajo
  // Si se mueve en la dirección incorrecta, simplemente intercambie los 2 pines IN1 e IN2 del motor en el controlador del motor
  // Agregue un mecanismo a la vez para evitar tener que intentar resolver el complejo cableado más tarde
  // Abreviaturas de notación, estas se agregarán a las funciones como un prefijo para realizar un seguimiento de todas las entradas y salidas
  // P = periscopio
  // BM = Mal motivador
  // Z = Domo Zapper
  // LS = sable de luz
  // LF = Escáner de formas de vida
  // DS = Servidor de Bebidas*/
//Libraries to include
#include <Wire.h>
#include <VarSpeedServo.h>
#include <math.h>
#include <EnableInterrupt.h>
#include <Adafruit_PWMServoDriver.h>

//Tableros de llamadas para i2c
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Definir constantes y valores para permanecer igual
#define SERIAL_PORT_SPEED 57600 // Definir la velocidad de comunicación serial de salida del puerto

// nuestro servo # contador
uint8_t servonum = 0;

// Estos son los puntos finales del servo para los paneles circulares, ajuste para abrir y cerrar más o menos
#define BMSERVOMIN 300 // ajuste para la posición del panel del Domo (150 - 600)
#define BMSERVOMAX 450 // ajuste para la posición del panel del Domo
#define ZSERVOMIN 300 // ajuste para la posición del panel del Domo (150 - 600)
#define ZSERVOMAX 450 // ajuste para la posición del panel del Domo
#define LSSERVOMIN 300 // ajuste para la posición del panel del Domo (150 - 600)
#define LSSERVOMAX 450 // ajuste para la posición del panel del Domo
#define LFSERVOMIN 300 // ajuste para la posición del panel del Domo (150 - 600)
#define LFSERVOMAX 450 // ajuste para la posición del panel del Domo
#define DSSERVOMIN 300 // ajuste para la posición del panel del Domo (150 - 600)
#define DSSERVOMAX 450 // ajuste para la posición del panel del Domo 
#define ZAPSERVOMIN 268 // ajuste la posición de brazo Zapper alto (150 - 600)//350
#define ZAPSERVOMAX 170 // ajuste la posición de brazo Zapper bajo//180
#define DRINKSERVOMIN 200 // / ajustar la posición del servidor de bebidas hacia abajo (150 - 600)
#define DRINKSERVOMAX 320 // ajustar la posición del servidor de bebidas hacia arriba
#define PTURNSERVOMIN 100 // ajustar la posición de giro Periscopio (150 - 600)//180
#define PTURNSERVOMAX 500 // ajuste la posición de giro de Periscopio//450
#define ZAPTURNSERVOMIN 500 // ajuste la posición de giro de Dome Zapper (150 - 600) //500//
#define ZAPTURNSERVOMAX 100 // ajuste la posición de giro de Dome Zapper//200
#define LFTURNSERVOMIN 100 // ajuste la posición de giro de Dome radar (150 - 600)//180
#define LFTURNSERVOMAX 500 // ajuste la posición de giro de Dome radar

//Configuraciones LED altas y bajas a continuación, conectadas a la línea Gnd y Signal en el PCA9685
#define ZLEDSERVOMIN 200 // ajuste la posición de giro de Dome Zapper(150 - 600)
#define ZLEDSERVOMAX 500 // ajuste la posición de giro de Dome Zapper
#define BMLEDSERVOMIN 200 // ajuste la posición de giro de Dome Zapper (150 - 600)
#define BMLEDSERVOMAX 500 // ajuste la posición de giro de Dome Zapper

//Conductores de motor
#define PIN1 2 // Controlador de motor de periscopio IN1
#define PIN2 3 // Controlador de motor de periscopio IN2
#define BMIN1 4 // Controlador de motor Bad Motivator IN1
#define BMIN2 5 // Controlador de motor Bad Motivator IN2
#define ZIN1 6 // Controlador de motor Zapper IN1
#define ZIN2 7 // Controlador de motor Zapper IN2
#define LSIN1 8 // Controlador de motor SableLuz IN1
#define LSIN2 9 // Controlador de motor SableLuz IN2
#define LFIN1 10 // Controlador de motor Lifeform IN1
#define LFIN2 11 // Controlador de motor Lifeform IN2
#define DSIN1 12 // Controlador de motor de servidor de bebidas IN1
#define DSIN2 13 // Controlador de motor de servidor de bebidas IN2

//Interruptores de límite para subir y bajar
#define PTop 23 //Interruptor de límite superior de periscopio
#define PBot 22 //Interruptor de límite inferior de periscopio
#define BMTop 25 //Interruptor de límite superior de Bad Motivator 
#define BMBot 24 //Interruptor de límite inferior de Bad Motivator 
#define ZTop 27 //Interruptor de límite superior de Zapper 
#define ZBot 26 //Interruptor de límite inferior de Zapper 
#define LSTop 29 //Interruptor de límite superior de Lightsaber 
#define LSBot 28 //Interruptor de límite inferior de Lightsaber
#define LFTop 31 //Interruptor de límite superior de Lifeform 
#define LFBot 30 //Interruptor de límite inferior de Lifeform 
#define DSTop 32 //Interruptor de límite superior de Drink Server 
#define DSBot 33 //Interruptor de límite inferior de Drink Server 

// Estos pines están conectados a tierra para disparar, esto se puede hacer con un botón o un código agregado más tarde para hacer este control remoto desde otra fuente
#define buttonPin 38 // pasador de botón para activar el mecanismo de elevación del zapper del domo
#define buttonPin1 34 //pasador de botón para activar el mecanismo de elevación del periscope del domo
#define buttonPin2 37 //pasador de botón para activar el mecanismo de elevación del scanner del domo
#define buttonPin3 35 //pasador de botón para activar el mecanismo de elevación del Lightsaber lift del domo
#define buttonPin4 36 //pasador de botón para activar el mecanismo de elevación del Bad Motivator del domo
#define buttonPin5 39 //pasador de botón para activar el mecanismo de elevación del Drink Server del domo 

// temporizadores
unsigned long currentMillis; // referencia de reloj en marcha
unsigned long zappreviousMillis; // zapper tiempo almacenamiento
long zapinterval = 30; // tiempo entre destellos
unsigned long zapturnpreviousMillis; // zapper almacenado tiempo de giro
long zapturninterval = 2000; // tiempo entre giros dome zapper
long zapturninterval2 = 4000; // más tiempo entre el giro del zapper del domo
static unsigned long zap_time_stopped; // es hora de que el zapper del domo se quede despierto
unsigned long zappause; // tiempo para almacenar la posición superior del zapper
byte zapflashcount = 0; // Contador para flashes Dome Zapper
byte zapperturncount = 0; // contador de vueltas de cúpula zapper

unsigned long bmpreviousMillis; // bad motivator tiempo almacenamiento
long bminterval = 200; // tiempo entre destellos

unsigned long pturnpreviousMillis; // Periscope tiempo almacenamiento
long pturninterval = 1200; // time between posotional turns
byte pturncount = 0; // count how many turns for the periscope

unsigned long lfturnpreviousMillis; // Lifeform scanner tiempo almacenamiento
long lfturninterval = 600; // tiempo entre turnos posocionales
byte lfturncount = 0; // cuenta cuantas vueltas da el radar
const long lfledinterval = 500; // tiempo entre parpadeos del led del escáner Lifeform
unsigned long lfledpreviousmillis = 0; // almacenamiento para flashes led

unsigned long dspreviousMillis; // drink server tiempo almacenamiento
long dsinterval = 4000; // time between drink server going up

// button timer for debounce if required
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 50; // the debounce time; increase if the output flickers

// sates to select from the different case functions
static enum {ZAP_MOVE_TOP, ZAP_TOP} statezapup; //zapper arriba
static enum {ZAP_MOVE_BOT, ZAP_BOT} statezapdown; // zapper down
static enum {P_MOVE_TOP, P_TOP} statepup; //periscope arriba
static enum {P_MOVE_BOT, P_BOT} statepdown; //periscope abajo
static enum {LF_MOVE_TOP, LF_TOP} statelfup; //lifeform scanner arriba
static enum {LF_MOVE_BOT, LF_BOT} statelfdown; //lifeform scanner abajo
static enum {BM_MOVE_TOP, BM_TOP} statebmup; //Bad Motivator arriba
static enum {BM_MOVE_BOT, BM_BOT} statebmdown; //Bad Motivator abajo
static enum {LS_MOVE_TOP, LS_TOP} statelsup; //Lightsaber arriba
static enum {LS_MOVE_BOT, LS_BOT} statelsdown; //Lightsaber abajo
static enum {DS_MOVE_TOP, DS_TOP} statedsup; //Drink Server arriba
static enum {DS_MOVE_BOT, DS_BOT} statedsdown; //Drink Server abajo

int statez; //levantamiento y giro del brazo zapper
int statezl; //zapper led
int statept; //periscope giro
int statelf; //lifeform scanner
int statelft; //lifeform scanner giro
int statebml; //bad motivator led
int stateds; //drink server arm

// establecer números enteros
// almacenamiento para valores de interruptor de límite
int PTopVal = LOW;
int PBotVal = LOW;
int BMTopVal = LOW;
int BMBotVal = LOW;
int ZTopVal = LOW;
int ZBotVal = LOW;
int LSTopVal = LOW;
int LSBotVal = LOW;
int LFTopVal = LOW;
int LFBotVal = LOW;
int DSTopVal = LOW;
int DSBotVal = LOW;

// botón de entrada
int buttonPushCounter = 0; // Contador para el número de pulsaciones de botones.
int buttonPushCounter1 = 0; // Contador para el número de pulsaciones de botones.
int buttonPushCounter2 = 0; // Contador para el número de pulsaciones de botones.
int buttonPushCounter3 = 0; // Contador para el número de pulsaciones de botones.
int buttonPushCounter4 = 0; // Contador para el número de pulsaciones de botones.
int buttonPushCounter5 = 0; // Contador para el número de pulsaciones de botones.
int ledState = LOW; // el estado actual del pin de salida
int ledState1 = LOW; // el estado actual del pin de salida
int ledState2 = LOW; // el estado actual del pin de salida
int buttonState = 0; // el estado actual del botón para Dome Zapper
int buttonState1 = 0; // el estado actual del botón para Periscope
int buttonState2 = 0; // el estado actual del botón para Lifeform Scanner
int buttonState3 = 0; // el estado actual del botón para Bad Motivator
int buttonState4 = 0; // el estado actual del botón para Lightsaber
int buttonState5 = 0; // el estado actual del botón para Drink Server
int lastButtonState = 0; // la lectura anterior del pin de entrada
int lastButtonState1 = 0; // la lectura anterior del pin de entrada
int lastButtonState2 = 0; // la lectura anterior del pin de entrada
int lastButtonState3 = 0; // la lectura anterior del pin de entrada
int lastButtonState4 = 0; // la lectura anterior del pin de entrada
int lastButtonState5 = 0; // la lectura anterior del pin de entrada


void setup() {

  Serial.begin(SERIAL_PORT_SPEED);// serial communicationpwm.begin();
  pwm.begin();
  pwm.setPWMFreq(50); // standard for analog servos

  //output pins
  pinMode(PIN1, OUTPUT);
  pinMode(PIN2, OUTPUT);
  pinMode(BMIN1, OUTPUT);
  pinMode(BMIN2, OUTPUT);
  pinMode(ZIN1, OUTPUT);
  pinMode(ZIN2, OUTPUT);
  pinMode(LSIN1, OUTPUT);
  pinMode(LSIN2, OUTPUT);
  pinMode(LFIN1, OUTPUT);
  pinMode(LFIN2, OUTPUT);
  pinMode(DSIN1, OUTPUT);
  pinMode(DSIN2, OUTPUT);

  //pinMode(ledPin, SALIDA); // pin led en el arduino para probar los pines de entrada
  //usando un interruptor de límite, el pin va a Normalmente abierto y el común va a tierra, el valor lee BAJO cuando el interruptor está cerrado
  pinMode(PTop, INPUT_PULLUP);
  pinMode(PBot, INPUT_PULLUP);
  pinMode(BMTop, INPUT_PULLUP);
  pinMode(BMBot, INPUT_PULLUP);
  pinMode(ZTop, INPUT_PULLUP);
  pinMode(ZBot, INPUT_PULLUP);
  pinMode(LSTop, INPUT_PULLUP);
  pinMode(LSBot, INPUT_PULLUP);
  pinMode(DSTop, INPUT_PULLUP);
  pinMode(DSBot, INPUT_PULLUP);
  pinMode(LFTop, INPUT_PULLUP);
  pinMode(LFBot, INPUT_PULLUP);
  
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin1, INPUT);
  pinMode(buttonPin2, INPUT);
  pinMode(buttonPin3, INPUT);
  pinMode(buttonPin4, INPUT);
  pinMode(buttonPin5, INPUT);

  // Escriba los pines del motor a nivel bajo para que no se inicien al encender
  digitalWrite(PIN1, LOW);
  digitalWrite(PIN2, LOW);
  digitalWrite(BMIN1, LOW);
  digitalWrite(BMIN2, LOW);
  digitalWrite(ZIN1, LOW);
  digitalWrite(ZIN2, LOW);
  digitalWrite(LSIN1, LOW);
  digitalWrite(LSIN2, LOW);
  digitalWrite(LFIN1, LOW);
  digitalWrite(LFIN2, LOW);
  digitalWrite(DSIN1, LOW);
  digitalWrite(DSIN2, LOW);
  //digitalWrite(ledPin, LOW); // apagamos el led
  
  //Establecer los servos en sus posiciones de inicio
  servoSetup(); // ver la función y el final del código
  
  delay(1000); // esperar a que todo llegue a sus posiciones en el encendido
}

//Código principal aquí

void loop() {

  // restablecer temporizadores
  currentMillis = millis();
  readlimits(); //leer y almacenar los valores de los interruptores de límite alto o bajo, función más abajo en el código
  buttonState = digitalRead(buttonPin);  // disparador principal para entradas de botón
  buttonState1 = digitalRead(buttonPin1); // disparador principal para entradas de botón
  buttonState2 = digitalRead(buttonPin2); // disparador principal para entradas de botón
  buttonState3 = digitalRead(buttonPin3); // disparador principal para entradas de botón
  buttonState4 = digitalRead(buttonPin4); // disparador principal para entradas de botón
  buttonState5 = digitalRead(buttonPin5); // disparador principal para entradas de botón

  if (buttonState != lastButtonState) {
    if (buttonState == HIGH) {
      buttonPushCounter++;
    }
  }
  if (buttonState1 != lastButtonState1) {
    if (buttonState1 == HIGH) {
      buttonPushCounter1++;
    }
  }
  if (buttonState2 != lastButtonState2) {
    if (buttonState2 == HIGH) {
      buttonPushCounter2++;
    }
  }
  if (buttonState3 != lastButtonState3) {
    if (buttonState3 == HIGH) {
      buttonPushCounter3++;
    }
  }
  if (buttonState4 != lastButtonState4) {
    if (buttonState4 == HIGH) {
      buttonPushCounter4++;
    }
  }
  if (buttonState5 != lastButtonState5) {
    if (buttonState5 == HIGH) {
      buttonPushCounter5++;
    }
  }

  //dome zapper
  if (buttonPushCounter == 1) {
    DomeZapperUp();
    if (ZTopVal == LOW && ZBotVal == HIGH) {
      DomeZapper();
    }
  }
  else if (buttonPushCounter == 2) {
    pwm.setPWM(5, 0, ZAPTURNSERVOMIN); //turn the zapper arm to original position
    pwm.setPWM(4, 0, ZAPSERVOMIN); //lower the arm
    pwm.setPWM(8, 0, 4096); // sets the led LOW
    DomeZapperDown();
  }
  else {
    buttonPushCounter = 0;
    statezapup = ZAP_MOVE_TOP; // reset states for next lift sequence
    statezapdown = ZAP_MOVE_BOT;
    statez = 1;
    statezl = 0;
  }

  //periscopio
  if (buttonPushCounter1 == 1) {
    PeriscopeUp();
    if (PTopVal == LOW && PBotVal == HIGH) {
      PeriscopeTurn();

    }
  }
  else if (buttonPushCounter1 == 2) {
    pwm.setPWM(6, 0, PTURNSERVOMIN); //periscope turn to original lift position
    PeriscopeDown();
  }
  else {
    buttonPushCounter1 = 0;
    statepup = P_MOVE_TOP;
    statepdown = P_MOVE_BOT;
    statept = 0;
  }
  
  //Lifeform scanner
  if (buttonPushCounter2 == 1) {
    LifeformUp();
    if (currentMillis - lfledpreviousmillis >= lfledinterval) {
      pwm.setPWM(10, 4096 , 0); //Lifeform LED HIGH
      lfledpreviousmillis = currentMillis;
    }
    else {
      pwm.setPWM(10, 0, 4096); //Lifeform LED LOW
    }
    if (LFTopVal == LOW && LFBotVal == HIGH) {
      LFTurn();

    }
  }
  else if (buttonPushCounter2 == 2) {
    pwm.setPWM(7, 0, LFTURNSERVOMIN); //Lifeform turn to original position
    LifeformDown();
  }
  else {
    buttonPushCounter2 = 0;
    statelfup = LF_MOVE_TOP;
    statelfdown = LF_MOVE_BOT;
    statelft = 0;
  }
  // Bad Motivator
  if (buttonPushCounter3 == 1) {
    BadMotivatorUp();
    pwm.setPWM(9, 4096, 0);
  }
  else if (buttonPushCounter3 == 2) {
    BadMotivatorDown();
    pwm.setPWM(9, 0, 4096);
  }
  else {
    buttonPushCounter3 = 0;
    statebmup = BM_MOVE_TOP;

    statebmdown = BM_MOVE_BOT;
  }
  
  //Lightsaber Lifter
  if (buttonPushCounter4 == 1) {
    LightsaberUp();
  }
  else if (buttonPushCounter4 == 2) {
    LightsaberDown();
  }
  else {
    buttonPushCounter4 = 0;
    statelsup = LS_MOVE_TOP;
    statelsdown = LS_MOVE_BOT;
  }
  
  //Drink Server
  if (buttonPushCounter5 == 1) {
    DrinkServerUp();
  }
  else if (buttonPushCounter5 == 2) {
    if (DSTopVal == LOW && DSBotVal == HIGH) {
      pwm.setPWM(11, 0, DRINKSERVOMAX); //raise the drink server arm
    }
  }
  else if (buttonPushCounter5 == 3) {
    if (DSTopVal == LOW && DSBotVal == HIGH) {
      pwm.setPWM(11, 0, DRINKSERVOMIN); //lower the drink server arm
    }
  }
  else if (buttonPushCounter5 == 4) {
    DrinkServerDown();
  }
  else {
    buttonPushCounter5 = 0;
    statedsup = DS_MOVE_TOP; // reset states for next lift sequence
    statedsdown = DS_MOVE_BOT;
  }
  SerialOut(); // print to serial all values for testing
  lastButtonState = buttonState; // reset the input button
  lastButtonState1 = buttonState1; // reset the input button
  lastButtonState2 = buttonState2; // reset the input button
  lastButtonState3 = buttonState3; // reset the input button
  lastButtonState4 = buttonState4; // reset the input button
  lastButtonState5 = buttonState5; // reset the input button
}

// Funciones a continuación aquí para ver las características

void DomeZapperUp() { // this function is for the dome zapper
  
  switch (statezapup) {
    case ZAP_MOVE_TOP:
      if (ZTopVal != LOW) {
        if (digitalRead(ZTop) == HIGH && digitalRead(ZBot) == LOW) {
          pwm.setPWM(1, 0, ZSERVOMAX); // open the pie panel
        }
        digitalWrite(ZIN1, HIGH); //turn the dc motor on
        digitalWrite(ZIN2, LOW);
        statezapup = ZAP_TOP;
      }
      break;
    case ZAP_TOP:
      if (ZTopVal == LOW) {
        digitalWrite(ZIN1, LOW); //turn the motor off
        digitalWrite(ZIN2, LOW); //turn the motor off
        //pwm.setPWM(4, 0, ZAPSERVOMAX); //lift zapper arm
      }
      break;
  }
}

void DomeZapperDown() { // this function is for the dome zapper
  switch (statezapdown) {
    case ZAP_MOVE_BOT:
      if (ZBotVal != LOW) {
        digitalWrite(ZIN1, LOW); //turn the dc motor on
        digitalWrite(ZIN2, HIGH);
        statezapdown = ZAP_BOT;
      }
      break;
    case ZAP_BOT:
      if (ZBotVal == LOW) {
        digitalWrite(ZIN1, LOW); //turn the dc motor on
        digitalWrite(ZIN2, LOW);
        if (digitalRead(ZBot) == LOW && digitalRead(ZTop) == HIGH) {
          pwm.setPWM(1, 0, ZSERVOMIN); // close the pie panel
        }
      }

      break;
  }
}
void DomeZapper() { //lift zapper arm servo, flash light, rotate to new position and flash, return to first position, arm down
  switch (statez) {
    case 1:
      currentMillis = millis();
      pwm.setPWM(4, 0, ZAPSERVOMAX); //lift zapper arm
      if (currentMillis - zapturnpreviousMillis >= zapturninterval2) {
        statez = 2;
        zapturnpreviousMillis = currentMillis;
      }
      break;
    case 2:
      currentMillis = millis();
      pwm.setPWM(5, 0, ZAPTURNSERVOMAX); //turn zapper arm
      ZapLed(); // flash the LED
      if (currentMillis - zapturnpreviousMillis >= zapturninterval2) {
        statez = 3;
        zapturnpreviousMillis = currentMillis;
      }
      break;
    case 3:
      currentMillis = millis();
      if (currentMillis - zapturnpreviousMillis >= zapturninterval2) {
        pwm.setPWM(5, 0, ZAPTURNSERVOMIN); //turn
        statez = 0;
        zapturnpreviousMillis = currentMillis;
      }
      break;
  }
}

void ZapLed() {
  switch (statezl)
  {
    case 0:
      currentMillis = millis();
      pwm.setPWM(8, 4096, 0); // sets the led HIGH from the PCA9685
      if (currentMillis - zappreviousMillis >= zapinterval) {
        zappreviousMillis = currentMillis;
        statezl = 1;
      }
      break;
    case 1:
      currentMillis = millis();
      pwm.setPWM(8, 0, 4096); // sets the led LOW from the PCA9685
      if (currentMillis - zappreviousMillis >= zapinterval) {
        zappreviousMillis = currentMillis;
        statezl = 2;
      }
      break;
    case 2:
      currentMillis = millis();
      zapflashcount++;
      if (zapflashcount == 80) {
        statezl = 3;
        zapflashcount = 0;
      }
      else {
        statezl = 0;
      }
      break;
  }
}

void PeriscopeUp() { //button tirggered Lift periscope, flash lights, rotate back and forwards, when button triggered again lower again in home position turn off lights
  switch (statepup) {
    case P_MOVE_TOP:
      if (PTopVal != LOW) {
        digitalWrite(PIN1, HIGH); //turn the dc motor on
        digitalWrite(PIN2, LOW);
        statepup = P_TOP;
      }
      break;
    case P_TOP:
      if (PTopVal == LOW) {
        digitalWrite(PIN1, LOW); //turn the motor off
        digitalWrite(PIN2, LOW); //turn the motor off
      }
      break;
  }
}

void PeriscopeDown() { // this function lowers the periscope
  switch (statepdown) {
    case P_MOVE_BOT:
      if (PBotVal != LOW) {
        digitalWrite(PIN1, LOW); //turn the dc motor on
        digitalWrite(PIN2, HIGH);
        statepdown = P_BOT;
      }
      break;
    case P_BOT:
      if (PBotVal == LOW) {
        digitalWrite(PIN1, LOW); //turn the dc motor on
        digitalWrite(PIN2, LOW);
      }
      break;
  }
}
void PeriscopeTurn() { // turn the periscope back and forwards
  switch (statept)
  {
    case 0:
      currentMillis = millis();
      if (currentMillis - pturnpreviousMillis >= pturninterval) {
        pwm.setPWM(6, 0, PTURNSERVOMAX); //turn
        pturnpreviousMillis = currentMillis;
        statept = 1;
      }
      break;
    case 1:
      currentMillis = millis();
      if (currentMillis - pturnpreviousMillis >= pturninterval) {
        pwm.setPWM(6, 0, PTURNSERVOMIN); //turn
        pturnpreviousMillis = currentMillis;
        statept = 2;
      }
      break;
    case 2:
      currentMillis = millis();
      pturncount++;
      if (pturncount == 3) {
        statept = 3;
        pturncount = 0;
      }
      else {
        statept = 0;
      }
      break;
  }
}

void LifeformUp() { //button tirggered Lift Lifeform Scanner, when button triggered again lower in home position
  switch (statelfup) {
    case LF_MOVE_TOP:
      if (LFTopVal != LOW) {
        if (digitalRead(LFTop) == HIGH && digitalRead(LFBot) == LOW) {
          pwm.setPWM(3, 0, LFSERVOMAX); // open the pie panel
        }
        digitalWrite(LFIN1, HIGH); //turn the dc motor on
        digitalWrite(LFIN2, LOW);
        statelfup = LF_TOP;
      }
      break;
    case LF_TOP:
      if (LFTopVal == LOW) {

        digitalWrite(LFIN1, LOW); //turn the motor off
        digitalWrite(LFIN2, LOW); //turn the motor off
      }
      break;
  }
}

void LifeformDown() { // this function lowers the Lifeform scanner
  switch (statelfdown) {
    case LF_MOVE_BOT:
      if (LFBotVal != LOW) {
        digitalWrite(LFIN1, LOW); //turn the dc motor on
        digitalWrite(LFIN2, HIGH);
        statelfdown = LF_BOT;
      }
      break;
    case LF_BOT:
      if (LFBotVal == LOW) {
        digitalWrite(LFIN1, LOW); //turn the dc motor on
        digitalWrite(LFIN2, LOW);
        if (digitalRead(LFBot) == LOW && digitalRead(LFTop) == HIGH) {
          pwm.setPWM(3, 0, LFSERVOMIN); // close the pie panel
        }

      }
      break;
  }
}

void LFTurn() { // turn the Lifeform scanner back and forwards
  switch (statelft)
  {
    case 0:
      currentMillis = millis();
      if (currentMillis - lfturnpreviousMillis >= lfturninterval) {
        pwm.setPWM(7, 0, LFTURNSERVOMAX); //Lifeform turn
        lfturnpreviousMillis = currentMillis;
        statelft = 1;
      }
      break;
    case 1:
      currentMillis = millis();
      if (currentMillis - lfturnpreviousMillis >= lfturninterval) {
        pwm.setPWM(7, 0, LFTURNSERVOMIN); //Lifeform turn

        lfturnpreviousMillis = currentMillis;
        statelft = 2;
      }
      break;
    case 2:
      currentMillis = millis();
      lfturncount++;
      if (lfturncount == 6) {
        statelft = 3;
        lfturncount = 0;
      }
      else {
        statelft = 0;
      }
      break;
  }
}

void BadMotivatorUp() { //button tirggered Lift Bad Motivator, when button triggered again lower in home position
  switch (statebmup) {

    case BM_MOVE_TOP:
      if (BMTopVal != LOW) {
        if (digitalRead(BMTop) == HIGH && digitalRead(BMBot) == LOW) {
          pwm.setPWM(0, 0, BMSERVOMAX); // open the pie panel
        }
        digitalWrite(BMIN1, HIGH); //turn the dc motor on
        digitalWrite(BMIN2, LOW);
        statebmup = BM_TOP;
      }
      break;
    case BM_TOP:
      if (BMTopVal == LOW) {
        digitalWrite(BMIN1, LOW); //turn the motor off
        digitalWrite(BMIN2, LOW); //turn the motor off
      }
      break;
  }
}

void BadMotivatorDown() { // this function lowers the Bad Motivator
  switch (statebmdown) {
    case BM_MOVE_BOT:
      if (BMBotVal != LOW) {

        digitalWrite(BMIN1, LOW); //turn the dc motor on
        digitalWrite(BMIN2, HIGH);
        statebmdown = BM_BOT;
      }
      break;
    case BM_BOT:
      if (BMBotVal == LOW) {
        digitalWrite(BMIN1, LOW); //turn the dc motor on
        digitalWrite(BMIN2, LOW);
        if (digitalRead(BMBot) == LOW && digitalRead(BMTop) == HIGH) {
          pwm.setPWM(0, 0, BMSERVOMIN); // close the pie panel
        }
      }
      break;
  }
}

//el botón se activó Levantar sable de luz, cuando el botón se activó de nuevo, baje
void LightsaberUp() {
  switch (statelsup) {
    case LS_MOVE_TOP:
      if (LSTopVal != LOW) {
        if (digitalRead(LSTop) == HIGH && digitalRead(LSBot) == LOW) {
          pwm.setPWM(2, 0, LSSERVOMAX); // abrimos panel circular
        }

        digitalWrite(LSIN1, HIGH); // enciende el motor de corriente continua
        digitalWrite(LSIN2, LOW);
        statelsup = LS_TOP;
      }
      break;
    case LS_TOP:
      if (LSTopVal == LOW) {
        digitalWrite(LSIN1, LOW); //apagamos el motor
        digitalWrite(LSIN2, LOW); //apagamos el motor
      }
      break;
  }
}

// esta función baja el sable de luz
void LightsaberDown() {
  switch (statelsdown) {
    case LS_MOVE_BOT:
      if (LSBotVal != LOW) {
        digitalWrite(LSIN1, LOW); // enciende el motor de corriente continua
        digitalWrite(LSIN2, HIGH);
        statelsdown = LS_BOT;
      }
      break;

    case LS_BOT:
      if (LSBotVal == LOW) {
        digitalWrite(LSIN1, LOW);// enciende el motor de corriente continua
        digitalWrite(LSIN2, LOW);
        if (digitalRead(LSBot) == LOW && digitalRead(LSTop) == HIGH) {
          pwm.setPWM(2, 0, LSSERVOMIN); // abre el panel circular
        }
      }
      break;
  }
}

// esta función es para el servidor de bebidas
void DrinkServerUp() {
  switch (statedsup) {
    case DS_MOVE_TOP:
      if (DSTopVal != LOW) {
        if (digitalRead(DSTop) == HIGH && digitalRead(DSBot) == LOW) {
          pwm.setPWM(12, 0, DSSERVOMAX);// abre el panel circular
        }
        digitalWrite(DSIN1, HIGH); // enciende el motor de corriente continua
        digitalWrite(DSIN2, LOW);
        statedsup = DS_TOP;

      }
      break;
    case DS_TOP:
      if (DSTopVal == LOW) {
        digitalWrite(DSIN1, LOW); //apagamos el motor
        digitalWrite(DSIN2, LOW); //apagamos el motor
      }
      break;
  }
}
void DrinkServerDown() { // esta función es para el servidor de bebidas
  switch (statedsdown) {
    case DS_MOVE_BOT:
      if (DSBotVal != LOW) {
        digitalWrite(DSIN1, LOW); // enciende el motor de corriente continua
        digitalWrite(DSIN2, HIGH);
        statedsdown = DS_BOT;
      }
      break;
    case DS_BOT:
      if (DSBotVal == LOW) {
        digitalWrite(DSIN1, LOW); // enciende el motor de corriente continua
        digitalWrite(DSIN2, LOW);
        if (digitalRead(DSBot) == LOW && digitalRead(DSTop) == HIGH) {
          pwm.setPWM(12, 0, DSSERVOMIN); // cerrar el panel circular
        }
      }
      break;
  }
}
void readlimits() { //esta función lee todos los interruptores de límite y almacena sus valores para compararlos al final
  //se detiene en el ciclo principal, solo se escribe aquí para mantener el loop code clean
  PBotVal = digitalRead(PBot);
  PTopVal = digitalRead(PTop);
  BMTopVal = digitalRead(BMTop);
  BMBotVal = digitalRead(BMBot);
  ZBotVal = digitalRead(ZBot);
  ZTopVal = digitalRead(ZTop);
  LSBotVal = digitalRead(LSBot);
  LSTopVal = digitalRead(LSTop);
  LFBotVal = digitalRead(LFBot);
  LFTopVal = digitalRead(LFTop);
  DSBotVal = digitalRead(DSBot);
  DSTopVal = digitalRead(DSTop);
}
void servoSetup() {

  //ServoStartPositions();
  pwm.setPWM(0, 0, BMSERVOMIN); //bad motivator pie panel
  pwm.setPWM(1, 0, ZSERVOMIN); //zapper pie panel
  pwm.setPWM(2, 0, LSSERVOMIN); //lightsaber pie panel
  pwm.setPWM(3, 0, LFSERVOMIN); //lifeform pie panel
  pwm.setPWM(4, 0, ZAPSERVOMIN);// zapper arm
  pwm.setPWM(5, 0, ZAPTURNSERVOMIN); //zapper arm turn
  pwm.setPWM(6, 0, PTURNSERVOMIN); //periscope turn
  pwm.setPWM(7, 0, LFTURNSERVOMIN); //Lifeform turn
  pwm.setPWM(8, 0, 4096); //Zapper LED low
  pwm.setPWM(9, 0, 4096); //Bad Motiviator LED low
  pwm.setPWM(10, 0, 4096); //Lifeform LED low
  pwm.setPWM(11, 0, DRINKSERVOMIN); //Drink Server Arm
  pwm.setPWM(12, 0, DSSERVOMIN); //Drink Server pie panel
}

// Función para mostrar valores para la prueba, desmarque cualquier valor que no quiera o quiera ver
void SerialOut() {

  Serial.print("B:"); Serial.print(buttonPushCounter); Serial.print("\t");
  Serial.print("B1:"); Serial.print(buttonPushCounter1); Serial.print("\t");
  Serial.print("B2:"); Serial.print(buttonPushCounter2); Serial.print("\t");
  Serial.print("B3:"); Serial.print(buttonPushCounter3); Serial.print("\t");
  Serial.print("B4:"); Serial.print(buttonPushCounter4); Serial.print("\t");
  Serial.print("B5:"); Serial.print(buttonPushCounter5); Serial.print("\t");
  Serial.print("St:"); Serial.print(buttonState); Serial.print("\t");
  Serial.print("St1:"); Serial.print(buttonState1); Serial.print("\t");
  Serial.print("St2:"); Serial.print(buttonState2); Serial.print("\t");
  Serial.print("St3:"); Serial.print(buttonState3); Serial.print("\t");
  Serial.print("St4:"); Serial.print(buttonState4); Serial.print("\t");
  Serial.print("St5:"); Serial.print(buttonState5); Serial.print("\t");
  //Serial.print("ZBot:"); Serial.print(ZBotVal); Serial.print("\t");
  //Serial.print("ZTop:"); Serial.print(ZTopVal); Serial.print("\t");
  //Serial.print("PBot:"); Serial.print(PBotVal); Serial.print("\t");
  //Serial.print("PTop:"); Serial.print(PTopVal); Serial.print("\t");
  //Serial.print("BMBot:"); Serial.print(BMBotVal); Serial.print("\t");
  //Serial.print("BMTop:"); Serial.print(BMTopVal); Serial.print("\t");
  //Serial.print("LFBot:"); Serial.print(LFBotVal); Serial.print("\t");
  //Serial.print("LFTop:"); Serial.print(LFTopVal); Serial.print("\t");
  //Serial.print("LSBot:"); Serial.print(LSBotVal); Serial.print("\t");
  //Serial.print("LSTop:"); Serial.print(LSTopVal); Serial.print("\t");
  //Serial.print("DSBot:"); Serial.print(DSBotVal); Serial.print("\t");
  //Serial.print("DSTop:"); Serial.print(DSTopVal); Serial.print("\t");
  Serial.print("statezl:"); Serial.print(statezl); Serial.print("\t");
  Serial.print("Millis:"); Serial.print(currentMillis); Serial.println("\t");

}
