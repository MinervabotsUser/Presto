//////// Inclusão de bibliotecas /////
#include "seguidor.h"
#include "PIDControler.h"
#include <QTRSensors.h>
#include <Button.h>
#include <SoftwareSerial.h>

#include "controlboard.h"
#include "sensor.h"
#include "sensorboard.h"
#include "ultrasonic.h"
#include "robot.h"
/***********************************/

#define DEBUG 1

////////////////////////////////////////////////////////////////////
///////////////////////// OBJETOS //////////////////////////////////
////////////////////////////////////////////////////////////////////

/************ Botao **************************/
Button button = Button(BUTTON_PIN, PULLDOWN);

/************ Sensor de Reflectancia *********/
QTRSensorsAnalog sensorDeLinha((unsigned char[]){A5, A4, A3, A2, A1, A0}, NUM_SENSORS);
unsigned int sensorLinha[NUM_SENSORS];

/************ Controle do Robo ***************/
Robot robot(R_ENABLE, R_MOTOR_1, R_MOTOR_2, L_ENABLE, L_MOTOR_1, L_MOTOR_2);
PIDControler pid;

/************ Bluetooth *********************/
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

////////////////////////////////////////////////////////////////////
///////////////////////// VARIAVEIS GLOBAIS ////////////////////////
////////////////////////////////////////////////////////////////////
unsigned long tempoInterrupcao;
unsigned long ultimaParada;
unsigned long tempoUltimoComando;

/************ Entrada serial *********************/
char inByte;

/************ Controle PID **********************/
long int lastRun;
float kp;
float ki;
float kd;
float linearSpeed;
/************ Parametros Robo *******************/

volatile boolean following = false;

////////////////////////////////////////////////////////////////////
///////////////////////// SETUP ////////////////////////////////////
////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  BluetoothSerial.begin(9600);
  while (!Serial); // wait for serial port to connect. Needed for Leonardo and Micro only
  
  Serial.println("Pressione botao para iniciar!");
  while (!button.isPressed());
  delay(500);
  
  // Calibrar sensores de reflectancia
  autoCalibrate(VEL_CALIBRACAO,TEMPO_CALIBRACAO);
  
  // Interrupcao usada para permitir parar robo usando o botão enquanto ele estiver seguindo linha.
  attachInterrupt(BUTTON_INTERRUPT, changeMode, RISING);

  //////////// Iniciando PID //////////////
  /* TODO: carregar e gravar constantes na EEPROM*/
  kp = 1;
  ki = 0;
  kd = 0;
  linearSpeed = 60.;
  pid.Init(kp, ki, kd);
  /////////////////////////////////////////
  
  digitalWrite(LED_PIN,HIGH);
  delay(800);
  digitalWrite(LED_PIN,LOW);
  
  //////////// Iniciar seguindo linha /////
  following = true;
  delay(1000);
}

////////////////////////////////////////////////////////////////////
///////////////////////// LOOP /////////////////////////////////////
////////////////////////////////////////////////////////////////////

void loop() {
  lastRun = millis();
  /////////////////////////// Modo Controlado ///////////////////////
  
  while (!following) {
    BluetoothSerial.listen();
    // Parar robo apos chamada da interrupcao
    if(millis() - tempoInterrupcao < 500)
      robot.stop();
    if(button.isPressed()){
      following = true;
      delay(500);
      pid.reset();
      break;
    }
    while (BluetoothSerial.available() > 0) {
      tempoUltimoComando = millis();
      inByte = BluetoothSerial.read();
      Serial.print("Bluetooth: ");
      Serial.println(inByte);
      switch (inByte) {
        case 'F':
        robot.moveForward();
        break;
        case 'B':
        robot.moveBackwards();
        break;
        case 'L':
        robot.rotateAntiClockwise();
        break;
        case 'R':
        robot.rotateClockwise();
        break;
        case 'S':
        robot.stop();
        break;
        case 'I':
        robot.moveForwardRight();
        break;
        case 'J':
        robot.moveBackwardsRight();
        break;
        case 'G':
        robot.moveForwardLeft();
        break;
        case 'H':
        robot.moveBackwardsLeft();
        break;
        case 'W':
        followLine();
        break;
        case 'C':
        autoCalibrate(VEL_CALIBRACAO, TEMPO_CALIBRACAO);
        break;
        case 'K':
        setPidConstants();
        break;
        default:  //Get velocity
        if(inByte=='q'){
          robot.setSpeed(100);
        }else if((inByte >= 48) && (inByte <= 57)){
            //Chars '0' - '9' have an integer equivalence of 48 - 57, accordingly.
            //Subtracting 48 changes the range from 48-57 to 0-9
            //Multiplying by 10 changes the range from 0-9 to 0-90.
          robot.setSpeed((inByte - 48)*10);
        }
        break;

      }
    }
  }
  //////////////////////////// Modo Autônomo /////////////////////////////////
  while (following == true) {
    if(button.isPressed()){
      robot.stop();
      following = false;
      delay(500);
      break;
    }
    
    
    // Obter a posição da linha e normalizar entre -10 e 10
    float erro = 10.0 * ((sensorDeLinha.readLine(sensorLinha, QTR_EMITTERS_ON, WHITE_LINE) - CENTER_POSITION) / (CENTER_POSITION*1.0));
    
    // Computar resposta do PID
    float output = pid.run(erro);
    
    // Girar motores

    if (erro < 0){
      // Curva Pra Esquerda 
      output = abs(output);

      if (linearSpeed - output > 0){
        robot.setPWM(linearSpeed + output,linearSpeed - output);
      }
      else{
        robot.setPWM(linearSpeed + output,abs(linearSpeed - output),false,true); // L_reverse true
      }
    }
    else{
      // Curva pra direita
      if (linearSpeed - output > 0){
        robot.setPWM(linearSpeed - output,linearSpeed + output);
      }
      else{
        robot.setPWM(abs(linearSpeed - output),linearSpeed + output,true,false); // R_reverse true
      }
    }
    
    ///////////////////////////////////////////////////////////
    // Os valores de debug no modo seguidor de linha         //
    // só serão exibidos caso a macro DEBUG esteja definida  //
    ///////////////////////////////////////////////////////////
    
    #ifdef DEBUG
    Serial.print(sensorLinha[0]);
    Serial.print("\t");
    Serial.print(sensorLinha[1]);
    Serial.print("\t");
    Serial.print(sensorLinha[2]);
    Serial.print("\t");
    Serial.print(sensorLinha[3]);
    Serial.print("\t");
    Serial.print(sensorLinha[4]);
    Serial.print("\t");
    Serial.print(sensorLinha[5]);
    Serial.print("\t");
    Serial.print("Erro: ");
    Serial.print(erro);
    Serial.print("\t");

    Serial.print("PID out: ");
    Serial.print(output);
    Serial.print("\t");
    
    if (erro < 0) {
      Serial.print("Curva Pra Esquerda");
      Serial.print("\t");
    }else{
      Serial.print("Curva Pra Direita");
      Serial.print("\t");
    }

    Serial.print("Speed - out: ");
    Serial.print(linearSpeed - output);
    Serial.print("\t");
    Serial.println();
    #endif
    //checkBatery();
  }
  
}
////////////////////////////////////////////////////////////////////
///////////////////////// DEFINIÇÃO DE FUNÇÕES /////////////////////
////////////////////////////////////////////////////////////////////

///////////////////// Controle dos Motores /////////////////////////
void changeMode() {
// função para parar alternar entre
  if (millis() - tempoInterrupcao < 1000)
    return;
  if (following) {
    robot.stop();
    following = false;
  }
  else {
    pid.reset();
    following = true;
  }
  tempoInterrupcao = millis();
}

// Função para robo começar seguir linha
void followLine(){
  following = true;
  pid.reset();
}


void setPidConstants() { 
// Função para configurar constantes do PID usando entrada serial  
  robot.stop();
  Serial.println("== Seting PID constants ==");
  String inputString;
  boolean settingUp = true;
  while (settingUp) {
    BluetoothSerial.listen();
    while (BluetoothSerial.available() > 0) {
      char inByte = BluetoothSerial.read();
      // Formato da string de configuracao do PID: KP<Kp>I<Ki>D<Kd>M<linearSpeed>
      if ((inByte != 'P') && (inByte != 'I') && (inByte != 'D') && (inByte != 'M'))
        inputString += inByte;
      else {
        inputString += '\0';
        char carray[inputString.length() + 1]; //determine size of the array
        inputString.toCharArray(carray, sizeof(carray)); //put readStringinto an array
        switch (inByte) {
          case 'P':
          Serial.println(inputString);
          if (inputString == "")
            break;
          else
              kp = atof(carray); //convert the array into a float
            break;
            case 'I':
            Serial.println(inputString);
            if (inputString != "")
              ki = atof(carray);
            break;
            case 'D':
            Serial.println(inputString);
            if (inputString != "")
              kd = atof(carray);
            break;
            case 'M':
            Serial.println(inputString);
            if (inputString != "") {
              linearSpeed = atoi(carray);
            }
            settingUp = false;
            break;
          }
          inputString = "";
        }
      }
    }

    pid.setConstants(kp,  kd,  ki);
    Serial.println();
    Serial.print("Kp = ");
    Serial.println(kp);
    Serial.print("Ki = ");
    Serial.println(ki);
    Serial.print("Kd = ");
    Serial.println(kd);
    Serial.print("Max Speed = ");
    Serial.println(linearSpeed);
  }

  void autoCalibrate( int velocidade, int tempoCalibracao ) {
    Serial.println("Calibrando...");
    unsigned long tempoInicial = millis();

    while ((millis() - tempoInicial) < tempoCalibracao) {
      if(((millis() - tempoInicial) < tempoCalibracao / 4) || ((millis() - tempoInicial) > (3 * tempoCalibracao) / 4))
        robot.setPWM(velocidade, velocidade,true,false);
      else
        robot.setPWM(velocidade, velocidade,false,true);
      sensorDeLinha.calibrate();
    }
    robot.setPWM(-velocidade, velocidade);
    while(10.0 * ((sensorDeLinha.readLine(sensorLinha, QTR_EMITTERS_ON, WHITE_LINE) - CENTER_POSITION) / (CENTER_POSITION*1.0)) < 0);
    robot.stop();
    Serial.println("Calibrado");
  }

  void manualCalibrate(){
    while(!button.isPressed()){
      Serial.println("calibrando");
      sensorDeLinha.calibrate();
    }
    delay(500);
  }

  void checkBatery(){
    unsigned tensao = analogRead(SENSOR_BATERIA_PIN);
    Serial.println(tensao);

    if(tensao < 550 && tensao > 300){
      robot.stop();
      for(int i = 0; i < 4; i++){
        analogWrite(LED_PIN,255);
        delay(200);
        analogWrite(LED_PIN,0);
        delay(200);
        pid.reset();
      }
    } 
  }
