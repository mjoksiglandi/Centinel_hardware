#include <Arduino.h>
#include <Servo.h>
//#include <BluetoothSerial.h>

//BluetoothSerial SerialBT;

//HardwareSerial SerialPort(2); esp32
#define SerialPort Serial2

//reles
int releAgua = 20;
int releGel = 21;

/////servos
int cont = 0;

Servo servoPitch;
Servo servoZeta;
int pinServoPitch = 23;
int pinServoZeta = 22;

int posIniPitch = 70;
int posSenPitch = 145;
int posIniZeta = 10;
int posSenZeta = 130;

//boton
int btnPin = 6;
int valBtn;
bool banderaBoton = false;

////////////////doble nucleo
//TaskHandle_t Tarea1;

//sensor tritex datos
String data;
String sub;
float fsub;

/*
void loop_tarea1(void *pvParameters) {
  while (1) {
    if (SerialPort.available()) {
      data = SerialPort.readStringUntil('\n');
      sub = data.substring(8, 13);
      fsub = sub.toFloat();
      fsub = fsub / 1000;
    }
  }
}
*/

void lectura() {
  valBtn = digitalRead(btnPin);

  if (valBtn == 0) {
    banderaBoton = true;
  }

  if (banderaBoton == true) {
    if (SerialPort.available()) {
      data = SerialPort.readStringUntil('\n');
      sub = data.substring(8, 13);
      fsub = sub.toFloat();
      fsub = fsub / 1000;
    }
    cont++;
    //SerialBT.println(cont);
    if (cont == 1) {
      //SerialBT.println("bomba gel");
      digitalWrite(releGel, HIGH);
    }
    if (cont == 21) {
      //SerialBT.println("Comenzando Sensado");
      digitalWrite(releGel, LOW);
      servoPitch.write(posIniPitch);
      servoZeta.write(posIniZeta);
    }
    if (cont == 31) {
      //SerialBT.println("posSenPitch");
      servoPitch.write(posSenPitch);
    }
    if (cont == 41) {
      //SerialBT.println("posSenZeta");
      servoZeta.write(posSenZeta);
    }
    if (cont == 81) {
      //SerialBT.println("posIniZeta");
      servoZeta.write(posIniZeta);
    }
    if (cont == 101) {
      //SerialBT.println("posIniPitch");
      servoPitch.write(posIniPitch);
    }
    if (cont == 120) {
      digitalWrite(releAgua, HIGH);
    }
    if (cont == 140) {
      digitalWrite(releAgua, LOW);
      banderaBoton = false;
      cont = 0;
    }
  }
}

void setupSensorTritex() {
  //SerialPort.begin(38400, SERIAL_8N1, 16, 17);
  //SerialBT.begin("ESPTritexROS");
  SerialPort.begin(38400);

  pinMode(releAgua, OUTPUT);
  pinMode(releGel, OUTPUT);

  digitalWrite(releAgua, LOW);
  digitalWrite(releGel, LOW);

  /*
  xTaskCreatePinnedToCore(
    loop_tarea1,
    "Tarea1",
    10000,
    NULL,
    0,
    &Tarea1,
    0);
    */

  servoPitch.attach(pinServoPitch);
  servoZeta.attach(pinServoZeta);

  servoPitch.write(posIniPitch);
  servoZeta.write(posIniZeta);

  pinMode(btnPin, INPUT_PULLUP);
}
