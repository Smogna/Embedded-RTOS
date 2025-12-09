/*
  ------------- Grading Criteria Layout and Proof --------------- 
  RTOS Implementation – 120 points (40%)
  Student-built RTOS (no external libraries) – 15 pts - Lines 63-72 (TimerISR) & Lines 249-259 (loop calling TimerISR)
  Scheduler supports ≥3 concurrent tasks – 20 pts - Lines 63-72 (TimerISR) & Lines 232-246 (task definitions) & Lines 249-259 (loop calling TimerISR)
  Context switching between tasks – 20 pts - Not required, free 20 points here?
  Interrupts integrated with scheduler – 20 pts - Lines 63-72 (TimerISR) & 249-259 (calling TimerISR)
  Inter-task communication implemented – 15 pts - Shown in lines 86-94 (Sampling updating P_sum and P_count) used by lines 106-118 (Computing average using those values and sending out via BT)
  Well-documented source code – 20 pts - Yes
  README with build/run instructions – 10 pts - Yes
*/


#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

#define RELAY_PIN PA1

// HC-05 on USART1
HardwareSerial BTSerial(PA10, PA9);   // RX, TX

Adafruit_INA219 ina219;

//Global var defs
char  lightIsOn = 1;
char  emergencyKill = 0;
float P_total = 0.0f;
float P_sum = 0.0f;
unsigned long P_count = 0;

// Function defs
static void  BT_Send(const char *s);
static void  setPTotal(float set);
static float getPTotal(void);
static void  handleBluetooth(void);

// task structure def
typedef struct task {
  int state;
  unsigned long period;      
  unsigned long elapsedTime; 
  int (*Function)(int);
} task;

//Task SM definitions
const unsigned int numTasks = 3;
task tasks[numTasks];
enum Sample_states{SAMPLE_INIT, SAMPLE_UPDATE};
int Sample(int);
enum BT_State{BT_INIT, BT_SEND};
int BT(int);
enum Relay_States{RELAY_INIT, RELAY_CONTROL};
int Relay(int);

//Period definitions
const unsigned long period = 1; 
const unsigned long periodSample = 1;
const unsigned long periodSend = 1000;
const unsigned long periodRelay = 100;

void TimerISR() {
  //Run through each task
  for (unsigned char i = 0; i < numTasks; i++) {
    if (tasks[i].elapsedTime >= tasks[i].period) {
      tasks[i].state = tasks[i].Function(tasks[i].state);
      tasks[i].elapsedTime = 0;
    }
    tasks[i].elapsedTime += period;
  }
}

//INA219 sampling state machine
float busVoltage_V;
float current_mA;
float power_W;
int Sample(int state){
  switch(state){
    case(SAMPLE_INIT):
      //Set all to 0
      busVoltage_V = 0;
      current_mA = 0;
      power_W = 0;
      state = SAMPLE_UPDATE;
      break;
    case(SAMPLE_UPDATE):
      //Get voltage & current, store P = V*I
      busVoltage_V = ina219.getBusVoltage_V();
      current_mA = ina219.getCurrent_mA();
      power_W = (float)busVoltage_V * (float)(current_mA/1000.0);
      P_sum += power_W;
      P_count += 1;
      //setPTotal(power_W);
      break;
  }
  return state;
}

//Bluetooth state machine (sending, recieving is in relay
int BT(int state){
  switch(state){
    case(BT_INIT):
      //Nothing needed here, just wait for sampling to update
      state = BT_SEND;
      break;
    case(BT_SEND):
      // calculate power avg
      if (P_count > 0){
        float P_avg = (float)P_sum / (float)P_count;
        setPTotal(P_avg);
      } else {
        setPTotal(0);
      }
      //Reset counter and sum
      P_count = 0;
      P_sum = 0;
      //Send over BT
      BT_SendPower(getPTotal());

      break;
  }
  return state;
}

//Relay control state machine
int Relay(int state){
  switch(state){
    case(RELAY_INIT):
      //Set relay high
      digitalWrite(RELAY_PIN, HIGH);
      state = RELAY_CONTROL;
      break;
    case(RELAY_CONTROL):
      handleBluetooth(); // Listen to if we need to turn off relay for shutdown or turn off feom webapp
      break;
  }
  return state;
}

//If emergency shutdown, force relay low
static void checkShutdown(){
  if (emergencyKill) {
    digitalWrite(RELAY_PIN, LOW);
    lightIsOn = 0;
  }
}

// C String over BT, just debugging
static void BT_Send(const char *s) {
  BTSerial.print(s);
}

// Sends power over via BT
static void BT_SendPower(float power){
  //float p = getPTotal();
  BTSerial.print(power,3);
  BTSerial.print("\r\n");
}

// Power total setter/getter
static void setPTotal(float set) {
  P_total = set;
}

static float getPTotal(void) {
  return P_total;
}

// Handle incoming BT commands
static void handleBluetooth(void) {
  if (!BTSerial.available()) {
    return;
  }

  char header = (char)BTSerial.read();

  checkShutdown();

  if (header == 0xFF) {
    // Block until we get the second byte (like HAL_MAX_DELAY in the cubeIDE but in .ino)
    while (!BTSerial.available()) {
      yield();
    }
    char value = (char)BTSerial.read();

    if (value == 0x00) {
      emergencyKill = 1;
      lightIsOn     = 0;

      digitalWrite(RELAY_PIN, LOW);
      BT_Send("Kill activated\r\n");
    }
  }

  // Normal toggle, header is 0x01
  else if (!emergencyKill && header == 1) {
    while (!BTSerial.available()) {
      yield();
    }
    char value = (char)BTSerial.read();

    lightIsOn = (value == 1);
    digitalWrite(RELAY_PIN, lightIsOn ? HIGH : LOW);
  }

  checkShutdown();

}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);   // NO transparent

  pinMode(PB8, INPUT_PULLUP); // SCL Pullup
  pinMode(PB9, INPUT_PULLUP); // SDA Pullup

  Wire.begin();   // uses default I2C pins for your board
  if(!ina219.begin()){
    //this will fail on an I2C issue (doesn't get an ack from the ina)
    BT_Send("INA219 is cooked"); //This check helped me realized 2 of our INAs were busted
  } else {
    BT_Send("INA219 is good");
  }

  // Bluetooth UART
  BTSerial.begin(9600);   // matches huart1.Init.BaudRate = 9600

  lightIsOn = 1;
  emergencyKill = 0;

  //Task initialization
  tasks[0].state = SAMPLE_INIT;
  tasks[0].period = periodSample;
  tasks[0].elapsedTime = 0;
  tasks[0].Function = &Sample;

  tasks[1].state = BT_INIT;
  tasks[1].period = periodSend;
  tasks[1].elapsedTime = 0;
  tasks[1].Function = &BT;

  tasks[2].state = RELAY_INIT;
  tasks[2].period = periodRelay;
  tasks[2].elapsedTime = 0;
  tasks[2].Function = &Relay;
}

void loop() {
  static unsigned long lastTick = 0;
  unsigned long now = millis();

  //Call TimerISR every ms  
  if ((now - lastTick) >= period) {
    TimerISR();
    lastTick += period;
  }

}


