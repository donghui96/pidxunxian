#include <EEPROM.h>

#define FIRMWARE "LineFollow Robot V1.05\r\n"

int pos;
int adcal[5] = {150,150,200,150,150};
int a[5]; // analog value for each sensor
  
float lr_ratio = 1.2;
void doDcSpeed(int spdL, int spdR) {
  //spdR = -spdR;
  if (spdL < 0) {
    analogWrite(5, 0);
    analogWrite(6, -spdL);
  } else {
    analogWrite(5, spdL);
    analogWrite(6, 0);
  }

  if (spdR < 0) {
    analogWrite(9, 0);
    analogWrite(10, -spdR);
  } else {
    analogWrite(9, spdR);
    analogWrite(10, 0);
  }
}

int bias = 0;
int outlineCnt = 0;

void stateMachine(int a) {
  switch (a) {
    case B00000:
      outlineCnt++;
      break;
    case B11111:
      outlineCnt++;
      break;
    case B00010:
    case B00110:
      outlineCnt = 0;
      bias = 1;
      break;
    case B00001:
    case B00011:
      outlineCnt = 0;
      bias = 2;
      break;
    case B00100:
      outlineCnt = 0;
      bias = 0;
      break;
    case B01000:
    case B01100:
      outlineCnt = 0;
      bias = -1;
      break;
    case B10000:
    case B11000:
      outlineCnt = 0;
      bias = -2;
      break;
    default:
      //Serial.println(a,BIN);
      outlineCnt++;
      break;
  }
  if (outlineCnt > 10) {
    doDcSpeed(0,0);
  } else {
    float ff = 150;
    float ctrl = -calcPid(bias);
    doDcSpeed(ff+ctrl,ff-ctrl);
  }
}

float Kp = 110;
float Ki = 0.15; // 0.15
float Kd = 1200; //1200
float error, errorLast, erroInte;

float calcPid(float input) {
  float errorDiff;
  float output;
  error = error * 0.7 + input * 0.3; // filter
  //error = input;
  errorDiff = error - errorLast;
  erroInte = constrain(erroInte + error, -50, 50);
  output = Kp * error + Ki * erroInte + Kd * errorDiff;
  /*
  Serial.print(error); Serial.print(' ');
  Serial.print(erroInte); Serial.print(' ');
  Serial.print(errorDiff); Serial.print(' ');
  Serial.println(output);
  */
  errorLast = error;

  return output;
}

// todo: use analog average to determine the position of black line other than threshold check
int echoTrace() {
  int ret = 0;
  for (int i = 0; i < 5; i++) {
    a[i] = analogRead(A0 + i);
    //Serial.print(a[i]);Serial.print(",");
    if (a[i] < adcal[i]) ret += (0x1 << i);
  }
  //Serial.println(ret,BIN);
  return ret;
}

void echoVersion() {
  Serial.print("M0 ");
  Serial.print(FIRMWARE);
}

void doGetSensor(){
  Serial.print("M1 ");
  for(int i=0;i<5;i++){
    Serial.print(a[i]);
    Serial.print(" ");  
  }
  Serial.println(pos,BIN);
}

void doGetPid(){
  Serial.print("M2 ");
  Serial.print(Kp);Serial.print(" ");
  Serial.print(Ki);Serial.print(" ");
  Serial.println(Kd);
}

void doSetPid(char * cmd){
  char * tmp;
  char * str;
  str = tmp = cmd;
  while (str != NULL) {
    str = strtok_r(0, " ", &tmp);
    if (str[0] == 'P') {
      Kp = atof(str + 1);
    } else if (str[0] == 'I') {
      Ki = atof(str + 1);
    } else if (str[0] == 'D') {
      Kd = atof(str + 1);
    }
  }
}

void doGetThreshold(){
  Serial.print("M4 ");
  for(int i=0;i<5;i++){
    Serial.print(adcal[i]);
    Serial.print(" ");  
  }
  Serial.println("");
}

void doSetThreshold(char * cmd){
  sscanf(cmd, "%d %d %d %d %d\n", &adcal[0], &adcal[1], &adcal[2], &adcal[3], &adcal[4]);
}


void doSoftReset() {
  asm volatile ("  jmp 0");
}

void parseMcode(char * cmd) {
  int code;
  char * tmp;
  code = atoi(cmd);
  cmd = strtok_r(cmd, " ", &tmp);
  switch (code) {
    case 0:
      echoVersion();
      break;
    case 1:
      doGetSensor();
      break;
    case 2:
      doGetPid();
      break;
    case 3:
      doSetPid(tmp);
      break;
    case 4:
      doGetThreshold();
      break;
    case 5:
      doSetThreshold(tmp);
      break;
    case 999:
      doSoftReset();
      break;
  }
}

void parseCmd(char * cmd){
  if (cmd[0] == 'M') { // mcode
    parseMcode(cmd + 1);
  }
  Serial.println("OK");
}

void setup() {
  Serial.begin(115200);
  echoVersion();
}

char buf[64];
int8_t bufindex;

void loop() {
  delay(5);
  pos = echoTrace();
  stateMachine(pos);
  while (Serial.available()) {
    char c = Serial.read();
    buf[bufindex++] = c;
    if (c == '\n') {
      buf[bufindex] = '\0';
      parseCmd(buf);
      memset(buf, 0, 64);
      bufindex = 0;
    }
    if (bufindex >= 64) {
      bufindex = 0;
    }
  }
}
