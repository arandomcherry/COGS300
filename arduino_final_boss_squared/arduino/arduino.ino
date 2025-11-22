// Author: Winslow Flandre <i@winsloweric.com>
// Arduino Final Boss SQUARED. Refactored.
#include <WiFiS3.h>
#include <Servo.h>

// Motors, notice that there is a swap in drive() due to wiring mistake.
#define enA_1 10
#define enA_2 11
// Encoders, 1 is left, 2 is right
#define in1_1 2
#define in2_1 4
#define in1_2 7
#define in2_2 8
// Servo
#define servopin 12
// Ultrasonics, 1 is leftside, 2 is frontside.
#define trig1 3
#define trig2 6
#define echo1 5
#define echo2 9
// IR Sensors
#define leftir A0
#define midir A1
#define rightir A2

// API Endpoint
IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GW(192, 168, 4, 1);
IPAddress AP_SN(255, 255, 255, 0);

const int SERIAL_BAUD = 115200;

const char* ssid = "Arduino Final Boss²™";
const char* pass = "letsalllovelain";
WiFiServer server(80);
const char page[] PROGMEM = R"HTML(
<meta charset=utf-8>
<style>
input{width:48px}
body{margin:8px;font:14px sans-serif}
.p{display:inline-block;padding:6px;border:1px solid #ccc;border-radius:4px}
.g{display:grid;grid-template-columns:repeat(4,52px);gap:4px;margin-bottom:4px}
button{height:40px;border:1px solid #ccc;background:#f8f8f8;cursor:pointer}
button.on{background:#cfe8ff;border-color:#7aa}
#state{margin-top:8px;padding:4px;border:1px solid #eee;background:#fafafa;font:12px monospace;max-height:160px;overflow:auto;white-space:pre}
</style>
<script>
const C={W:1,A:4,S:2,D:5,Q:8,E:9,STOP:7,M:255,N:254,B:101,X:100,Z:50},p=new Set,m=new Set("WASD"),B={};
function S(b){fetch("/api/cmd?b="+b).catch(()=>{})}
function hi(k,on){const b=B[k];if(b)b.classList[on?"add":"remove"]("on")}
function d(k){if(!p.has(k)){p.add(k);hi(k,1);S(C[k])}}
function u(k){if(!p.has(k))return;p.delete(k);hi(k,0);if(m.has(k)){for(const x of p)if(m.has(x))return;S(C.STOP)}}
function refreshState(){
  fetch("/api/state").then(r=>r.json()).then(s=>{
    const el=document.getElementById("state");if(!el)return;
    el.textContent=JSON.stringify(s,null,2);
  }).catch(()=>{});
}
function swapScan(){
  fetch("/api/swap").then(r=>r.json()).then(s=>{
    const el=document.getElementById("state");if(!el)return;
    el.textContent=JSON.stringify(s,null,2);
  }).catch(()=>{});
}
addEventListener("keydown",e=>{const k=e.key.toUpperCase();if(C[k]){e.preventDefault();d(k)}})
addEventListener("keyup",e=>{const k=e.key.toUpperCase();if(C[k]){e.preventDefault();u(k)}})
addEventListener("pointerdown",e=>{const k=e.target.dataset.k;if(k&&C[k]){e.preventDefault();d(k)}})
addEventListener("pointerup",e=>{const k=e.target.dataset.k;if(k&&C[k]){e.preventDefault();u(k)}})
addEventListener("DOMContentLoaded",()=>{
  for(const x of document.querySelectorAll("[data-k]"))B[x.dataset.k]=x;

  const r=document.getElementById("refresh");
  if(r)r.onclick=e=>{e.preventDefault();refreshState()};

  const swap=document.getElementById("swap");
  if(swap)swap.onclick=e=>{e.preventDefault();swapScan()};

  const servoGo=document.getElementById("servoGo");
  if(servoGo)servoGo.onclick=e=>{
    e.preventDefault();
    const v=parseInt(document.getElementById("servoIndex").value,10);
    if(Number.isNaN(v))return;
    fetch("/api/servo?i="+v).catch(()=>{});
  };

  const driveGo=document.getElementById("driveGo");
  if(driveGo)driveGo.onclick=e=>{
    e.preventDefault();
    const v=parseInt(document.getElementById("driveIndex").value,10);
    if(Number.isNaN(v))return;
    fetch("/api/driveIndex?i="+v).catch(()=>{});
  };
});
</script>
<p>Z→Main/Auto,M→Wall,N→Obj,B→Line,X→Tournament</p>
<div class=p>
  <div class=g>
    <button data-k=W>W</button><button data-k=A>A</button><button data-k=S>S</button><button data-k=D>D</button>
    <button data-k=STOP>■</button><button data-k=M>M</button><button data-k=N>N</button><button data-k=B>B</button>
    <button data-k=Z>Z</button><button data-k=X>X</button>
  </div>
  <button id=refresh>⟳ State</button>
  <button id=swap>⟳ Swapscan</button>

  <div style="margin-top:4px">
    <label>Servo index:
      <input id="servoIndex" type="number" min="0" max="8">
    </label>
    <button id="servoGo">Set servo</button>
  </div>

  <div style="margin-top:4px">
    <label>Drive index:
      <input id="driveIndex" type="number" min="0" max="8">
    </label>
    <button id="driveGo">Run drive</button>
  </div>
</div>
<pre id=state></pre>
)HTML";


// The state that the car is in,
// if AUTONOMOUS_SEQ is on, the car would assume tournament and execute state transition check.
enum MetaState {
  MANUAL,
  AUTONOMOUS_SEQ,
  AUTONOMOUS
};

// Specific autonomous state currently running
enum AutonomousState {
  WALL,
  OBJECT,
  LINE
};

MetaState systemState;
AutonomousState autoState;

// Possible moving state, BACKWARD is not used.
// Only updated under autonomous mode
enum MovState {
  FORWARD,
  ROTLEFT,
  ROTRIGHT,
  BACKWARD
};

MovState prevState = FORWARD;
MovState currState = FORWARD;

Servo servo1;  // The servo for the front ultrasonic sonar

const int numPosition = 9;        // For object tracking. Number of positions to check and navigate.
volatile int servoDirection = 0;  // Current direction for servo (front ultrasonic) to take.
int servoStep;                    // The amount of angle for each step for object tracking

// All parameters needed for driving
struct MotorParam {
  PinStatus motor1In1;
  PinStatus motor1In2;
  PinStatus motor2In1;
  PinStatus motor2In2;
  int motor1Power;
  int motor2Power;
};

// Translation table for motor control parameter when object tracking.
struct MotorParamDelayed {
  MotorParam params;
  int delay;
};
MotorParamDelayed objTrackingTranslationTable[numPosition][2];
double objDistance[numPosition];
double objProbability[numPosition];

// Wall-following PD Controller parameters
int ePrev;       // e[k-1], e[k] should be measured by loop
float kP = 8;  // K_p
float kD = 2;  // K_d
int baseSpeed = 120;
int tS = 50;    // 0.05s
int dRef = 20;  // 30cm
int dK;
int uK;

void setup() {
  Serial.begin(SERIAL_BAUD);

  if (WiFi.beginAP(ssid, pass) != WL_AP_LISTENING) {
    Serial.println("AP start failed");
    while (true)
      ;
  }
  delay(10);
  servo1.attach(servopin);
  servo1.write(180);  // To very left for wall following detection.
  delay(10);
  servoStep = 180 / (numPosition - 1);

  for (int i = 0; i < numPosition; i++) {
    objDistance[i] = 0;
  }

  systemState = MANUAL;
  autoState = LINE;
  objTrackingTranslationTable[0][0] = { { HIGH, LOW, LOW, HIGH, 160, 160 }, 400 };
  objTrackingTranslationTable[0][1] = { { LOW, HIGH, LOW, HIGH, 133, 133 }, 200 };
  objTrackingTranslationTable[1][0] = { { HIGH, LOW, LOW, HIGH, 160, 160 }, 280 };
  objTrackingTranslationTable[1][1] = { { LOW, HIGH, LOW, HIGH, 133, 133 }, 200 };
  objTrackingTranslationTable[2][0] = { { HIGH, LOW, LOW, HIGH, 160, 160 }, 220 };
  objTrackingTranslationTable[2][1] = { { LOW, HIGH, LOW, HIGH, 133, 133 }, 200 };
  objTrackingTranslationTable[3][0] = { { HIGH, LOW, LOW, HIGH, 160, 160 }, 170 };
  objTrackingTranslationTable[3][1] = { { LOW, HIGH, LOW, HIGH, 133, 133 }, 200 };
  objTrackingTranslationTable[4][0] = { { LOW, HIGH, LOW, HIGH, 133, 133 }, 200 };
  objTrackingTranslationTable[4][1] = { { LOW, LOW, LOW, LOW, 0, 0 }, 0 };  // TODO: optimization to skip 0 timeout
  objTrackingTranslationTable[5][0] = { { LOW, HIGH, HIGH, LOW, 160, 160 }, 100 };
  objTrackingTranslationTable[5][1] = { { LOW, HIGH, LOW, HIGH, 133, 133 }, 200 };
  objTrackingTranslationTable[6][0] = { { LOW, HIGH, HIGH, LOW, 160, 160 }, 160 };
  objTrackingTranslationTable[6][1] = { { LOW, HIGH, LOW, HIGH, 133, 133 }, 200 };
  objTrackingTranslationTable[7][0] = { { LOW, HIGH, HIGH, LOW, 160, 160 }, 230 };
  objTrackingTranslationTable[7][1] = { { LOW, HIGH, LOW, HIGH, 133, 133 }, 200 };
  objTrackingTranslationTable[8][0] = { { LOW, HIGH, HIGH, LOW, 160, 160 }, 300 };
  objTrackingTranslationTable[8][1] = { { LOW, HIGH, LOW, HIGH, 133, 133 }, 200 };

  pinMode(enA_1, OUTPUT);
  pinMode(enA_2, OUTPUT);
  pinMode(in1_1, OUTPUT);
  pinMode(in2_1, OUTPUT);
  pinMode(in1_2, OUTPUT);
  pinMode(in2_2, OUTPUT);
  pinMode(in1_2, OUTPUT);
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(leftir, INPUT);
  pinMode(midir, INPUT);
  pinMode(rightir, INPUT);
  // System all ready.
  server.begin();
}

void loop() {
  if (systemState == MANUAL) {
    // Do nothing.
  } else {  // System is at least in an auto state, SEQ mode only enable automatic state transition.
    switch (autoState) {
      case LINE:
        if (lineFollowingFrame()) {
          autoState = WALL;
        }
        break;
      case WALL:
        if (wallFollowingFrame()) {
          autoState = OBJECT;
          servo1.write(0);
        }
        break;
      case OBJECT:
        if (objectFollowingFrame()) {
          systemState = MANUAL;
          autoState = WALL;
          blinkBuiltinLED();
        }
        break;
      default:
        Serial.println("Impossibility");
        blinkBuiltinLED();
    }
  }
  WiFiClient client = server.available();
  if (client) {
    wifiHandler(client);
  }
}

void blinkBuiltinLED() {
  for (int i = 0; i < 20; i++) {
    digitalWrite(13, HIGH);
    delay(15);
    digitalWrite(13, LOW);
    delay(15);
  }
  return;
}

boolean fineturnDirection(bool direction) {  // 0 = left, 1 = right
  //WiFiClient client = server.available();
  stopDrive();
  delay(30);
  int counter = 0;  // Force quit on 200000;
  while (true) {
    counter++;
    if (counter >= 30000) {
      goForward();
      delay(5);
      stopDrive();
      return 0;
    }
    if (!direction) {
      rotLeft();
      delay(5);
      stopDrive();
      currState = ROTLEFT;
    } else {
      rotRight();
      delay(5);
      stopDrive();
      currState = ROTRIGHT;
    }
    int leftJudge = digitalRead(leftir);
    int midJudge = digitalRead(midir);
    int rightJudge = digitalRead(rightir);

    if (leftJudge == 1 && midJudge == 0 && rightJudge == 1) {  // back on line
      stopDrive();
      int score = 0;
      for (int i = 0; i < 4; i++) {
        int leftJudgeTmp = digitalRead(leftir);
        int midJudgeTmp = digitalRead(midir);
        int rightJudgeTmp = digitalRead(rightir);
        if (leftJudgeTmp == 1 && midJudgeTmp == 0 && rightJudgeTmp == 1) {
          score++;
        }
        delay(4);
      }
      if (score >= 3) {
        goForward();
        return false;
      } else if (leftJudge == 1 && midJudge == 0 && rightJudge == 0) {
        if (!direction) {
          goForward();
          delay(20);
          stopDrive();
          continue;
        } else {
          continue;
        }
      } else if (leftJudge == 0 && midJudge == 0 && rightJudge == 1) {
        if (!direction) {
          continue;
        } else {
          goForward();
          delay(20);
          stopDrive();
          continue;
        }
      } else if (leftJudge == 0 && midJudge == 0 && rightJudge == 0) {
        goForward();
        delay(20);
        stopDrive();
      } else {
        if (currState == FORWARD) {
          goForward();
        } else if (currState == ROTLEFT) {
          rotLeft();
        } else if (currState == ROTRIGHT) {
          rotRight();
        }
        continue;
      }
    } else if (midJudge == 1 && leftJudge == 1 && rightJudge == 1) {
      // if (systemState == AUTONOMOUS_SEQ && wallFollowingCriteria()) {
      //   digitalWrite(13, HIGH);
      //   return true;
      // } else {
      continue;
      // }
    }
  }
}

void rotLeft() {
  drive(LOW, HIGH, HIGH, LOW, 0, 82);
}

void rotRight() {
  drive(HIGH, LOW, LOW, HIGH, 82, 0);
}

boolean lineFollowingFrame() {
  int leftJudge = digitalRead(leftir);
  int midJudge = digitalRead(midir);
  int rightJudge = digitalRead(rightir);

  if (leftJudge == 1 && midJudge == 0 && rightJudge == 1) {
    prevState = currState;
    currState = FORWARD;
    goForward();
    delay(17);
    stopDrive();
    delay(2);
  } else if (leftJudge == 0 && midJudge == 0 && rightJudge == 1) {
    prevState = currState;
    fineturnDirection(0);
  } else if (leftJudge == 0 && midJudge == 1 && rightJudge == 1) {
    goForward();
    delay(6);
    stopDrive();
    delay(2);
    prevState = currState;
    fineturnDirection(0);
  } else if (leftJudge == 1 && midJudge == 0 && rightJudge == 0) {
    prevState = currState;
    fineturnDirection(1);
  } else if (leftJudge == 1 && midJudge == 1 && rightJudge == 0) {
    goForward();
    delay(6);
    stopDrive();
    delay(2);
    prevState = currState;
    fineturnDirection(1);
  } else if (leftJudge == 1 && rightJudge == 1 && midJudge == 1) {
    // Out of line, recover, and potentially we want to check for wall following now
    if (systemState == AUTONOMOUS_SEQ && wallFollowingCriteria()) {
      return true;
    } else {
      // Not following wall and out of line
      stopDrive();
      delay(10);
      if (prevState == FORWARD) {
        rotLeft();  // TODO: better return logic;
        delay(20);
      } else if (prevState == ROTLEFT) {
        rotRight();
        delay(20);
      } else if (prevState == ROTRIGHT) {
        rotLeft();
        delay(20);
      }
    }
  } else {  // 000 010
    if (prevState == FORWARD) {
      delay(10);
      goForward();
    } else if (prevState == ROTLEFT) {
      delay(10);
      rotLeft();
    } else if (prevState == ROTRIGHT) {
      delay(10);
      rotRight();
    }
    goForward();
    delay(5);
  }
  return false;
}

boolean wallFollowingFrame() {
  double frontLeftReading = read_ultrasonic(trig2, echo2);
  double rightReading = read_ultrasonic(trig1, echo1);
  int leftJudge = digitalRead(leftir);
  int midJudge = digitalRead(midir);
  int rightJudge = digitalRead(rightir);

  float e = dRef - rightReading;
  if (frontLeftReading < 20) {
    rotLeft();
    delay(10);
    return false;
  } else {
    float de = (eCurr - ePrev) / tS;
    float u_raw = kP * e + kD * de;

    if (u_raw > 40)  u_raw = 40;
    if (u_raw < -40) u_raw = -40;

    left_pwm  = constrain(left_pwm,  0, 255);
    right_pwm = constrain(right_pwm, 0, 255);

    drive(LOW, HIGH, LOW, HIGH, left_pwm, right_pwm);
  }
  if (objectFollowingCriteria(leftJudge, midJudge, rightJudge, frontLeftReading, rightReading)) {
    stopDrive();
    delay(10);
    goForward();
    delay(500);
    stopDrive();
    return true;
  } else {
    return false;
  }
}

boolean wallFollowingCriteria() {
  // Front ultrasonic should point to left
  double frontLeftReading = read_ultrasonic(trig2, echo2);
  double rightReading = read_ultrasonic(trig1, echo1);

  if (frontLeftReading + rightReading < 70) {
    return true;
  } else {
    return false;
  }
}

boolean objectFollowingFrame() {
  // Sweeping
  servo1.write(0);
  delay(500);
  for (int i = 0; i < numPosition; i++) {
    int angle = i * servoStep;
    servo1.write(angle);
    delay(200);
    objDistance[i] = read_ultrasonic(trig2, echo2, 50000UL);
    if (objDistance[i] < 6) {
      return true;
    }
  }
  double maxVal = arr_maxi(objDistance, numPosition);
  if (maxVal < 0.0001) maxVal = 1.0;
  arr_div(objDistance, numPosition, maxVal);
  arr_mirror1(objDistance, objProbability, numPosition);
  int bestIndex = arr_maxi(objProbability, numPosition);
  if (objTrackingTranslationTable[bestIndex][0].delay != 0) {
    delayedDrive(objTrackingTranslationTable[bestIndex][0]);
  }
  if (objTrackingTranslationTable[bestIndex][1].delay != 0) {
    delayedDrive(objTrackingTranslationTable[bestIndex][1]);
  }
  stopDrive();
  delay(10);
  return false;
}

boolean objectFollowingCriteria(int IR1, int IR2, int IR3, int sonic1, int sonic2) {
  if ((ir1 == 0 || ir2 == 0 || ir3 == 0) && sonic1 > 100) { 
    // hit marker and front sonic reads high
    return true;
  } else {
    return false;
  }
}

void wifiHandler(WiFiClient client) {
  String req = client.readStringUntil('\r');
  client.readStringUntil('\n');

  if (req.startsWith("GET /api/cmd")) {
    int bIndex = req.indexOf("b=");
    if (bIndex > 0) {
      int val = req.substring(bIndex + 2).toInt();
      if (val >= 0 && val <= 255) {
        switch (val) {
          case 50:
            if (systemState == MANUAL) {
              systemState = AUTONOMOUS;
            } else {
              stopDrive();
              systemState = MANUAL;
            }
            break;
          case 1:  //W
            goForward();
            break;
          case 4:  //A
            drive(LOW, HIGH, LOW, LOW, 150, 150);
            break;
          case 2:  //S
            goBackward();
            break;
          case 5:  //D
            drive(LOW, LOW, LOW, HIGH, 150, 150);
            break;
          case 8:
            rotLeft();
            break;
          case 9:
            rotRight();
            break;
          case 255:  //Wall following
            stopDrive();
            autoState = WALL;
            break;
          case 254:  //Object following
            servo1.write(0);
            stopDrive();
            autoState = OBJECT;
            delay(100);
            break;
          case 101:  //Line following
            stopDrive();
            autoState = LINE;
            break;
          case 100:  //Tournament
            stopDrive();
            if (systemState == AUTONOMOUS_SEQ) {
              systemState = MANUAL;
            } else {
              autoState = LINE;
              systemState = AUTONOMOUS_SEQ;
            }
            break;
          case 7:
            stopDrive();
            break;
          default:
            break;
        }
      }
    }
    client.println("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nOK");
  } else if (req.startsWith("GET /api/state")) {
    int leftJudge = digitalRead(leftir);
    int midJudge = digitalRead(midir);
    int rightJudge = digitalRead(rightir);
    double frontDistance = read_ultrasonic(trig2, echo2);
    double sideDistance = read_ultrasonic(trig1, echo1);
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close\r\n");
    client.print("{\"systemState\":");
    client.print((int)systemState);
    client.print(",\"autoState\":");
    client.print((int)autoState);
    client.print(",\"currState\":");
    client.print((int)currState);
    client.print(",\"prevState\":");
    client.print((int)prevState);
    client.print(",\"irLeft\":");
    client.print(leftJudge);
    client.print(",\"irMid\":");
    client.print(midJudge);
    client.print(",\"irRight\":");
    client.print(rightJudge);
    client.print(",\"frontDistance\":");
    client.print(frontDistance);
    client.print(",\"sideDistance\":");
    client.print(sideDistance);
    client.print("}");
  } else if (req.startsWith("GET /api/swap")) {
    servo1.write(0);
    delay(500);
    for (int i = 0; i < numPosition; i++) {
      int angle = i * servoStep;
      servo1.write(angle);
      delay(200);
      objDistance[i] = read_ultrasonic(trig2, echo2, 50000UL);
    }
    double maxVal = arr_maxi(objDistance, numPosition);
    if (maxVal < 0.0001) maxVal = 1.0;
    arr_div(objDistance, numPosition, maxVal);
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close\r\n");
    client.print("{\"result\":\"");
    for (int i = 0; i < numPosition; i++) {
      client.print(objDistance[i]);
      client.print(" ");
    }
    client.print("\"}");
  } else if (req.startsWith("GET /api/servo")) {
    int iPos = req.indexOf("i=");
    int idx = 0;
    if (iPos > 0) {
      idx = req.substring(iPos + 2).toInt();
    }
    if (idx < 0) idx = 0;
    if (idx >= numPosition) idx = numPosition - 1;

    int angle = idx * servoStep;
    servo1.write(angle);

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close\r\n");
    client.print("{\"idx\":");
    client.print(idx);
    client.print(",\"angle\":");
    client.print(angle);
    client.print("}");
  } else if (req.startsWith("GET /api/driveIndex")) {
    int iPos = req.indexOf("i=");
    int idx = 0;
    if (iPos > 0) {
      idx = req.substring(iPos + 2).toInt();
    }
    if (idx < 0) idx = 0;
    if (idx >= numPosition) idx = numPosition - 1;

    delayedDrive(objTrackingTranslationTable[idx][0]);
    delayedDrive(objTrackingTranslationTable[idx][1]);

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close\r\n");
    client.print("{\"idx\":");
    client.print(idx);
    client.print("}");
  } else {  // root page
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close\r\n");
    client.print(page);
  }
  client.stop();
}

void delayedDrive(MotorParamDelayed d) {
  paramDrive(d.params);
  delay(d.delay);
  stopDrive();
  return;
}

void paramDrive(MotorParam d) {
  drive(d.motor1In1, d.motor1In2, d.motor2In1, d.motor2In2, d.motor1Power, d.motor2Power);
  return;
}
void drive(PinStatus p1, PinStatus p2, PinStatus p3, PinStatus p4, int power1, int power2) {
  analogWrite(enA_1, power1);
  analogWrite(enA_2, power2);

  digitalWrite(in1_1, p3);
  digitalWrite(in2_1, p4);

  digitalWrite(in1_2, p1);
  digitalWrite(in2_2, p2);
  return;
}

double read_ultrasonic(int trigPin, int echoPin, unsigned long timeout) {
  digitalWrite(trigPin, LOW);   // start low to ensure no pulse is sent
  delayMicroseconds(5);         // ensure 5 microseconds of no signal to avoid interference
  digitalWrite(trigPin, HIGH);  // start pulse high
  delayMicroseconds(10);        // continue for 10 microseconds
  digitalWrite(trigPin, LOW);   // stop pulse

  // Measure length of time before pulse comes in
  double duration = pulseIn(echoPin, HIGH, timeout);

  // Convert the time into a distance
  double cm = (duration / 2) / 29.1;  // Divide by 29.1 or multiply by 0.0343

  if (duration == 0) {
    return 9999.0;
  }

  return cm;
}

double read_ultrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);   // start low to ensure no pulse is sent
  delayMicroseconds(5);         // ensure 5 microseconds of no signal to avoid interference
  digitalWrite(trigPin, HIGH);  // start pulse high
  delayMicroseconds(10);        // continue for 10 microseconds
  digitalWrite(trigPin, LOW);   // stop pulse

  // Measure length of time before pulse comes in
  double duration = pulseIn(echoPin, HIGH, 18000UL);

  // Convert the time into a distance
  double cm = (duration / 2) / 29.1;  // Divide by 29.1 or multiply by 0.0343

  if (duration == 0) {
    return 9999.0;
  }

  return cm;
}
// Predefined driving parameters
void stopDrive() {
  drive(LOW, LOW, LOW, LOW, 0, 0);
}
void goForward() {
  drive(LOW, HIGH, LOW, HIGH, 152, 152);
}
void goBackward() {
  drive(HIGH, LOW, HIGH, LOW, 120, 120);
}

// Division over all elements
void arr_div(double* arr, size_t n, double a) {
  for (int i = 0; i < n; i++) {
    arr[i] = arr[i] / a;
  }
}
// Return the maximum item index.
int arr_maxi(const double* arr, size_t n) {
  int idx = 0;
  double mx = arr[0];
  for (size_t i = 1; i < n; ++i) {
    if (arr[i] > mx) {
      mx = arr[i];
      idx = (int)i;
    }
  }
  return idx;
}
// Convert every element to be 1 - x
void arr_mirror1(double* arr1, double* arr2, size_t n) {
  for (int i = 0; i < n; i++) {
    arr2[i] = 1 - arr1[i];
  }
}