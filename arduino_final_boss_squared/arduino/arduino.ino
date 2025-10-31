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
WiFiServer server(80);
const char page[] PROGMEM = R"HTML(
<meta charset=utf-8>
<style>body{margin:8px;font:14px sans-serif}.g{display:grid;grid-template-columns:repeat(4,52px);gap:6px}button{height:44px;border:1px solid #ccc}</style>
<script>
const C={W:1,A:4,S:2,D:5,STOP:7,M:255,N:254,B:101,F:65,G:66,Q:128,E:129},p=new Set(),S=b=>fetch('/api/cmd?b='+b).catch(()=>{}),d=k=>{if(!p.has(k)){p.add(k);S(C[k])}},u=k=>{p.delete(k);p.size||S(C.STOP)};
addEventListener('keydown',e=>{const k=e.key.toUpperCase();if(C[k]){e.preventDefault();d(k)}});addEventListener('keyup',e=>{const k=e.key.toUpperCase();if(C[k]){e.preventDefault();u(k)}});addEventListener('pointerdown',e=>{const k=e.target.dataset.k;if(k){e.preventDefault();d(k)}});addEventListener('pointerup',e=>{const k=e.target.dataset.k;if(k){e.preventDefault();u(k)}})
</script>
<div class=g>
<p>M → Wall following, N → Object following, B → Line following, X → Tournament</p>
<button data-k=W>W</button><button data-k=A>A</button><button data-k=S>S</button><button data-k=D>D</button>
<button onclick="S(C.STOP)">■</button>
<button onclick="S(C.M)">M</button><button onclick="S(C.N)">N</button><button onclick="S(C.B)">B</button><button onclick="S(C.F)">F</button><button onclick="S(C.G)">G</button><button onclick="S(C.Q)">Q</button><button onclick="S(C.E)">E</button>
</div>
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

Servo servo1; // The servo for the front ultrasonic sonar

const int numPosition = 9; // For object tracking. Number of positions to check and navigate.
volatile int servoDirection = 0; // Current direction for servo (front ultrasonic) to take.

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

void setup() {
  objTrackingTranslationTable[0][0] = {{HIGH, LOW, LOW, HIGH, 160, 160}, 400};
  objTrackingTranslationTable[0][1] = {{LOW, HIGH, LOW, HIGH, 133, 133}, 200};
  objTrackingTranslationTable[1][0] = {{HIGH, LOW, LOW, HIGH, 160, 160}, 280};
  objTrackingTranslationTable[1][1] = {{LOW, HIGH, LOW, HIGH, 133, 133}, 200};
  objTrackingTranslationTable[2][0] = {{HIGH, LOW, LOW, HIGH, 160, 160}, 220};
  objTrackingTranslationTable[2][1] = {{LOW, HIGH, LOW, HIGH, 133, 133}, 200};
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    wifiHandler(client);
  }
}

void wifiHandler(WiFiClient client) {
  String req = client.readStringUntil('\r');
  client.readStringUntil('\n');
}

void delayedDrive(MotorParamDelayed d) {
  paramDrive(d.params);
  delay(d.delay);
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

float read_ultrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);   // start low to ensure no pulse is sent
  delayMicroseconds(5);         // ensure 5 microseconds of no signal to avoid interference
  digitalWrite(trigPin, HIGH);  // start pulse high
  delayMicroseconds(10);        // continue for 10 microseconds
  digitalWrite(trigPin, LOW);   // stop pulse

  // Measure length of time before pulse comes in
  float duration = pulseIn(echoPin, HIGH);

  // Convert the time into a distance
  float cm = (duration / 2) / 29.1;  // Divide by 29.1 or multiply by 0.0343

  return cm;
}