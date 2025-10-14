// === Pins ===
#define enA_1 10  // RIGHT motor PWM
#define enA_2 11  // LEFT motor PWM
#define in1_1 4
#define in2_1 2
#define in1_2 7
#define in2_2 8
#define encleft 12
#define encright 13
#define trig1 3   // side sensor (right wall)
#define trig2 6   // front sensor
#define echo1 5
#define echo2 9
#include <avr/interrupt.h>

volatile bool tick100ms = false;

#include <WiFiS3.h>
#include <math.h>

// === WiFi AP / Server ===
const char* ssid = "ARDUINO_FINAL_BOSS";  // open AP (no password)
WiFiServer server(80);
IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GW(192, 168, 4, 1);
IPAddress AP_SN(255, 255, 255, 0);

// === Serial ===
const int SERIAL_BAUD = 115200;

bool autonomous_mode        = false;

// Telemetry cache
int   lastLeft = 0, lastRight = 0;

// === Commands ===
const uint8_t CMD_W    = 0b00000001;
const uint8_t CMD_A    = 0b00000100;
const uint8_t CMD_S    = 0b00000010;
const uint8_t CMD_D    = 0b00000011;
const uint8_t CMD_STOP = 0b00000111;

// === UI Page ===
String htmlPage() {
  return R"HTML(
<!doctype html><html><head><meta charset="utf-8"/>
<title>WASD Control</title>
<style>
  body{font-family:system-ui,Arial;margin:2rem;}
  button{font-size:1.1rem;padding:.6rem 1rem;margin:.3rem;}
</style>
<script>
const CMD={W:1,A:4,S:2,D:5,STOP:7,M:255};
let pressed=new Set();
async function send(b){ fetch('/api/cmd?b='+b).catch(()=>{}); }
function down(k){ if(!pressed.has(k)){ pressed.add(k); send(CMD[k]); } }
function up(k){ if(pressed.has(k)){ pressed.delete(k); } if(pressed.size==0) send(CMD.STOP); }
window.addEventListener('keydown',e=>{ const k=e.key.toUpperCase(); if(CMD[k]){ e.preventDefault(); down(k); }});
window.addEventListener('keyup',e=>{ const k=e.key.toUpperCase(); if(CMD[k]){ e.preventDefault(); up(k); }});
</script></head><body>
<h1>WASD Controller</h1>
<p>Use W/A/S/D. Release keys to STOP. <button onclick="send(CMD.M)">Toggle Autonomy</button></p>
</body></html>
)HTML";
}

float readCm(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW); delayMicroseconds(3);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long d = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  float cm = (d * 0.0343f) / 2.0f;
  return cm;
}

// drive(p1,p2,p3,p4, powerRight, powerLeft)
// enA_1 == RIGHT PWM, enA_2 == LEFT PWM
void drive(int p1, int p2, int p3, int p4, int powerRight, int powerLeft) {
  powerRight = constrain(powerRight, 0, 255);
  powerLeft  = constrain(powerLeft , 0, 255);
  analogWrite(enA_1, powerRight);   // RIGHT
  analogWrite(enA_2, powerLeft );   // LEFT
  digitalWrite(in1_1, p3); // RIGHT DIR
  digitalWrite(in2_1, p4);
  digitalWrite(in1_2, p1); // LEFT DIR
  digitalWrite(in2_2, p2);
}

inline void stopAll(){ drive(LOW, LOW, LOW, LOW, 0, 0); }
inline void slideLeft(){ drive(LOW, HIGH, LOW, HIGH, 255, 200); }
inline void slideLeft(){ drive(LOW, HIGH, LOW, HIGH, 200, 255); }

// === Manual control handlers ===
bool handleSerialManual() {
  if (!Serial.available()) return false;
  char command = Serial.read();
  switch (command) {
    case 0b00000001: forwardDifferential(BASE_SPEED, BASE_SPEED); break;                 // W
    case 0b00000010: drive(LOW, HIGH, LOW, HIGH, BASE_SPEED, BASE_SPEED); break;        // S
    case 0b00000011: forwardDifferential(BASE_SPEED, 0); break;                         // D
    case 0b00000100: forwardDifferential(0, BASE_SPEED); break;                         // A
    case 0b00000111: stopAll(); break;                                                  // STOP
    case 0b01111111: autonomous_mode = !autonomous_mode; break;                         // toggle autonomy
    default: break;
  }
  return true;
}

bool handleHttpManual() {
  WiFiClient client = server.available();
  if (!client) return false;

  String req = client.readStringUntil('\r');
  client.readStringUntil('\n'); // discard

  if (req.startsWith("GET /api/cmd")) {
    int bIndex = req.indexOf("b=");
    if (bIndex > 0) {
      int val = req.substring(bIndex + 2).toInt();
      switch (val) {
        case 1:   forwardDifferential(BASE_SPEED, BASE_SPEED); break;                    // W
        case 2:   drive(LOW, HIGH, LOW, HIGH, BASE_SPEED, BASE_SPEED); break;            // S
        case 4:   forwardDifferential(0, BASE_SPEED); break;                             // A
        case 5:   forwardDifferential(BASE_SPEED, 0); break;                             // D
        case 7:   stopAll(); break;                                                      // STOP
        case 255: autonomous_mode = !autonomous_mode; break;                             // toggle autonomy
        default:  stopAll(); break;
      }
    }
    client.println("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nOK");
  } else {
    String page = htmlPage();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close\r\n");
    client.print(page);
  }
  client.stop();
  return true;
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  // Optional extra serial for your board, preserved from original
  Serial1.begin(SERIAL_BAUD);

  // WiFi AP
  if (WiFi.beginAP(ssid) != WL_AP_LISTENING) {
    Serial.println("AP start failed");
    while(true);
  }
  server.begin();

  pinMode(trig1, OUTPUT); pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT); pinMode(echo2, INPUT);

  pinMode(enA_1, OUTPUT); pinMode(enA_2, OUTPUT);
  pinMode(in1_1, OUTPUT); pinMode(in2_1, OUTPUT);
  pinMode(in1_2, OUTPUT); pinMode(in2_2, OUTPUT);
  pinMode(encleft, INPUT_PULLUP);
  pinMode(encright, INPUT_PULLUP);

  stopAll();
}

// === Main Loop ===
void loop() {
  // 1) Sensors (read & filter)
  float cmSide  = readCm(trig1, echo1);   // right wall
  float cmFront = readCm(trig2, echo2);   // ahead

  // 2) Manual handlers (manual wins this cycle)
  bool manual = handleSerialManual();
  manual = handleHttpManual() || manual;

  // 3) Autonomous control
  if (autonomous_mode) {
    if (cm1 > 8) {
      turnLeft();
    } else if (cm1 <= 10){
      turnRight();
    } else {
      goForward();
    }
  }

  delay(80); // loop cadence ~12.5 Hz (tune DIFF_SLEW with this)
}
