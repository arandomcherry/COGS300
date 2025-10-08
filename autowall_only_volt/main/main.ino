// ====== PINS ======
#define enA_1 10  // PWM for RIGHT motor (see drive() mapping note below)
#define enA_2 11  // PWM for LEFT motor
#define in1_1 4
#define in2_1 2
#define in1_2 7
#define in2_2 8
#define encleft 12
#define encright 13
#define trig1 3  // side sensor (wall following)
#define trig2 6  // front sensor
#define echo1 5
#define echo2 9

// ====== WIFI / SERVER ======
#include <WiFiS3.h>
#include <math.h>
IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GW(192, 168, 4, 1);
IPAddress AP_SN(255, 255, 255, 0);

const char* ssid = "ARDUINO_FINAL_BOSS";  // open AP (no password)
WiFiServer server(80);

// ====== SERIAL ======
const int SERIAL_BAUD = 115200;

// ====== AUTONOMY TUNING ======
// Wall following target on side sensor:
const float TARGET_CM = 18.0;
const float DEAD_BAND = 3.0;    // ignore tiny errors
const float MAX_ERR_CM = 12.0;  // clamp error for scaling

// Differential steering (voltage control):
const int BASE_SPEED = 220;  // 0..255
const int MAX_DIFF = 100;    // 0..255 (max differential)
const float KP = 4.0;        // speed-per-cm (tweak with MAX_DIFF)

// Burst + Cooldown:
const int MIN_TURN_CYCLES = 1;
const int MAX_TURN_CYCLES = 3;
const int FORWARD_COOLDOWN = 2;  // straight cycles before next turn burst

// --- Right-wall following behavior tweaks ---
const bool HUG_RIGHT = true;        // keep true (right-hand wall)
const bool USE_FRONT_STOP = false;  // still disabled per your request

const float BREAK_COOLDOWN_ERR = 2.5;  // cm beyond DEAD_BAND to interrupt cooldown
const float NEAR_CLAMP_CM = 3.0;       // clamp silly-near readings (ultrasonic min)

// Track sign of the current burst (+1 turn right, -1 turn left, 0 none)
int burst_dir = 0;

inline int sgn(float x) {
  return (x > 0) - (x < 0);
}

const float FRONT_STOP_CM = 15.0;  // used only when USE_FRONT_STOP = true

const float FAR_CLAMP_CM = 60.0;               // clamp noisy long reads
const unsigned long ECHO_TIMEOUT_US = 25000UL;  // ~4m max (HC-SR04-ish)


// ====== STATE ======
enum AutoState { AUTO_STEER_BURST,
                 AUTO_COOLDOWN };
AutoState auto_state = AUTO_COOLDOWN;

int turn_cycles_left = 0;
int cooldown_cycles_left = 0;
bool autonomous_mode = false;

// Keeps last commanded speeds for debug print
int lastLeft = 0, lastRight = 0;

// ====== COMMAND BYTES (kept from your code) ======
const uint8_t CMD_W = 0b00000001;
const uint8_t CMD_A = 0b00000011;
const uint8_t CMD_S = 0b00000010;
const uint8_t CMD_D = 0b00000100;
const uint8_t CMD_STOP = 0b00000111;

// ====== UI ======
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
<p>Use keyboard W/A/S/D. Release all keys to STOP.</p>
<div>
  <button onclick="send(CMD.W)">W</button>
  <button onclick="send(CMD.A)">A</button>
  <button onclick="send(CMD.S)">S</button>
  <button onclick="send(CMD.D)">D</button>
  <button onclick="send(CMD.STOP)">STOP</button>
  <button onclick="send(CMD.M)">Toggle Autonomy</button>
</div>
</body></html>
)HTML";
}

// ====== MOTOR DRIVER ======
// drive(p1,p2,p3,p4, powerRight, powerLeft)
// NOTE: enA_1 == RIGHT PWM, enA_2 == LEFT PWM (to match your earlier usage)
void drive(int p1, int p2, int p3, int p4, int powerRight, int powerLeft) {
  powerRight = constrain(powerRight, 0, 255);
  powerLeft = constrain(powerLeft, 0, 255);

  analogWrite(enA_1, powerRight);  // RIGHT
  analogWrite(enA_2, powerLeft);   // LEFT

  digitalWrite(in1_1, p3);  // motor block 1 (RIGHT DIR)
  digitalWrite(in2_1, p4);
  digitalWrite(in1_2, p1);  // motor block 2 (LEFT DIR)
  digitalWrite(in2_2, p2);
}

inline void forwardDifferential(int rightPWM, int leftPWM) {
  drive(HIGH, LOW, HIGH, LOW, rightPWM, leftPWM);
}

inline void stopAll() {
  drive(LOW, LOW, LOW, LOW, 0, 0);
}

// ====== SENSORS ======
float readCm(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(3);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long d = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (d == 0) return FAR_CLAMP_CM;  // timeout → treat as far
  float cm = (d * 0.0343f) / 2.0f;
  if (cm > FAR_CLAMP_CM) cm = FAR_CLAMP_CM;
  // after computing cm:
  if (cm < NEAR_CLAMP_CM) cm = NEAR_CLAMP_CM;

  return cm;
}

int mapErrToCycles(float absErr) {
  float e = absErr;
  if (e < DEAD_BAND) return 0;
  if (e > MAX_ERR_CM) e = MAX_ERR_CM;
  float t = (e - DEAD_BAND) / (MAX_ERR_CM - DEAD_BAND);  // 0..1
  int cycles = (int)round(MIN_TURN_CYCLES + t * (MAX_TURN_CYCLES - MIN_TURN_CYCLES));
  cycles = constrain(cycles, MIN_TURN_CYCLES, MAX_TURN_CYCLES);
  return cycles;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(SERIAL_BAUD);

  // WiFi AP
  if (WiFi.beginAP(ssid) != WL_AP_LISTENING) {
    Serial.println("AP start failed");
    while (true)
      ;
  }
  server.begin();

  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);

  pinMode(enA_1, OUTPUT);
  pinMode(enA_2, OUTPUT);
  pinMode(in1_1, OUTPUT);
  pinMode(in2_1, OUTPUT);
  pinMode(in1_2, OUTPUT);
  pinMode(in2_2, OUTPUT);
  pinMode(encleft, INPUT_PULLUP);
  pinMode(encright, INPUT_PULLUP);

  stopAll();
  cooldown_cycles_left = 0;
  auto_state = AUTO_COOLDOWN;
}

// Returns true if a manual/HTTP command took control this loop
bool handleSerialManual() {
  if (!Serial.available()) return false;
  char command = Serial.read();
  switch (command) {
    case 0b00000001: forwardDifferential(BASE_SPEED, BASE_SPEED); break;          // W
    case 0b00000010: drive(LOW, HIGH, LOW, HIGH, BASE_SPEED, BASE_SPEED); break;  // S
    case 0b00000011: forwardDifferential(BASE_SPEED, 0); break;                   // D (pivot-ish)
    case 0b00000100: forwardDifferential(0, BASE_SPEED); break;                   // A (pivot-ish)
    case 0b00000101: drive(LOW, HIGH, LOW, LOW, BASE_SPEED, 0); break;            // custom
    case 0b00000110: drive(LOW, LOW, LOW, HIGH, 0, BASE_SPEED); break;            // custom
    case 0b00000111: stopAll(); break;                                            // STOP
    case 0b01111111: autonomous_mode = true; break;                               // enable autonomy
    default: break;
  }
  return true;
}

bool handleHttpManual() {
  WiFiClient client = server.available();
  if (!client) return false;

  // crude request parsing
  String req = client.readStringUntil('\r');
  client.readStringUntil('\n');  // discard

  if (req.startsWith("GET /api/cmd")) {
    int bIndex = req.indexOf("b=");
    if (bIndex > 0) {
      int val = req.substring(bIndex + 2).toInt();
      switch (val) {
        case 1: forwardDifferential(BASE_SPEED, BASE_SPEED); break;          // W
        case 2: drive(LOW, HIGH, LOW, HIGH, BASE_SPEED, BASE_SPEED); break;  // S
        case 4: forwardDifferential(0, BASE_SPEED); break;                   // A
        case 5: forwardDifferential(BASE_SPEED, 0); break;                   // D
        case 7: stopAll(); break;                                            // STOP
        case 255:
          autonomous_mode = !autonomous_mode;
          stopAll();
          break;  // toggle autonomy
        default: stopAll(); break;
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

void loop() {
  // 1) Read sensors
  float cmSide = readCm(trig1, echo1);   // side/right
  float cmFront = readCm(trig2, echo2);  // front

  // Optional extra clamp for ultrasonic near-field weirdness
  if (cmSide < NEAR_CLAMP_CM) cmSide = NEAR_CLAMP_CM;
  if (cmFront < NEAR_CLAMP_CM) cmFront = NEAR_CLAMP_CM;

  // 2) Manual handlers (manual wins this cycle)
  bool manual = handleSerialManual();
  manual = handleHttpManual() || manual;

  // 3) Autonomous control
  if (!manual && autonomous_mode) {
    if (USE_FRONT_STOP && cmFront <= FRONT_STOP_CM) {
      stopAll();
    } else {
      // Error: + => too far from RIGHT wall; - => too close
      float err = cmSide - TARGET_CM;
      float absErr = fabs(err);
      int signE = sgn(err);  // -1,0,+1

      // Differential steering (right-wall mapping):
      float diffF = KP * err;
      if (diffF > MAX_DIFF) diffF = MAX_DIFF;
      if (diffF < -MAX_DIFF) diffF = -MAX_DIFF;
      int diff = (int)round(diffF);

      int rightSpeed, leftSpeed;
      if (HUG_RIGHT) {
        // Too far (+diff): slow RIGHT, speed LEFT -> turn RIGHT towards wall
        rightSpeed = BASE_SPEED - diff;
        leftSpeed = BASE_SPEED + diff;
      } else {
        rightSpeed = BASE_SPEED + diff;
        leftSpeed = BASE_SPEED - diff;
      }
      rightSpeed = constrain(rightSpeed, 0, 255);
      leftSpeed = constrain(leftSpeed, 0, 255);

      // --- Preemptable burst + breakable cooldown ---
      switch (auto_state) {
        case AUTO_COOLDOWN:
          {
            // If error gets large again, break cooldown and start a new burst
            if (absErr > (DEAD_BAND + BREAK_COOLDOWN_ERR)) {
              int want = mapErrToCycles(absErr);
              if (want > 0 && signE != 0) {
                burst_dir = signE;  // remember direction at burst start
                turn_cycles_left = want;
                auto_state = AUTO_STEER_BURST;
                forwardDifferential(rightSpeed, leftSpeed);
                turn_cycles_left--;
              } else {
                forwardDifferential(BASE_SPEED, BASE_SPEED);
              }
            } else {
              // Otherwise keep going straight during cooldown
              forwardDifferential(BASE_SPEED, BASE_SPEED);
              if (cooldown_cycles_left > 0) cooldown_cycles_left--;
              else cooldown_cycles_left = 0;
            }
          }
          break;

        case AUTO_STEER_BURST:
          {
            // End the burst immediately if we overshoot or get back in-range
            if (signE == 0 || (signE != burst_dir) || absErr <= DEAD_BAND) {
              auto_state = AUTO_COOLDOWN;
              cooldown_cycles_left = FORWARD_COOLDOWN;
              burst_dir = 0;
              forwardDifferential(BASE_SPEED, BASE_SPEED);
            } else if (turn_cycles_left > 0) {
              forwardDifferential(rightSpeed, leftSpeed);
              turn_cycles_left--;
            } else {
              // normal end of burst -> cooldown
              auto_state = AUTO_COOLDOWN;
              cooldown_cycles_left = FORWARD_COOLDOWN;
              burst_dir = 0;
              forwardDifferential(BASE_SPEED, BASE_SPEED);
            }
          }
          break;
      }

      // (Optional) telemetry remembers what we *intend* to use
      lastLeft = leftSpeed;
      lastRight = rightSpeed;
    }
  }

  // 4) Telemetry (unchanged)
  int leftEncoderVal = digitalRead(encleft);
  int rightEncoderVal = digitalRead(encright);
  Serial.print("Side(cm): ");
  Serial.print(cmSide);
  Serial.print("  Front(cm): ");
  Serial.print(cmFront);
  Serial.print("  L:");
  Serial.print(lastLeft);
  Serial.print("  R:");
  Serial.print(lastRight);
  Serial.print("  State:");
  Serial.print(auto_state == AUTO_STEER_BURST ? "STEER" : "COOL");
  Serial.print("  tc:");
  Serial.print(turn_cycles_left);
  Serial.print("  cd:");
  Serial.print(cooldown_cycles_left);
  Serial.print("  burst:");
  Serial.print(burst_dir);
  Serial.print("  EncL:");
  Serial.print(leftEncoderVal);
  Serial.print("  EncR:");
  Serial.println(rightEncoderVal);

  delay(80);
}
