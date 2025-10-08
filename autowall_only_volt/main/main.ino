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

// === Behavior Tuning ===
// Target distance to right wall (cm)
const float TARGET_CM       = 18.0f;
const float DEAD_BAND       = 3.0f;   // ignore tiny errors
const float MAX_ERR_CM      = 12.0f;  // scaling cap

// Base forward speed and general limits
const int   BASE_SPEED      = 220;    // 0..255

// PD steering + filtering (to avoid wall slam)
const float SIDE_FILTER_ALPHA = 0.35f; // 0..1; higher=snappier, lower=smoother
const float KP = 2.6f;                 // proportional gain (cm -> PWM)
const float KD = 1.1f;                 // derivative per loop (Δcm -> PWM)

// Dynamic steering limits (small near target, larger when far)
const int   MAX_DIFF_NEAR = 35;        // PWM delta cap when near target
const int   MAX_DIFF_FAR  = 100;       // PWM delta cap when far
const float NEAR_ZONE_CM  = 8.0f;      // |err| <= this → "near"

// Slew limit on steering updates (per loop ~80ms)
const int   DIFF_SLEW     = 12;        // PWM per loop

// Burst + Cooldown (to avoid dithering)
const int   MIN_TURN_CYCLES = 2;
const int   MAX_TURN_CYCLES = 6;
const int   FORWARD_COOLDOWN= 6;

// Cooldown early-break threshold (so we correct promptly)
const float BREAK_COOLDOWN_ERR = 2.5f; // extra cm beyond dead band

// Safety / sensor
const bool  USE_FRONT_STOP   = false;  // keep false per your request
const float FRONT_STOP_CM    = 15.0f;  // used only if USE_FRONT_STOP = true
const float FAR_CLAMP_CM     = 200.0f; // clamp noisy long reads
const float NEAR_CLAMP_CM    = 3.0f;   // clamp silly-near reads
const unsigned long ECHO_TIMEOUT_US = 25000UL; // ~4m timeout

// Right-hand wall hugging
const bool  HUG_RIGHT = true;

// === State ===
enum AutoState { AUTO_STEER_BURST, AUTO_COOLDOWN };
AutoState auto_state = AUTO_COOLDOWN;
int  turn_cycles_left       = 0;
int  cooldown_cycles_left   = 0;
int  burst_dir              = 0; // +1/-1/0 (direction at burst start)
bool autonomous_mode        = false;

// Telemetry cache
int   lastLeft = 0, lastRight = 0;

// Filter / PD memory
float cmSideFilt  = TARGET_CM;
float prevErr     = 0.0f;
int   prevDiffCmd = 0;

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

// === Helpers ===
inline int sgn(float x){ return (x>0) - (x<0); }

inline float mapf(float x, float in_min, float in_max, float out_min, float out_max){
  if (x <= in_min) return out_min;
  if (x >= in_max) return out_max;
  return out_min + (out_max - out_min) * (x - in_min) / (in_max - in_min);
}

float readCm(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW); delayMicroseconds(3);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long d = pulseIn(echo, HIGH, ECHO_TIMEOUT_US);
  if (d == 0) return FAR_CLAMP_CM; // timeout → treat as far
  float cm = (d * 0.0343f) / 2.0f;
  if (cm > FAR_CLAMP_CM) cm = FAR_CLAMP_CM;
  if (cm < NEAR_CLAMP_CM) cm = NEAR_CLAMP_CM;
  return cm;
}

int mapErrToCycles(float absErr){
  if (absErr < DEAD_BAND) return 0;
  if (absErr > MAX_ERR_CM) absErr = MAX_ERR_CM;
  float t = (absErr - DEAD_BAND) / (MAX_ERR_CM - DEAD_BAND); // 0..1
  int cycles = (int)round(MIN_TURN_CYCLES + t*(MAX_TURN_CYCLES - MIN_TURN_CYCLES));
  return constrain(cycles, MIN_TURN_CYCLES, MAX_TURN_CYCLES);
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

inline void forwardDifferential(int rightPWM, int leftPWM){
  drive(HIGH, LOW, HIGH, LOW, rightPWM, leftPWM);
}
inline void stopAll(){ drive(LOW, LOW, LOW, LOW, 0, 0); }

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
  auto_state = AUTO_COOLDOWN;
  cooldown_cycles_left = 0;
  burst_dir = 0;
  cmSideFilt = TARGET_CM;
  prevErr = 0.0f;
  prevDiffCmd = 0;
}

// === Main Loop ===
void loop() {
  // 1) Sensors (read & filter)
  float cmSide  = readCm(trig1, echo1);   // right wall
  float cmFront = readCm(trig2, echo2);   // ahead

  // IIR filter on side distance
  cmSideFilt += SIDE_FILTER_ALPHA * (cmSide - cmSideFilt);

  // 2) Manual handlers (manual wins this cycle)
  bool manual = handleSerialManual();
  manual = handleHttpManual() || manual;

  // 3) Autonomous control
  if (!manual && autonomous_mode) {
    if (USE_FRONT_STOP && cmFront <= FRONT_STOP_CM) {
      stopAll();
    } else {
      // Error: + => too far from RIGHT wall; - => too close
      float err    = cmSideFilt - TARGET_CM;
      float absErr = fabs(err);
      int   signE  = sgn(err);

      // --- PD steering ---
      float derr  = err - prevErr;                 // per loop (~80ms)
      float diffF = KP * err + KD * derr;          // PWM delta
      // Dynamic cap based on |err|
      int dynMax  = (int)round(mapf(absErr, 0.0f, MAX_ERR_CM, (float)MAX_DIFF_NEAR, (float)MAX_DIFF_FAR));

      int diffCmd = (int)round(diffF);
      if (diffCmd >  dynMax) diffCmd =  dynMax;
      if (diffCmd < -dynMax) diffCmd = -dynMax;

      // Slew limit steering command
      if (diffCmd > prevDiffCmd + DIFF_SLEW) diffCmd = prevDiffCmd + DIFF_SLEW;
      else if (diffCmd < prevDiffCmd - DIFF_SLEW) diffCmd = prevDiffCmd - DIFF_SLEW;

      // Convert to wheel PWMs (10=RIGHT, 11=LEFT), right-wall mapping:
      // +err (too far) → turn RIGHT: slow RIGHT, speed LEFT
      int rightSpeed = BASE_SPEED - diffCmd;
      int leftSpeed  = BASE_SPEED + diffCmd;
      rightSpeed = constrain(rightSpeed, 0, 255);
      leftSpeed  = constrain(leftSpeed , 0, 255);

      // --- Preemptable burst + breakable cooldown ---
      switch (auto_state) {
        case AUTO_COOLDOWN: {
          // Break cooldown early if error grows beyond margin
          if (absErr > (DEAD_BAND + BREAK_COOLDOWN_ERR)) {
            int want = mapErrToCycles(absErr);
            if (want > 0 && signE != 0) {
              burst_dir = signE;
              turn_cycles_left = want;
              auto_state = AUTO_STEER_BURST;
              forwardDifferential(rightSpeed, leftSpeed);
              turn_cycles_left--;
            } else {
              forwardDifferential(BASE_SPEED, BASE_SPEED);
            }
          } else {
            forwardDifferential(BASE_SPEED, BASE_SPEED);
            if (cooldown_cycles_left > 0) cooldown_cycles_left--;
            else cooldown_cycles_left = 0;
          }
        } break;

        case AUTO_STEER_BURST: {
          // End burst if overshoot (sign flip), in-range, or cycles done
          if (signE == 0 || (signE != burst_dir) || absErr <= DEAD_BAND) {
            auto_state = AUTO_COOLDOWN;
            cooldown_cycles_left = FORWARD_COOLDOWN;
            burst_dir = 0;
            forwardDifferential(BASE_SPEED, BASE_SPEED);
          } else if (turn_cycles_left > 0) {
            forwardDifferential(rightSpeed, leftSpeed);
            turn_cycles_left--;
          } else {
            auto_state = AUTO_COOLDOWN;
            cooldown_cycles_left = FORWARD_COOLDOWN;
            burst_dir = 0;
            forwardDifferential(BASE_SPEED, BASE_SPEED);
          }
        } break;
      }

      // Telemetry cache & PD memory
      lastLeft     = leftSpeed;
      lastRight    = rightSpeed;
      prevErr      = err;
      prevDiffCmd  = diffCmd;
    }
  }

  // 4) Telemetry
  int leftEncoderVal  = digitalRead(encleft);
  int rightEncoderVal = digitalRead(encright);

  Serial.print("Side(cm): ");   Serial.print(cmSide);
  Serial.print("  Front(cm): ");Serial.print(cmFront);
  Serial.print("  L:");         Serial.print(lastLeft);
  Serial.print("  R:");         Serial.print(lastRight);
  Serial.print("  State:");     Serial.print(auto_state==AUTO_STEER_BURST?"STEER":"COOL");
  Serial.print("  tc:");        Serial.print(turn_cycles_left);
  Serial.print("  cd:");        Serial.print(cooldown_cycles_left);
  Serial.print("  burst:");     Serial.print(burst_dir);
  Serial.print("  EncL:");      Serial.print(leftEncoderVal);
  Serial.print("  EncR:");      Serial.println(rightEncoderVal);

  delay(80); // loop cadence ~12.5 Hz (tune DIFF_SLEW with this)
}
