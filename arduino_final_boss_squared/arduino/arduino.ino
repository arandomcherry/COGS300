// Author: Winslow Flandre <i@winsloweric.com>
// Arduino Final Boss SQUARED. Refactored.
#include <WiFiS3.h>
#include <Servo.h>

// Motors, notice that there is a swap in drive() due to wiring mistake.
#define enA_1 10
#define enA_2 11
#define in1_1 A0
#define in2_1 4
#define in1_2 7
#define in2_2 8
// Servo
#define servopin 12
// Ultrasonics, 1 is leftside, 2 is frontside.
#define trig1 A2
#define trig2 6
#define echo1 5
#define echo2 9
// IR Sensors
#define leftir 2
#define midir A1
#define rightir 3

// syslog
WiFiUDP udp;

// API Endpoint
IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GW(192, 168, 4, 1);
IPAddress AP_SN(255, 255, 255, 0);

const int SERIAL_BAUD = 115200;

const char* ssid = "Arduino Final Boss²™";
const char* pass = "letsalllovelain";
WiFiServer server(80);

boolean inhibitMovement = false;
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
#heatmapBox{margin-top:8px;padding:4px;border:1px solid #eee;border-radius:4px;background:#fdfdfd}
#heatmapTitle{font-size:13px;margin-bottom:2px}
.heatmap{display:flex;gap:2px;align-items:flex-end;margin-top:4px}
.heatcell{flex:1;height:28px;border-radius:3px;background:#eee;transition:background .15s}
#heatmapLegend{font-size:11px;color:#666;margin-top:4px}
</style>
<script>
const C={W:1,A:4,S:2,D:5,Q:8,E:9,STOP:7,M:255,N:254,B:101,X:100,Z:50,I:150},p=new Set,m=new Set("WASD"),B={};
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
function ensureHeatmap(count){
  const box=document.getElementById("heatmap");
  if(!box)return;
  if(box.children.length===count)return;
  box.innerHTML="";
  for(let i=0;i<count;i++){
    const cell=document.createElement("div");
    cell.className="heatcell";
    box.appendChild(cell);
  }
}
function renderHeatmap(values){
  const box=document.getElementById("heatmap");
  if(!box||!values.length)return;
  ensureHeatmap(values.length);

  let min=values[0],max=values[0];
  for(const v of values){
    if(!Number.isFinite(v))continue;
    if(v<min)min=v;
    if(v>max)max=v;
  }
  const span=(max-min)||1;

  // Left (index 0) → servo at 0°, right → 180°
  const cells=[...box.children];
  cells.forEach((cell,idx)=>{
    const v=values[idx];
    if(!Number.isFinite(v)){
      cell.style.background="#eee";
      return;
    }
    // Normalize 0–1
    const t=(v-min)/span;
    // We want **closer = darker**:
    // If your values are normalized (0=near, 1=far), flip t first:
    //   const closeness = 1 - t;
    // If values are raw cm (small=near), closeness = 1 - t still works.
    const closeness=1-t;
    const lightness=80-closeness*40; // 40% (very close) → 80% (far)
    cell.style.backgroundColor=`hsl(210,70%,${lightness}%)`;
  });

  const legend=document.getElementById("heatmapLegend");
  if(legend){
    legend.textContent=
      `min: ${min.toFixed(1)}  max: ${max.toFixed(1)}  (darker = closer, left = servo 0°)`;
  }
}
function swapScan(){
  fetch("/api/swap").then(r=>r.json()).then(s=>{
    const stateEl=document.getElementById("state");
    if(stateEl) stateEl.textContent=JSON.stringify(s,null,2);
    if(typeof s.result==="string"){
      const vals=s.result.trim().split(/\s+/).map(Number).filter(v=>Number.isFinite(v));
      if(vals.length) renderHeatmap(vals);
    }
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
<p>Z→Main/Auto,M→Wall,N→Obj,B→Line,X→Tournament,(I)nhibitMovements</p>
<div class=p>
  <div class=g>
    <button data-k=W>W</button><button data-k=A>A</button><button data-k=S>S</button><button data-k=D>D</button>
    <button data-k=STOP>■</button><button data-k=M>M</button><button data-k=N>N</button><button data-k=B>B</button>
    <button data-k=Z>Z</button><button data-k=X>X</button>
    <button data-k=I>I</button>
  </div>
  <button id=refresh>⟳ State</button>
  <button id=swap>⟳ Swapscan</button>

  <!-- NEW: heatmap box -->
  <div id="heatmapBox">
    <div id="heatmapTitle">Swapscan heatmap</div>
    <div id="heatmap" class="heatmap"></div>
    <div id="heatmapLegend"></div>
  </div>

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
const int numPositionDebug = 30;
volatile int servoDirection = 0;  // Current direction for servo (front ultrasonic) to take.
volatile int servoDirectionDebug = 0; 
double servoStep;                    // The amount of angle for each step for object tracking
double servoStepDebug;                    

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
double objDistanceDebug[numPositionDebug];
double objProbabilityDebug[numPositionDebug];

volatile int leftIRVal;
volatile int rightIRVal;

// Wall-following PD Controller parameters
float ePrev;  // e[k-1], e[k] should be measured by loop
float kP = 5.0f;
float kD = 1.2f;
int base_pwm = 80;
float tS = 0.05f;  // 0.05s
float dRef = 18.0;     // 10cm
float dK;
float uK;
int left_pwm;
int right_pwm;
const float FRONT_MIN = 10;  // cm: "we're about to hit front wall"
const float RIGHT_MIN = 4;   // cm: "we're scraping right wall"

void syslog(char* msg, size_t len) {
  // Serial.print("syslog\n");
  if (!udp.beginPacket("255.255.255.255", 1514)) {
    Serial.print("beginPacket error.\n");
    return;
  }
  udp.write(msg, len ? len : strlen(msg));
  // Serial.write("udp.write() done\n");
  if (!udp.endPacket()) {
    Serial.print("endPacket error.\n");
    return;
  }
  // Serial.print("syslog done.\n");
}

void log(const char* fmt, ...) {
  static char buf[1446];
  size_t s;
  size_t s0 = snprintf(buf, sizeof(buf), "[%12ul]: ", millis());
  va_list list;
  va_start(list, fmt);
  s = vsnprintf(buf + s0, sizeof(buf) + s0, fmt, list);
  va_end(list);
  syslog(buf, s + s0);
}

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

  servoStepDebug = 180 / (numPositionDebug - 1);

  for (int i = 0; i < numPositionDebug; i++) {
    objDistanceDebug[i] = 0;
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
  objTrackingTranslationTable[4][1] = { { LOW, LOW, LOW, LOW, 0, 0 }, 0 };
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
  pinMode(leftir, INPUT_PULLUP);
  pinMode(midir, INPUT);
  pinMode(rightir, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(leftir), IRInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightir), IRInterrupt, CHANGE);
  // System all ready.
  udp.begin(11451);
  server.begin();
  log("Setup done.");
}

void loop() {
  if (systemState == MANUAL) {
    // Do nothing.
  } else {  // System is at least in an auto state, SEQ mode only enable automatic state transition.
    switch (autoState) {
      case LINE:
        if (lineFollowingFrame()) {
          autoState = WALL;
          servo1.write(90);
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
  if (systemState == AUTONOMOUS_SEQ && wallFollowingCriteria()) {
    return true;
  } 
  stopDrive();
  delay(20);
  int counter = 0;  // Force quit on 200000;
  while (true) {
    counter++;
    if (counter >= 20000) {
      goForward();
      delay(10);
      stopDrive();
      return 0;
    }
    if (!direction) {
      rotLeft();
      delay(3);
      stopDrive();
      currState = ROTLEFT;
    } else {
      rotRight();
      delay(3);
      stopDrive();
      currState = ROTRIGHT;
    }
    int leftJudge = digitalRead(leftir);
    int midJudge = digitalRead(midir);
    int rightJudge = digitalRead(rightir);

    if (midJudge == 0) {
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
    }

    if (leftJudge == 1 && midJudge == 0 && rightJudge == 1) {  // back on line
      stopDrive();
      if (!direction) {
        rotRight();
        delay(5);
        stopDrive();
      } else {
        rotLeft();
        delay(5);
        stopDrive();
      }
      goForward();
      return false;
    } else if (leftJudge == 1 && rightJudge == 0) {
      if (!direction) {
        return false;
      } else {
        continue;
      }
    } else if (leftJudge == 0 && rightJudge == 1) {
      if (!direction) {
        continue;
      } else {
        return false;
        continue;
      }
    } else if (leftJudge == 0 && midJudge == 0 && rightJudge == 0) {
      continue;
    } else if (midJudge == 1 && leftJudge == 1 && rightJudge == 1) {
      goBackward();
      delay(80);
      stopDrive();
      continue;
    }
  }
}

void rotLeft() {
  drive(LOW, HIGH, HIGH, LOW, 0, 84);
}

void rotRight() {
  drive(HIGH, LOW, LOW, HIGH, 84, 0);
}

void rotLeftTotal() {
  drive(LOW, HIGH, HIGH, LOW, 90, 90);
}

void rotRightTotal() {
  drive(HIGH, LOW, LOW, HIGH, 90, 90);
}

void IRInterrupt() {
  leftIRVal = digitalRead(leftir);
  rightIRVal = digitalRead(rightir);
  if ((leftIRVal == 0 || rightIRVal == 0) && autoState == LINE && systemState != MANUAL) {
    fullBackward();
  }
}

boolean lineFollowingFrame() {
  int midJudge = digitalRead(midir);
  if (midJudge == 0) {
    digitalWrite(13, HIGH);
    if (leftIRVal == 0) {
      prevState = currState;
      return fineturnDirection(0);
    } else if (rightIRVal == 0) {
      prevState = currState;
      return fineturnDirection(1);
    } else {  // 1 0 1
      prevState = currState;
      currState = FORWARD;
      goForward();
    }
  } else {  // midJudge = 1
    digitalWrite(13, LOW);
    if (leftIRVal == 0) {
      prevState = currState;
      return fineturnDirection(0);
    } else if (rightIRVal == 0) {
      prevState = currState;
     return  fineturnDirection(1);
    } else {  // 1 1 1
      stopDrive();
      delay(20);
      if (systemState == AUTONOMOUS_SEQ && wallFollowingCriteria()) {
        return true;
      } else {
        // Not following wall and out of line
        if (prevState == FORWARD) {
          goBackward();
          delay(10);
        } else if (prevState == ROTLEFT) {
          currState = ROTRIGHT;
          rotRight();
          delay(15);
        } else if (prevState == ROTRIGHT) {
          currState = ROTLEFT;
          rotLeft();
          delay(15);
        }
      }
    }
  }
  return false;
}
// boolean lineFollowingFrame() {
//   int leftJudge = digitalRead(leftir);
//   int midJudge = digitalRead(midir);
//   int rightJudge = digitalRead(rightir);

//   log("Line status: %s - %s - %s\n", leftJudge ? "off" : "on",
//   midJudge ? "off" : "on", rightJudge ? "off" : "on");

//   if(midJudge == 0) {
//     digitalWrite(13, HIGH);
//   } else {
//     digitalWrite(13, LOW);
//   }
//   if (leftJudge == 1 && midJudge == 0 && rightJudge == 1) { // Back on line
//     log("Back on line. Moving forward.\n");
//     prevState = currState;
//     currState = FORWARD;
//     goForward();
//     delay(10);
//     stopDrive();
//     delay(1);
//   } else if (leftJudge == 0 && midJudge == 0 && rightJudge == 1) { // On the right
//     log("To the right.\n");
//     prevState = currState;
//     fineturnDirection(0);
//   } else if (leftJudge == 0 && midJudge == 1 && rightJudge == 1) {
//     log("To the right 2.\n");
//     goForward();
//     delay(5);
//     stopDrive();
//     delay(2);
//     prevState = currState;
//     fineturnDirection(0);
//   } else if (leftJudge == 1 && midJudge == 0 && rightJudge == 0) {
//     log("To the left.\n");
//     prevState = currState;
//     fineturnDirection(1);
//   } else if (leftJudge == 1 && midJudge == 1 && rightJudge == 0) {
//     log("To the left 2.\n");
//     goForward();
//     delay(5);
//     stopDrive();
//     delay(2);
//     prevState = currState;
//     fineturnDirection(1);
//   } else if (leftJudge == 1 && rightJudge == 1 && midJudge == 1) {
//     log("No line.\n");
//     // Out of line, recover, and potentially we want to check for wall following now
//     if (systemState == AUTONOMOUS_SEQ && wallFollowingCriteria()) {
//       return true;
//     } else {
//       // Not following wall and out of line
//       stopDrive();
//       delay(10);
//       if (prevState == FORWARD) {
//         rotLeft();  // TODO: better return logic;
//         delay(20);
//       } else if (prevState == ROTLEFT) {
//         rotRight();
//         delay(20);
//       } else if (prevState == ROTRIGHT) {
//         rotLeft();
//         delay(20);
//       }
//     }
//   } else {  // 000 010
//     log("??\n");
//     if (prevState == FORWARD) {
//       delay(10);
//       goForward();
//     } else if (prevState == ROTLEFT) {
//       delay(10);
//       rotLeft();
//     } else if (prevState == ROTRIGHT) {
//       delay(10);
//       rotRight();
//     }
//     goForward();
//     delay(5);
//   }
//   return false;
// }

boolean wallFollowingFrame() {
  double frontLeftReading = read_ultrasonic(trig2, echo2, 30000UL);
  double rightReading = read_ultrasonic(trig1, echo1, 30000UL);
  int leftJudge = digitalRead(leftir);
  int midJudge = digitalRead(midir);
  int rightJudge = digitalRead(rightir);

  float e = dRef - rightReading;
  if (frontLeftReading < FRONT_MIN) {
    if (rightReading > 130) {
      rotRightTotal();
      delay(100);    // rotate for a fixed time
      stopDrive();   // stop rotation
    } else {
      rotLeftTotal();
      delay(100);    // rotate for a fixed time
      stopDrive();   // stop rotation
    }
    return false;  // then next loop PD will take over again
  } else {
    if (fabs(e) < 1.5) e = 0.0f;  // use absolute value
    float de = (e - ePrev) / tS;
    float u_raw = kP * e + kD * de;

    if (u_raw > 30) u_raw = 30;
    if (u_raw < -30) u_raw = -30;

    left_pwm = base_pwm + (int)u_raw;
    right_pwm = base_pwm - (int)u_raw;
    left_pwm = constrain(left_pwm, 0, 255);
    right_pwm = constrain(right_pwm, 0, 255);

    drive(LOW, HIGH, LOW, HIGH, right_pwm, left_pwm);
    ePrev = e;
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

  if ((frontLeftReading + rightReading < 60) || rightReading < 20 || frontLeftReading < 20) {
    stopDrive();
    delay(80);
    goForward();
    delay(1000);
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
  double maxVal = arr_max(objDistance, numPosition);
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

boolean objectFollowingCriteria(int ir1, int ir2, int ir3, int sonic1, int sonic2) {
  return false;
  if (ir2 == 0 && sonic1 > 100) {
    // ir1 and 3 get triggered when hits wall...
    // hit marker and front sonic reads high
    return true;
  } else {
    return false;
  }
}

void wifiHandler(WiFiClient client) {
  String req = client.readStringUntil('\r');
  client.readStringUntil('\n');
  log("WiFi request\n");

  if (req.startsWith("GET /api/cmd")) {
    int bIndex = req.indexOf("b=");
    if (bIndex > 0) {
      int val = req.substring(bIndex + 2).toInt();
      if (val >= 0 && val <= 255) {
        switch (val) {
          case 50:
            if (systemState == MANUAL) {
              log("Start autonomous.");
              systemState = AUTONOMOUS;
            } else {
              log("Start manual.");
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
            servo1.write(90);
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
          case 150:
            inhibitMovement = !inhibitMovement;
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
    client.print(",\"left_pwm\":");
    client.print(left_pwm);
    client.print(",\"right_pwm\":");
    client.print(right_pwm);
    client.print("}");
  } else if (req.startsWith("GET /api/swap")) {
    servo1.write(0);
    delay(500);
    for (int i = 0; i < numPositionDebug; i++) {
      int angle = i * servoStepDebug;
      servo1.write(angle);
      delay(80);
      objDistanceDebug[i] = read_ultrasonic(trig2, echo2, 100000UL);
      delay(80);
    }
    double maxVal = arr_max(objDistanceDebug, numPositionDebug);
    if (maxVal < 0.0001) maxVal = 1.0;
    arr_div(objDistanceDebug, numPositionDebug, maxVal);
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close\r\n");
    client.print("{\"result\":\"");
    for (int i = 0; i < numPositionDebug; i++) {
      client.print(objDistanceDebug[i]);
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
  if (!inhibitMovement) {
    analogWrite(enA_1, power1);
    analogWrite(enA_2, power2);

    digitalWrite(in1_1, p3);
    digitalWrite(in2_1, p4);

    digitalWrite(in1_2, p1);
    digitalWrite(in2_2, p2);
    return;
  }
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
  drive(LOW, HIGH, LOW, HIGH, 155, 155);
}
void goBackward() {
  drive(HIGH, LOW, HIGH, LOW, 134, 134);
}
void fullBackward() {
  drive(HIGH, LOW, HIGH, LOW, 200, 200);
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
// Return the maximum value
double arr_max(const double* arr, size_t n) {
  double mx = arr[0];
  for (size_t i = 1; i < n; ++i) {
    if (arr[i] > mx) {
      mx = arr[i];
    }
  }
  return mx;
}

// Convert every element to be 1 - x
void arr_mirror1(double* arr1, double* arr2, size_t n) {
  for (int i = 0; i < n; i++) {
    arr2[i] = 1 - arr1[i];
  }
}