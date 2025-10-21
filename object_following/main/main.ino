#define enA_1 10
#define enA_2 11
#define in1_1 2
#define in2_1 4
#define in1_2 7
#define in2_2 8
#define servopin 13
#define trig1 3
#define trig2 6
#define echo1 5
#define echo2 9
//sensor 1 side, sensor 2 front
#include <WiFiS3.h>
#include <Servo.h>
IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GW(192, 168, 4, 1);
IPAddress AP_SN(255, 255, 255, 0);


Servo servo1;
const int num_positions = 8;
int servo_dir = 0;
int servo_step;


float beliefs[num_positions];
float dis[num_positions];
float prob[num_positions];

// --- 8 direction helpers ---
// Tip: tune the delays (ms) to taste or replace with your own milder/stronger turn functions.

void dir_right_hard()   { strongRightTurn(); }                 // pivot/faster right
void dir_right()        { strongRightTurn(); delay(80); goForward(); }
void dir_right_slight() { strongRightTurn(); delay(40); goForward(); }

void dir_forward_right(){ goForward(); /* optionally bias right motor a bit slower */ }
void dir_forward_left() { goForward(); /* optionally bias left motor a bit slower  */ }

void dir_left_slight()  { strongLeftTurn();  delay(40); goForward(); }
void dir_left()         { strongLeftTurn();  delay(80); goForward(); }
void dir_left_hard()    { strongLeftTurn(); }                  // pivot/faster left


void goForward() {
  drive(0, HIGH, 0, HIGH, 133, 133);
}

void strongLeftTurn() {
  drive(0, HIGH, HIGH, 0, 170, 170);
}

void strongRightTurn() {
  drive(HIGH, 0, 0, HIGH, 170, 170);
}

void turnLeft(int speed) {
  drive(0, HIGH, 0, HIGH, 60, speed);
}
void turnRight(int speed) {
  drive(0, HIGH, 0, HIGH, speed, 60);
}
void stopDrive() {
  drive(0, 0, 0, 0, 0, 0);
}

boolean autonomous_mode = 0;
boolean autonomous_mode_objectfollow = 0;

const char* ssid = "ARDUINO_FINAL_BOSS";  // open AP (no password)
WiFiServer server(80);

const int SERIAL_BAUD = 115200;

// Command bytes
const uint8_t CMD_W = 0b00000001;
const uint8_t CMD_A = 0b00000100;
const uint8_t CMD_S = 0b00000010;
const uint8_t CMD_D = 0b00000011;
const uint8_t CMD_STOP = 0b00000111;

String htmlPage() {
  return R"HTML(
  <!doctype html><html><head><meta charset="utf-8"/>
  <title>WASD Control</title>
  <script>
  const CMD={W:1,A:4,S:2,D:5,STOP:7, M:255, N:254};
  let pressed=new Set();
  async function send(b){
    fetch('/api/cmd?b='+b).catch(()=>{});
  }
  function down(k){
    if(!pressed.has(k)){ pressed.add(k); send(CMD[k]); }
  }
  function up(k){
    if(pressed.has(k)){ pressed.delete(k); }
    if(pressed.size==0) send(CMD.STOP);
  }
  window.addEventListener('keydown',e=>{
    const k=e.key.toUpperCase(); if(CMD[k]){ e.preventDefault(); down(k); }
  });
  window.addEventListener('keyup',e=>{
    const k=e.key.toUpperCase(); if(CMD[k]){ e.preventDefault(); up(k); }
  });
  </script></head><body>
  <h1>WASD Controller</h1>
  <p>Use keyboard W/A/S/D. Releases → Stop. M Autonomous, N Object following</p>
  </body></html>
  )HTML";
}

int speed = 255;
int lowVol = 0;

float arr_max (const float* arr, size_t n) {
  float max = 0;
  for (int i = 0; i<n; i++) {
    if (arr[i] > max) max = arr[i];
  }
  return max;
}

int arr_maxi(const float* arr, size_t n) {
  int idx = 0;
  float mx = arr[0];
  for (size_t i = 1; i < n; ++i) {
    if (arr[i] > mx) { mx = arr[i]; idx = (int)i; }
  }
  return idx;
}

void arr_div (float* arr, size_t n, float a) {
  for (int i = 0; i<n; i++) {
    arr[i] = arr[i] / a;
  }
}

void arr_mul (float* arr1, float* arr2, size_t n) {
  for (int i = 0; i<n; i++) {
    arr1[i] = arr1[i] * arr2[i];
  }
}

void arr_mirror1 (float* arr1, float* arr2, size_t n) {
  for (int i = 0; i<n; i++) {
    arr2[i] = 1 - arr1[i];
  }
}

void arr_probn (float* arr, size_t n) {
  float sum = 0;
  for (int i = 0; i<n; i++) {
    sum += arr[i];
  }
  for (int i = 0; i<n; i++) {
    arr[i] = arr[i] / sum;
  }
}

void setup() {
  servo1.attach(servopin);
  servo_step = 360 / num_positions;

  servo1.write(0);
  delay(15);    
  for (int i = 0; i < num_positions; i++) {
    beliefs[i] = 1.0 / num_positions;
    dis[i] = 0.0;
  }
  Serial1.begin(SERIAL_BAUD);  // UART to robot
  if (WiFi.beginAP(ssid) != WL_AP_LISTENING) {
    Serial.println("AP start failed");
    while (true)
      ;
  }

  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);

  server.begin();
  pinMode(enA_1, OUTPUT);
  pinMode(enA_2, OUTPUT);
  pinMode(in1_1, OUTPUT);
  pinMode(in2_1, OUTPUT);
  pinMode(in1_2, OUTPUT);
  pinMode(in2_2, OUTPUT);
  pinMode(in1_2, OUTPUT);
//  pinMode(encleft, INPUT_PULLUP);
  //pinMode(encright, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  if (autonomous_mode_objectfollow) {
    // 1) Sweep: sample distances at evenly spaced angles
    for (int i = 0; i < num_positions; i++) {
      int angle = i * servo_step;      // map index to angle
      servo1.write(angle);
      delay(100);
      dis[i] = read_ultrasonic(trig2, echo2); // cm
    }

    // 2) Normalize distances to 0..1 by dividing by max
    float maxVal = arr_max(dis, num_positions);
    if (maxVal <= 0.0001) maxVal = 1.0;
    arr_div(dis, num_positions, maxVal);

    // 3) Convert to likelihood: closer -> higher (1 - normalized distance)
    arr_mirror1(dis, prob, num_positions);

    // 4) Multiply prior (beliefs) by likelihoods (Bayes update)
    arr_mul(beliefs, prob, num_positions);

    // 5) Renormalize beliefs to sum to 1
    arr_probn(beliefs, num_positions);

    int best_idx =
      /* preferred if you fixed arr_maxi: */ arr_maxi(beliefs, num_positions);
      /* or, if you kept your old arr_maxi:  argmax_idx(beliefs, num_positions); */

    int best_angle = best_idx * servo_step;      // 0..180, 0 = rightmost
    int bin = best_angle / (180 / 8);            // 8 bins across the scan (0..7)
    if (bin < 0) bin = 0; if (bin > 7) bin = 7;

    switch (bin) {
      case 0: dir_right_hard();   break; // ~0°–22°
      case 1: dir_right();        break; // ~22°–45°
      case 2: dir_right_slight(); break; // ~45°–67°
      case 3: dir_forward_right();break; // ~67°–90°
      case 4: dir_forward_left(); break; // ~90°–112°
      case 5: dir_left_slight();  break; // ~112°–135°
      case 6: dir_left();         break; // ~135°–157°
      default: dir_left_hard();   break; // ~157°–180°
    }
  }
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 0b00000001) {
      // FORWARD FORWARD
      drive(lowVol, HIGH, lowVol, HIGH, speed, speed);
    } else if (command == 0b00000010) {
      // BACKWARD BACKWARD
      drive(HIGH, lowVol, HIGH, lowVol, speed, speed);
    } else if (command == 0b00000011) {
      // FORWARD STATIC
      drive(lowVol, HIGH, HIGH, HIGH, speed, speed);
    } else if (command == 0b00000100) {
      // STATIC FORWARD
      drive(HIGH, HIGH, lowVol, HIGH, speed, speed);
    } else if (command == 0b00000101) {
      // BACKWARD STATIC
      drive(HIGH, lowVol, HIGH, HIGH, speed, speed);
    } else if (command == 0b00000110) {
      // STATIC BACKWARD
      drive(HIGH, HIGH, HIGH, lowVol, speed, speed);
    } else if (command == 0b00000111) {
      // STATIC STATIC
      drive(HIGH, HIGH, HIGH, HIGH, speed, speed);
      digitalWrite(enA_1, HIGH);
      digitalWrite(enA_2, HIGH);
    } else if (command == 0b01000000) {
      // FORWARD BACKWARD
      drive(HIGH, speed, speed, HIGH, speed, speed);
    } else if (command == 0b01100000) {
      // BACKWARD FORWARD
      drive(speed, HIGH, HIGH, speed, speed, speed);
    } else if (command == 0b01110000) {
      //speed control init
      while (!Serial.available()) {}  // Wait for speed control signal
      char speedVal = Serial.read();
      speed = speedVal;
      Serial.print(speed);
    } else if (command == 0b01111111) {
      autonomous_mode = 1;
    }
  }

  WiFiClient client = server.available();
  if (!client) return;

  // Wait for request
  String req = client.readStringUntil('\r');
  client.readStringUntil('\n');  // discard

  if (req.startsWith("GET /api/cmd")) {
    int bIndex = req.indexOf("b=");
    if (bIndex > 0) {
      int val = req.substring(bIndex + 2).toInt();
      if (val >= 0 && val <= 255) {
        switch (val) {
          case 1:
            drive(lowVol, HIGH, lowVol, HIGH, speed, speed);
            break;
          case 4:
            drive(lowVol, HIGH, lowVol, lowVol, speed, speed);
            break;
          case 2:
            drive(HIGH, lowVol, HIGH, lowVol, speed, speed);
            break;
          case 5:
            drive(lowVol, lowVol, lowVol, HIGH, speed, speed);
            break;
          case 254:
            autonomous_mode_objectfollow = !autonomous_mode_objectfollow;
            break;
          case 255:
            autonomous_mode = !autonomous_mode;
            drive(lowVol, lowVol, lowVol, lowVol, speed, speed);
            break;
          default:
            drive(lowVol, lowVol, lowVol, lowVol, speed, speed);
        }
      }
    }
    client.println("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nOK");
  } else {  // root page
    String page = htmlPage();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close\r\n");
    client.print(page);
  }
  client.stop();
}

void drive(int p1, int p2, int p3, int p4, int power1, int power2) {
  analogWrite(enA_1, power1);
  analogWrite(enA_2, power2);

  digitalWrite(in1_1, p3);
  digitalWrite(in2_1, p4);

  digitalWrite(in1_2, p1);
  digitalWrite(in2_2, p2);
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