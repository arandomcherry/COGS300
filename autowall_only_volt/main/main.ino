#define enA_1 10  
#define enA_2 11
#define in1_1 4
#define in2_1 2 
#define in1_2 7 
#define in2_2 8
#define encleft 12
#define encright 13
#define trig1 3
#define trig2 6
#define echo1 5
#define echo2 9
//sensor 1 side, sensor 2 front
#include <WiFiS3.h>
IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GW(192, 168, 4, 1);
IPAddress AP_SN(255, 255, 255, 0);

int turning_counter = 0;
int forward_counter = 0;
//int COOLDOWN_TURNING_THRESHOLD = 3;
//int COOLDOWN_FORWARD_THRESHOLD = 16;

boolean autonomous_mode = 0;

const char* ssid = "ARDUINO_FINAL_BOSS";   // open AP (no password)
WiFiServer server(80);

const int SERIAL_BAUD = 115200;

// Command bytes
const uint8_t CMD_W     = 0b00000001;
const uint8_t CMD_A     = 0b00000100;
const uint8_t CMD_S     = 0b00000010;
const uint8_t CMD_D     = 0b00000011;
const uint8_t CMD_STOP  = 0b00000111;

String htmlPage() {
  return R"HTML(
<!doctype html><html><head><meta charset="utf-8"/>
<title>WASD Control</title>
<script>
const CMD={W:1,A:4,S:2,D:5,STOP:7, M:255};
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
<p>Use keyboard W/A/S/D. Releases → Stop.</p>
</body></html>
)HTML";
}

int speed = 255;
int lowVol = 0;

void setup() {
  Serial1.begin(SERIAL_BAUD); // UART to robot
  if (WiFi.beginAP(ssid) != WL_AP_LISTENING) { Serial.println("AP start failed"); while (true); }

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
  pinMode(encleft, INPUT_PULLUP);
  pinMode(encright, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  digitalWrite(trig1, LOW);
  delayMicroseconds(5);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
  unsigned long duration1 = pulseIn(echo1, HIGH);
  digitalWrite(trig2, LOW);
  delayMicroseconds(5);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig2, LOW);
  unsigned long duration2 = pulseIn(echo2, HIGH);

  float cm1 = (duration1/2) / 29.1;
  float cm2 = (duration2/2) / 29.1;

  if (autonomous_mode) {

    // Test: voltage control

    int vol1, vol2;
    if (cm1 <= 25) {
      vol1 = 255 - map(cm1, 0, 25, 64, 255);
    } else if (cm1 >= 40) {
      vol2 = map(cm1, 40, 255, 64, 255);
    } else {
      vol1 = 255;
      vol2 = 255;
    }
    drive(speed, lowVol, speed, lowVol, vol1, vol2);

    // Old code, with cooldown control
    // if (cm1 <= 25) {
    //   if (turning_counter <= COOLDOWN_TURNING_THRESHOLD && forward_counter >= COOLDOWN_FORWARD_THRESHOLD) {
    //     drive(speed, lowVol, speed, lowVol);
    //     turning_counter ++;
    //     forward_counter = min(forward_counter - 1, 0);
    //   } else {
    //     drive(lowVol, lowVol, speed, lowVol);
    //     turning_counter = min(turning_counter - 1, 0);
    //     forward_counter = max(forward_counter + 1, COOLDOWN_FORWARD_THRESHOLD);
    //   }
    // } else if (cm1 > 40) {
    //   if (turning_counter <= COOLDOWN_TURNING_THRESHOLD && forward_counter >= COOLDOWN_FORWARD_THRESHOLD) {
    //     drive(speed, lowVol, lowVol, lowVol);
    //     turning_counter ++;
    //     forward_counter = min(forward_counter - 1, 0);
    //   } else {
    //     drive(speed, lowVol, lowVol, lowVol);
    //     turning_counter = min(turning_counter - 1, 0);
    //     forward_counter = max(forward_counter + 1, COOLDOWN_FORWARD_THRESHOLD);
    //   }
    // } else {
    //   drive(speed, lowVol, speed, lowVol);
    //   forward_counter = max(forward_counter + 1, COOLDOWN_FORWARD_THRESHOLD);
    // }
  } 
  Serial.print("Side: ");  
  Serial.print(cm1);
  Serial.print(" Front: ");  
  Serial.print(cm2);
  Serial.println();  
  delay(100);
  int leftEncoderVal = digitalRead(encleft);
  int rightEncoderVal = digitalRead(encright);
  // Serial.print('E');
  // Serial.print(leftEncoderVal); 
  // Serial.print('#');
  // Serial.print(rightEncoderVal);
  // Serial.println();
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 0b00000001) {
      // FORWARD FORWARD
      drive(HIGH, lowVol, HIGH, lowVol, speed, speed);
    } else if (command == 0b00000010) {
      // BACKWARD BACKWARD
      drive(lowVol, HIGH, lowVol, HIGH, speed, speed);
    } else if (command == 0b00000011) {
      // FORWARD STATIC
      drive(HIGH, lowVol, lowVol, lowVol, speed, speed);
    } else if (command == 0b00000100) {
      // STATIC FORWARD
      drive(lowVol, lowVol, HIGH, lowVol, speed, speed);
    } else if (command == 0b00000101) {
      // BACKWARD STATIC
      drive(lowVol, HIGH, lowVol, lowVol, speed, speed);
    } else if (command == 0b00000110) {
      // STATIC BACKWARD
      drive(lowVol, lowVol, lowVol, HIGH, speed, speed);
    } else if (command == 0b00000111) {
      // STATIC STATIC
      drive(lowVol, lowVol, lowVol, lowVol, speed, speed);
      digitalWrite(enA_1, lowVol);
      digitalWrite(enA_2, lowVol);
    } else if (command == 0b01000000) {
      // FORWARD BACKWARD
      drive(lowVol, speed, speed, lowVol, speed, speed);
    } else if (command == 0b01100000) {
      // BACKWARD FORWARD
      drive(speed, lowVol, lowVol, speed, speed, speed);
    } else if (command == 0b01110000) {
      //speed control init
      while (!Serial.available()) {} // Wait for speed control signal
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
  client.readStringUntil('\n'); // discard

  // Very basic parsing
  if (req.startsWith("GET /api/cmd")) {
    int bIndex = req.indexOf("b=");
    if (bIndex > 0) {
      int val = req.substring(bIndex + 2).toInt();
      if (val >= 0 && val <= 255) {
        switch (val) {
	case 1:
          drive(HIGH, lowVol, HIGH, lowVol, speed, speed);
          break;
	case 4:
          drive(lowVol, lowVol, HIGH, lowVol, speed, speed);
          break;
	case 2:
          drive(lowVol, HIGH, lowVol, HIGH, speed, speed);
          break;
	case 5:
          drive(HIGH, lowVol, lowVol, lowVol, speed, speed);
          break;
	case 255:
          autonomous_mode = !autonomous_mode;
          break;
	default:
          drive(lowVol, lowVol, lowVol, lowVol, speed, speed);
        } 
      }
    }
    client.println("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nOK");
  } else { // root page
    String page = htmlPage();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("Connection: close\r\n");
    client.print(page);
  }
  client.stop();
  // delay(20); // remove because of echo delay
}

void drive(int p1, int p2, int p3, int p4, int power1, int power2) {
  analogWrite(enA_1, power1);
  analogWrite(enA_2, power2);

  digitalWrite(in1_1, p3);
  digitalWrite(in2_1, p4);

  digitalWrite(in1_2, p1);
  digitalWrite(in2_2, p2);
}