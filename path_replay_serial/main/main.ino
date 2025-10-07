#define enA_1 10  
#define enA_2 11
#define in1_1 4
#define in2_1 2 
#define in1_2 7 
#define in2_2 8
#define encleft 12
#define encright 13

#include <WiFiS3.h>
IPAddress AP_IP(192, 168, 4, 1);
IPAddress AP_GW(192, 168, 4, 1);
IPAddress AP_SN(255, 255, 255, 0);

const char* ssid = "ARDUINO_FINAL_BOSS";   // open AP (no password)
WiFiServer server(80);

const int SERIAL_BAUD = 115200;

// Command bytes
const uint8_t CMD_W     = 0b00000001;
const uint8_t CMD_A     = 0b00000100;
const uint8_t CMD_S     = 0b00000010;
const uint8_t CMD_D     = 0b00000101;
const uint8_t CMD_STOP  = 0b00000111;

String htmlPage() {
  return R"HTML(
<!doctype html><html><head><meta charset="utf-8"/>
<title>WASD Control</title>
<script>
const CMD={W:1,A:4,S:2,D:5,STOP:7};
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
    int leftEncoderVal = digitalRead(encleft);
    int rightEncoderVal = digitalRead(encright);
    Serial.print('E');
    Serial.print(leftEncoderVal); 
    Serial.print('#');
    Serial.print(rightEncoderVal);
    Serial.println();
    if (Serial.available()) {
    char command = Serial.read();
    if (command == 0b00000001) {
      // FORWARD FORWARD
      drive(lowVol, speed, lowVol, speed);
    } else if (command == 0b00000010) {
      // BACKWARD BACKWARD
      drive(speed, lowVol, speed, lowVol);
    } else if (command == 0b00000011) {
      // FORWARD STATIC
      drive(lowVol, speed, lowVol, lowVol);
    } else if (command == 0b00000100) {
      // STATIC FORWARD
      drive(lowVol, lowVol, lowVol, speed);
    } else if (command == 0b00000101) {
      // BACKWARD STATIC
      drive(speed, lowVol, lowVol, lowVol);
    } else if (command == 0b00000110) {
      // STATIC BACKWARD
      drive(lowVol, lowVol, speed, lowVol);
    } else if (command == 0b00000111) {
      // STATIC STATIC
      drive(lowVol, lowVol, lowVol, lowVol);
      digitalWrite(enA_1, lowVol);
      digitalWrite(enA_2, lowVol);
    } else if (command == 0b01000000) {
      // FORWARD BACKWARD
      drive(speed, lowVol, lowVol, speed);
    } else if (command == 0b01100000) {
      // BACKWARD FORWARD
      drive(lowVol, speed, speed, lowVol);
    } else if (command == 0b01110000) {
      //speed control init
      while (!Serial.available()) {} // Wait for speed control signal
      char speedVal = Serial.read();
      speed = speedVal;
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
          drive(lowVol, speed, lowVol, speed);
          break;
          case 4:
          drive(lowVol, lowVol, lowVol, speed);
          break;
          case 2:
          drive(speed, lowVol, speed, lowVol);
          break;
          case 5:
          drive(lowVol, lowVol, speed, lowVol);
          break;
          default:
          drive(lowVol, lowVol, lowVol, lowVol);
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

  delay(20);
}

void drive(int p1, int p2, int p3, int p4) {
      digitalWrite(enA_1, HIGH);
      digitalWrite(enA_2, HIGH);

      digitalWrite(in1_1, p1);
      digitalWrite(in2_1, p2);

      digitalWrite(in1_2, p3);
      digitalWrite(in2_2, p4);
}