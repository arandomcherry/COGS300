  #define enA_1 10
  #define enA_2 11
  #define in1_1 2
  #define in2_1 4
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

  void goForward() {
    drive(0, HIGH, 0, HIGH, 133, 133);
  }

  void strongLeftTurn() {
    drive(0, HIGH, HIGH, 0, 170, 170);
  }

  void strongRightTurn() {
    drive(HIGH,0,0,HIGH, 170, 170);
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

  float sideData[4];
  int sideCounter = 0;
  float frontData[4];
  int frontCounter = 0;

  boolean autonomous_mode = 0;

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
    pinMode(encleft, INPUT_PULLUP);
    pinMode(encright, INPUT_PULLUP);
    Serial.begin(115200);
  }

  float arrAvg(const float* arr, size_t n) {
    if (!arr || n == 0) return NAN;  // avoid div-by-zero / bad input
    double sum = 0.0;                // better precision while summing
    for (size_t i = 0; i < n; ++i) {
      sum += arr[i];
    }
    return (float)(sum / (double)n);
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

    float cm1 = (duration1 / 2) / 29.1;
    float cm2 = (duration2 / 2) / 29.1;

  // emergency right turn
    if (cm1 > 1000 && cm2 > 600 && autonomous_mode) {
          Serial.println("strong right");
          strongRightTurn();
          delay(400);
          stopDrive();
          delay(5);
          goForward();
          delay(5);
    }
    if (sideCounter == 4 && frontCounter == 4) {
      sideCounter = 0;
      frontCounter = 0;

      float sideAvg = arrAvg(sideData, 4);
      float frontAvg = arrAvg(frontData, 4);

    
    Serial.print(sideAvg);
      Serial.print(" ");
      Serial.print(frontAvg);
      Serial.println();

      if (autonomous_mode) {
        // if (frontAvg > 600 && sideAvg > 70) {
        //   turnLeft(100);
        //   delay(20);
        //   goForward();
        //   delay(500);
        // } else 
        if (frontAvg < 25) {
          Serial.println("strong left");
          strongLeftTurn();
          delay(140);
          stopDrive();
          delay(5);
          goForward();
          delay(10);
        } else {
          if (sideAvg < 20) {
            Serial.println("left");
            turnLeft(120);
            delay(500);
            goForward();
            delay(180);
          } else if (sideAvg > 30 && sideAvg < 800) {
            Serial.println("right");
            turnRight(120);
            delay(100);
            goForward();
            delay(180);
          } else if (sideAvg > 800) {
          Serial.println("strong right");
          strongRightTurn();
          delay(400);
          stopDrive();
          delay(5);
          goForward();
          delay(5);
          } else {
            goForward();
          }
        }
      }
    }
    sideData[sideCounter] = cm1;
    sideCounter++;
    frontData[frontCounter] = cm2;
    frontCounter++;


    // int error = cm1 - 25;
    // if (error > 2) {
    //   if (autonomous_mode) {
    //     turnLeft(200);
    //   }
    //   Serial.print(" going left ");
    // } else if (error < -2) {
    //   if (autonomous_mode) {
    //     turnRight(200);
    //   }
    //   Serial.print(" going right ");
    // } else {
    //   if (autonomous_mode) {
    //     goForward();
    //   }
    //   Serial.print(" going forward ");
    // }
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
