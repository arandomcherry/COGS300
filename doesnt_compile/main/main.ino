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

const char* ssid = "ARDUINO_FINAL_BOSS";  // open AP (no password)
WiFiServer server(80);

const int SERIAL_BAUD = 115200;

// ---------- Encoder pulse event buffer ----------
struct PulseEvent {
  uint32_t ms;  // server-side millis()
  uint8_t lr;   // bit0 = left rising, bit1 = right rising
};

const int EV_CAP = 128;

// Buffer is NON-volatile; indices are volatile
static PulseEvent evbuf[EV_CAP];
static volatile uint16_t ev_head = 0;  // next write
static volatile uint16_t ev_tail = 0;  // next read

inline void evPush(uint32_t ms, bool lRise, bool rRise) {
  uint8_t lr = (lRise ? 1 : 0) | (rRise ? 2 : 0);
  if (!lr) return;

  noInterrupts();  // protect indices (and paired write)
  uint16_t head = ev_head;
  uint16_t nxt = (uint16_t)((head + 1) % EV_CAP);
  if (nxt == ev_tail) {
    // full -> drop oldest
    ev_tail = (uint16_t)((ev_tail + 1) % EV_CAP);
  }

  // write event fields (no struct copy of volatile involved)
  evbuf[head].ms = ms;
  evbuf[head].lr = lr;

  ev_head = nxt;
  interrupts();
}

inline bool evPop(PulseEvent& out) {
  bool ok = false;
  noInterrupts();
  if (ev_tail != ev_head) {
    uint16_t tail = ev_tail;
    out.ms = evbuf[tail].ms;  // copy fields out
    out.lr = evbuf[tail].lr;
    ev_tail = (uint16_t)((tail + 1) % EV_CAP);
    ok = true;
  }
  interrupts();
  return ok;
}


volatile int prevLeft = -1, prevRight = -1;
// Command bytes
const uint8_t CMD_W = 0b00000001;
const uint8_t CMD_A = 0b00000100;
const uint8_t CMD_S = 0b00000010;
const uint8_t CMD_D = 0b00000101;
const uint8_t CMD_STOP = 0b00000111;

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
  Serial1.begin(SERIAL_BAUD);  // UART to robot
  if (WiFi.beginAP(ssid) != WL_AP_LISTENING) {
    Serial.println("AP start failed");
    while (true)
      ;
  }

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

  // Edge detection and event enqueue
  if (prevLeft < 0 || prevRight < 0) {
    prevLeft = leftEncoderVal;
    prevRight = rightEncoderVal;
  } else {
    bool lRise = (prevLeft == LOW && leftEncoderVal == HIGH);
    bool rRise = (prevRight == LOW && rightEncoderVal == HIGH);
    if (lRise || rRise) {
      evPush(millis(), lRise, rRise);
    }
    prevLeft = leftEncoderVal;
    prevRight = rightEncoderVal;
  }
  Serial.print('E');
  Serial.print(leftEncoderVal);
  Serial.print('#');
  Serial.println(rightEncoderVal);

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
      while (!Serial.available()) {}  // Wait for speed control signal
      char speedVal = Serial.read();
      speed = speedVal;
    }
  }
  WiFiClient client = server.available();
  if (!client) return;

  // Wait for request line
  String req = client.readStringUntil('\r');
  client.readStringUntil('\n');  // discard remainder of line

  auto getParam = [&](const String& key, int defVal) -> int {
    int idx = req.indexOf(key + "=");
    if (idx < 0) return defVal;
    // stop at space or & or end
    int start = idx + key.length() + 1;
    int end = req.indexOf(' ', start);
    int amp = req.indexOf('&', start);
    if (amp >= 0 && (end < 0 || amp < end)) end = amp;
    if (end < 0) end = req.length();
    return req.substring(start, end).toInt();
  };

  if (req.startsWith("GET /api/telemetry")) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close\r\n");

    // Drain all events currently in the buffer
    PulseEvent ev;
    int count = 0;
    while (evPop(ev)) {
      // P<l>#<r>@<ms>
      int l = (ev.lr & 1) ? 1 : 0;
      int r = (ev.lr & 2) ? 1 : 0;
      client.print("P");
      client.print(l);
      client.print("#");
      client.print(r);
      client.print("@");
      client.print(ev.ms);
      client.print("\n");
      count++;
    }

    // If no events happened, you may optionally emit a heartbeat
    // so the client can keep its timing fresh. Comment out if not desired.
    if (count == 0) {
      client.print("H@");
      client.print(millis());
      client.print("\n");
    }

    client.stop();
    return;
  }


  if (req.startsWith("GET /api/cmd")) {
    int bIndex = req.indexOf("b=");
    if (bIndex > 0) {
      int val = req.substring(bIndex + 2).toInt();

      // Optional speed override via HTTP, e.g. /api/cmd?b=112&speed=180
      int sp = getParam("speed", -1);
      if (sp >= 0 && sp <= 255) speed = sp;

      switch (val) {
        case 1:  // W
          drive(lowVol, speed, lowVol, speed);
          break;
        case 4:  // A  (right wheel only)
          drive(lowVol, lowVol, lowVol, speed);
          break;
        case 2:  // S
          drive(speed, lowVol, speed, lowVol);
          break;
        case 3:  // D  (left wheel only)
          drive(lowVol, speed, lowVol, lowVol);
          break;
        case 7:  // STOP
          drive(lowVol, lowVol, lowVol, lowVol);
          digitalWrite(enA_1, lowVol);
          digitalWrite(enA_2, lowVol);
          break;
        case 112:  // 0b01110000 speed-dec set already via ?speed=
        case 224:  // 0b11100000 speed-inc set already via ?speed=
          // no immediate motion change; just acknowledge
          break;
        default:
          // Fallback to stop for unknowns
          drive(lowVol, lowVol, lowVol, lowVol);
          break;
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