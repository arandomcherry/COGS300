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
struct PulseEvt {
  uint32_t ms;  // server-side millis()
  uint8_t lr;   // bit0 = left rising, bit1 = right rising
};

const int EV_CAP = 128;

// Buffer is NON-volatile; indices are volatile
static PulseEvt evbuf[EV_CAP];
static volatile uint16_t ev_head = 0;  // next write
static volatile uint16_t ev_tail = 0;  // next read

inline void pulsePush(uint32_t ms, bool lRise, bool rRise) {
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

inline bool pulsePop(PulseEvt& out) {
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
// ===== Queued replay (preload path and autonomous playback) =====
struct Segment { uint32_t dur_ms; uint8_t flags; }; // flags: bit0=left, bit1=right
const int QUEUE_CAP = 256;
Segment qbuf[QUEUE_CAP];
volatile uint16_t q_head = 0, q_tail = 0;
bool q_running = false;
bool q_hasCurrent = false;
Segment q_current;
uint32_t q_segStartMs = 0;

inline void qClear() {
  q_head = q_tail = 0;
  q_running = false;
  q_hasCurrent = false;
}

inline bool qPush(uint32_t dur_ms, uint8_t flags) {
  uint16_t nxt = (q_head + 1) % QUEUE_CAP;
  if (nxt == q_tail) return false;
  qbuf[q_head].dur_ms = dur_ms;
  qbuf[q_head].flags = flags;
  q_head = nxt;
  return true;
}

inline bool qPop(struct Segment &out) {
  if (q_tail == q_head) return false;
  out.dur_ms = qbuf[q_tail].dur_ms;
  out.flags  = qbuf[q_tail].flags;
  q_tail = (q_tail + 1) % QUEUE_CAP;
  return true;
}

inline void qApplyFlags(uint8_t f) {
  extern int speed;
  extern int lowVol;
  bool L = (f & 1) != 0;
  bool R = (f & 2) != 0;
  if (L && R)       { drive(lowVol, speed, lowVol, speed); }
  else if (!L && !R){ drive(lowVol, lowVol, lowVol, lowVol);
                      digitalWrite(enA_1, lowVol); digitalWrite(enA_2, lowVol); }
  else if (L && !R) { drive(lowVol, speed, lowVol, lowVol); }
  else if (!L && R) { drive(lowVol, lowVol, lowVol, speed); }
}

void qStart() {
  if (q_tail == q_head) { q_running = false; q_hasCurrent = false; return; }
  q_running = true; q_hasCurrent = false;
}

void qStop() {
  q_running = false; q_hasCurrent = false;
  qApplyFlags(0);
}

void qTick() {
  if (!q_running) return;
  uint32_t now = millis();
  if (!q_hasCurrent) {
    if (!qPop(q_current)) { q_running = false; qApplyFlags(0); return; }
    q_hasCurrent = true;
    q_segStartMs = now;
    qApplyFlags(q_current.flags);
  } else {
    uint32_t elapsed = now - q_segStartMs;
    if (elapsed >= q_current.dur_ms) {
      q_hasCurrent = false;
    }
  }
}


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
      pulsePush(millis(), lRise, rRise);
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
    PulseEvt ev;
    int count = 0;
    while (pulsePop(ev)) {
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


  

  // ---- Queue control endpoints ----
  if (req.startsWith("GET /api/q/clear")) {
    qClear();
    client.println("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nCLEARED");
    client.stop(); return;
  }

  if (req.startsWith("GET /api/q/start")) {
    qStart();
    client.println("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nSTARTED");
    client.stop(); return;
  }

  if (req.startsWith("GET /api/q/status")) {
    int count = (q_head >= q_tail) ? (q_head - q_tail) : (QUEUE_CAP - (q_tail - q_head));
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close\r\n");
    client.print("{\"queued\":"); client.print(count);
    client.print(",\"running\":"); client.print(q_running ? 1 : 0);
    client.print("}\n");
    client.stop(); return;
  }

  // Bulk push via POST: body contains many lines "dur,left,right\n"
  if (req.startsWith("POST /api/q/push")) {
    int len = contentLengthFromHeaders(client);
    String body;
    body.reserve(len);
    while (len-- > 0) { while (!client.available()) {} body += (char)client.read(); }

    int start=0;
    while (start < body.length()) {
      int nl = body.indexOf('\n', start);
      if (nl < 0) nl = body.length();
      String line = body.substring(start, nl);
      start = nl + 1;

      uint32_t dur; uint8_t flags;
      if (parseSegLine(line, dur, flags)) {
        qPush(dur, flags); // if full, newest overwrite can be added if you prefer
      }
    }
    client.println("HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\nOK");
    client.stop(); return;
  }

if (req.startsWith("GET /api/cmd")) {
    qStop(); // abort queued playback on manual command

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


  qTick();
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