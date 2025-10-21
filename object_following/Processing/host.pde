import processing.serial.*;

Serial port;
PrintWriter out;

/* ------------------- CONFIG ------------------- */
// Sliding window used to judge "wheel is running"
final int   WINDOW_MS          = 150;   // look this far back for pulses
final float RATE_THRESHOLD_HZ  = 2.0;   // pulses/sec needed to be "running"
final int   MIN_TICKS_IN_WIN   = 1;     // OR: running if >= this many pulses in window
// How often to re-evaluate state even without new serial input
final int   HEARTBEAT_MS       = 30;

/* ------------------- STATE ------------------- */
// For edge detection (incoming lines provide instantaneous 0/1)
int encLeftPrev  = -1;
int encRightPrev = -1;

// Recent pulse times (ms) per wheel, pruned to WINDOW_MS
ArrayList<Long> leftTicks  = new ArrayList<Long>();
ArrayList<Long> rightTicks = new ArrayList<Long>();

// Compressed-state logging (duration,left,right)
boolean haveSegState = false;
boolean segLeft  = false, segRight = false;
long    segStartMs = 0;

// For periodic checks
long lastHeartbeatMs = 0;

// Misc
boolean loggingReady = false;
int speed = 255;

/* ------------------- SETUP ------------------- */

void setup() {
  size(300, 200);
  println(Serial.list());
  String portName = Serial.list()[0];
  port = new Serial(this, portName, 115200);
  port.bufferUntil('\n');

  String fn = nf(year(),4)+nf(month(),2)+nf(day(),2)+"_"+nf(hour(),2)+nf(minute(),2)+nf(second(),2);
  out = createWriter("segments_"+fn+".csv");
  out.println("# duration_ms,left,right");
  out.flush();
  loggingReady = true;

  textAlign(LEFT, TOP);
  textSize(12);
}

/* ------------------- DRAW (heartbeat) ------------------- */

void draw() {
  background(250);
  fill(20);
  text("Sliding-window logging → segments CSV\n" +
       "Window: " + WINDOW_MS + " ms\n" +
       "Rate threshold: " + nf(RATE_THRESHOLD_HZ,0,2) + " Hz\n" +
       "Press 'p' to replay a file (replay.csv)\n" +
       "Press 'F' to flush the current open segment", 10, 10);

  // Heartbeat: even if no serial input, advance/compress states
  long now = millis();
  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    updateAndMaybeEmit(now);
    lastHeartbeatMs = now;
  }
}

/* ------------------- SERIAL: parse encoder and feed window ------------------- */

void serialEvent(Serial p) {
  String raw = p.readStringUntil('\n');
  if (raw == null) return;
  String line = trim(raw);
  if (line.length() == 0) return;

  try {
    if (line.charAt(0) == 'E') {
      // Expect: E<leftBit>#<rightBit> e.g. "E0#1"
      String payload = line.substring(1);
      String[] parts = split(payload, '#');
      if (parts != null && parts.length == 2) {
        int d1 = parseInt(parts[0].trim());
        int d2 = parseInt(parts[1].trim());

        long now = millis();

        // First sample initializes previous state
        if (encLeftPrev < 0 || encRightPrev < 0) {
          encLeftPrev = d1;
          encRightPrev = d2;
          segStartMs = now;   // start timing from first sample
          haveSegState = false;
          return;
        }

        // Rising edges → a pulse
        if (encLeftPrev == 0 && d1 == 1)  leftTicks.add(now);
        if (encRightPrev == 0 && d2 == 1) rightTicks.add(now);

        encLeftPrev = d1;
        encRightPrev = d2;

        // Update moving/static decision and emit segment changes if any
        updateAndMaybeEmit(now);
      }
    }
  } catch (Exception ex) {
    println("ERROR parsing line: '" + line + "' -> " + ex);
  }
}

/* ------------------- SLIDING-WINDOW DECISION + SEGMENT COMPRESSION ------------------- */

void pruneWindow(ArrayList<Long> buf, long now) {
  long cutoff = now - WINDOW_MS;
  // remove from head while too old
  while (!buf.isEmpty() && buf.get(0) < cutoff) {
    buf.remove(0);
  }
}

boolean isRunning(ArrayList<Long> buf, long now) {
  pruneWindow(buf, now);
  int n = buf.size();
  if (n >= MIN_TICKS_IN_WIN) return true;
  // Sliding-window average rate (Hz)
  float rate = (n * 1000.0f) / max(1, WINDOW_MS);
  return rate >= RATE_THRESHOLD_HZ;
}

void updateAndMaybeEmit(long now) {
  if (!loggingReady || out == null) return;

  boolean L = isRunning(leftTicks, now);
  boolean R = isRunning(rightTicks, now);

  if (!haveSegState) {
    // start first segment
    segLeft = L; segRight = R;
    segStartMs = now;
    haveSegState = true;
    return;
  }

  if (L != segLeft || R != segRight) {
    // state changed ⇒ emit previous segment duration
    float dur = max(0, now - segStartMs);
    out.println(dur + "," + (segLeft?1:0) + "," + (segRight?1:0));
    out.flush();

    // start new segment
    segLeft = L; segRight = R;
    segStartMs = now;
  }
}

void flushOpenSegment() {
  if (!loggingReady || out == null || !haveSegState) return;
  long now = millis();
  // Make sure the decision reflects any timeouts
  boolean L = isRunning(leftTicks, now);
  boolean R = isRunning(rightTicks, now);

  // If current view differs from stored seg, close old seg first
  if (L != segLeft || R != segRight) {
    float midDur = max(0, now - segStartMs);
    out.println(midDur + "," + (segLeft?1:0) + "," + (segRight?1:0));
    segLeft = L; segRight = R;
    segStartMs = now;
  }
  // Close the active segment
  float dur = max(0, now - segStartMs);
  out.println(dur + "," + (segLeft?1:0) + "," + (segRight?1:0));
  out.flush();
}

/* ------------------- KEY CONTROLS ------------------- */

void keyReleased() {
  // stop
  safeWrite(0b00000111);
}

void keyPressed() {
  if (key == 'w') {
    safeWrite(0b00000001); // forward
  } else if (key == 'a') {
    safeWrite(0b00000100); // left wheel only
  } else if (key == 's') {
    safeWrite(0b00000010); // backward (unused in replay)
  } else if (key == 'd') {
    safeWrite(0b00000101); // right wheel only
  } else if (key == 'z') {
    speed = constrain(speed - 30, 0, 255);
    safeWrite(0b01110000); safeWrite(speed);
    print(speed);
  } else if (key == 'x') {
    speed = constrain(speed + 30, 0, 255);
    safeWrite(0b11100000); safeWrite(speed);
    print(speed);
  } else if (key == 'p') {
    // optional: finalize any open segment before replaying something else
    flushOpenSegment();
    startReplay("replay.csv");  // expects [duration,left,right]
  } else if (key == 'm') {
    safeWrite(0b01111111);
  } else if (key == 'F') {
    // manual flush of current open segment to the CSV
    flushOpenSegment();
    println("Flushed current segment.");
  } else {
    println("Key pressed: " + key);
  }
}

void safeWrite(int b) {
  try { port.write(b); } catch (Exception e) { println("Serial write failed: " + e); }
}

/* ------------------- REPLAY: [duration,left,right] ------------------- */

void startReplay(final String csvPath) {
  new Thread(new Runnable() { public void run() { replayFromCSV(csvPath); } }).start();
}

// Map (leftMoving,rightMoving) -> your motor command bytes
int cmdFor(boolean left, boolean right) {
  if (left && right)   return 0b00000001; // forward
  if (!left && !right) return 0b00000111; // stop
  if (left && !right)  return 0b00000100; // left only (curve right)
  if (!left && right)  return 0b00000101; // right only (curve left)
  return 0b00000111;
}

void replayFromCSV(String csvPath) {
  try {
    String[] lines = loadStrings(csvPath);
    if (lines == null || lines.length == 0) {
      println("Replay: file empty or not found: " + csvPath);
      return;
    }

    int lastCmd = -999;

    for (String raw : lines) {
      if (raw == null) continue;
      String s = trim(raw);
      if (s.length() == 0 || s.startsWith("#")) continue;

      String[] p = split(s, ',');
      if (p.length < 3) { println("Replay: bad line (need duration,left,right): " + s); continue; }

      long duration = (long)Double.parseDouble(p[0].trim());
      boolean L = parseInt(p[1].trim()) != 0;
      boolean R = parseInt(p[2].trim()) != 0;

      int cmd = cmdFor(R, L);
      if (cmd != lastCmd) {
        try { port.write(cmd); } catch (Exception ignore) {}
        lastCmd = cmd;
      }

      // Hold the state for 'duration' ms
      delay((int)min(duration, (long)Integer.MAX_VALUE));
    }

    // ensure stop at the end
    try { port.write(0b00000111); } catch (Exception ignore) {}
    println("Replay: done.");
  } catch (Exception e) {
    println("Replay: error -> " + e);
  }
}

/* ------------------- CLEANUP ------------------- */

void dispose() {
  // Try to close out any open segment on exit
  flushOpenSegment();
  if (out != null) {
    out.flush();
    out.close();
  }
}
