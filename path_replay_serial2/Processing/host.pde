import processing.serial.*;
import java.util.HashSet;
import java.util.Set;

// Track motion keys currently held to avoid repeats
Set<Character> heldMotionKeys = new HashSet<Character>();

boolean isMotionKey(char k) {
  return k=='w' || k=='a' || k=='s' || k=='d';
}

Serial port;
boolean remote_mode = false;
String remote_address = "http://192.168.4.1/api/cmd";
boolean haveRemoteEpoch = false;
long remoteEpochServerMs = 0; // Arduino millis at epoch
long remoteEpochClientMs = 0; // Processing millis at same instant

PrintWriter out;

int   WINDOW_MS         = 150;
float RATE_THRESHOLD_HZ = 2.0;
int   MIN_TICKS_IN_WIN  = 1;
int   HEARTBEAT_MS      = 30;

// Remote telemetry polling
int TELEMETRY_POLL_MS = 10;
long lastPollMs = 0;
volatile boolean pollInFlight = false;

int encLeftPrev = -1;
int encRightPrev = -1;

// --- Fixed window (tumbling) counters ---
int leftCount = 0;
int rightCount = 0;
long windowStartMs = 0;   // local (mapped) time of the current window start
boolean windowInit = false;

// Compressed-state logging (duration,left,right)
boolean haveSegState = false;
boolean segLeft = false, segRight = false;
long segStartMs = 0;
// For periodic checks
long lastHeartbeatMs = 0;
// Misc
boolean loggingReady = false;
int speed = 255;

long mapServerMsToLocal(long serverMs) {
  if (!haveRemoteEpoch) {
    // If somehow called early, fall back to local clock
    return millis();
  }
  return remoteEpochClientMs + (serverMs - remoteEpochServerMs);
}
void setup() {
if (remote_mode) {
  WINDOW_MS         = 100;
  RATE_THRESHOLD_HZ = 1;
  HEARTBEAT_MS      = 30;
  TELEMETRY_POLL_MS = 10;
}

  size(300, 200);
  try {
    String portName = Serial.list()[0];
    port = new Serial(this, portName, 115200);
    port.bufferUntil('\n');
  }
  catch (Exception e) {
  }
  String fn = nf(year(), 4)+nf(month(), 2)+nf(day(), 2)+"_"+nf(hour(), 2)+nf(minute(), 2)+nf(second(), 2);
  out = createWriter("segments_"+fn+".csv");
  out.println("# duration_ms,left,right");
  out.flush();
  loggingReady = true;
  textAlign(LEFT, TOP);
  textSize(12);
} 

void draw() {
  background(250);
  fill(20);
  text("Sliding-window logging → segments CSV\n" + "Window: " + WINDOW_MS + " ms\n" + "Rate threshold: " + nf(RATE_THRESHOLD_HZ, 0, 2) + " Hz\n" + "Press 'p' to replay a file (replay.csv)\n" + "Press 'F' to flush the current open segment", 10, 10);
  // Heartbeat: even if no serial input, advance/compress states
  long now = millis();
  if (now - lastHeartbeatMs >= HEARTBEAT_MS) {
    updateAndMaybeEmit(now);
    lastHeartbeatMs = now;
  }
  // Remote telemetry polling
  if (remote_mode) {
    if (now - lastPollMs >= TELEMETRY_POLL_MS) {
      pollTelemetryOnce();
      lastPollMs = now;
    }
  }
}

void serialEvent(Serial p) {
  String raw = p.readStringUntil('\n');
  if (raw == null) return;
  handleIncomingLine(raw);
}

String telemetryUrl() {
  // derive http://.../api/telemetry from remote_address (which points to /api/cmd)
  String base = remote_address;
  int i = base.indexOf("/api/cmd");
  if (i >= 0) base = base.substring(0, i);
  if (!base.endsWith("/")) base += "/";
  return base + "api/telemetry";
}

void pollTelemetryOnce() {
  if (pollInFlight) return;
  pollInFlight = true;

  final String url = telemetryUrl();
  new Thread(new Runnable() {
    public void run() {
      try {
        String[] lines = loadStrings(url);
        if (lines != null) {
          for (String line : lines) {
            handleIncomingLine(line);
          }
        }
      }
      catch (Exception e) {
        // quiet network error; optional: 
        println("Telemetry HTTP error: " + e);
      }
      finally {
        pollInFlight = false;
      }
    }
  }
  ).start();
}


// Decide "running" for the *closed* window ending at (windowStartMs + WINDOW_MS)
boolean decideRunning(int count) {
  // Option A: count-only (simple & robust)
  if (count >= MIN_TICKS_IN_WIN) return true;

  // Option B: keep your rate as a secondary guard (optional)
  float rate = (count * 1000.0f) / max(1, WINDOW_MS);
  return rate >= RATE_THRESHOLD_HZ;
}

// Close the current window, update segment state, and start a new empty window.
// closeEndMs is the end time of the window being closed.
void closeWindow(long closeEndMs) {
  if (!loggingReady || out == null) return;

  boolean L = decideRunning(leftCount);
  boolean R = decideRunning(rightCount);

  if (!haveSegState) {
    segLeft = L;
    segRight = R;
    segStartMs = closeEndMs;   // start timing from end of first window
    haveSegState = true;
  } else if (L != segLeft || R != segRight) {
    float dur = max(0, closeEndMs - segStartMs);
    out.println(dur + "," + (segLeft?1:0) + "," + (segRight?1:0));
    out.flush();

    segLeft = L;
    segRight = R;
    segStartMs = closeEndMs;
  }

  // New empty window starts at closeEndMs
  windowStartMs = closeEndMs;
  leftCount = 0;
  rightCount = 0;
}

// Ensure our current window is advanced so that 't' lies inside it.
// If time jumped across multiple windows, we close each empty-in-between window.
void advanceTo(long t) {
  if (!windowInit) {
    windowStartMs = t - (t % WINDOW_MS); // align to grid (optional)
    leftCount = rightCount = 0;
    windowInit = true;
    return;
  }
  while (t >= windowStartMs + WINDOW_MS) {
    long closeEnd = windowStartMs + WINDOW_MS;
    closeWindow(closeEnd);
  }
}

void notePulseAt(boolean left, boolean right, long tLocal) {
  advanceTo(tLocal);
  if (left)  leftCount++;
  if (right) rightCount++;
}

void pruneWindow(ArrayList<Long> buf, long now) {
  long cutoff = now - WINDOW_MS; // remove from head while too old
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
  // No sliding prune; we just ensure windows close up to 'now'
  advanceTo(now);
  // Do not close the current *open* window here; that’s intentional.
  // We close it when time crosses the boundary (handled by advanceTo).
}

void handleIncomingLine(String line) {
  if (line == null) return;
  String s = trim(line);
  if (s.length() == 0) return;

  try {
    if (s.charAt(0) == 'P') { // P<l>#<r>@<ms>
      int hash = s.indexOf('#');
      int at   = s.indexOf('@');
      if (hash > 0 && at > hash) {
        int lPulse = parseInt(s.substring(1, hash).trim());
        int rPulse = parseInt(s.substring(hash + 1, at).trim());
        long srvMs = Long.parseLong(s.substring(at + 1).trim());

        if (!haveRemoteEpoch) {
          haveRemoteEpoch = true;
          remoteEpochServerMs = srvMs;
          remoteEpochClientMs = millis();
        }
        long tLocal = mapServerMsToLocal(srvMs);
        notePulseAt(lPulse != 0, rPulse != 0, tLocal);
        updateAndMaybeEmit(tLocal);
      }
      return;
    }

    if (s.charAt(0) == 'H') { // heartbeat H@ms (optional)
      int at = s.indexOf('@');
      if (at > 0) {
        long srvMs = Long.parseLong(s.substring(at + 1).trim());
        if (!haveRemoteEpoch) {
          haveRemoteEpoch = true;
          remoteEpochServerMs = srvMs;
          remoteEpochClientMs = millis();
        }
        long tLocal = mapServerMsToLocal(srvMs);
        updateAndMaybeEmit(tLocal);
      }
      return;
    }

    // Legacy E<leftBit>#<rightBit> (convert levels -> edges -> pulses)
    if (s.charAt(0) == 'E') {
      String[] parts = split(s.substring(1), '#');
      if (parts != null && parts.length == 2) {
        int d1 = parseInt(parts[0].trim());
        int d2 = parseInt(parts[1].trim());
        long now = millis();

        if (encLeftPrev < 0 || encRightPrev < 0) {
          encLeftPrev = d1;
          encRightPrev = d2;
          segStartMs = now;
          haveSegState = false;
          // also align the window
          advanceTo(now);
          return;
        }
        boolean lRise = (encLeftPrev == 0 && d1 == 1);
        boolean rRise = (encRightPrev == 0 && d2 == 1);
        encLeftPrev = d1;
        encRightPrev = d2;

        if (lRise || rRise) {
          notePulseAt(lRise, rRise, now);
        }
        updateAndMaybeEmit(now);
      }
      return;
    }

  } catch (Exception ex) {
    println("ERROR parsing line: '" + s + "' -> " + ex);
  }
}


void flushOpenSegment() {
  if (!loggingReady || out == null) return;
  long now = millis();

  // Advance so that any boundary up to 'now' is processed
  advanceTo(now);

  // Optionally close the current (partial) window so its counts contribute
  // Comment out if you want to keep current window “open” and not counted.
  long closeEnd = windowStartMs + WINDOW_MS;
  if (now >= closeEnd) {
    closeWindow(closeEnd);
  }

  if (haveSegState) {
    float dur = max(0, now - segStartMs);
    out.println(dur + "," + (segLeft?1:0) + "," + (segRight?1:0));
    out.flush();
  }
}


/* ------------------- KEY CONTROLS ------------------- */
void keyReleased() {
  char k = Character.toLowerCase(key);

  // Clear held set so the next press will send again
  if (isMotionKey(k)) {
    heldMotionKeys.remove(k);
  } else {
    // non-motion releases don't affect motion; no-op
  }

  // Keep your “release → stop” behavior
  if (remote_mode) {
    httpGetAsync("b=7");  // stop
  } else {
    safeWrite(0b00000111);
  }
}


void keyPressed() {
  char k = Character.toLowerCase(key);

  // If it's a motion key and it's already held, ignore repeat
  if (isMotionKey(k)) {
    if (heldMotionKeys.contains(k)) return; // already sent for this key
    heldMotionKeys.add(k);                   // first press → send
  }

  if (remote_mode) {
    if (k == 'w') {
      httpGetAsync("b=1");     // forward
    } else if (k == 'a') {
      httpGetAsync("b=4");     // left wheel only
    } else if (k == 's') {
      httpGetAsync("b=2");     // backward
    } else if (k == 'd') {
      httpGetAsync("b=5");     // right wheel only
    } else if (k == 'z') {
      speed = constrain(speed - 30, 0, 255);
      httpGetAsync("b=112&speed=" + speed); // 0b01110000
    } else if (k == 'x') {
      speed = constrain(speed + 30, 0, 255);
      httpGetAsync("b=224&speed=" + speed); // 0b11100000
    } else if (k == 'f') {
      flushOpenSegment();
      println("Flushed current segment.");
    } else if (k == 'p') {
      flushOpenSegment();
      uploadSegmentsCsv("replay.csv");
    } else {
      println("Key pressed: " + k);
    }
    return; // don't fall through to serial writes
  }

  // --- Local serial mode ---
  if (k == 'w') {
    safeWrite(0b00000001);
  } else if (k == 'a') {
    safeWrite(0b00000100);
  } else if (k == 's') {
    safeWrite(0b00000010);
  } else if (k == 'd') {
    safeWrite(0b00000011);
  } else if (k == 'z') {
    speed = constrain(speed - 30, 0, 255);
    safeWrite(0b01110000); safeWrite(speed);
  } else if (k == 'x') {
    speed = constrain(speed + 30, 0, 255);
    safeWrite(0b11100000); safeWrite(speed);
  } else if (k == 'p') {
    flushOpenSegment();
    uploadSegmentsCsv("replay.csv");
  } else if (k == 'f') {
    flushOpenSegment();
    println("Flushed current segment.");
  } else {
    println("Key pressed: " + k);
  }
}

void safeWrite(int b) {
  try {
    port.write(b);
  }
  catch (Exception e) {
    println("Serial write failed: " + e);
  }
} /* ------------------- REPLAY: [duration,left,right] ------------------- */
void startReplay(final String csvPath) {
  new Thread(new Runnable() {
    public void run() {
      replayFromCSV(csvPath);
    }
  }
  ).start();
} // Map (leftMoving,rightMoving) -> your motor command bytes
int cmdFor(boolean left, boolean right) {
  if (left && right) return 0b00000001;
  // forward
  if (!left && !right) return 0b00000111;
  // stop
  if (left && !right) return 0b00000011; // left only (curve right)
  if (!left && right) return 0b00000100; // right only (curve left)
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
      if (p.length < 3) {
        println("Replay: bad line (need duration,left,right): " + s);
        continue;
      }

      long duration = (long)Double.parseDouble(p[0].trim());
      boolean L = parseInt(p[1].trim()) != 0;
      boolean R = parseInt(p[2].trim()) != 0;

      int cmd = cmdFor(R, L); // keep your existing mapping
      if (cmd != lastCmd) {
        try {
          if (remote_mode) {
            // send over Wi-Fi
            httpGetAsync("b=" + cmd);
          } else {
            // send over serial
            port.write(cmd);
          }
        }
        catch (Exception ignore) {
        }
        lastCmd = cmd;
      }

      // Hold the state for 'duration' ms
      delay((int)min(duration, (long)Integer.MAX_VALUE));
    }

    // ensure stop at the end
    try {
      if (remote_mode) {
        httpGetAsync("b=7"); // 0b00000111
      } else {
        port.write(0b00000111);
      }
    }
    catch (Exception ignore) {
    }

    println("Replay: done.");
  }
  catch (Exception e) {
    println("Replay: error -> " + e);
  }
}

// --- add this helper ---
void httpGetAsync(final String query) {
  println(query);
  new Thread(new Runnable() {
    public void run() {
      try {
        String url = remote_address + (remote_address.contains("?") ? "&" : "?") + query;
        String[] lines = loadStrings(url);
        if (lines != null) for (String line : lines) println(line);
      }
      catch (Exception e) {
        println("HTTP error: " + e);
      }
    }
  }
  ).start();
}

// --- HTTP POST helper to send bulk segment data to Arduino ---
void httpPostAsync(final String path, final String body) {
  new Thread(new Runnable() {
    public void run() {
      try {
        // Derive base http://.../ from remote_address (which points to /api/cmd)
        String base = remote_address;
        int i = base.indexOf("/api/cmd");
        if (i >= 0) base = base.substring(0, i);
        if (!base.endsWith("/")) base += "/";
        String urlStr = base + path;  // e.g. "api/q/push"

        java.net.URL url = new java.net.URL(urlStr);
        java.net.HttpURLConnection conn = (java.net.HttpURLConnection) url.openConnection();
        conn.setRequestMethod("POST");
        conn.setDoOutput(true);
        byte[] bytes = body.getBytes("UTF-8");
        conn.setFixedLengthStreamingMode(bytes.length);
        conn.setRequestProperty("Content-Type", "text/plain; charset=UTF-8");
        conn.connect();
        java.io.OutputStream os = conn.getOutputStream();
        os.write(bytes);
        os.flush();
        os.close();
        conn.getInputStream().close();
        conn.disconnect();
      } catch (Exception e) {
        println("HTTP POST error: " + e);
      }
    }
  }).start();
}

// --- Upload segments CSV to Arduino queue and start playback ---
void uploadSegmentsCsv(String csvPath) {
  try {
    // 1) Clear queue
    httpGetAsync("api/q/clear");

    // 2) Read file and batch POST ~4KB chunks
    String[] lines = loadStrings(csvPath);
    if (lines == null || lines.length == 0) { println("Upload: file empty or not found: " + csvPath); return; }

    StringBuilder batch = new StringBuilder();
    int bytesLimit = 4096;
    int pushed = 0;

    for (String raw : lines) {
      if (raw == null) continue;
      String s = trim(raw);
      if (s.length() == 0 || s.startsWith("#")) continue;

      String[] p = split(s, ',');
      if (p.length < 3) continue;

      long dur = (long)Double.parseDouble(p[0].trim());
      int L = parseInt(p[1].trim());
      int R = parseInt(p[2].trim());
      if (dur < 0) dur = 0;

      String line = (dur + "," + (L!=0?1:0) + "," + (R!=0?1:0) + "\n");
      if (batch.length() + line.length() > bytesLimit) {
        httpPostAsync("api/q/push", batch.toString());
        batch.setLength(0);
      }
      batch.append(line);
      pushed++;
    }
    if (batch.length() > 0) httpPostAsync("api/q/push", batch.toString());

    println("Upload queued " + pushed + " segments.");
    // 3) Start
    httpGetAsync("api/q/start");
  } catch (Exception e) {
    println("Upload error: " + e);
  }
}
/* ------------------- CLEANUP ------------------- */
void dispose() {
  flushOpenSegment();
  if (out != null) {
    out.flush();
    out.close();
  }
}
