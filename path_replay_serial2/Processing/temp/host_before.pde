import processing.serial.*;

Serial port;
PrintWriter out;

int encLeftPrev = 0;
int encRightPrev = 0;
boolean haveEncPrev = false;

boolean loggingReady = false;
boolean warning = false;
char speed = 255;

void setup() {
  size(300, 200);

  println(Serial.list()); 
  String portName = Serial.list()[0]; 
  port = new Serial(this, portName, 115200);
  port.bufferUntil('\n');
  
  String fn = nf(year(),4)+nf(month(),2)+nf(day(),2)+"_"+nf(hour(),2)+nf(minute(),2)+nf(second(),2);
  out = createWriter("arduino_log_"+fn+".csv");
  out.println("t_ms,d1,d2");
  out.flush();
  loggingReady = true;
}

void draw() {

}

void serialEvent(Serial p) {
    if (!loggingReady || out == null) {
      return;
    }

  String raw = p.readStringUntil('\n');
  if (raw == null || raw.length() == 0) return;
  String line = trim(raw);
  try {
    if (line.charAt(0) == 'E') {
      // E = Encoder telemetry
      line = line.substring(1);
      String[] parts = split(line, '#');
      if (parts.length == 2) {
        int d1 = int(parts[0]);
        int d2 = int(parts[1]);
        
        if (!haveEncPrev) {
          encLeftPrev = d1;
          encRightPrev = d2;
          haveEncPrev = true;
          return;
        }
        int leftMov = 0;
        int rightMov = 0;
        if (encLeftPrev == 0 && d1 == 1) {
          leftMov = 1;
        }        
        if (encRightPrev == 0 && d2 == 1) {
          rightMov = 1;
        }        

        out.println(millis()+","+leftMov+","+rightMov);
        out.flush();
        
        encLeftPrev = d1;
        encRightPrev = d2;
      }
    }
  } catch (Exception ex) {
    // Any exception here would otherwise disable serialEvent() — keep it alive
    println("ERROR parsing line: '" + line + "' -> " + ex);
  }
}

void keyReleased() {
  port.write(0b00000111);
}


void keyPressed() {
  if (key == 'w') {
     port.write(0b00000001);
  } else if (key == 'a') {
     port.write(0b00000100);
  } else if (key == 's') {
    port.write(0b00000010);
  } else if (key == 'd') {
    port.write(0b00000101);
  } else if (key == 'q') {
    println("leftrot");
    port.write(0b01000000);
  } else if (key == 'e') {
    println("rightwrot");
    port.write(0b01100000);
  } else if (key == 'z') {
    port.write(0b01110000);
    speed -= 30;
    if (speed <= 0) {
    speed = 0;
    }
    port.write(speed);
  } else if (key == 'p') {
    startReplay("replay.csv");
  } else if (key == 'x') {
    port.write(0b11100000);
    speed += 30;
    if (speed >= 255) {
    speed = 255;
    }
    port.write(speed);
  } else {
    println("Key pressed: " + key);
  }
}

void startReplay(final String csvPath) {
  // Run the replay in a background thread
  new Thread(new Runnable() {
    public void run() { replayFromCSV(csvPath); }
  }).start();
}

// Map (left,right) -> your existing control bytes
int cmdFor(int left, int right) {
  if (left == 1 && right == 1) return 0b00000001; // forward (like 'w')
  if (left == 0 && right == 0) return 0b00000111; // stop (like keyReleased)
  if (left == 1 && right == 0) return 0b00000100; // turn left (like 'a')
  if (left == 0 && right == 1) return 0b00000101; // turn right (like 'd')
  return 0b00000111; // default to stop on weird values
}

void replayFromCSV(String csvPath) {
  try {
    String[] lines = loadStrings(csvPath); // put file in sketch folder or give absolute path
    if (lines == null || lines.length == 0) {
      println("Replay: file empty or not found: " + csvPath);
      return;
    }

    // If first line is a header, skip it automatically
    int startIdx = 0;
    if (lines[0].toLowerCase().contains("t") || lines[0].toLowerCase().contains("left")) {
      startIdx = 1;
    }

    long lastT = -1;
    for (int i = startIdx; i < lines.length; i++) {
      String line = trim(lines[i]);
      if (line.length() == 0) continue;     // skip blanks
      if (line.startsWith("#")) continue;   // skip comments

      String[] parts = split(line, ',');
      if (parts == null || parts.length < 3) {
        println("Replay: bad line (need t,left,right): " + line);
        continue;
      }

      long t = (long)Double.parseDouble(parts[0].trim()); // tolerate floats in time
      int left = parseInt(parts[1].trim());
      int right = parseInt(parts[2].trim());

      // Delay by delta from previous timestamp
      if (lastT >= 0) {
        float delta = max(0, t - lastT);
        // Cap to int for delay(); large deltas still okay
        delay((int)min(delta, (long)Integer.MAX_VALUE));
      }
      lastT = t;

      int cmd = cmdFor(left, right);
      try {
        port.write(cmd);
      } catch (Exception e) {
        println("Replay: serial write failed at t=" + t + " -> " + e);
        // optional: break; 
      }
    }

    // Ensure we end stopped
    try { port.write(0b00000111); } catch (Exception ignore) {}

    println("Replay: done (" + (lines.length - startIdx) + " steps).");
  } catch (Exception e) {
    println("Replay: error -> " + e);
  }
}
