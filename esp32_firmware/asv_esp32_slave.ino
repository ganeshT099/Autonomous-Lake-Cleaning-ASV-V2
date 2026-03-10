// =============================================================
// ASV ESP32 Slave Firmware
// Board  : ESP32 WROOM 38-pin
// Sensors: 3x JSN-SR04T Ultrasonic + DFRobot C4001 Radar
// Output : USB Serial to Jetson at 115200 baud
// Format : $ASV,F:0.82,R:1.20,L:1.45,RDR:1.20,SAFE*\n
// =============================================================

// ── PIN DEFINITIONS — fill these in ─────────────────────────
// Ultrasonic FRONT
#define TRIG_FRONT   // e.g. 23
#define ECHO_FRONT   // e.g. 22

// Ultrasonic RIGHT
#define TRIG_RIGHT   // e.g. 19
#define ECHO_RIGHT   // e.g. 18

// Ultrasonic LEFT
#define TRIG_LEFT    // e.g. 17
#define ECHO_LEFT    // e.g. 16

// DFRobot C4001 Radar — connected to ESP32 Serial2
#define RADAR_TX  17  // ESP32 TX2 → Radar RX
#define RADAR_RX  16  // ESP32 RX2 → Radar TX
// ─────────────────────────────────────────────────────────────

// Safety thresholds (metres)
#define DANGER_THRESHOLD  0.5
#define WARNING_THRESHOLD 1.0

// Radar serial
HardwareSerial RadarSerial(2);

// Store latest readings
float dist_front = 0.0;
float dist_right = 0.0;
float dist_left  = 0.0;
float dist_radar = 0.0;
bool  radar_presence = false;

// ── Ultrasonic read function ──────────────────────────────────
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0) return -1.0; // no echo

  float distance = (duration * 0.0343) / 2.0;
  if (distance < 0.25 || distance > 4.5) return -1.0;
  return distance;
}

// ── Radar parse function ──────────────────────────────────────
// DFRobot C4001 format: $JYBSS,1,0.82,0,0*FF
void readRadar() {
  if (RadarSerial.available()) {
    String line = RadarSerial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("$JYBSS")) {
      // Strip checksum
      int star = line.indexOf('*');
      if (star > 0) line = line.substring(0, star);
      // Parse fields
      int c1 = line.indexOf(',');
      int c2 = line.indexOf(',', c1 + 1);
      int c3 = line.indexOf(',', c2 + 1);
      if (c1 > 0 && c2 > 0) {
        int presence = line.substring(c1 + 1, c2).toInt();
        float dist   = line.substring(c2 + 1, c3).toFloat();
        radar_presence = (presence == 1);
        dist_radar     = radar_presence ? dist : 0.0;
      }
    }
  }
}

// ── Safety level ──────────────────────────────────────────────
String getSafetyLevel() {
  float vals[4] = {dist_front, dist_right, dist_left, dist_radar};
  float closest = 9999.0;
  for (int i = 0; i < 4; i++) {
    if (vals[i] > 0 && vals[i] < closest) closest = vals[i];
  }
  if (closest < DANGER_THRESHOLD)  return "DANGER";
  if (closest < WARNING_THRESHOLD) return "WARNING";
  return "SAFE";
}

// ── Checksum ──────────────────────────────────────────────────
uint8_t calcChecksum(String data) {
  uint8_t cs = 0;
  for (int i = 0; i < data.length(); i++) cs ^= data[i];
  return cs;
}

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  // USB Serial to Jetson
  Serial.begin(115200);

  // Radar Serial
  RadarSerial.begin(115200, SERIAL_8N1, RADAR_RX, RADAR_TX);

  // Ultrasonic pins
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);

  Serial.println("ASV ESP32 Slave Ready");
}

// ── Loop ──────────────────────────────────────────────────────
void loop() {
  // Read all sensors
  dist_front = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  delay(15); // avoid cross-talk
  dist_right = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  delay(15);
  dist_left  = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  delay(15);
  readRadar();

  // Build message
  String safety = getSafetyLevel();
  String msg = "ASV,";
  msg += "F:" + String(dist_front, 2) + ",";
  msg += "R:" + String(dist_right, 2) + ",";
  msg += "L:" + String(dist_left,  2) + ",";
  msg += "RDR:" + String(dist_radar, 2) + ",";
  msg += safety;

  // Send with checksum
  uint8_t cs = calcChecksum(msg);
  char csHex[3];
  sprintf(csHex, "%02X", cs);
  Serial.print("$" + msg + "*" + csHex + "\n");

  delay(100); // 10Hz
}
