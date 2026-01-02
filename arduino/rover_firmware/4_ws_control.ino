#include <Servo.h>

// ================== PIN DEFINITIONS ==================
#define FL_DIR 2
#define FL_PUL 3

#define FR_DIR 4
#define FR_PUL 5

#define RL_DIR 6
#define RL_PUL 7

#define RR_DIR 8
#define RR_PUL 9

// Servo pins
#define FL_SERVO A0
#define FR_SERVO A1
#define RL_SERVO A2
#define RR_SERVO A3

// ================== CONSTANTS ==================
const unsigned long PPR = 156522;

// ================== MOTOR STRUCT ==================
struct Motor {
  uint8_t pulPin;
  uint8_t dirPin;
  unsigned long interval;
  unsigned long lastTime;
  bool state;
};

Motor motors[4] = {
  {FL_PUL, FL_DIR, 0, 0, LOW},
  {FR_PUL, FR_DIR, 0, 0, LOW},
  {RL_PUL, RL_DIR, 0, 0, LOW},
  {RR_PUL, RR_DIR, 0, 0, LOW}
};

// ================== SERVOS ==================
Servo servos[4];

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);

  pinMode(FL_DIR, OUTPUT); pinMode(FL_PUL, OUTPUT);
  pinMode(FR_DIR, OUTPUT); pinMode(FR_PUL, OUTPUT);
  pinMode(RL_DIR, OUTPUT); pinMode(RL_PUL, OUTPUT);
  pinMode(RR_DIR, OUTPUT); pinMode(RR_PUL, OUTPUT);

  digitalWrite(FL_DIR, HIGH);
  digitalWrite(FR_DIR, HIGH);
  digitalWrite(RL_DIR, HIGH);
  digitalWrite(RR_DIR, HIGH);

  // Attach servos
  servos[0].attach(FL_SERVO);
  servos[1].attach(FR_SERVO);
  servos[2].attach(RL_SERVO);
  servos[3].attach(RR_SERVO);

  Serial.println("Stepper + Steering controller READY");
}

// ================== LOOP ==================
void loop() {
  readCommand();
  runMotors();
}

// ================== SERIAL PARSING ==================
void readCommand() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();

  Serial.print("RAW RX: ");
  Serial.println(line);

  // -------- Parse V --------
  int vIndex = line.indexOf('V');
  int sIndex = line.indexOf('S');

  if (vIndex == -1 || sIndex == -1) {
    Serial.println("Invalid format (V or S missing)");
    return;
  }

  // -------- Parse wheel velocities --------
  float w[4];
  int start = vIndex + 2;

  for (int i = 0; i < 4; i++) {
    int space = line.indexOf(' ', start);
    if (space == -1) return;
    w[i] = line.substring(start, space).toFloat();
    start = space + 1;
  }

  // -------- Parse servo angles --------
  float s[4];
  start = sIndex + 2;

  for (int i = 0; i < 4; i++) {
    int space = line.indexOf(' ', start);
    if (space == -1 && i < 3) return;
    s[i] = (i < 3) ? line.substring(start, space).toFloat()
                   : line.substring(start).toFloat();
    start = space + 1;
  }

  // -------- Apply steering --------
  for (int i = 0; i < 4; i++) {
  int angle = s[i];          // force +ve
  servos[i].write(angle);
  Serial.print(angle);
  }


  // -------- Apply wheel velocities --------
  for (int i = 0; i < 4; i++) {
    if (w[i] >= 0) digitalWrite(motors[i].dirPin, HIGH);
    else           digitalWrite(motors[i].dirPin, LOW);

    float omega = abs(w[i]);

    if (omega < 1e-6) {
      motors[i].interval = 0;
    } else {
      motors[i].interval =
        (unsigned long)(3.141592653e6 / (omega * PPR));
    }
  }

  Serial.println("Command applied");
}

// ================== NON-BLOCKING STEPPER RUN ==================
void runMotors() {
  unsigned long now = micros();

  for (int i = 0; i < 4; i++) {
    if (motors[i].interval == 0) continue;

    if (now - motors[i].lastTime >= motors[i].interval) {
      motors[i].lastTime = now;
      motors[i].state = !motors[i].state;
      digitalWrite(motors[i].pulPin, motors[i].state);
    }
  }
}