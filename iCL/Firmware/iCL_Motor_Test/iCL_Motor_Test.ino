/*
  iCL_Motor_Test

  Teensy 4.1 serial console for jogging a StepperOnline iCL57/60 integrated
  closed-loop stepper through its PUL/DIR inputs.

  Wiring: NPN sinking, common-anode. A Teensy pin HIGH turns the transistor on,
  which turns the drive's optocoupler on. Both pins idle LOW.

  Drive timing (iCL Series User Manual, section 3.1):
    - PUL pulse width >= 2.5 us        -> this sketch uses 10 us pulses
    - DIR set >= 5 us before next PUL  -> this sketch waits at least 10 us
    - max 200 kHz pulse rate           -> this sketch caps at kMaxStepRate

  Type "help" in the Serial Monitor (115200, newline line ending) for commands.
*/

// ---------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------

const uint8_t kPulPin = 39;
const uint8_t kDirPin = 33;

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------

const uint32_t kSerialBaud = 115200;

const uint32_t kTickMicros = 10;                       // step ISR period, also the pulse width
const float kTickSeconds = kTickMicros / 1000000.0f;
const float kMaxStepRate = 40000.0f;                   // steps/s; ISR ceiling is 50k

const uint32_t kRampIntervalMicros = 1000;
const uint32_t kStreamIntervalMs = 50;                 // 20 Hz

const uint32_t kDefaultStepsPerRev = 400;              // iCL57/60 default, SW1-SW4 all on
const float kDefaultAccelRpmPerSec = 300.0f;
const float kDefaultDecelRpmPerSec = 300.0f;
const float kDefaultMaxRpm = 600.0f;
const float kHardMaxRpm = 3000.0f;

// ---------------------------------------------------------------------------
// Step generator state (shared with the ISR)
// ---------------------------------------------------------------------------

IntervalTimer stepTimer;

volatile float isrStepRate = 0.0f;   // signed steps/s; positive = forward
volatile int32_t stepCount = 0;
volatile bool dirInvert = false;

bool pulseHigh = false;              // ISR-only
bool dirForward = false;             // ISR-only; matches DIR LOW at boot
float stepPhase = 0.0f;              // ISR-only

// ---------------------------------------------------------------------------
// Motion state
// ---------------------------------------------------------------------------

enum MotionState { STATE_IDLE, STATE_ACCEL, STATE_DECEL, STATE_CRUISE };
const char* const kStateNames[] = {"idle", "accel", "decel", "cruise"};

float targetRpm = 0.0f;
float currentRpm = 0.0f;
float accelRpmPerSec = kDefaultAccelRpmPerSec;
float decelRpmPerSec = kDefaultDecelRpmPerSec;
float maxRpm = kDefaultMaxRpm;
uint32_t stepsPerRev = kDefaultStepsPerRev;
MotionState motionState = STATE_IDLE;

bool streamEnabled = false;

char serialCommandBuffer[48] = {0};
uint8_t serialCommandLength = 0;

elapsedMicros rampTimer;
elapsedMillis streamTimer;

void setup() {
  pinMode(kPulPin, OUTPUT);
  pinMode(kDirPin, OUTPUT);
  digitalWriteFast(kPulPin, LOW);
  digitalWriteFast(kDirPin, LOW);

  Serial.begin(kSerialBaud);
  delay(250);

  stepTimer.begin(stepTick, kTickMicros);
  stepTimer.priority(32);

  Serial.println();
  Serial.println("# iCL_Motor_Test");
  Serial.print("# PUL pin ");
  Serial.print(kPulPin);
  Serial.print(", DIR pin ");
  Serial.println(kDirPin);
  printCommandHelp();
}

void loop() {
  processSerialInput();

  if (rampTimer >= kRampIntervalMicros) {
    const float dt = rampTimer / 1000000.0f;
    rampTimer = 0;
    updateRamp(dt);
  }

  if (streamEnabled && streamTimer >= kStreamIntervalMs) {
    streamTimer = 0;
    printStreamLine();
  }
}

// ---------------------------------------------------------------------------
// Step ISR
//
// Runs every kTickMicros. A pulse is HIGH for exactly one tick. When the
// direction changes, DIR is written and the tick is skipped, so DIR always
// leads the next pulse by at least one tick.
// ---------------------------------------------------------------------------

void stepTick() {
  if (pulseHigh) {
    digitalWriteFast(kPulPin, LOW);
    pulseHigh = false;
    return;
  }

  const float rate = isrStepRate;

  if (rate == 0.0f) {
    stepPhase = 0.0f;
    return;
  }

  const bool forward = rate > 0.0f;

  if (forward != dirForward) {
    dirForward = forward;
    digitalWriteFast(kDirPin, (forward != dirInvert) ? HIGH : LOW);
    stepPhase = 0.0f;
    return;
  }

  stepPhase += fabsf(rate) * kTickSeconds;

  if (stepPhase >= 1.0f) {
    stepPhase -= 1.0f;
    digitalWriteFast(kPulPin, HIGH);
    pulseHigh = true;
    stepCount += forward ? 1 : -1;
  }
}

// ---------------------------------------------------------------------------
// Ramping
// ---------------------------------------------------------------------------

// Moves currentRpm toward targetRpm. Speeding up uses accel; slowing down,
// including slowing to 0 before a reversal, uses decel.
void updateRamp(float dt) {
  if (currentRpm == targetRpm) {
    motionState = (currentRpm == 0.0f) ? STATE_IDLE : STATE_CRUISE;
    applyStepRate();
    return;
  }

  const bool reversing = (currentRpm > 0.0f && targetRpm < 0.0f) ||
                         (currentRpm < 0.0f && targetRpm > 0.0f);
  const bool slowing = currentRpm != 0.0f &&
                       (reversing || fabsf(targetRpm) < fabsf(currentRpm));

  const float goal = reversing ? 0.0f : targetRpm;
  const float rate = slowing ? decelRpmPerSec : accelRpmPerSec;
  const float change = rate * dt;
  const float remaining = goal - currentRpm;

  if (fabsf(remaining) <= change) {
    currentRpm = goal;
  } else {
    currentRpm += (remaining > 0.0f) ? change : -change;
  }

  motionState = slowing ? STATE_DECEL : STATE_ACCEL;
  applyStepRate();
}

void applyStepRate() {
  float rate = currentRpm * stepsPerRev / 60.0f;

  if (rate > kMaxStepRate) {
    rate = kMaxStepRate;
  } else if (rate < -kMaxStepRate) {
    rate = -kMaxStepRate;
  }

  isrStepRate = rate;
}

void emergencyStop() {
  targetRpm = 0.0f;
  currentRpm = 0.0f;
  isrStepRate = 0.0f;
  motionState = STATE_IDLE;
}

bool motorStopped() {
  return currentRpm == 0.0f && targetRpm == 0.0f;
}

float clampRpm(float rpm) {
  if (rpm > maxRpm) {
    return maxRpm;
  }

  if (rpm < -maxRpm) {
    return -maxRpm;
  }

  return rpm;
}

// ---------------------------------------------------------------------------
// Serial output
// ---------------------------------------------------------------------------

// Machine-readable status for a future Web Serial UI:
//   S,<targetRpm>,<currentRpm>,<steps>,<state>
void printStreamLine() {
  Serial.printf("S,%.1f,%.1f,%ld,%s\n",
                targetRpm, currentRpm, (long)stepCount, kStateNames[motionState]);
}

void printStatus() {
  Serial.printf("# state=%s  target=%.1f rpm  current=%.1f rpm  steps=%ld\n",
                kStateNames[motionState], targetRpm, currentRpm, (long)stepCount);
  Serial.printf("# accel=%.1f rpm/s  decel=%.1f rpm/s  max=%.1f rpm  spr=%lu  invert=%d\n",
                accelRpmPerSec, decelRpmPerSec, maxRpm, (unsigned long)stepsPerRev, dirInvert ? 1 : 0);
  Serial.printf("# step rate=%.1f steps/s (limit %.0f)\n", isrStepRate, kMaxStepRate);
}

void printCommandHelp() {
  Serial.println("#");
  Serial.println("# Commands:");
  Serial.println("#   rpm <n>          Target RPM, negative = other direction (ramps)");
  Serial.println("#   rev              Reverse the current target (ramps)");
  Serial.println("#   stop             Ramp down to 0 using decel");
  Serial.println("#   x | estop        Stop pulses immediately");
  Serial.println("#   accel <rpm/s>    Ramp-up rate, works while spinning");
  Serial.println("#   decel <rpm/s>    Ramp-down rate, works while spinning");
  Serial.println("#   max <rpm>        Speed cap (1-3000)");
  Serial.println("#   spr <n>          Steps per revolution, match the DIP switches (stopped only)");
  Serial.println("#   invert 0|1       Flip DIR polarity (stopped only)");
  Serial.println("#   zero             Reset the step counter");
  Serial.println("#   status           Print settings and state");
  Serial.println("#   stream on|off    Print S,<target>,<current>,<steps>,<state> at 20 Hz");
  Serial.println("#   help             Print this help");
}

// ---------------------------------------------------------------------------
// Serial commands
// ---------------------------------------------------------------------------

void processSerialInput() {
  while (Serial.available() > 0) {
    const char incoming = Serial.read();

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      serialCommandBuffer[serialCommandLength] = '\0';
      handleSerialCommand(serialCommandBuffer);
      serialCommandLength = 0;
      serialCommandBuffer[0] = '\0';
      continue;
    }

    if (serialCommandLength < sizeof(serialCommandBuffer) - 1) {
      serialCommandBuffer[serialCommandLength] = incoming;
      serialCommandLength++;
    }
  }
}

void handleSerialCommand(char* commandLine) {
  char* command = strtok(commandLine, " \t");

  if (command == nullptr) {
    return;
  }

  lowercaseText(command);

  if (strcmp(command, "x") == 0 || strcmp(command, "estop") == 0) {
    emergencyStop();
    Serial.println("# emergency stop");
    return;
  }

  if (strcmp(command, "stop") == 0) {
    targetRpm = 0.0f;
    Serial.println("# stopping");
    return;
  }

  if (strcmp(command, "rev") == 0) {
    targetRpm = (targetRpm == 0.0f) ? 0.0f : -targetRpm;
    Serial.printf("# target %.1f rpm\n", targetRpm);
    return;
  }

  if (strcmp(command, "zero") == 0) {
    stepCount = 0;
    Serial.println("# step counter zeroed");
    return;
  }

  if (strcmp(command, "status") == 0) {
    printStatus();
    return;
  }

  if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
    printCommandHelp();
    return;
  }

  char* valueText = strtok(nullptr, " \t");

  if (valueText == nullptr) {
    Serial.println("# Missing value. Type 'help' for commands.");
    return;
  }

  lowercaseText(valueText);
  const float value = atof(valueText);

  if (strcmp(command, "rpm") == 0) {
    targetRpm = clampRpm(value);

    if (targetRpm != value) {
      Serial.printf("# clamped to max %.1f rpm\n", maxRpm);
    }

    Serial.printf("# target %.1f rpm\n", targetRpm);
  } else if (strcmp(command, "accel") == 0) {
    setRampRate(accelRpmPerSec, value, "accel");
  } else if (strcmp(command, "decel") == 0) {
    setRampRate(decelRpmPerSec, value, "decel");
  } else if (strcmp(command, "max") == 0) {
    setMaxRpm(value);
  } else if (strcmp(command, "spr") == 0) {
    setStepsPerRev(valueText);
  } else if (strcmp(command, "invert") == 0) {
    setDirInvert(valueText);
  } else if (strcmp(command, "stream") == 0) {
    setStream(valueText);
  } else {
    Serial.println("# Unknown command. Type 'help' for commands.");
  }
}

void setRampRate(float& setting, float value, const char* name) {
  if (value <= 0.0f || value > 100000.0f) {
    Serial.printf("# %s must be > 0 rpm/s\n", name);
    return;
  }

  setting = value;
  Serial.printf("# %s %.1f rpm/s\n", name, setting);
}

void setMaxRpm(float value) {
  if (value < 1.0f || value > kHardMaxRpm) {
    Serial.printf("# max must be 1..%.0f rpm\n", kHardMaxRpm);
    return;
  }

  maxRpm = value;
  targetRpm = clampRpm(targetRpm);
  Serial.printf("# max %.1f rpm\n", maxRpm);
}

void setStepsPerRev(const char* valueText) {
  if (!motorStopped()) {
    Serial.println("# stop the motor before changing spr");
    return;
  }

  const long value = atol(valueText);

  if (value < 1 || value > 100000) {
    Serial.println("# spr must be 1..100000");
    return;
  }

  stepsPerRev = value;
  Serial.printf("# spr %lu\n", (unsigned long)stepsPerRev);

  const float topStepRate = maxRpm * stepsPerRev / 60.0f;

  if (topStepRate > kMaxStepRate) {
    Serial.printf("# note: max rpm needs %.0f steps/s, output is limited to %.0f\n", topStepRate, kMaxStepRate);
  }
}

void setDirInvert(const char* valueText) {
  if (!motorStopped()) {
    Serial.println("# stop the motor before changing invert");
    return;
  }

  // Motor is stopped, so the ISR isn't writing DIR right now.
  noInterrupts();
  dirInvert = atoi(valueText) != 0;
  digitalWriteFast(kDirPin, (dirForward != dirInvert) ? HIGH : LOW);
  interrupts();

  Serial.printf("# invert %d\n", dirInvert ? 1 : 0);
}

void setStream(const char* valueText) {
  if (strcmp(valueText, "on") == 0 || strcmp(valueText, "1") == 0) {
    streamEnabled = true;
  } else if (strcmp(valueText, "off") == 0 || strcmp(valueText, "0") == 0) {
    streamEnabled = false;
  } else {
    Serial.println("# stream must be on or off");
    return;
  }

  Serial.printf("# stream %s\n", streamEnabled ? "on" : "off");
}

void lowercaseText(char* text) {
  while (*text != '\0') {
    if (*text >= 'A' && *text <= 'Z') {
      *text = *text - 'A' + 'a';
    }

    text++;
  }
}
