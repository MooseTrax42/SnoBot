/*
  Single encoder test (quadrature)
  - Encoder A: pin 2 (interrupt)
  - Encoder B: pin 4 (direction sample)

  Serial commands:
    r  -> reset count to 0

  Output (10 Hz):
    count=<ticks> delta=<ticks_since_last_print> dir=<FWD|REV|STOP> a=<0|1> b=<0|1> irq=<count>
*/

const uint8_t PIN_ENC_A = 2;
const uint8_t PIN_ENC_B = 4;

volatile long encoderCount = 0;
volatile unsigned long irqCount = 0;
volatile uint8_t lastA = HIGH;

unsigned long lastPrintMs = 0;
long lastPrintedCount = 0;

void onEncoderAChange() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  irqCount++;

  // Count only when A actually toggles.
  if (a == lastA) {
    return;
  }

  // If using channel A interrupts only, this relation gives direction.
  if (a == b) {
    encoderCount++;
  } else {
    encoderCount--;
  }

  lastA = a;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  lastA = digitalRead(PIN_ENC_A);

  int aInterrupt = digitalPinToInterrupt(PIN_ENC_A);
  if (aInterrupt < 0) {
    Serial.println("ERROR: pin 2 is not an interrupt pin on this board.");
    while (true) {
      delay(1000);
    }
  }

  attachInterrupt(aInterrupt, onEncoderAChange, CHANGE);

  Serial.println("Single encoder test started");
  Serial.println("Pins: A=2, B=4");
  Serial.println("Send 'r' to reset count");
}

void loop() {
  if (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'r' || c == 'R') {
      noInterrupts();
      encoderCount = 0;
      irqCount = 0;
      interrupts();
      lastPrintedCount = 0;
      Serial.println("count reset");
    }
  }

  unsigned long now = millis();
  if (now - lastPrintMs >= 100) {
    lastPrintMs = now;

    noInterrupts();
    long count = encoderCount;
    unsigned long irqs = irqCount;
    interrupts();

    long delta = count - lastPrintedCount;
    lastPrintedCount = count;

    const char* dir = "STOP";
    if (delta > 0) {
      dir = "FWD";
    } else if (delta < 0) {
      dir = "REV";
    }

    int aState = digitalRead(PIN_ENC_A);
    int bState = digitalRead(PIN_ENC_B);

    Serial.print("count=");
    Serial.print(count);
    Serial.print(" delta=");
    Serial.print(delta);
    Serial.print(" dir=");
    Serial.print(dir);
    Serial.print(" a=");
    Serial.print(aState);
    Serial.print(" b=");
    Serial.print(bState);
    Serial.print(" irq=");
    Serial.println(irqs);
  }
}
