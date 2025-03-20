enum Colours {
  OFF,
  RED,
  ORANGE,
  GREEN,
};

#define RED_LED 1
#define GREEN_LED 3

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);   // Turn the LED on once initialised
}

void loop() {
  setLED(RED);
  delay(1000);
  setLED(ORANGE);
  delay(1000);
  setLED(GREEN);
  delay(1000);
}

void setLED(Colours colour) {
  switch (colour) {
    case (OFF):
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, LOW);
      break;
    case (RED):
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      break;
    case (ORANGE):
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, HIGH);
      break;
    case (GREEN):
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
      break;
  }
}
