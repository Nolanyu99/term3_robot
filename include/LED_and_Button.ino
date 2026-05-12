const int redPin = 9;
const int bluePin = 10;
const int greenPin = 11;
const int buttonPin = 2;

bool previousStateRed = false;
bool state = true;
bool Reviving = true;
int previousButtonPressed = 1;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  
  pinMode(buttonPin, INPUT_PULLUP);
  
  Serial.begin(115200); delay(2000);
  Serial.println("starting...");
}

void Flash(bool previousStateRed) {
  if (previousStateRed) {
    Serial.print("Red\n");
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
  }
  else {
    Serial.print("Off\n");
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
  }
}

void loop() {
  int buttonPressed = digitalRead(buttonPin);

  if (buttonPressed == 0 && previousButtonPressed = 0) {
    state = !state;
  }
  
  if (!state) {
    Flash(previousStateRed);
    previousStateRed = !previousStateRed; delay(1000);
  }
  else {
    //rest of code
    //Serial.print("Running...\n");
    if (Reviving) {
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, HIGH);
      digitalWrite(bluePin, LOW);
    }
  }
  previousButtonPressed = buttonPressed
}