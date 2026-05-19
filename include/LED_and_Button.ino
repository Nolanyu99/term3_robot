const int redPin = 9;
const int bluePin = 10;
const int greenPin = 11;
const int OffButtonPin = 2;
const int RevButtonPin = 40

bool previousStateRed = false;
bool state = true;
bool Reviving = false;
int previousOffButtonPressed = 1;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  
  pinMode(OffButtonPin, INPUT_PULLUP);
  pinMode(RevButtonPin, INPUT_PULLUP);
  
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
  int OffButtonPressed = digitalRead(OffButtonPin);
  int RevButtonPressed = digitalRead(RevButtonPin);
  
  Reviving = false

  if (OffButtonPressed == 0 && previousOffButtonPressed == 0) {
    state = !state;
  }
  else if (RevButtonPressed == 0) {
    Reviving = true;
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
  previousOffButtonPressed = OffButtonPressed;
}
