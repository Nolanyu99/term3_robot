const int redPin = 39;
const int greenPin = 35;
const int bluePin = 37;
const int OffButtonPin = 33;
const int RevButtonPin = 13;

int previousStateRed = 0;
bool state = true;
bool Reviving = false;
int previousOffButtonPressed = 1;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  
  pinMode(OffButtonPin, INPUT_PULLUP);
  pinMode(RevButtonPin, INPUT_PULLUP);
  
  Serial.begin(115200);
  delay(2000);
  Serial.println("starting...");
}

void Flash(int previousStateRed) {
  if (previousStateRed % 20 <= 9) {
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
  Serial.print("OffButtonPressed ");
  Serial.println(OffButtonPressed);
  Serial.print("previousOffButtonPressed: ");
  Serial.println(previousOffButtonPressed);
  int RevButtonPressed = digitalRead(RevButtonPin);
  Serial.print("RevButtonPressed ");
  Serial.println(RevButtonPressed);
  Serial.println("------------");
  
  Reviving = false;

  if (OffButtonPressed == 0 && previousOffButtonPressed != 0) {
    state = !state;
    delay(50);
  }
  else if (RevButtonPressed == 0) {
    Reviving = true;
    delay(50);
  }
  
  if (!state) {
    Flash(previousStateRed);
    previousStateRed++;
    delay(25);
  }
  else {
    //rest of code
    //Serial.print("Running...\n");
    if (Reviving) {
      digitalWrite(redPin, LOW);
      digitalWrite(greenPin, HIGH);
      digitalWrite(bluePin, LOW);
      delay(500);
    }
    else {
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, LOW);
      delay(50);
    }
  }
  previousOffButtonPressed = OffButtonPressed;
}
