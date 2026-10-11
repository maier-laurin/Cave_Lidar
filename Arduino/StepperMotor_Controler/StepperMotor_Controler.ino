const int stepPin = 4;
const int dirPin = 3;
const int sensPin = 2;

// Filtering variables
const int WINDOW_SIZE = 10;
bool readings[WINDOW_SIZE]; // Array to store last n values
int readIndex = 0;          // The index of the current reading
int totalHighs = 0;         // Running sum of HIGH readings
bool last_filtered_state = LOW;
int rotationsteps = 0;

void setup() {
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(sensPin, INPUT);

  // Initialize array to 0
  for (int i = 0; i < WINDOW_SIZE; i++) {
    readings[i] = 0;
  }
}

void loop() {
  digitalWrite(dirPin, HIGH); // Set direction clockwise

  // Step the motor
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(stepPin, LOW);

  // --- Filtering Logic ---
  
  // 1. Subtract the oldest reading from the total
  totalHighs -= readings[readIndex];
  
  // 2. Read the new value and add it to the total
  readings[readIndex] = digitalRead(sensPin);
  totalHighs += readings[readIndex];
  
  // 3. Advance the index
  readIndex = (readIndex + 1) % WINDOW_SIZE;

  // 4. Determine the filtered state (Quantile/Threshold)
  // If more than 7 out of 10 are HIGH, we consider the signal HIGH.
  // This helps ignore small spikes (jitter).
  bool current_filtered_state = (totalHighs > 7); 

  rotationsteps++;
  // --- Rising Edge Detection ---
  if (current_filtered_state == HIGH && last_filtered_state == LOW) {
    Serial.print("Rising Edge Detected! :");
    Serial.println(rotationsteps);
    rotationsteps = 0;
  }

  last_filtered_state = current_filtered_state;

  delayMicroseconds(490); 
}