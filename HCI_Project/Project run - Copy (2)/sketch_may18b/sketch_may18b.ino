#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
int16_t ax, ay, az, gx, gy, gz;
int16_t base_gx = 0, base_gy = 0, base_gz = 0;

const int threshold = 3;
const int changeThreshold = 3; // Change threshold for output
const int buttonPin = 7; // Input button pin for mouse click
const int calibrateButtonPin = 8; // Calibration button pin
const int buttonGroundPin = 10; // Ground pin for calibration button
int prev_x = 960; // Assuming initial mouse position at center
int prev_y = 540; // Assuming initial mouse position at center

void setup() {
  Serial.begin(115200); // Use a higher baud rate for faster communication
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    while (1); // Halt if MPU6050 is not connected
  }

  // Set MPU6050 to highest possible data rate (1kHz)
  mpu.setRate(0);  // 1kHz sample rate
  mpu.setDLPFMode(MPU6050_DLPF_BW_256);  // Set digital low pass filter to 256Hz
  
  calibrateMPU();
  Serial.println("Calibration complete");

  pinMode(buttonPin, INPUT_PULLUP); // Set mouse click button pin as input with internal pull-up resistor
  pinMode(calibrateButtonPin, INPUT_PULLUP); // Set calibration button pin as input with internal pull-up resistor
  pinMode(buttonGroundPin, OUTPUT); // Set calibration button ground pin as output
  digitalWrite(buttonGroundPin, LOW); // Set ground pin low to complete the circuit
}

void loop() {
  // Check for calibration button press
  if (digitalRead(calibrateButtonPin) == LOW) { // Button press will read LOW due to internal pull-up resistor
    calibrateMPU();
    Serial.println("Recalibration complete");
    prev_x = 960; // Reset cursor to center X
    prev_y = 540; // Reset cursor to center Y
    Serial.print("NX:");
    Serial.print(prev_x);
    Serial.print(",NY:");
    Serial.print(prev_y);
    Serial.print(",B:");
    Serial.println(0); // Button state 0
    delay(500); // Debounce delay
    return; // Skip the rest of the loop to avoid unwanted movements
  }

  // Read MPU6050 data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Adjust readings by baseline
  int vx = (gx - base_gx) / 150;
  int vy = (gz - base_gz) / 150;

  // Determine velocity based on direction of movement
  int velocity_x = 0;
  int velocity_y = 0;
  if (abs(vx) > threshold || abs(vy) > threshold) {
    velocity_x = map(vx, -threshold, threshold, -1, 1); // Map to desired range with increased sensitivity
    velocity_y = map(vy, -threshold, threshold, -1, 1); // Map to desired range with increased sensitivity

  }  

  // Calculate new mouse position
  int new_x = max(0, min(1920, prev_x + velocity_x)); // Assuming screen width is 1920
  int new_y = max(0, min(1080, prev_y + velocity_y)); // Assuming screen height is 1080

  // Read button state for mouse click
  int buttonState = digitalRead(buttonPin);

  // Send preprocessed data to Python script only if there's a change of at least 5 units or mouse button state change
  if (abs(new_x - prev_x) >= changeThreshold || abs(new_y - prev_y) >= changeThreshold || buttonState == 1) {
    Serial.print("NX:");
    Serial.print(new_x);
    Serial.print(",NY:");
    Serial.print(new_y);
    Serial.print(",B:");
    Serial.println(buttonState);

    // Update previous position
    prev_x = new_x;
    prev_y = new_y;
  }
}

void calibrateMPU() {
  const int numReadings = 100;
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;

  for (int i = 0; i < numReadings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;
    delay(10); // Reduced delay for faster calibration
  }

  base_gx = sum_gx / numReadings;
  base_gy = sum_gy / numReadings;
  base_gz = sum_gz / numReadings;
}
