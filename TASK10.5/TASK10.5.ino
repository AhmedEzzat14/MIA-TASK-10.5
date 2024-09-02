#include <Wire.h>                                            // Initialize I2C communication library


const int MPU6050_Address = 0x68;                         // Default I2C address for the MPU6050

const int MPU6050_Power_Management = 0x6B;              // Register address of Power Management to set it to 0 to initialize the module

const int MPU6050_Gyroscope_Z_High = 0x47;            // Register address of High byte of Z-Gyroscope to retrieve the sensor data
//const int MPU6050_Gyroscope_Z_Low = 0x48;          // Register address of Low byte of Z-Gyroscope to retrieve the sensor data        


const float Gyroscope_Scale_Modifier = 131.0;     // Gyroscope scale modifier (for 250 degrees/second) (±250 dps: 1 raw unit = 1/131.0 dps)


float Yaw;
float Gyroscope_Z_Offset = 0;                 // This offset is a correction factor that is subtracted from the raw gyroscope data to improve the accuracy

long Previous_Time = 0;                     // Calculating estimated time between successive reading from MPU, which for accurately integrating the gyroscope data

void setup() {
  Wire.begin();                                           // Begin I2C communication
  Serial.begin(9600);                                    // Set baudrate at 9600  
  Wire.beginTransmission(MPU6050_Address);              // Wake up the MPU6050 as it starts in sleep mode
  Wire.write(MPU6050_Power_Management);                // MPU6050. When the MPU6050 is powered on, this register is in sleep mode by default to save power
  Wire.write(0);                                      // Write the value 0 to this register
  Wire.endTransmission(true);
  
  
  Calibrate_Gyroscope();                          // Calibrate the gyro to find the offset
  
  
  Previous_Time = millis();                    // Record the start time
}

void loop() {
  int16_t Gyroscope_Z = Read_MPU6050_GyroscopeZ();                                                      // Get the reading from sensor
  
  float Gyroscope_Z_corrected = (Gyroscope_Z / Gyroscope_Scale_Modifier) - Gyroscope_Z_Offset;        // Apply offsets to increase accuracy
  
  long Current_Time = millis();                                                                     // Calculate the time difference
  
  float Elapsed_Time = (Current_Time - Previous_Time) / 1000.0;                                   // Convert (ms) to (s)
  Previous_Time = Current_Time;
  

  Yaw += Gyroscope_Z_corrected * Elapsed_Time;                                                // Integration to get yaw
  
 
  Serial.print("Yaw: ");
  Serial.println(Yaw);                                                                    // Print yaw angle
  
  delay(500);                                                                           // Delay (2s) to control update rate
}

int16_t Read_MPU6050_GyroscopeZ() {
  
  Wire.beginTransmission(MPU6050_Address);                        // Request the gyro Z high bytes
  Wire.write(MPU6050_Gyroscope_Z_High);                          // Z-axis high byte register is at address 0x47
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_Address, 2, true);                  // Request 2 bytes from the specified register
  
  
  int16_t Gyroscope_Z = Wire.read() << 8 | Wire.read();     // Read the two bytes and combine them into one 16-bit integer
  
  return Gyroscope_Z;
}

void Calibrate_Gyroscope() {
  const int Num_Readings = 1000;
  long Gyroscope_Z_sum = 0;
  
  Serial.println("Calibrating Gyroscope...");

  // Collect multiple samples to calculate an average offset
  for (int i = 0; i < Num_Readings; i++) {
    int16_t Gyroscope_Z = Read_MPU6050_GyroscopeZ();
    Gyroscope_Z_sum += Gyroscope_Z;
    delay(3);                                                                         // Small delay to optimize the code
  }
  
  
  Gyroscope_Z_Offset = (Gyroscope_Z_sum / Num_Readings) / Gyroscope_Scale_Modifier;      // Calculate the average offset
  
  Serial.print("Gyroscope Z Offset: ");
  Serial.println(Gyroscope_Z_Offset);
}
