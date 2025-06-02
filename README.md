# Withforce Lumbar Flexion, Breathing Rate, Energy Expenditure Validation module

during bluetooth connection initation, the PC must send weight as a uint8_t on the second byte of the data (for EE calculation)

1) lumbar flexion with Kalman filter and 30 degrees threshold
2) breathing rate from FSR sensor with 30s buffer
3) energey expenditure from lumbar flexion + step count (from IMU pedometer)

4) ADS data is buffered 30s

5) data is sent every 30s via bluetooth