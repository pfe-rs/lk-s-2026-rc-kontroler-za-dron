
int battery_pin = 11; // Pin to witch voltage divider is connected
bool battery_protection;

#include <Wire.h> // Library for I2C communication
float roll_rate, pitch_rate, yaw_rate; // Gyro rates
float desired_roll_rate, desired_pitch_rate, desired_yaw_rate; // Desired Gyro rates
float error_roll_rate, error_pitch_rate, error_yaw_rate; // Error Gyro rates
float previous_error_roll_rate, previous_error_pitch_rate, previous_error_yaw_rate; // Previous error Gyro rates

float x_acceleration, y_acceleration, z_acceleration;
float roll_angle, pitch_angle;

float roll_kalman_angle = 0;
float pitch_kalman_angle = 0;
float roll_kalman_angle_uncertainty = 2 * 2;
float pitch_kalman_angle_uncertainty = 2 * 2;
float kalman_output[] = {0, 0};
float desired_roll_angle = 0, desired_pitch_angle = 0;
float error_roll_angle, error_pitch_angle;
float previous_error_roll_angle, previous_error_pitch_angle;

float previous_roll_i, previous_pitch_i, previous_yaw_i; // Previous I values
float roll_input, pitch_input, yaw_input; //Angle inputs
float pid_return[] = {0, 0, 0};
float pid_parameters [3][3] = { {0,     0,      0},     // Roll   PID parameter
                                {0.04,  0.025,  0.03},  // Pitch  PID parameter
                                {0,     0,      0}};    // Yaw    PID parameter


#include <ESP32Servo.h> // Library used for PWM signal
bool arm;
float throttle; // Used for vertical stabilization
int min_motor_speed = 15;
int max_motor_speed = 180;
Servo motors[4];
int motor_pins [4] = {7, 8, 9, 10};
int motor_values [4] = {0, 0, 0, 0};


uint32_t loop_timer;


#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

unsigned long last_ble_time = 0;
bool deviceConnected = false;

// Add a server callbacks class
class Server_callbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) { deviceConnected = true; }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pServer->getAdvertising()->start();
  }
};

void Rx() {

  String rxValue = String(pRxCharacteristic->getValue().c_str());
  pRxCharacteristic->setValue("");

  if (rxValue.length() > 0) {

    if (rxValue.charAt(0) == 'a') {
      Arm();
    }

    if (rxValue.charAt(0) == 't') {
      throttle = rxValue.substring(1, rxValue.length()).toInt();
    }

    if (rxValue.charAt(0) == 'p') {
      pid_parameters[1][0] = rxValue.substring(1, rxValue.length()).toDouble();
    }

    if (rxValue.charAt(0) == 'i') {
      pid_parameters[1][1] = rxValue.substring(1, rxValue.length()).toDouble();
    }

    if (rxValue.charAt(0) == 'd') {
      pid_parameters[1][2] = rxValue.substring(1, rxValue.length()).toDouble();
    }
  }
}

void Tx() {

  char txString[128];
  snprintf(txString, sizeof(txString), "%.2f,%.2f,%.5f,%.5f,%.5f,%.2f, %d,%d,%d,%d", error_pitch_angle, Battery_voltage(), pid_parameters[1][0], pid_parameters[1][1], pid_parameters[1][2], 0, motor_values[0], motor_values[1], motor_values[2], motor_values[3]);
  pTxCharacteristic->setValue(txString);
  pTxCharacteristic->notify();
}

void Ble_setup() {

  BLEDevice::init("UART Service");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new Server_callbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE_NR);
  pService->start();
  pServer->getAdvertising()->start();

}

void PID(float error, float p, float i, float d, float previous_error, float previous_i) {

  float p_value = p * error;

  float i_value;
  
  if(error > 15 || error < -15) i_value = previous_i;
  else i_value = previous_i + i * (error + previous_error) * 0.004 / 2;
    
  if (i_value > 15) i_value = 15;
  else if (i_value < -15) i_value = -15;

  float d_value = d * (error - previous_error) / 0.004;

  float pid_output = p_value + i_value + d_value;
  if (pid_output > 800) pid_output = 800;
  else if (pid_output < -800) pid_output = -800;

  pid_return[0] = pid_output;
  pid_return[1] = error;
  pid_return[2] = i_value;
}

void PID_restart(void) {

  previous_error_roll_rate = 0;
  previous_error_pitch_rate = 0;
  previous_error_yaw_rate = 0;

  previous_roll_i = 0;
  previous_pitch_i = 0;
  previous_yaw_i = 0;
}

void IMU() {

  Wire.beginTransmission(0x68); // MPU6050 Adress
  Wire.write(0x1A); // Low-Pass Filter
  Wire.write(0x05); // Filter Bandwidth Frequency (10Hz)
  Wire.endTransmission();

  Wire.beginTransmission(0x68); // MPU6050 Adress
  Wire.write(0x1B); // Sensitivity
  Wire.write(0x8); // Sensitivity value (+-500 Degree/s <--> 65.5 LSB/s)
  Wire.endTransmission();

  Wire.beginTransmission(0x68); // MPU6050 Adress
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68, 6); // Reading Gyro registers

  int16_t gyro_roll = Wire.read() << 8 | Wire.read(); // Combining registers for Roll
  int16_t gyro_pitch = Wire.read() << 8 | Wire.read(); // Combining registers for Pitch
  int16_t gyro_yaw = Wire.read() << 8 | Wire.read(); // Combining registers for Yaw

  roll_rate = (float)gyro_roll / 65.5; // Converting to Degree/s
  pitch_rate = (float)gyro_pitch / 65.5; // Converting to Degree/s
  yaw_rate = (float)gyro_yaw / 65.5; // Converting to Degree/s

  roll_rate += 0.04;
  pitch_rate += 3.93;
  yaw_rate -= 0.37;


  //Accelerometer
  Wire.beginTransmission(0x68); // MPU6050 Adress
  Wire.write(0x1C); // Accelerometer Range
  Wire.write(0x10); // Range Value (+-8G)
  Wire.endTransmission();

  Wire.beginTransmission(0x68); // MPU6050 Adress
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  int16_t x_acceleration_lsb = Wire.read() << 8 | Wire.read();
  int16_t y_acceleration_lsb = Wire.read() << 8 | Wire.read();
  int16_t z_acceleration_lsb = Wire.read() << 8 | Wire.read();

  x_acceleration = (float)x_acceleration_lsb / 4096 - 0.035;
  y_acceleration = (float)y_acceleration_lsb / 4096 + 0.000;
  z_acceleration = (float)z_acceleration_lsb / 4096 + 0.035;

  roll_angle = atan(y_acceleration / sqrt(x_acceleration * x_acceleration + z_acceleration * z_acceleration)) * 1 / (3.142 / 180);
  pitch_angle = -atan(x_acceleration / sqrt(y_acceleration * y_acceleration + z_acceleration * z_acceleration)) * 1 / (3.142 / 180);
}

void Kalman(float kalman_state, float kalman_uncertainty, float kalman_input, float kalman_measurement) {

  kalman_state = kalman_state + 0.004 * kalman_input;
  kalman_uncertainty = kalman_uncertainty + 0.004 * 0.004 * 0.5 * 0.5;

  float kalman_gain = kalman_uncertainty * 1 / (1 * kalman_uncertainty + 0.5 * 0.5);
  kalman_state = kalman_state + kalman_gain * (kalman_measurement - kalman_state);
  kalman_uncertainty = (1 - kalman_gain) * kalman_uncertainty;
  kalman_output[0] = kalman_state;
  kalman_output[1] = kalman_uncertainty;
}

float Battery_voltage() {

  uint16_t mv = analogReadMilliVolts(battery_pin);
  return (mv * 6.4) / 1000.0;
}

void Battery_protection(float battery_voltage) {

  if (battery_voltage < 10 || battery_voltage > 13) battery_protection = 0;
  else battery_protection = 1;
}

void Motor_setup() {

  for (int i = 0; i < 4; i++) {
    motors[i].setPeriodHertz(250);
    motors[i].attach(motor_pins[i], 1000, 2000);
  }
}

void Motor_power() {

  motor_values[0] = battery_protection * arm * (throttle + pitch_input); // + roll_input - yaw_input
  motor_values[1] = battery_protection * arm * (throttle - pitch_input); // + roll_input + yaw_input
  motor_values[2] = battery_protection * arm * (throttle + pitch_input); // - roll_input + yaw_input
  motor_values[3] = battery_protection * arm * (throttle - pitch_input); // - roll_input - yaw_input

  for (int i = 0; i < 4; i++) {
    motor_values[i] *= 1.8;
    if (motor_values[i] < 0) motor_values[i] = 0;
    else if (motor_values[i] > max_motor_speed) motor_values[i] = max_motor_speed;
    motors[i].write(max(motor_values[i], min_motor_speed * arm));
  }
}

void Arm() {
  arm = !arm;
  if (arm) PID_restart();
}

void setup() {

  rgbLedWrite(21, 255, 0, 0);
  Serial.begin(115200);

  Wire.setClock(400000);
  Wire.begin(13, 12);
  delay(1000);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();


  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  Motor_setup();
  Ble_setup();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  loop_timer = micros();
}

void loop() {

  Battery_protection(Battery_voltage());
  IMU();

  //desired_roll_rate = 0;
  //desired_pitch_rate = 0;
  //desired_yaw_rate = 0;

  Kalman(roll_kalman_angle, roll_kalman_angle_uncertainty, roll_rate, roll_angle);
  roll_kalman_angle = kalman_output[0];
  roll_kalman_angle_uncertainty = kalman_output[1];

  Kalman(pitch_kalman_angle, pitch_kalman_angle_uncertainty, pitch_rate, pitch_angle);
  pitch_kalman_angle = kalman_output[0];
  pitch_kalman_angle_uncertainty = kalman_output[1];

  error_roll_angle = desired_roll_angle - roll_kalman_angle;
  error_pitch_angle = desired_pitch_angle - pitch_kalman_angle - 7  ;

  //error_roll_rate = desired_roll_rate - roll_rate;
  //error_pitch_rate = desired_pitch_rate - pitch_rate;
  //error_yaw_rate = desired_yaw_rate - yaw_rate;

  PID(error_roll_angle, pid_parameters[0][0], pid_parameters[0][1], pid_parameters[0][2], previous_error_roll_angle, previous_roll_i);
  roll_input = pid_return[0];
  previous_error_roll_angle = pid_return[1];
  previous_roll_i = pid_return[2];

  PID(error_pitch_angle, pid_parameters[1][0], pid_parameters[1][1], pid_parameters[1][2], previous_error_pitch_angle, previous_pitch_i);
  pitch_input = pid_return[0];
  previous_error_pitch_angle = pid_return[1];
  previous_pitch_i = pid_return[2];

  PID(error_yaw_rate, pid_parameters[2][0], pid_parameters[2][1], pid_parameters[2][2], previous_error_yaw_rate, previous_yaw_i);
  yaw_input = pid_return[0];
  previous_error_yaw_rate = pid_return[1];
  previous_yaw_i = pid_return[2];

  Motor_power();

  if (deviceConnected && millis() - last_ble_time > 100) {
    Rx();
    Tx();
    last_ble_time = millis();
  }

  while (micros() - loop_timer < 4000);
  loop_timer = micros();
}
