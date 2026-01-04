#include <Arduino.h>
#include <AS5600.h>
#include <PacerController.h>
#include <QTRSensors.h>
#include <Adafruit_NeoPixel.h>
#include <WebServer.h>
#include <WiFi.h>
#include <PID_v1.h>

const double WHEEL_CIRCUMFERENCE = PI * (121.9 / 1000.0);
const double GEAR_RATIO = (40.0 / 20.0) * (38.0 / 13.0);
const uint8_t LED_COUNT = 1;
const uint8_t LED_PIN = 38;
const uint8_t SENSOR_COUNT = 16;
const uint8_t SENSOR_PINS[] = { 48, 10, 39, 40, 41, 42, 2, 1, 18, 17, 16, 15, 7, 6, 5, 4 };
const uint8_t ESC_PIN = 21;
const uint8_t SERVO_PIN = 47;
const char* SSID = "PacerESP32";
const char* PASSWORD = "04082008";

uint16_t sensorValues[SENSOR_COUNT];
int distance = 400;
int pace = 60;
double velocityKp = 1.0;
double velocityKd = 0.0;
double velocityKi = 2.5;
double steeringKp = 5.0;
double steeringKd = 1.1;
double steeringKi = 0.0;
double velocityInput, velocityOutput, velocitySetpoint;
double steeringInput, steeringOutput, steeringSetpoint;

PID velocityPID(&velocityInput, &velocityOutput, &velocitySetpoint, velocityKp, velocityKi, velocityKd, DIRECT);
PID steeringPID(&steeringInput, &steeringOutput, &steeringSetpoint, steeringKp / 1000, steeringKi / 1000, steeringKd / 1000, REVERSE);
AS5600 as5600("REVERSE", GEAR_RATIO, WHEEL_CIRCUMFERENCE);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
PacerController pacer(ESC_PIN, SERVO_PIN, GEAR_RATIO, WHEEL_CIRCUMFERENCE, strip);
WebServer server(80);
QTRSensors qtr;

void debug();
void updateInputs();
void handleRoot();
void handleStart();
void handleStop();
void handleCalibrate();
void handlePID();

void setup() {
  Serial.begin(921600);

  velocityPID.SetSampleTime(20);
  velocityPID.SetOutputLimits(0, 500);
  velocityPID.SetMode(AUTOMATIC);
  steeringSetpoint = 7500;
  steeringPID.SetSampleTime(20);
  steeringPID.SetOutputLimits(-90, 90);
  steeringPID.SetMode(AUTOMATIC);

  WiFi.softAP(SSID, PASSWORD);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/start", HTTP_POST, handleStart);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/calibrate", HTTP_POST, handleCalibrate);
  server.on("/pid", HTTP_POST, handlePID);
  server.begin();

  qtr.setTypeRC();
  qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);
  qtr.setDimmingLevel(0);
  qtr.setTimeout(3000);

  as5600.begin();

  pacer.begin();
}

void loop() {
  server.handleClient();
  if (pacer.getPacingStatus()) {
    as5600.update();

    debug();

    steeringInput = qtr.readLineWhite(sensorValues);
    steeringPID.Compute();
    velocityInput = as5600.getRotationsPerSecond();
    velocityPID.Compute();

    pacer.update(steeringOutput, velocityOutput, as5600.getDistanceTraveled());
  }
}

void updateInputs() {
  if (server.hasArg("distance")) {
    distance = server.arg("distance").toInt();
  }
  if (server.hasArg("pace")) {
    pace = server.arg("pace").toInt();
  }
  if (server.hasArg("velocityKp")) {
    velocityKp = server.arg("velocityKp").toDouble();
  }
  if (server.hasArg("velocityKd")) {
    velocityKd = server.arg("velocityKd").toDouble();
  }
  if (server.hasArg("velocityKi")) {
    velocityKi = server.arg("velocityKi").toDouble();
  }
  if (server.hasArg("steeringKp")) {
    steeringKp = server.arg("steeringKp").toDouble();
  }
  if (server.hasArg("steeringKd")) {
    steeringKd = server.arg("steeringKd").toDouble();
  }
  if (server.hasArg("steeringKi")) {
    steeringKi = server.arg("steeringKi").toDouble();
  }
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><body>";
  html += "<form method='POST'>";
  html += "<label for='distance'>Distance (meters): </label>";
  html += "<input type='number' id='distance' name='distance' step='1' value='" + String(distance) + "'><br>";
  html += "<label for='pace'>Goal Pace (seconds): </label>";
  html += "<input type='number' id='pace' name='pace' step='1' value='" + String(pace) + "'><br>";
  html += "<button type='submit' formaction='/start'>Start Pacing</button><br>";
  html += "<button type='submit' formaction='/stop'>Stop Pacing</button><br>";
  html += "<button type='submit' formaction='/calibrate'>Calibrate Sensor</button>";
  html += "</form>";

  html += "<form method='POST'>";
  html += "<h2>Velocity PID</h2>";
  html += "<label for='velocityKp'>Proportional (Kp): </label>";
  html += "<input type='number' id='velocityKp' name='velocityKp' step='0.01' value='" + String(velocityKp, 2) + "'><br>";
  html += "<label for='velocityKd'>Derivative (Kd): </label>";
  html += "<input type='number' id='velocityKd' name='velocityKd' step='0.01' value='" + String(velocityKd, 2) + "'><br>";
  html += "<label for='velocityKi'>Integral (Ki): </label>";
  html += "<input type='number' id='velocityKi' name='velocityKi' step='0.01' value='" + String(velocityKi, 2) + "'><br>";
  html += "<h2>Steering PID</h2>";
  html += "<label for='steeringKp'>Proportional (Kp): </label>";
  html += "<input type='number' id='steeringKp' name='steeringKp' step='0.01' value='" + String(steeringKp, 2) + "'><br>";
  html += "<label for='steeringKd'>Derivative (Kd): </label>";
  html += "<input type='number' id='steeringKd' name='steeringKd' step='0.01' value='" + String(steeringKd, 2) + "'><br>";
  html += "<label for='steeringKi'>Integral (Ki): </label>";
  html += "<input type='number' id='steeringKi' name='steeringKi' step='0.01' value='" + String(steeringKi, 2) + "'><br>";
  html += "<button type='submit' formaction='/pid'>Update PID</button>";
  html += "</form>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void debug() {
  Serial.print("20");
  Serial.print(",");
  Serial.print(as5600.getRotationsPerSecond());
  Serial.print(",");
  Serial.println("30");
}

void handleStart() {
  updateInputs();
  as5600.resetDistanceTraveled();
  velocityPID.SetMode(MANUAL);
  velocityOutput = 0;
  velocityPID.SetMode(AUTOMATIC);
  steeringPID.SetMode(MANUAL);
  steeringOutput = 0;
  steeringPID.SetMode(AUTOMATIC);
  pacer.start(distance, pace);
  velocitySetpoint = pacer.getGoalRotationsPerSecond();
  server.send(204);
}

void handleStop() {
  updateInputs();
  pacer.stop();
  server.send(204);
}

void handleCalibrate() {
  updateInputs();
  qtr.resetCalibration();
  pacer.setStatusColor(255, 255, 0);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  pacer.setStatusColor(0, 255, 0);
  server.send(204);
}

void handlePID() {
  updateInputs();
  steeringPID.SetTunings(steeringKp / 1000, steeringKi / 1000, steeringKd / 1000);
  velocityPID.SetTunings(velocityKp, velocityKi, velocityKd);
  server.send(204);
}
