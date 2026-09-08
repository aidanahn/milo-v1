/**
 * @file web_interface.cpp
 * @brief Wi-Fi access point, control-page rendering, and form handling.
 */

#include "web_interface.hpp"

#include <WiFi.h>

WebInterface::WebInterface(PacerSettings& settings)
    : settings_(settings), server(80) {}

void WebInterface::begin(Action start, Action stop, Action calibrate, Action tune) {
  WiFi.softAP(config::SSID, config::PASSWORD);
  server.on("/", HTTP_GET, [this]() { handleRoot(); });
  server.on("/start", HTTP_POST, [this, start]() { handleAction(start); });
  server.on("/stop", HTTP_POST, [this, stop]() { handleAction(stop); });
  server.on("/calibrate", HTTP_POST, [this, calibrate]() { handleAction(calibrate); });
  server.on("/pid", HTTP_POST, [this, tune]() { handleAction(tune); });
  server.begin();
}

void WebInterface::update() {
  server.handleClient();
}

void WebInterface::handleAction(Action action) {
  // Preserve command ordering: callbacks must observe submitted settings.
  updateInputs();
  action();
  server.send(204);
}

void WebInterface::updateInputs() {
  if (server.hasArg("distance")) {
    settings_.distance = server.arg("distance").toInt();
  }
  if (server.hasArg("pace")) {
    settings_.pace = server.arg("pace").toInt();
  }
  if (server.hasArg("velocityKp")) {
    settings_.velocityKp = server.arg("velocityKp").toDouble();
  }
  if (server.hasArg("velocityKd")) {
    settings_.velocityKd = server.arg("velocityKd").toDouble();
  }
  if (server.hasArg("velocityKi")) {
    settings_.velocityKi = server.arg("velocityKi").toDouble();
  }
  if (server.hasArg("steeringKp")) {
    settings_.steeringKp = server.arg("steeringKp").toDouble();
  }
  if (server.hasArg("steeringKd")) {
    settings_.steeringKd = server.arg("steeringKd").toDouble();
  }
  if (server.hasArg("steeringKi")) {
    settings_.steeringKi = server.arg("steeringKi").toDouble();
  }
}

void WebInterface::handleRoot() {
  String html = "<!DOCTYPE html><html><body>";
  html += "<form method='POST'>";
  html += "<label for='distance'>Distance (meters): </label>";
  html += "<input type='number' id='distance' name='distance' step='1' value='" + String(settings_.distance) + "'><br>";
  html += "<label for='pace'>Goal Pace (seconds): </label>";
  html += "<input type='number' id='pace' name='pace' step='1' value='" + String(settings_.pace) + "'><br>";
  html += "<button type='submit' formaction='/start'>Start Pacing</button><br>";
  html += "<button type='submit' formaction='/stop'>Stop Pacing</button><br>";
  html += "<button type='submit' formaction='/calibrate'>Calibrate Sensor</button>";
  html += "</form>";

  html += "<form method='POST'>";
  html += "<h2>Velocity PID</h2>";
  html += "<label for='velocityKp'>Proportional (Kp): </label>";
  html += "<input type='number' id='velocityKp' name='velocityKp' step='0.01' value='" + String(settings_.velocityKp, 2) + "'><br>";
  html += "<label for='velocityKd'>Derivative (Kd): </label>";
  html += "<input type='number' id='velocityKd' name='velocityKd' step='0.01' value='" + String(settings_.velocityKd, 2) + "'><br>";
  html += "<label for='velocityKi'>Integral (Ki): </label>";
  html += "<input type='number' id='velocityKi' name='velocityKi' step='0.01' value='" + String(settings_.velocityKi, 2) + "'><br>";
  html += "<h2>Steering PID</h2>";
  html += "<label for='steeringKp'>Proportional (Kp): </label>";
  html += "<input type='number' id='steeringKp' name='steeringKp' step='0.01' value='" + String(settings_.steeringKp, 2) + "'><br>";
  html += "<label for='steeringKd'>Derivative (Kd): </label>";
  html += "<input type='number' id='steeringKd' name='steeringKd' step='0.01' value='" + String(settings_.steeringKd, 2) + "'><br>";
  html += "<label for='steeringKi'>Integral (Ki): </label>";
  html += "<input type='number' id='steeringKi' name='steeringKi' step='0.01' value='" + String(settings_.steeringKi, 2) + "'><br>";
  html += "<button type='submit' formaction='/pid'>Update PID</button>";
  html += "</form>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}
