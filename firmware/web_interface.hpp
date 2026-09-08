#ifndef PACER_FIRMWARE_WEB_INTERFACE_HPP_
#define PACER_FIRMWARE_WEB_INTERFACE_HPP_

#include <WebServer.h>
#include "config.hpp"

// Control actions remain in the sketch; this class owns HTTP and page rendering.
class WebInterface {
 public:
  using Action = void (*)();
  explicit WebInterface(PacerSettings& settings);
  void begin(Action start, Action stop, Action calibrate, Action tune);
  void update();

 private:
  void updateInputs();
  void handleRoot();
  void handleAction(Action action);

  PacerSettings& settings_;
  WebServer server;
};

#endif
