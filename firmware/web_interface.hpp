/**
 * @file web_interface.hpp
 * @brief HTTP control interface and callbacks for pacing commands.
 */

#ifndef PACER_FIRMWARE_WEB_INTERFACE_HPP_
#define PACER_FIRMWARE_WEB_INTERFACE_HPP_

#include <WebServer.h>
#include "config.hpp"

/**
 * @brief Serve a browser control page and dispatch synchronous control actions.
 * @details Holds a reference to shared settings; the caller owns their lifetime.
 * Each POST updates any supplied fields, invokes its callback, and returns 204.
 * Values are converted using Arduino String methods without range validation.
 */
class WebInterface {
 public:
  /**
   * @brief Callback executed synchronously while servicing a control request.
   */
  using Action = void (*)();
  /**
   * @brief Create an HTTP server on port 80 using the supplied settings.
   * @param settings Mutable run settings; must outlive this interface.
   */
  explicit WebInterface(PacerSettings& settings);
  /**
   * @brief Start the configured Wi-Fi access point and register HTTP routes.
   * @param start Non-null callback for POST /start.
   * @param stop Non-null callback for POST /stop.
   * @param calibrate Non-null callback for POST /calibrate.
   * @param tune Non-null callback for POST /pid.
   * @note Callbacks execute on the caller's loop; long actions delay other requests.
   */
  void begin(Action start, Action stop, Action calibrate, Action tune);
  /**
   * @brief Service pending HTTP work; call regularly from the main loop.
   */
  void update();

 private:
  /**
   * @brief Copy supplied form fields into settings, retaining omitted values.
   */
  void updateInputs();
  /**
   * @brief Render the current settings as HTML and respond with HTTP 200.
   */
  void handleRoot();
  /**
   * @brief Update settings, execute the action, and respond with HTTP 204.
   * @param action Non-null control callback supplied to begin().
   */
  void handleAction(Action action);

  PacerSettings& settings_;
  WebServer server;
};

#endif
