#if defined(SERIAL_IS_SERIALUSB) || defined(SERIAL_IS_CONFIGURABLE)
bool wait_for_usb_configured(unsigned long timeout) {
    unsigned long start = millis();
    while (!USBDevice.connected() && millis() - start < timeout) /* wait */;

    return USBDevice.connected();
}

bool wait_for_serialusb_opened(unsigned long timeout) {
    unsigned long start = millis();
    while (!SerialUSB && millis() - start < timeout) /* wait */;

    return (bool)SerialUSB;
}

bool setup_serialusb() {
  // First, see if we are configured at the USB device level. This
  // happens automatically when USB is plugged into an USB host without
  // any user interaction, typically within a couple of hundred ms. If
  // that doesn't happen quickly, assume we're not connected to USB at
  // all and fail.
  if (wait_for_usb_configured(1000)) {
    // USB device was configured, so start serial port.
    SerialUSB.begin(0);

    // Then, see if the serial port is actually opened. Because this
    // needs action from the user, give them ample time for this.
    if (wait_for_serialusb_opened(10000)) {
      // Serial port was opened, we can use it.
      //
      // Wait a bit more, otherwise some clients (e.g. minicom on Linux) seem to miss the first bit of output...
      delay(250);

      return true;
    } else {
      // Stop the serial port again so any prints to it are ignored,
      // rather than filling up the buffer and eventually locking up
      // when the buffer is full.
      SerialUSB.end();
    }
  }

  return false;
}
#endif // defined(SERIAL_IS_SERIALUSB) || defined(SERIAL_IS_CONFIGURABLE)

bool setup_serial() {
  #if defined(ARDUINO_MJS_V1) || defined(SERIAL_IS_SERIAL1)
  // For a hardware serial port, just call begin and be done.
  // Anyone interested in the output can start listening before power-up
  // or reset and if nobody is listening, then the messages are just
  // lost without blocking.
  Serial.begin(SERIAL_BAUD);
  #elif defined(SERIAL_IS_SERIALUSB)
  // For SerialUSB, things are more tricky, since the computer can only
  // open the serial port *after* the board started. This delays
  // startup to give the computer a chance to open the port so it can
  // see all messages printed to serial.
  setup_serialusb();
  #elif defined(SERIAL_IS_CONFIGURABLE)
  if (setup_serialusb()) {
    // SerialUSB port was openend, use that
    ConfigurableSerial = &SerialUSB;
  } else {
    // SerialUSB port was not opened, use hardware serial port instead
    Serial1.begin(SERIAL_BAUD);
    ConfigurableSerial = &Serial1;
  }
  #else
  #error "Unknown serial setup"
  #endif
}
