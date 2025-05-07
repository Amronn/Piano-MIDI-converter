# 🎹 Piano MIDI Converter

The Modular Acoustic Piano MIDI Converter converts key and pedal movements from an acoustic piano into MIDI messages via USB.

## 🧩 System Overview
- **Main Controller (ESP32-S3):** Manages communication, processing, and MIDI output.
- **Sensor Modules:** Read data from the piano keys and pedals via SPI and UART.

## 🎛️ Hardware Details
- **Modules:** 5 sensor modules connected in series.
- **Each module contains:**
  - 8× CNY70 reflective sensors for key press detection
  - MCP3008 ADC for analog-to-digital conversion

![Piano MIDI Converter](images/piano_midi_converter.jpg)

![External Converter Device](images/piano_midi_external_device.jpg)

## 🎼 Features
- Modular, expandable architecture
- Real-time monitoring of keys and pedals
- Converts mechanical actions to MIDI messages
- Precise, low-latency performance

**Supports:** 40 piano keys and 2 pedals.

