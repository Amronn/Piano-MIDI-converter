# 🎹 Piano MIDI Converter
Modular Acoustic Piano MIDI Converter is a system designed to convert the key and pedal movements of an acoustic piano into MIDI messages via USB.

## 🧩 System Overview
The system consists of three main types of modules:

### Main Controller
Built with an ESP32-S3, it manages communication, processing, and integration of all sensor modules. It handles MIDI output and coordinates the entire system.

### Sensor Modules
Two types of sensor modules are responsible for reading data from the piano. They communicate with the main controller via SPI and UART.

## 🎛️ Hardware Details
The current setup uses 5 sensor modules connected in series.

### Each module contains:
8× CNY70 reflective sensors for key press detection

MCP3008 ADC to convert analog readings for the microcontroller

## The system currently supports:
40 piano keys

2 piano pedals

## 🎼 Features
Modular architecture – easy to expand or modify

Real-time key and pedal monitoring

Converts mechanical actions to standard MIDI messages

Designed for precise, low-latency performance
