# 🕒 Smart Health Watch | Embedded C Project

A **bare-metal C** smartwatch system designed using the **ATmega328P**, which monitors **step count**, **heart rate**, and **skin temperature** using the **MPU6050**, an **ADC-based heart rate sensor**, and an **OLED display**. Ideal for embedded systems and wearable health monitoring applications.

---

## 🔧 Features

- 🦶 **Step Counting** using MPU6050 accelerometer with threshold filtering  
- ❤️ **Heart Rate Monitoring (BPM)** using ADC pulse detection (PPG method)  
- 🌡️ **Skin Temperature Sensing** using analog temperature sensor  
- 📺 **SSD1306 OLED Display** with I2C communication  
- 🔄 **UART Output** for Bluetooth/Serial communication to mobile/PC  
- ⏱️ **Timer-Driven ISR Architecture** for precise timing & low CPU load  
- 🧠 **Adaptive Threshold Algorithm** for noise-resistant heart rate calculation

---

## 🛠️ Hardware Components

| Component           | Description                                |
|--------------------|--------------------------------------------|
| ATmega328P         | Microcontroller (Arduino Uno core)         |
| MPU6050            | 3-axis Accelerometer + Gyroscope (I2C)     |
| SSD1306 OLED       | 128x64 pixel I2C OLED Display              |
| Pulse Sensor       | Photodiode + LED connected to ADC          |
| Temp Sensor        | Analog temperature sensor                  |
| Push Buttons       | Trigger step/BPM readings (PB0 and PD7)    |
| UART Module        | For serial or Bluetooth communication      |

---

## 🧩 System Architecture

User Inputs (PB0, PD7)
       │
       ▼
ATmega328P ───> MPU6050 (I2C)
       │
       ├─> ADC Input (Pulse + Temp)
       │
       ├─> SSD1306 OLED (I2C)
       │
       └─> UART Output (Bluetooth/Serial

#🧪 How It Works

##  Step Counting
         Vector magnitude of acceleration (√Xa² + Ya² + Za²)
         
         Threshold-based detection (> 1.2g) with a 200ms debounce window
         
         Steps shown live on the OLED screen
  
##  Heart Rate Monitoring
         ADC samples PPG signal every 2ms
         
         Rolling window-based mean & variance used to set adaptive threshold
         
         Rising & falling edges count heartbeats
         
         Heartbeats in 5s × 12 = BPM estimation

# 🧰 How to Build
  Toolchain Required:
  
  AVR-GCC
  avrdude
  Optional: Atmel Studio / PlatformIO / VS Code

Compile:

  avr-gcc -mmcu=atmega328p -Os main.c -o smartwatch.elf
  avr-objcopy -O ihex smartwatch.elf smartwatch.hex

Upload:
  avrdude -c arduino -p m328p -P COMx -b 115200 -U flash:w:smartwatch.hex
  
# 🖥️ Display Overview
  |OLED Line	  |Information                      |
  |------------|---------------------------------|
  |Line 0	 |Title: Medi-care                 |
  |Line 1	 |Status: Normal                   |
  |Line 2	 |BPM result or step prompt/result |
  |Line 3	 |Skin temperature                 | 
  |Line 7	 |Version information              |

# 🎮 Button Controls

  |Button	|Action                              |  
  |------------|-----------------------------------|
  |PB0	        |Reset  BPM and trigger HR reading  |
  |PD7	        |Start and stop step counter        |

