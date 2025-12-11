🌱 Advanced Intelligent Irrigation System
A smart, automated irrigation and water management system using real-time sensors, microcontrollers, and communication modules to optimize water usage in agriculture and domestic environments.

⭐ Features
🌡️ Real-Time Soil Moisture Monitoring
Continuously reads soil moisture to determine exact irrigation needs.

🕒 Scheduled Irrigation with RTC
Uses DS3231 RTC to run irrigation cycles at precise times.

📡 Remote Alerts via GSM (SIM800)
Sends SMS updates about tank level, pump status, power availability, and faults.

🚰 Automatic Pump & Valve Control
Activates pump and drip valves based on moisture, tank level, and timing.

📊 Dual Flow Measurement
Measures water delivery across two irrigation lines for accurate monitoring.

🔌 Power Availability Detection
Mains power detection ensures pump operates only when electricity is present.

⚡ Current Monitoring
Detects pump running status and protects against dry-run or overload.

📉 Tank & Well Water Level Monitoring
Ultrasonic sensor tracks tank capacity; well sensor checks water availability before pumping.

🖥️ Real-Time LCD Display
A 20x4 I2C LCD shows moisture, flow rate, water level, pump status, and system messages.

🔔 Fault Detection & Alerts
Alerts for low tank level, dry well, power failure, blocked flow, or pump malfunction.

♻️ Energy-Efficient Operation
Irrigation avoids peak hours and reduces unnecessary pumping.

🌿 Supports Precision Agriculture
Helps maintain optimal soil conditions with minimal water waste.

🔄 Fully Automated with Minimal Manual Intervention
Smart logic handles decisions, improving reliability and ease of use.

📘 Introduction

Water is one of the most vital natural resources, and its efficient management is crucial in modern agriculture. Due to rising water demand and climate change, smart irrigation systems have become essential.

This project automates irrigation and water control using sensors, microcontrollers, and GSM-based communication. It collects data such as:

Soil moisture

Water levels

Flow rates

Power status

Based on this data, the system controls pumps, drip irrigation lines, and alerts users remotely. It ensures sustainable water usage with minimal manual effort.

🪜STEPS

Step 1: Collect all the components and connect those to the Arduino Atmega 2560(you can use any arduino boards)(List of all components given in PPT)

Step 2: Install the Required Arduino Libraries(Given in LIBRARY Folder)

Step 3:Upload the Main Arduino Code to the atmega 2560( I have used Ardiuno IDE)

Step 4: Install System in Real Field / Garden

To run this project, download all required Arduino libraries here:
👉 Download libraries.zip

Unzip it and place the contents inside your Arduino `libraries` folder:
Documents → Arduino → libraries

Then restart the Arduino IDE.

