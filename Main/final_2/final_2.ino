#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>  // Added for watchdog timer
//***********************
// Configuration Constants
//***********************
namespace Config {
  // Pump control parameters
  const unsigned int PEAK_TIME_START = 1200;    // 6:00 PM (18:00)
  const unsigned int PEAK_TIME_END   = 1510;    // 7:00 PM (19:00)
  const unsigned long MAX_PUMP_RUN_TIME_MS = 180000UL;  // 3 minutes
  const unsigned long PUMP_ON_DELAY_MS     = 3000UL;    // 3 seconds
  // Tank parameters
  const int MAINS_VOLTAGE = 230;             // Volts
  const float TANK_CAPACITY = 150.0;          // liters
  const float TANK_HEIGHT   = 17;            // cm
  const float TANK_MAX_LEVEL = 130.0;          // liters
  const float TANK_MIN_LEVEL = 30.0;           // liters
  const float ULTRASONIC_MOUNT_OFFSET = 3.0;     // cm offset from tank top
  const float DRIP_ACTIVATION_THRESHOLD = 100.0; // liters (minimum level to start drip)
  // Well water level parameters
  const int WELL_MIN_LEVEL = 5;  // Minimum analog reading to run pump
  const int WELL_MAX_LEVEL = 220; // Level to resume pump after low water condition
  // Soil moisture parameters
  const int DRIP_MOIST_LOW = 500;  // Dry soil reading
  const int DRIP_MOIST_HIGH = 220; // Wet soil reading
  const int DRIP_MOIST_DRY = 420;  // Threshold for dry condition (above this is DRY)
  const int DRIP_MOIST_WET = 300;  // Threshold for wet condition (below this is WET)
  // Drip valve parameters
  const unsigned int DRIP_MORNING_START = 1226;  // 5:00 AM
  const unsigned int DRIP_MORNING_END   = 1430;  // 7:30 AM
  const unsigned int DRIP_EVENING_START = 1540; // 4:00 PM
  const unsigned int DRIP_EVENING_END   = 1841; // 6:30 PM
  const float DRIP_LOW_FLOW_THRESHOLD  = 1.0;    // L/min
  const float DRIP_HIGH_FLOW_THRESHOLD = 3.0;    // L/min
  const float DRIP_VOLUME_THRESHOLD    = 5.0;  // liters
  const unsigned long FLOW_RATE_DELAY_MS = 5000UL; // delay (ms) for low-flow during startup
  // Flow sensor calibration and measurement
  const float FLOW_SENSOR_CALIBRATION = 1500;  // pulses per liter
  const int NUM_SAMPLES = 5;
  const unsigned long FLOW_MEASURE_INTERVAL_MS = 1000UL; //(1 second interval)
  // Debounce parameters
  const unsigned long DEBOUNCE_DELAY_MS = 50; // 50ms debounce time
   // Non-blocking timing parameters
  const unsigned long LCD_UPDATE_INTERVAL_MS = 1000;    // 1 second
  const unsigned long SERIAL_UPDATE_INTERVAL_MS = 1000; // 1 second
  const unsigned long SOIL_READING_INTERVAL_MS = 2000;  // 2 seconds
  const unsigned long WELL_READING_INTERVAL_MS = 2000;  // 2 seconds
  const unsigned long ULTRASONIC_READING_INTERVAL_MS = 100; // 100ms
}
// Hardware Pin Definitions
namespace Pins {
  const uint8_t PUMP_RELAY      = 22;
  const uint8_t MAINS_SENSOR    = 23;
  const uint8_t DRIP_VALVE1     = 25;
  const uint8_t DRIP_VALVE2     = 26;
  const uint8_t DRIP_RELAY      = 27;
  const uint8_t FLOW_SENSOR1    = 2;    // Must be interrupt-capable on Mega
  const uint8_t FLOW_SENSOR2    = 3;    // Must be interrupt-capable on Mega
  const uint8_t ULTRASONIC_TRIG = 11;
  const uint8_t ULTRASONIC_ECHO = 10;
  const uint8_t CURRENT_SENSOR  = A0;
  const uint8_t WELL_WATER      = A1;   // Analog pin for well water pressure
  const uint8_t SOIL_MOISTURE1  = A2;   // Capacitive soil moisture for valve 1
  const uint8_t SOIL_MOISTURE2  = A3;   // Capacitive soil moisture for valve 2
}
//***********************
// Global Objects
//***********************
RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27, 20, 4);
// Forward declarations for ISRs
void flowSensor1_ISR();
void flowSensor2_ISR();
//***********************
// Flow Sensor State - Volatile for ISR
//***********************
namespace FlowSensorState {
  volatile unsigned long flowCount1 = 0;
  volatile unsigned long flowCount2 = 0;
}
//***********************
// Time Utilities
//***********************
class TimeUtils {
public:
  // Convert DateTime to HHMM format (0-2359)
  static unsigned int timeToHHMM(const DateTime &dt) {
    return dt.hour() * 100 + dt.minute();
  }
  // Check if current time is between startTime and endTime in HHMM format
  static bool isTimeBetween(unsigned int startTime, unsigned int endTime, const DateTime &nowTime) {
    unsigned int currentTime = timeToHHMM(nowTime);
    if (startTime < endTime)
      return (currentTime >= startTime && currentTime < endTime);
    else
      return (currentTime >= startTime || currentTime < endTime);
  }
  // Extract hours and minutes from HHMM format
  static void extractHoursAndMinutes(unsigned int timeHHMM, int &hours, int &minutes) {
    hours = timeHHMM / 100;
    minutes = timeHHMM % 100;
  }
   // Handle millis() rollover safely
  static unsigned long getTimeDifference(unsigned long start, unsigned long current) {
    return (current >= start) ? (current - start) : (0xFFFFFFFFUL - start + current + 1);
  }
   // Format current time as string
  static void getTimeString(char* buffer, const DateTime &dt) {
    sprintf(buffer, "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second());
  }
};
//***********************
// Sensor Manager Class
//***********************
class SensorManager {
private:
  // Temperature variables
  float currentTemperature = 0.0;
  // Ultrasonic sensor readings
  float ultrasonicReadings[Config::NUM_SAMPLES];
  int ultrasonicReadIndex = 0;
  unsigned long lastUltrasonicReadTime = 0;
  // Soil moisture variables
  int soilMoisture1 = 0;
  int soilMoisture2 = 0;
  unsigned long lastSoilReadingTime = 0;
  String moisture1Status = ""; // "DRY", "MOIST", or "WET"
  String moisture2Status = ""; // "DRY", "MOIST", or "WET"
  // Well water level monitoring
  int wellWaterReading = 0;         // Current analog reading from well water sensor
  bool wellWaterLow = false;        // Flag to indicate if well water is low
  unsigned long lastWellReadingTime = 0;
  // Flow rates
  float flowRate1 = 0.0;  // L/min from sensor 1 (for drip valve 1)
  float flowRate2 = 0.0;  // L/min from sensor 2 (for drip valve 2)
  unsigned long lastFlowMeasureTime = 0;
  // Mains power detection
  unsigned long lastMainsDebounceTime = 0;
  int mainsLastReading = LOW;
  int mainsState = LOW;  // The debounced state (LOW = mains ON, HIGH = mains OFF)

public:
  SensorManager() {
    // Initialize ultrasonic readings array
    for (int i = 0; i < Config::NUM_SAMPLES; i++) {
      ultrasonicReadings[i] = 0.0;
    }
  }
  
  void init() {
    // Set pin modes
    pinMode(Pins::ULTRASONIC_TRIG, OUTPUT);
    pinMode(Pins::ULTRASONIC_ECHO, INPUT);
    pinMode(Pins::MAINS_SENSOR, INPUT);
    // Initialize readings
    updateSoilMoisture();
    updateWellWaterLevel();
    updateTemperature();
    lastFlowMeasureTime = millis();
    // Initialize mains power detection
    mainsLastReading = digitalRead(Pins::MAINS_SENSOR);
    mainsState = mainsLastReading;
    lastMainsDebounceTime = millis();
  }
  
  void updateAll() {
    updateSoilMoisture();
    updateWellWaterLevel();
    updateUltrasonicSensor();
    updateFlowRates();
    updateTemperature();
    updateMainsState();
  }
   // Read temperature from the DS3231 RTC
  void updateTemperature() {
    currentTemperature = rtc.getTemperature();
  }
  // Non-blocking ultrasonic sensor reading
  void updateUltrasonicSensor() {
    unsigned long currentMillis = millis();
    if (TimeUtils::getTimeDifference(lastUltrasonicReadTime, currentMillis) >= Config::ULTRASONIC_READING_INTERVAL_MS) {
      digitalWrite(Pins::ULTRASONIC_TRIG, LOW);
      delayMicroseconds(2);
      digitalWrite(Pins::ULTRASONIC_TRIG, HIGH);
      delayMicroseconds(50);
      digitalWrite(Pins::ULTRASONIC_TRIG, LOW);
      long duration = pulseIn(Pins::ULTRASONIC_ECHO, HIGH, 30000);
      float distance = (duration * 0.0343) / 2.0;
      // Store in circular buffer
      ultrasonicReadings[ultrasonicReadIndex] = distance;
      ultrasonicReadIndex = (ultrasonicReadIndex + 1) % Config::NUM_SAMPLES;
      lastUltrasonicReadTime = currentMillis;
    }
  }
  // Calculate tank volume
  float getTankVolume() { 
    // Calculate average distance from readings buffer
    float sum = 0;
    for (int i = 0; i < Config::NUM_SAMPLES; i++) {
      sum += ultrasonicReadings[i];
    }
    float avgDistance = sum / Config::NUM_SAMPLES;
    // Adjust for ultrasonic mount offset
    float adjustedDistance = avgDistance - Config::ULTRASONIC_MOUNT_OFFSET;
    // Calculate water level and volume
    float waterLevel = Config::TANK_HEIGHT - adjustedDistance;
    float volume = waterLevel * (Config::TANK_CAPACITY / Config::TANK_HEIGHT);
    // Constrain the volume
    return constrain(volume, 0, Config::TANK_CAPACITY);
  }
  // Read soil moisture sensors
  void updateSoilMoisture() {
    unsigned long currentMillis = millis();
    if (TimeUtils::getTimeDifference(lastSoilReadingTime, currentMillis) >= Config::SOIL_READING_INTERVAL_MS) {
      // Read sensors
      soilMoisture1 = analogRead(Pins::SOIL_MOISTURE1);
      soilMoisture2 = analogRead(Pins::SOIL_MOISTURE2);
      // Determine moisture status for sensor 1
      if (soilMoisture1 >= Config::DRIP_MOIST_DRY) {
        moisture1Status = "DRY";
      } else if (soilMoisture1 <= Config::DRIP_MOIST_WET) {
        moisture1Status = "WET";
      } else {
        moisture1Status = "MOIST";
      }
      // Determine moisture status for sensor 2
      if (soilMoisture2 >= Config::DRIP_MOIST_DRY) {
        moisture2Status = "DRY";
      } else if (soilMoisture2 <= Config::DRIP_MOIST_WET) {
        moisture2Status = "WET";
      } else {
        moisture2Status = "MOIST";
      }
      lastSoilReadingTime = currentMillis;
    }
  }
  // Read and assess well water level
  void updateWellWaterLevel() {
    unsigned long currentMillis = millis();
    if (TimeUtils::getTimeDifference(lastWellReadingTime, currentMillis) >= Config::WELL_READING_INTERVAL_MS) {
      wellWaterReading = analogRead(Pins::WELL_WATER);
      // Update well water state based on hysteresis thresholds
      bool previousState = wellWaterLow;
      if (wellWaterReading < Config::WELL_MIN_LEVEL) {
        wellWaterLow = true;
      } 
      else if (wellWaterReading >= Config::WELL_MAX_LEVEL) {
        wellWaterLow = false;
      }
      // Log state change
      if (previousState != wellWaterLow) {
        char timeStr[20];
        TimeUtils::getTimeString(timeStr, rtc.now());
        Serial.print(F("["));
        Serial.print(timeStr);
        Serial.print(F("] WELL WATER: "));
        Serial.print(wellWaterLow ? F("LOW") : F("NORMAL"));
        Serial.print(F(" (Reading: "));
        Serial.print(wellWaterReading);
        Serial.print(F(", Threshold: "));
        Serial.print(wellWaterLow ? Config::WELL_MIN_LEVEL : Config::WELL_MAX_LEVEL);
        Serial.println(F(")"));
      }
      lastWellReadingTime = currentMillis;
    }
  }
  // Update flow sensor rates using interrupts
  void updateFlowRates() {
    unsigned long currentMillis = millis();
    if (TimeUtils::getTimeDifference(lastFlowMeasureTime, currentMillis) >= Config::FLOW_MEASURE_INTERVAL_MS) {
      noInterrupts();
      unsigned long count1 = FlowSensorState::flowCount1;
      unsigned long count2 = FlowSensorState::flowCount2;
      FlowSensorState::flowCount1 = 0;
      FlowSensorState::flowCount2 = 0;
      interrupts();
      float elapsedSeconds = TimeUtils::getTimeDifference(lastFlowMeasureTime, currentMillis) / 1000.0;
      // Calculate flow rates
      float liters1 = count1 / Config::FLOW_SENSOR_CALIBRATION;
      flowRate1 = (liters1 * 60.0) / elapsedSeconds;
      float liters2 = count2 / Config::FLOW_SENSOR_CALIBRATION;
      flowRate2 = (liters2 * 60.0) / elapsedSeconds;
      lastFlowMeasureTime = currentMillis;
    }
  }
  // Improved debounced mains power detection
  void updateMainsState() {
    // Read the current state of the mains sensor pin
    int reading = digitalRead(Pins::MAINS_SENSOR);
    unsigned long currentMillis = millis();
    // If the reading has changed, reset the debounce timer
    if (reading != mainsLastReading) {
      lastMainsDebounceTime = currentMillis;
    }
    // If enough time has passed since the last change, update the debounced state
    if (TimeUtils::getTimeDifference(lastMainsDebounceTime, currentMillis) > Config::DEBOUNCE_DELAY_MS) {
      // Only update the state if it's actually changed
      if (mainsState != reading) {
        mainsState = reading;
        // Log state change when debounced state changes
        char timeStr[20];
        TimeUtils::getTimeString(timeStr, rtc.now());
        Serial.print(F("["));
        Serial.print(timeStr);
        Serial.print(F("] MAINS POWER: "));
        Serial.println(mainsState ? F("OFF") : F("ON"));
      }
    }
    // Save the current reading for next comparison
    mainsLastReading = reading;
  }
  // Read pump current
  float readPumpCurrent() {
    int sensorValue = analogRead(Pins::CURRENT_SENSOR);
    float voltage = sensorValue * (5.0 / 1024.0);
    float current = (voltage - 2.5) / 0.185;
    return abs(current);
  }
  // Getters for sensor values
  float getTemperature() const { return currentTemperature; }
  int getSoilMoisture1() const { return soilMoisture1; }
  int getSoilMoisture2() const { return soilMoisture2; }
  const String& getMoisture1Status() const { return moisture1Status; }
  const String& getMoisture2Status() const { return moisture2Status; }
  int getWellWaterReading() const { return wellWaterReading; }
  bool isWellWaterLow() const { return wellWaterLow; }
  float getFlowRate1() const { return flowRate1; }
  float getFlowRate2() const { return flowRate2; }
  bool isMainsOff() const { return mainsState == HIGH; }
  // Function to check if it's time to refill based on soil moisture
  bool isDripNeeded(int valve) const {
    if (valve == 1) {
      return moisture1Status != "WET";
    } else {
      return moisture2Status != "WET";
    }
  }
  // Free memory monitoring
  int getFreeRam() {
    extern int __heap_start, *__brkval; 
    int v; 
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
  }
};
//***********************
// Drip Valve Class (Improved)
//***********************
class DripValve {
private:
  uint8_t pin;
  float cumulative;            // Delivered volume in liters
  unsigned long badFlowStart;  // Timestamp when low flow condition was first detected
  bool abnormal;               // Latch: once abnormal during drip period, valve will not re-activate
  unsigned long lastUpdate;    // For integrating cumulative volume
  unsigned long activationTime; // Timestamp when valve was turned on
  String shutoffReason;        // Tracks the reason for abnormal shutoff
  unsigned long lastLowTankMsg; // Timestamp of last low tank message
  bool moistureWetDetected;    // Flag to indicate if moisture WET status was detected during this drip period
public:
  DripValve(uint8_t _pin)
    : pin(_pin), cumulative(0.0), badFlowStart(0), abnormal(false),
      lastUpdate(millis()), activationTime(0), shutoffReason(""), lastLowTankMsg(0),
      moistureWetDetected(false) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH); // Initialize to HIGH (inactive)
  }
  // Reset the valve state (called when drip period is not active)
  void reset() {
    cumulative = 0.0;
    badFlowStart = 0;
    abnormal = false;
    activationTime = 0;
    lastUpdate = millis();
    shutoffReason = "";
    moistureWetDetected = false; // Reset moisture wet detection for new drip period // Don't reset lastLowTankMsg to avoid message spam when drip time starts/stops
    digitalWrite(pin, HIGH); // Set to HIGH (inactive)
  }
  // Update the valve based on the instantaneous flow rate, tank volume, and current time.
  void update(float flowRate, float tankVolume, const DateTime &now, int moistureReading, const String &moistureStatus) {
    // Split the complex update function into smaller functions for clarity
    bool dripTime = isDripTime(now);
    if (!dripTime) {
      reset();  // Reset state when drip period ends.
      return;
    }
    bool valveIsOn = isValveOn();
    bool valveShouldBeOn = shouldValveBeOn(tankVolume, moistureStatus, valveIsOn);
    // Monitor flow rate and latch abnormal conditions
    if (valveIsOn) {
      monitorFlowRate(flowRate, now);
    }
    // Check for maximum volume threshold
    if (cumulative >= Config::DRIP_VOLUME_THRESHOLD) {
      handleMaxVolumeReached(now);
      valveShouldBeOn = false;
    }
    // Calculate elapsed time since the last update for flow integration
    unsigned long currentMillis = millis();
    unsigned long dt = TimeUtils::getTimeDifference(lastUpdate, currentMillis);
    lastUpdate = currentMillis;
    // Update valve state and integrate cumulative volume
    setValveState(valveShouldBeOn, flowRate, dt);
  }
  // Get the current volume delivered
  float getCumulativeVolume() const {
    return cumulative;
  }
  // Get the reason for shutoff
  const String& getShutoffReason() const {
    return shutoffReason;
  }
  // Check if valve had an abnormal condition
  bool isAbnormal() const {
    return abnormal;
  }
private:
  // Check if current valve is ON
  bool isValveOn() const {
    return digitalRead(pin) == LOW; // Valve is ON when pin is LOW
  }
  // Check if the valve should be turned ON based on conditions
  bool shouldValveBeOn(float tankVolume, const String &moistureStatus, bool valveIsOn) {
    // Check if we've detected WET soil during this drip period
    if (moistureStatus == "WET" && !moistureWetDetected) {
      // Log the first time we detect WET soil in this period
      if (isValveOn()) {
        char timeStr[20];
        TimeUtils::getTimeString(timeStr, rtc.now());
        Serial.print(F("["));
        Serial.print(timeStr);
        Serial.print(F("] DRIP VALVE "));
        Serial.print(pin == Pins::DRIP_VALVE1 ? F("1") : F("2"));
        Serial.println(F(": SHUTOFF - Wet Soil Detected"));
      }
      moistureWetDetected = true;
    }
    // Basic conditions for activation - now includes check for moistureWetDetected
    bool basicOnCondition = (cumulative < Config::DRIP_VOLUME_THRESHOLD) &&
                            (!abnormal) &&
                            (!moistureWetDetected); // Don't activate if wet soil was detected during this period
    // Only check tank level threshold if the valve is currently OFF
    if (!valveIsOn) {
      bool tankLevelOK = (tankVolume > Config::DRIP_ACTIVATION_THRESHOLD);
      // If other conditions are good but tank level is too low, log it (but limit to once every 5 seconds)
      if (!tankLevelOK && basicOnCondition) {
        unsigned long currentTime = millis();
        if (TimeUtils::getTimeDifference(lastLowTankMsg, currentTime) >= 5000) { // 5 seconds interval
          char timeStr[20];
          TimeUtils::getTimeString(timeStr, rtc.now());
          Serial.print(F("["));
          Serial.print(timeStr);
          Serial.print(F("] DRIP VALVE "));
          Serial.print(pin == Pins::DRIP_VALVE1 ? F("1") : F("2"));
          Serial.print(F(": ACTIVATION PREVENTED - Low Tank Level - "));
          Serial.print(tankVolume);
          Serial.print(F(" L (Threshold: "));
          Serial.print(Config::DRIP_ACTIVATION_THRESHOLD);
          Serial.println(F(" L)"));
          // Update timestamp of last message
          lastLowTankMsg = currentTime;
        }
      }
      basicOnCondition = basicOnCondition && tankLevelOK;
    }
    return basicOnCondition;
  }
  // Monitor flow rate and set abnormal flag if needed
  void monitorFlowRate(float flowRate, const DateTime &now) {
    // Record activation time if not already set
    if (activationTime == 0) {
      activationTime = millis();
    }
    // Always check for high flow condition
    if (flowRate > Config::DRIP_HIGH_FLOW_THRESHOLD) {
      // Log the high flow condition if this is a new abnormal condition
      if (!abnormal) {
        shutoffReason = "High Flow Rate";
        char timeStr[20];
        TimeUtils::getTimeString(timeStr, now);
        Serial.print(F("["));
        Serial.print(timeStr);
        Serial.print(F("] DRIP VALVE "));
        Serial.print(pin == Pins::DRIP_VALVE1 ? F("1") : F("2"));
        Serial.print(F(": SHUTOFF - High Flow Rate - "));
        Serial.print(flowRate);
        Serial.print(F(" L/min (Threshold: "));
        Serial.print(Config::DRIP_HIGH_FLOW_THRESHOLD);
        Serial.println(F(" L/min)"));
      }
      abnormal = true;
    }
    else {
      unsigned long onDuration = TimeUtils::getTimeDifference(activationTime, millis());
      // In the initial activation period, wait for FLOW_RATE_DELAY_MS // before shutting off if the flow is low
      if (onDuration < Config::FLOW_RATE_DELAY_MS) {
        if (flowRate < Config::DRIP_LOW_FLOW_THRESHOLD) {
          if (badFlowStart == 0) {
            badFlowStart = millis();
          } else if (TimeUtils::getTimeDifference(badFlowStart, millis()) >= Config::FLOW_RATE_DELAY_MS) {
            logLowFlowShutoff(now, flowRate);
          }
        } 
        else {
          badFlowStart = 0;
        }
      } 
      else {
        // After the initial period, any low flow triggers immediate shutdown
        if (flowRate < Config::DRIP_LOW_FLOW_THRESHOLD) {
          logLowFlowShutoff(now, flowRate);
        }
      }
    }
  }
  // Log low flow shutoff condition
  void logLowFlowShutoff(const DateTime &now, float flowRate) {
    // Log the low flow condition if this is a new abnormal condition
    if (!abnormal) {
      shutoffReason = "Low Flow Rate";
      char timeStr[20];
      TimeUtils::getTimeString(timeStr, now);
      Serial.print(F("["));
      Serial.print(timeStr);
      Serial.print(F("] DRIP VALVE "));
      Serial.print(pin == Pins::DRIP_VALVE1 ? F("1") : F("2"));
      Serial.print(F(": SHUTOFF - Low Flow Rate - "));
      Serial.print(flowRate);
      Serial.print(F(" L/min (Threshold: "));
      Serial.print(Config::DRIP_LOW_FLOW_THRESHOLD);
      Serial.println(F(" L/min)"));
    }
    abnormal = true;
  }
  // Handle maximum volume reached condition
  void handleMaxVolumeReached(const DateTime &now) {
    // Log the maximum volume condition if this is a new abnormal condition
    if (!abnormal) {
      shutoffReason = "Maximum Volume Reached";
      char timeStr[20];
      TimeUtils::getTimeString(timeStr, now);
      Serial.print(F("["));
      Serial.print(timeStr);
      Serial.print(F("] DRIP VALVE "));
      Serial.print(pin == Pins::DRIP_VALVE1 ? F("1") : F("2"));
      Serial.print(F(": SHUTOFF - Maximum Volume Reached - "));
      Serial.print(cumulative);
      Serial.print(F(" L (Threshold: "));
      Serial.print(Config::DRIP_VOLUME_THRESHOLD);
      Serial.println(F(" L)"));
    }
    abnormal = true;
  }
  // Set valve state and update cumulative volume
  void setValveState(bool valveShouldBeOn, float flowRate, unsigned long dt) {
    if (valveShouldBeOn) {
      if (digitalRead(pin) == HIGH) { // If valve is currently inactive (HIGH)
        activationTime = millis();
      }
      digitalWrite(pin, LOW); // Set to LOW (active)
      // Integrate delivered volume: (flowRate in L/min) to L/sec * dt in seconds
      float volumeDelivered = (flowRate / 60.0) * (dt / 1000.0);
      cumulative += volumeDelivered;
    } else {
      digitalWrite(pin, HIGH); // Set to HIGH (inactive)
      if (!abnormal) {
        cumulative = 0.0;
      }
    }
  }
  // Check if the current time is within one of the drip periods
  bool isDripTime(const DateTime &now) {
    return TimeUtils::isTimeBetween(Config::DRIP_MORNING_START, Config::DRIP_MORNING_END, now) ||
           TimeUtils::isTimeBetween(Config::DRIP_EVENING_START, Config::DRIP_EVENING_END, now);
  }
};
//***********************
// Pump Manager Class
//***********************
class PumpManager {
private:
  // Pump control globals
  unsigned long pumpOnTime = 0;
  bool pumpRunning = false;
  float initialTankVolume = 0.0;
  unsigned long pumpRequestTime = 0;
  bool pumpStoppedDueToMains = false;
  bool pumpStoppedDueToWellWater = false;
  // Variables for pump run time tracking
  unsigned long cumulativePumpRunTime = 0;
  unsigned long lastRunDuration = 0;
  bool activePumpCycle = false;
  // Variables for drip period tracking
  bool morningDripActive = false;
  bool eveningDripActive = false;
  bool postMorningDripPumpStarted = false;
  bool postEveningDripPumpStarted = false;
  unsigned long morningDripEndTime = 0;
  unsigned long eveningDripEndTime = 0;
  // Variables for pre-drip pump activation
  bool preMorningDripPumpStarted = false;
  bool preEveningDripPumpStarted = false;
  unsigned int lastDayChecked = 0;
  // Track state for event detection
  bool prevPumpState = false;

public:
  PumpManager() {}
  void init() {
    pinMode(Pins::PUMP_RELAY, OUTPUT);
    digitalWrite(Pins::PUMP_RELAY, HIGH); // Initialize OFF
    pumpRequestTime = millis();
    prevPumpState = false;
    // Initialize pump control flags
    pumpStoppedDueToMains = false;
    pumpStoppedDueToWellWater = false;
    cumulativePumpRunTime = 0;
    activePumpCycle = false;
    // Initialize post-drip pump flags
    morningDripActive = false;
    eveningDripActive = false;
    postMorningDripPumpStarted = false;
    postEveningDripPumpStarted = false;
    // Initialize pre-drip pump flags
    preMorningDripPumpStarted = false;
    preEveningDripPumpStarted = false;
    lastDayChecked = 0;
  }
  // Update drip period state tracking
  void updateDripPeriodTracking(const DateTime &now) {
    bool inMorningDrip = TimeUtils::isTimeBetween(Config::DRIP_MORNING_START, Config::DRIP_MORNING_END, now);
    bool inEveningDrip = TimeUtils::isTimeBetween(Config::DRIP_EVENING_START, Config::DRIP_EVENING_END, now);
    char timeStr[20];
    // Check morning drip period state
    if (inMorningDrip && !morningDripActive) {
      morningDripActive = true;
      postMorningDripPumpStarted = false;
      TimeUtils::getTimeString(timeStr, now);
      Serial.print(F("["));
      Serial.print(timeStr);
      Serial.println(F("] MORNING DRIP PERIOD: STARTED"));
    }
    else if (!inMorningDrip && morningDripActive) {
      morningDripActive = false;
      morningDripEndTime = millis();
      TimeUtils::getTimeString(timeStr, now);
      Serial.print(F("["));
      Serial.print(timeStr);
      Serial.println(F("] MORNING DRIP PERIOD: ENDED"));
    }
    // Check evening drip period state
    if (inEveningDrip && !eveningDripActive) {
      eveningDripActive = true;
      postEveningDripPumpStarted = false;
      TimeUtils::getTimeString(timeStr, now);
      Serial.print(F("["));
      Serial.print(timeStr);
      Serial.println(F("] EVENING DRIP PERIOD: STARTED"));
    }
    else if (!inEveningDrip && eveningDripActive) {
      eveningDripActive = false;
      eveningDripEndTime = millis();
      TimeUtils::getTimeString(timeStr, now);
      Serial.print(F("["));
      Serial.print(timeStr);
      Serial.println(F("] EVENING DRIP PERIOD: ENDED"));
    }
  }
  // Check for pre-drip pump activation
  bool checkPreDripPumpActivation(float tankVolume, const DateTime &now) {
    unsigned int currentTimeHHMM = TimeUtils::timeToHHMM(now);
    unsigned int currentDay = now.day();
    // Calculate time to check before drip periods (using MAX_PUMP_RUN_TIME_MS in minutes)
    unsigned int preCheckTimeMinutes = Config::MAX_PUMP_RUN_TIME_MS / 60000;
    // Calculate the time to check before morning drip
    int morningHours = 0;
    int morningMinutes = 0;
    TimeUtils::extractHoursAndMinutes(Config::DRIP_MORNING_START, morningHours, morningMinutes);
    int preCheckMorningHours = morningHours;
    int preCheckMorningMinutes = morningMinutes - preCheckTimeMinutes;
    // Handle time rollover correctly
    while (preCheckMorningMinutes < 0) {
      preCheckMorningMinutes += 60;
      preCheckMorningHours--;
    }
    if (preCheckMorningHours < 0) {
      preCheckMorningHours += 24;
    }
    unsigned int preMorningCheckTime = preCheckMorningHours * 100 + preCheckMorningMinutes;
    // Calculate the time to check before evening drip
    int eveningHours = 0;
    int eveningMinutes = 0;
    TimeUtils::extractHoursAndMinutes(Config::DRIP_EVENING_START, eveningHours, eveningMinutes);
    int preCheckEveningHours = eveningHours;
    int preCheckEveningMinutes = eveningMinutes - preCheckTimeMinutes;
    // Handle time rollover correctly
    while (preCheckEveningMinutes < 0) {
      preCheckEveningMinutes += 60;
      preCheckEveningHours--;
    }
    if (preCheckEveningHours < 0) {
      preCheckEveningHours += 24;
    }
    unsigned int preEveningCheckTime = preCheckEveningHours * 100 + preCheckEveningMinutes;
    // Check if we need to activate pump for morning drip period
    bool activateForMorning = false;
    if (currentTimeHHMM == preMorningCheckTime && !preMorningDripPumpStarted && (lastDayChecked != currentDay)) {
      if (tankVolume < Config::DRIP_ACTIVATION_THRESHOLD) {
        activateForMorning = true;
        preMorningDripPumpStarted = true;
        lastDayChecked = currentDay;
        char timeStr[20];
        TimeUtils::getTimeString(timeStr, now);
        Serial.print(F("["));
        Serial.print(timeStr);
        Serial.println(F("] PRE-MORNING DRIP: Tank level below threshold, pump activation required"));
        return true;
      }
    }
    // Reset the morning flag if we're past the morning check time but before the next day
    if (currentTimeHHMM > preMorningCheckTime && currentTimeHHMM < Config::DRIP_MORNING_START) {
      if (preMorningDripPumpStarted && lastDayChecked == currentDay) {
        // Keep the flag active until the drip period starts
      }
    } else if (!TimeUtils::isTimeBetween(preMorningCheckTime, Config::DRIP_MORNING_START, now)) {
      preMorningDripPumpStarted = false;
    }
    // Check if we need to activate pump for evening drip period
    bool activateForEvening = false;
    if (currentTimeHHMM == preEveningCheckTime && !preEveningDripPumpStarted && (lastDayChecked != currentDay || morningDripActive)) {
      if (tankVolume < Config::DRIP_ACTIVATION_THRESHOLD) {
        activateForEvening = true;
        preEveningDripPumpStarted = true;
        char timeStr[20];
        TimeUtils::getTimeString(timeStr, now);
        Serial.print(F("["));
        Serial.print(timeStr);
        Serial.println(F("] PRE-EVENING DRIP: Tank level below threshold, pump activation required"));
        return true;
      }
    }
    // Reset the evening flag if we're past the evening check time but before the next day
    if (currentTimeHHMM > preEveningCheckTime && currentTimeHHMM < Config::DRIP_EVENING_START) {
      if (preEveningDripPumpStarted) {
        // Keep the flag active until the drip period starts
      }
    } else if (!TimeUtils::isTimeBetween(preEveningCheckTime, Config::DRIP_EVENING_START, now)) {
      preEveningDripPumpStarted = false;
    }
    return activateForMorning || activateForEvening;
  }
  // Main pump control logic
  void updatePumpControl(float tankVolume, const SensorManager &sensors, const DateTime &now) {
    bool pumpOff = false;
    bool isPeakOrDripTime = false;
    bool isMainsOff = false;
    bool isWellWaterLow = false;
    bool postDripPumpActivation = false;
    bool preDripPumpActivation = false;
    String pumpActivationReason = "";
    // Check time constraints
    if (TimeUtils::isTimeBetween(Config::PEAK_TIME_START, Config::PEAK_TIME_END, now) ||
        TimeUtils::isTimeBetween(Config::DRIP_MORNING_START, Config::DRIP_MORNING_END, now) ||
        TimeUtils::isTimeBetween(Config::DRIP_EVENING_START, Config::DRIP_EVENING_END, now)) {
      isPeakOrDripTime = true;
      pumpOff = true;
    }
    // Check mains power
    isMainsOff = sensors.isMainsOff();
    if (isMainsOff) {
      pumpOff = true;
    }
    // Check well water
    isWellWaterLow = sensors.isWellWaterLow();
    if (isWellWaterLow) {
      pumpOff = true;
    }
    // Check for pre-drip pump activation
    preDripPumpActivation = checkPreDripPumpActivation(tankVolume, now);
    if (preDripPumpActivation) {
      pumpActivationReason = "Pre-Drip Tank Filling";
    }
    // Check for post-drip refill condition (morning)
    if (!morningDripActive && !postMorningDripPumpStarted && 
        morningDripEndTime > 0 && (TimeUtils::getTimeDifference(morningDripEndTime, millis()) < 300000)) { // Within 5 minutes of drip end
      if (tankVolume < Config::TANK_MAX_LEVEL && !isPeakOrDripTime && !isMainsOff && !isWellWaterLow) {
        postDripPumpActivation = true;
        pumpActivationReason = "Post-Morning Drip Refill";
      }
    }
    // Check for post-drip refill condition (evening)
    if (!eveningDripActive && !postEveningDripPumpStarted && 
        eveningDripEndTime > 0 && (TimeUtils::getTimeDifference(eveningDripEndTime, millis()) < 300000)) { // Within 5 minutes of drip end
      if (tankVolume < Config::TANK_MAX_LEVEL && !isPeakOrDripTime && !isMainsOff && !isWellWaterLow) {
        postDripPumpActivation = true;
        pumpActivationReason = "Post-Evening Drip Refill";
      }
    }
    // Other pump-off conditions
    // Check if total run time (cumulative) has exceeded the maximum
    if (pumpRunning && (cumulativePumpRunTime + TimeUtils::getTimeDifference(pumpOnTime, millis()) >= Config::MAX_PUMP_RUN_TIME_MS))
      pumpOff = true;
    if (tankVolume >= Config::TANK_MAX_LEVEL)
      pumpOff = true;
    // Pump control implementation
    if (pumpOff) {
      handlePumpOff(tankVolume, isPeakOrDripTime, isMainsOff, isWellWaterLow, now);
    } else {
      handlePumpOn(tankVolume, postDripPumpActivation, preDripPumpActivation, 
                  isPeakOrDripTime, isMainsOff, isWellWaterLow, pumpActivationReason, now);
    }
    // Check for state changes and log events
    checkPumpEvents(now);
  }
  // Handle pump off conditions
  void handlePumpOff(float tankVolume, bool isPeakOrDripTime, bool isMainsOff, bool isWellWaterLow, const DateTime &now) {
    if (pumpRunning) {
      // Before turning off, update cumulative run time
      lastRunDuration = TimeUtils::getTimeDifference(pumpOnTime, millis());
      cumulativePumpRunTime += lastRunDuration;
      // Record reason for pump shutdown for potential restart
      if (isMainsOff) {
        pumpStoppedDueToMains = true;
      }
      if (isWellWaterLow) {
        pumpStoppedDueToWellWater = true;
      }
      digitalWrite(Pins::PUMP_RELAY, HIGH);
      pumpRunning = false;
      char timeStr[20];
      TimeUtils::getTimeString(timeStr, now);
      Serial.print(F("["));
      Serial.print(timeStr);
      Serial.print(F("] PUMP: OFF - Reason: "));
      if (isPeakOrDripTime) {
        logPeakOrDripTimeShutoff(now);
      }
      else if (isMainsOff) {
        Serial.println(F("Mains Power Failure"));
        // Make sure we maintain the active pump cycle flag during power failure
        if (cumulativePumpRunTime > 0) {
          activePumpCycle = true;
        }
      }
      else if (isWellWaterLow) {
        Serial.println(F("Well Water Low"));
        // Make sure we maintain the active pump cycle flag during well water issues
        if (cumulativePumpRunTime > 0) {
          activePumpCycle = true;
        }
      }
      else if (tankVolume >= Config::TANK_MAX_LEVEL) {
        Serial.println(F("Tank Full"));
        // Reset cumulative time when the pump cycle completes naturally (tank full)
        cumulativePumpRunTime = 0;
        activePumpCycle = false;
      }
      else if (cumulativePumpRunTime >= Config::MAX_PUMP_RUN_TIME_MS) {
        // More detailed output for maximum run time condition
        Serial.print(F("Max Run Time Exceeded - "));
        Serial.print(F("Total runtime: "));
        Serial.print(cumulativePumpRunTime / 1000.0);
        Serial.print(F(" seconds ("));
        Serial.print(cumulativePumpRunTime / 60000.0);
        Serial.println(F(" minutes)"));
        // Reset cumulative time when max run time is reached
        cumulativePumpRunTime = 0;
        activePumpCycle = false;
      }
      else Serial.println(F("Unknown"));
      // Log cumulative run time
      Serial.print(F("Cumulative pump run time (s): "));
      Serial.println(cumulativePumpRunTime / 1000.0);
    }
    pumpRequestTime = millis();
  }
  // Handle pump on conditions
  void handlePumpOn(float tankVolume, bool postDripPumpActivation, bool preDripPumpActivation, 
                   bool isPeakOrDripTime, bool isMainsOff, bool isWellWaterLow, 
                   const String &pumpActivationReason, const DateTime &now) {
    // Check if we should turn the pump ON
    bool shouldActivatePump = false;
    String activationReason = "";
    // Check pre-drip activation conditions first (highest priority)
    if (preDripPumpActivation && !isPeakOrDripTime) {
      shouldActivatePump = true;
      activationReason = pumpActivationReason;
      // This is a new pump cycle
      if (!activePumpCycle) {
        cumulativePumpRunTime = 0;
        activePumpCycle = true;
      }
    }
    // Check post-drip refill condition second (next priority)
    else if (postDripPumpActivation && !isPeakOrDripTime) {
      shouldActivatePump = true;
      activationReason = pumpActivationReason;
      // This is a new pump cycle
      if (!activePumpCycle) {
        cumulativePumpRunTime = 0;
        activePumpCycle = true;
      }
    }
    // Normal case - tank level is low
    else if (tankVolume < Config::TANK_MIN_LEVEL) {
      shouldActivatePump = true;
      activationReason = "Tank Low Level";
      // If this is a new pump cycle (not a restart from failure), reset cumulative time
      if (!activePumpCycle) {
        cumulativePumpRunTime = 0;
        activePumpCycle = true;
      }
    }
    // Special case - pump was stopped due to mains failure but now it's restored
    else if (pumpStoppedDueToMains && !isMainsOff && !isPeakOrDripTime) {
      shouldActivatePump = true;
      activationReason = "Mains Power Restored";
      // Don't reset the flag yet - only reset after the pump actually starts
    }
    // Special case - pump was stopped due to well water low but now it's available
    else if (pumpStoppedDueToWellWater && !isWellWaterLow && !isPeakOrDripTime) {
      shouldActivatePump = true;
      activationReason = "Well Water Restored";
      // Don't reset the flag yet - only reset after the pump actually starts
    }
    // Activate pump if conditions are met
    if (!pumpRunning && shouldActivatePump) {
      if (TimeUtils::getTimeDifference(pumpRequestTime, millis()) >= Config::PUMP_ON_DELAY_MS) {
        digitalWrite(Pins::PUMP_RELAY, LOW);
        pumpOnTime = millis();
        initialTankVolume = tankVolume;
        pumpRunning = true;
        char timeStr[20];
        TimeUtils::getTimeString(timeStr, now);
        Serial.print(F("["));
        Serial.print(timeStr);
        Serial.print(F("] PUMP: ON - Reason: "));
        Serial.print(activationReason);
        // Set the post-drip flags if this was a post-drip activation
        if (activationReason == "Post-Morning Drip Refill") {
          postMorningDripPumpStarted = true;
        } else if (activationReason == "Post-Evening Drip Refill") {
          postEveningDripPumpStarted = true;
        }
        // Reset the flags now that the pump has actually started
        if (activationReason == "Mains Power Restored") {
          pumpStoppedDueToMains = false;
        } else if (activationReason == "Well Water Restored") {
          pumpStoppedDueToWellWater = false;
        }
        // Log cumulative run time at restart
        Serial.print(F(" | Resuming from cumulative time (s): "));
        Serial.println(cumulativePumpRunTime / 1000.0);
      }
    }
    // Handle flag maintenance
    maintainPumpFlags(isPeakOrDripTime, isMainsOff, isWellWaterLow, shouldActivatePump);
  }
  // Maintain pump control flags based on current conditions
  void maintainPumpFlags(bool isPeakOrDripTime, bool isMainsOff, bool isWellWaterLow, bool shouldActivatePump) {
    // If we're in a time or condition where we can't run the pump, // but we could later, keep the flags set
    if (isPeakOrDripTime) {
      // Keep the flags intact if we're just in peak/drip time // Don't reset any flags during restricted time periods
    } else if (!pumpRunning) {
      // Only consider clearing flags when the pump is not running // If we have an active pump cycle, we should preserve flags until // the cycle completes or the pump starts
      if (!activePumpCycle) {
        // If there's no active pump cycle, and both mains and well water // are available, then we can clear the flags
        if (!isMainsOff && !isWellWaterLow) {
          pumpStoppedDueToMains = false;
          pumpStoppedDueToWellWater = false;
        }
      } else {
        // If we have an active pump cycle, we only clear flags if // the pump should be running but isn't within the delay period
        if (shouldActivatePump && TimeUtils::getTimeDifference(pumpRequestTime, millis()) < Config::PUMP_ON_DELAY_MS) {
          // We're in the delay period before pump activation // Keep the flags since we'll check again soon
        } else if (!shouldActivatePump) {
          // We have an active cycle but no reason to activate pump // (not due to failures, because we checked those above)
          pumpStoppedDueToMains = false;
          pumpStoppedDueToWellWater = false;
        }
      }
    }
  }
  // Log detailed information about peak or drip time shutoff
  void logPeakOrDripTimeShutoff(const DateTime &now) {
    // More detailed output for peak/drip hours restriction
    DateTime dt = now;
    unsigned int currentTime = TimeUtils::timeToHHMM(dt);
    int currentHour, currentMinute, startHour, startMinute, endHour, endMinute;
    TimeUtils::extractHoursAndMinutes(currentTime, currentHour, currentMinute);
    if (TimeUtils::isTimeBetween(Config::PEAK_TIME_START, Config::PEAK_TIME_END, dt)) {
      TimeUtils::extractHoursAndMinutes(Config::PEAK_TIME_START, startHour, startMinute);
      TimeUtils::extractHoursAndMinutes(Config::PEAK_TIME_END, endHour, endMinute);
      Serial.print(F("Peak Hours Restriction - Current time: "));
      Serial.print(currentHour);
      Serial.print(F(":"));
      if (currentMinute < 10) Serial.print(F("0"));
      Serial.print(currentMinute);
      Serial.print(F(" (Peak period: "));
      Serial.print(startHour);
      Serial.print(F(":"));
      if (startMinute < 10) Serial.print(F("0"));
      Serial.print(startMinute);
      Serial.print(F("-"));
      Serial.print(endHour);
      Serial.print(F(":"));
      if (endMinute < 10) Serial.print(F("0"));
      Serial.print(endMinute);
      Serial.println(F(")"));
    } 
    else if (TimeUtils::isTimeBetween(Config::DRIP_MORNING_START, Config::DRIP_MORNING_END, dt)) {
      TimeUtils::extractHoursAndMinutes(Config::DRIP_MORNING_START, startHour, startMinute);
      TimeUtils::extractHoursAndMinutes(Config::DRIP_MORNING_END, endHour, endMinute);
      Serial.print(F("Morning Drip Period - Current time: "));
      Serial.print(currentHour);
      Serial.print(F(":"));
      if (currentMinute < 10) Serial.print(F("0"));
      Serial.print(currentMinute);
      Serial.print(F(" (Drip period: "));
      Serial.print(startHour);
      Serial.print(F(":"));
      if (startMinute < 10) Serial.print(F("0"));
      Serial.print(startMinute);
      Serial.print(F("-"));
      Serial.print(endHour);
      Serial.print(F(":"));
      if (endMinute < 10) Serial.print(F("0"));
      Serial.print(endMinute);
      Serial.println(F(")"));
    }
    else if (TimeUtils::isTimeBetween(Config::DRIP_EVENING_START, Config::DRIP_EVENING_END, dt)) {
      TimeUtils::extractHoursAndMinutes(Config::DRIP_EVENING_START, startHour, startMinute);
      TimeUtils::extractHoursAndMinutes(Config::DRIP_EVENING_END, endHour, endMinute);
      Serial.print(F("Evening Drip Period - Current time: "));
      Serial.print(currentHour);
      Serial.print(F(":"));
      if (currentMinute < 10) Serial.print(F("0"));
      Serial.print(currentMinute);
      Serial.print(F(" (Drip period: "));
      Serial.print(startHour);
      Serial.print(F(":"));
      if (startMinute < 10) Serial.print(F("0"));
      Serial.print(startMinute);
      Serial.print(F("-"));
      Serial.print(endHour);
      Serial.print(F(":"));
      if (endMinute < 10) Serial.print(F("0"));
      Serial.print(endMinute);
      Serial.println(F(")"));
    }
  }
  // Check for pump state changes
  void checkPumpEvents(const DateTime &now) {
    bool currentState = pumpRunning;
    
    if (currentState != prevPumpState) {
      prevPumpState = currentState;
      // We don't need to log state changes here because they're already logged in handlePumpOn/Off
    }
  }
  // Get pump status
  bool isPumpRunning() const { return pumpRunning; }
  // Get cumulative run time
  unsigned long getCumulativePumpRunTime() const { return cumulativePumpRunTime; }
  // Calculate current run time
  unsigned long getCurrentRunTime() const {
    if (!pumpRunning) return 0;
    return TimeUtils::getTimeDifference(pumpOnTime, millis());
  }
  // Calculate pump rate if running
  float getPumpRate(float currentTankVolume) const {
    if (!pumpRunning) return 0.0;
    float currentRunTime = getCurrentRunTime() / 1000.0; // Convert to seconds
    if (currentRunTime <= 0.0) return 0.0;
    return (currentTankVolume - initialTankVolume) / currentRunTime;
  }
};
//***********************
// Display Manager Class (Modified)
//***********************
class DisplayManager {
private:
  unsigned long lastLCDUpdateTime = 0;
  unsigned long lastSerialUpdateTime = 0;
  unsigned long lastSystemStatusTime = 0;  // Track last system status message time
  // Previous values for LCD optimization
  int prevHour = -1;
  int prevMinute = -1;
  float prevTemp = -99.0;
  bool prevPumpState = false;
  float prevTankVolume = -1.0;
  bool prevWellLowState = false;
  bool prevMainsState = false;
  bool prevDrip1State = false;
  bool prevDrip2State = false;
  String prevMoisture1Status = "";
  String prevMoisture2Status = "";
  // Flag to track if LCD layout has been initialized
  bool lcdInitialized = false;

public:
  DisplayManager() {}
  void init() {
    lcd.init();
    lcd.backlight();
    initializeLCDLayout();
  }
  // Initialize the static parts of the LCD layout
  void initializeLCDLayout() {
    lcd.clear();
    // Set up static labels
    lcd.setCursor(0, 0);
    lcd.print(F("Time:      Tmp:    C"));
    lcd.setCursor(0, 1);
    lcd.print(F("Pump:    Tank:     L"));
    lcd.setCursor(0, 2);
    lcd.print(F("Well:    Mains:"));
    lcd.setCursor(0, 3);
    lcd.print(F("D1:      D2:"));
    lcdInitialized = true;
  }
  // Update only the variable parts of the LCD display
  void updateLCD(float tankVolume, const SensorManager &sensors, const PumpManager &pumpManager) {
    unsigned long now = millis();
    if (TimeUtils::getTimeDifference(lastLCDUpdateTime, now) >= Config::LCD_UPDATE_INTERVAL_MS) {
      DateTime dt = rtc.now();
      // If LCD hasn't been initialized, do it now
      if (!lcdInitialized) {
        initializeLCDLayout();
      }
      // Update time (only if changed)
      if (dt.hour() != prevHour || dt.minute() != prevMinute) {
        lcd.setCursor(5, 0);
        lcd.print(dt.hour());
        lcd.print(F(":"));
        if (dt.minute() < 10)
          lcd.print(F("0"));
        lcd.print(dt.minute());
        prevHour = dt.hour();
        prevMinute = dt.minute();
      }
      // Update temperature (only if changed)
      float currentTemp = sensors.getTemperature();
      if (abs(currentTemp - prevTemp) >= 0.2) { // Update if changed by 0.2C or more
        lcd.setCursor(15, 0);
        lcd.print(currentTemp, 1);
        lcd.print(F(""));  // Clear any extra digits
        prevTemp = currentTemp;
      }
      // Update pump status (only if changed)
      bool pumpRunning = pumpManager.isPumpRunning();
      if (pumpRunning != prevPumpState) {
        lcd.setCursor(5, 1);
        lcd.print(pumpRunning ? F("ON ") : F("OFF"));
        prevPumpState = pumpRunning;
      }
      // Update tank level (only if changed significantly)
      if (abs(tankVolume - prevTankVolume) >= 1.0) { // Update if changed by 1L or more
        lcd.setCursor(15, 1);
        lcd.print(int(tankVolume));
        lcd.print(F(" ")); // Clear any extra digits
        prevTankVolume = tankVolume;
      }
      // Update well status (only if changed)
      bool wellLow = sensors.isWellWaterLow();
      if (wellLow != prevWellLowState) {
        lcd.setCursor(5, 2);
        lcd.print(wellLow ? F("LOW") : F("OK "));
        prevWellLowState = wellLow;
      }
      // Update mains status (only if changed)
      bool mainsOff = sensors.isMainsOff();
      if (mainsOff != prevMainsState) {
        lcd.setCursor(15, 2);
        lcd.print(mainsOff ? F("Off") : F("On "));
        prevMainsState = mainsOff;
      }
      // Update drip valve 1 status
      bool drip1Active = digitalRead(Pins::DRIP_VALVE1) == LOW;
      String moisture1 = sensors.getMoisture1Status().substring(0,1);
      if (drip1Active != prevDrip1State || moisture1 != prevMoisture1Status) {
        lcd.setCursor(3, 3);
        lcd.print(drip1Active ? F("ON:") : F("OFF:"));
        lcd.print(moisture1);
        prevDrip1State = drip1Active;
        prevMoisture1Status = moisture1;
      }
      // Update drip valve 2 status
      bool drip2Active = digitalRead(Pins::DRIP_VALVE2) == LOW;
      String moisture2 = sensors.getMoisture2Status().substring(0,1);
      if (drip2Active != prevDrip2State || moisture2 != prevMoisture2Status) {
        lcd.setCursor(12, 3);
        lcd.print(drip2Active ? F("ON:") : F("OFF:"));
        lcd.print(moisture2);
        prevDrip2State = drip2Active;
        prevMoisture2Status = moisture2;
      }
      lastLCDUpdateTime = now;
    }
  }
  // Modified Serial output to show logs only during events
  void updateSerial(float tankVolume, const SensorManager &sensors, const PumpManager &pumpManager, 
                   const DripValve &dripValve1, const DripValve &dripValve2) {
    unsigned long now = millis();
    if (TimeUtils::getTimeDifference(lastSerialUpdateTime, now) >= Config::SERIAL_UPDATE_INTERVAL_MS) {
      bool pumpRunning = pumpManager.isPumpRunning();
      bool drip1Active = digitalRead(Pins::DRIP_VALVE1) == LOW; // Active LOW
      bool drip2Active = digitalRead(Pins::DRIP_VALVE2) == LOW; // Active LOW
      bool hasActivity = pumpRunning || drip1Active || drip2Active;
      // If pump is running, show pump details
      if (pumpRunning) {
        float currentRunTime = pumpManager.getCurrentRunTime() / 1000.0;
        float totalRunTime = (pumpManager.getCumulativePumpRunTime() + pumpManager.getCurrentRunTime()) / 1000.0;
        float pumpRate = pumpManager.getPumpRate(tankVolume);
        float current = sensors.readPumpCurrent();
        float pumpLoad = current * Config::MAINS_VOLTAGE;
        Serial.print(F("PUMP EVENT | Run(s): "));
        Serial.print(currentRunTime);
        Serial.print(F(" | Total(s): "));
        Serial.print(totalRunTime);
        Serial.print(F(" | Rate(L/s): "));
        Serial.print(pumpRate);
        Serial.print(F(" | Current(A): "));
        Serial.print(current);
        Serial.print(F(" | Load(W): "));
        Serial.print(pumpLoad);
        Serial.print(F(" | Tank: "));
        Serial.print(tankVolume);
        Serial.println(F(" L"));
      } 
      // If any drip valve is active, show drip-related information in a single line
      else if (drip1Active || drip2Active) {
        Serial.print(F("DRIP EVENT | Tank: "));
        Serial.print(tankVolume);
        Serial.print(F(" L | "));
        if (drip1Active) {
          Serial.print(F("Drip1: "));
          Serial.print(sensors.getFlowRate1());
          Serial.print(F("L/m, Vol: "));
          Serial.print(dripValve1.getCumulativeVolume());
          Serial.print(F("L, "));
          Serial.print(sensors.getMoisture1Status());
          Serial.print(F(" | "));
        }
        if (drip2Active) {
          Serial.print(F("Drip2: "));
          Serial.print(sensors.getFlowRate2());
          Serial.print(F("L/m, Vol: "));
          Serial.print(dripValve2.getCumulativeVolume());
          Serial.print(F("L, "));
          Serial.print(sensors.getMoisture2Status());
        }
        Serial.println();
      }
      // If no activity, show system status only once every 15 seconds
      else {
        // Only show status if 15 seconds have passed since last status message
        if (TimeUtils::getTimeDifference(lastSystemStatusTime, now) >= 10000) {
          Serial.print(F("SYSTEM STATUS | Temp: "));
          Serial.print(sensors.getTemperature(), 1);
          Serial.print(F("C | Tank: "));
          Serial.print(tankVolume);
          Serial.print(F("L | Well: "));
          Serial.print(sensors.isWellWaterLow() ? F("LOW") : F("OK"));
          Serial.print(F(" | Mains: "));
          Serial.print(sensors.isMainsOff() ? F("OFF") : F("ON"));
          Serial.print(F(" | RAM: "));
          Serial.print(sensors.getFreeRam());
          Serial.println(F(" bytes"));
          // Update the timestamp
          lastSystemStatusTime = now;
        }
      }
      lastSerialUpdateTime = now;
    }
  }
};

//***********************
// SMS Manager Class
//***********************
class SMSManager {
private:
  // Phone number to send alerts to
  const char* phoneNumber;
  
  // Track last sent time for each alert type to implement cooldown
  unsigned long lastAlertTimes[15] = {0};
  
  // Cooldown periods for different alert types (in milliseconds)
  const unsigned long CRITICAL_COOLDOWN_MS = 900000;  // 15 minutes for critical alerts
  const unsigned long STANDARD_COOLDOWN_MS = 3600000; // 1 hour for standard alerts
  const unsigned long INFO_COOLDOWN_MS = 14400000;    // 4 hours for information alerts
  
  // Track previous states to detect changes
  bool prevMainsOff = false;
  bool prevWellWaterLow = false;
  bool prevPumpRunning = false;
  bool prevDripValve1State = false;
  bool prevDripValve2State = false;
  String prevMoisture1Status = "";
  String prevMoisture2Status = "";
  
  // Buffer to store SMS messages
  char smsBuffer[160];
  
  // GSM module status flag
  bool gsmReady = false;

public:
  // Alert type enum
  enum AlertType {
    POWER_FAILURE,
    POWER_RESTORED,
    WELL_WATER_LOW,
    WELL_WATER_RESTORED,
    TANK_LOW,
    TANK_CRITICAL,
    PUMP_MAX_RUNTIME,
    DRIP_ABNORMAL_FLOW,
    DRIP_NO_FLOW,
    DRIP_PERIOD_START,
    DRIP_PERIOD_END,
    SOIL_DRY,
    SOIL_WET,
    TANK_FULL,
    SYSTEM_STATUS
  };
  
  SMSManager(const char* number = "+919188773337") : phoneNumber(number) {}
  
  void init() {
    // Initialize the GSM module using Serial1 on the Arduino Mega
    Serial1.begin(9600);
    
    // Clear any garbage in the buffer
    while (Serial1.available()) {
      Serial1.read();
    }
    
    // Wait for GSM module to initialize - longer delay
    delay(2000);
    
    char timeStr[20];
    TimeUtils::getTimeString(timeStr, rtc.now());
    Serial.print(F("["));
    Serial.print(timeStr);
    Serial.println(F("] Initializing GSM module..."));
    
    // Reset the GSM module
    Serial1.println("AT+CFUN=1,1");
    delay(10000); // Longer delay after reset
    
    // Test AT command
    Serial.println(F("Testing AT command..."));
    if (sendATCommand("AT", "OK", 1000)) {
      Serial.println(F("GSM responding to AT command"));
    } else {
      Serial.println(F("WARNING: GSM not responding to AT"));
    }
    
    // Set SMS text mode
    Serial.println(F("Setting SMS text mode..."));
    if (sendATCommand("AT+CMGF=1", "OK", 1000)) {
      Serial.println(F("SMS text mode set"));
    } else {
      Serial.println(F("WARNING: Failed to set SMS text mode"));
    }
    
    // Check network registration
    Serial.println(F("Checking network registration..."));
    delay(5000); // Wait for network registration
    bool networkReady = false;
    
    // Try up to 5 times to connect to the network
    for (int i = 0; i < 5; i++) {
      Serial1.println("AT+CREG?");
      delay(1000);
      String regStatus = readResponse(1000);
      
      // +CREG: 0,1 or +CREG: 0,5 means registered
      if (regStatus.indexOf("+CREG: 0,1") > -1 || regStatus.indexOf("+CREG: 0,5") > -1) {
        networkReady = true;
        Serial.println(F("Network registered!"));
        break;
      }
      
      Serial.print(F("Waiting for network registration... Attempt "));
      Serial.println(i+1);
      delay(5000);
    }
    
    // Check signal quality
    Serial.println(F("Checking signal strength..."));
    Serial1.println("AT+CSQ");
    delay(1000);
    String signal = readResponse(1000);
    Serial.print(F("Signal strength: "));
    Serial.println(signal);
    
    // Set gsmReady flag based on all checks
    gsmReady = networkReady;
    
    if (gsmReady) {
      Serial.println(F("GSM module initialized and ready"));
      
      // Send startup message
      sprintf(smsBuffer, "[%s] Drip Irrigation System: Startup complete. GSM alerts activated.", timeStr);
      if (sendSMS(smsBuffer)) {
        Serial.println(F("Startup SMS sent successfully"));
      } else {
        Serial.println(F("WARNING: Failed to send startup SMS"));
      }
    } else {
      Serial.println(F("WARNING: GSM module not ready, check SIM card and signal"));
    }
  }
  
  bool setPhoneNumber(const char* number) {
    phoneNumber = number;
    return true;
  }
  
  void sendAlert(AlertType type, const String &message) {
    if (!gsmReady) {
      Serial.println(F("WARNING: Cannot send alert, GSM module not ready"));
      return;
    }
    
    unsigned long currentTime = millis();
    unsigned long cooldownPeriod;
    
    // Determine cooldown period based on alert type
    switch (type) {
      case POWER_FAILURE:
      case POWER_RESTORED:
      case WELL_WATER_LOW:
      case WELL_WATER_RESTORED:
      case TANK_CRITICAL:
      case PUMP_MAX_RUNTIME:
      case DRIP_ABNORMAL_FLOW:
      case DRIP_NO_FLOW:
        cooldownPeriod = CRITICAL_COOLDOWN_MS;
        break;
        
      case TANK_LOW:
      case DRIP_PERIOD_START:
      case DRIP_PERIOD_END:
      case SOIL_DRY:
      case SOIL_WET:
      case TANK_FULL:
        cooldownPeriod = STANDARD_COOLDOWN_MS;
        break;
        
      case SYSTEM_STATUS:
      default:
        cooldownPeriod = INFO_COOLDOWN_MS;
        break;
    }
    
    // Check cooldown period before sending
    if (TimeUtils::getTimeDifference(lastAlertTimes[type], currentTime) >= cooldownPeriod) {
      if (sendSMS(message)) {
        lastAlertTimes[type] = currentTime;
        
        char timeStr[20];
        TimeUtils::getTimeString(timeStr, rtc.now());
        Serial.print(F("["));
        Serial.print(timeStr);
        Serial.print(F("] SMS ALERT SENT: "));
        Serial.println(message);
      }
    } else {
      // Alert is still in cooldown period
      unsigned long remainingCooldown = cooldownPeriod - TimeUtils::getTimeDifference(lastAlertTimes[type], currentTime);
      char timeStr[20];
      TimeUtils::getTimeString(timeStr, rtc.now());
      Serial.print(F("["));
      Serial.print(timeStr);
      Serial.print(F("] SMS ALERT COOLDOWN: "));
      Serial.print(remainingCooldown / 60000); // Convert to minutes
      Serial.println(F(" minutes remaining"));
    }
  }
  
  bool sendATCommand(const char* command, const char* expectedResponse, unsigned long timeout) {
    // Clear buffer first
    while (Serial1.available()) {
      Serial1.read();
    }
    
    Serial1.println(command);
    
    return waitForResponse(expectedResponse, timeout);
  }
  
  String readResponse(unsigned long timeout) {
    String response = "";
    unsigned long start = millis();
    
    while (TimeUtils::getTimeDifference(start, millis()) < timeout) {
      if (Serial1.available()) {
        char c = Serial1.read();
        response += c;
        
        // Small delay to allow buffer to fill
        delay(10);
      }
    }
    
    return response;
  }
  
  bool sendSMS(const String &message) {
    if (!gsmReady) {
      Serial.println(F("WARNING: Cannot send SMS, GSM module not ready"));
      return false;
    }
    
    char timeStr[20];
    TimeUtils::getTimeString(timeStr, rtc.now());
    
    // Clear buffer first
    while (Serial1.available()) {
      Serial1.read();
    }
    
    // Command to set SMS recipient
    Serial.print(F("["));
    Serial.print(timeStr);
    Serial.print(F("] Sending SMS to "));
    Serial.println(phoneNumber);
    
    Serial1.print("AT+CMGS=\"");
    Serial1.print(phoneNumber);
    Serial1.println("\"");
    
    // Wait for > prompt (critical step)
    delay(1000);
    bool promptReceived = false;
    unsigned long startTime = millis();
    
    while (TimeUtils::getTimeDifference(startTime, millis()) < 5000) {
      if (Serial1.available()) {
        char c = Serial1.read();
        if (c == '>') {
          promptReceived = true;
          break;
        }
      }
    }
    
    if (!promptReceived) {
      Serial.println(F("WARNING: SMS prompt '>' not received"));
      return false;
    }
    
    // Send message content
    Serial.print(F("SMS content: "));
    Serial.println(message);
    Serial1.print(message);
    delay(500);
    
    // End SMS with Ctrl+Z (ASCII 26)
    Serial1.write(26);
    delay(1000);
    
    // Wait for OK response indicating message was sent
    bool sent = waitForResponse("OK", 20000);
    
    if (sent) {
      Serial.print(F("["));
      Serial.print(timeStr);
      Serial.println(F("] SMS sent successfully"));
    } else {
      Serial.print(F("["));
      Serial.print(timeStr);
      Serial.println(F("] WARNING: SMS sending failed or timed out"));
    }
    
    return sent;
  }
  
  bool waitForResponse(const char* expected, unsigned long timeout) {
    unsigned long start = millis();
    String response = "";
    
    while (TimeUtils::getTimeDifference(start, millis()) < timeout) {
      if (Serial1.available()) {
        char c = Serial1.read();
        response += c;
        
        // Check if the expected response is in what we've received so far
        if (response.indexOf(expected) != -1) {
          return true;
        }
      }
    }
    
    // Timeout occurred - print what we did receive for debugging
    if (response.length() > 0) {
      Serial.print(F("Response received: "));
      Serial.println(response);
    } else {
      Serial.println(F("No response received within timeout"));
    }
    
    return false;
  }
  
  // Update and check for alert conditions
  void checkAlerts(float tankVolume, const SensorManager &sensors, const PumpManager &pumpManager, 
                  const DripValve &dripValve1, const DripValve &dripValve2) {
    
    DateTime now = rtc.now();
    char timeStr[20];
    TimeUtils::getTimeString(timeStr, now);
    
    // Check mains power status
    bool currentMainsOff = sensors.isMainsOff();
    if (currentMainsOff != prevMainsOff) {
      if (currentMainsOff) {
        sprintf(smsBuffer, "[%s] ALERT: Power failure detected. System on backup power.", timeStr);
        sendAlert(POWER_FAILURE, smsBuffer);
      } else {
        sprintf(smsBuffer, "[%s] INFO: Power restored. System resumed normal operation.", timeStr);
        sendAlert(POWER_RESTORED, smsBuffer);
      }
      prevMainsOff = currentMainsOff;
    }
    
    // Check well water level
    bool currentWellWaterLow = sensors.isWellWaterLow();
    if (currentWellWaterLow != prevWellWaterLow) {
      if (currentWellWaterLow) {
        sprintf(smsBuffer, "[%s] ALERT: Well water level critically low. Pump operation suspended.", timeStr);
        sendAlert(WELL_WATER_LOW, smsBuffer);
      } else {
        sprintf(smsBuffer, "[%s] INFO: Well water level restored to normal.", timeStr);
        sendAlert(WELL_WATER_RESTORED, smsBuffer);
      }
      prevWellWaterLow = currentWellWaterLow;
    }
    
    // Check tank level
    if (tankVolume <= Config::TANK_MIN_LEVEL) {
      sprintf(smsBuffer, "[%s] ALERT: Tank level critically low (%.1f L). Below minimum threshold of %.1f L.", 
              timeStr, tankVolume, (float)Config::TANK_MIN_LEVEL);
      sendAlert(TANK_CRITICAL, smsBuffer);
    } else if (tankVolume <= Config::DRIP_ACTIVATION_THRESHOLD) {
      sprintf(smsBuffer, "[%s] WARNING: Tank level low (%.1f L). Below drip activation threshold of %.1f L.", 
              timeStr, tankVolume, Config::DRIP_ACTIVATION_THRESHOLD);
      sendAlert(TANK_LOW, smsBuffer);
    } else if (tankVolume >= Config::TANK_MAX_LEVEL) {
      sprintf(smsBuffer, "[%s] INFO: Tank filled to maximum level (%.1f L).", timeStr, tankVolume);
      sendAlert(TANK_FULL, smsBuffer);
    }
    
    // Check pump status
    bool currentPumpRunning = pumpManager.isPumpRunning();
    if (currentPumpRunning != prevPumpRunning) {
      if (!currentPumpRunning && prevPumpRunning) {
        // Pump just stopped
        unsigned long runTime = pumpManager.getCumulativePumpRunTime();
        
        if (runTime >= Config::MAX_PUMP_RUN_TIME_MS) {
          sprintf(smsBuffer, "[%s] ALERT: Pump maximum run time exceeded (%.1f min). Check system for leaks or failures.", 
                  timeStr, runTime / 60000.0);
          sendAlert(PUMP_MAX_RUNTIME, smsBuffer);
        }
      }
      prevPumpRunning = currentPumpRunning;
    }
    
    // Check drip valve 1 status
    bool currentDripValve1State = (digitalRead(Pins::DRIP_VALVE1) == LOW);  // LOW = Active
    if (currentDripValve1State != prevDripValve1State) {
      if (currentDripValve1State) {
        // Valve 1 just opened
        sprintf(smsBuffer, "[%s] INFO: Drip valve 1 activated. Soil moisture: %s", 
                timeStr, sensors.getMoisture1Status().c_str());
        sendAlert(DRIP_PERIOD_START, smsBuffer);
      } else if (prevDripValve1State) {
        // Valve 1 just closed
        sprintf(smsBuffer, "[%s] INFO: Drip valve 1 deactivated. Volume delivered: %.2f L", 
                timeStr, dripValve1.getCumulativeVolume());
        sendAlert(DRIP_PERIOD_END, smsBuffer);
        
        // Check if valve was deactivated due to abnormal condition
        if (dripValve1.isAbnormal()) {
          sprintf(smsBuffer, "[%s] ALERT: Drip valve 1 abnormal shutdown. Reason: %s", 
                  timeStr, dripValve1.getShutoffReason().c_str());
          sendAlert(DRIP_ABNORMAL_FLOW, smsBuffer);
        }
      }
      prevDripValve1State = currentDripValve1State;
    }
    
    // Check drip valve 2 status
    bool currentDripValve2State = (digitalRead(Pins::DRIP_VALVE2) == LOW);  // LOW = Active
    if (currentDripValve2State != prevDripValve2State) {
      if (currentDripValve2State) {
        // Valve 2 just opened
        sprintf(smsBuffer, "[%s] INFO: Drip valve 2 activated. Soil moisture: %s", 
                timeStr, sensors.getMoisture2Status().c_str());
        sendAlert(DRIP_PERIOD_START, smsBuffer);
      } else if (prevDripValve2State) {
        // Valve 2 just closed
        sprintf(smsBuffer, "[%s] INFO: Drip valve 2 deactivated. Volume delivered: %.2f L", 
                timeStr, dripValve2.getCumulativeVolume());
        sendAlert(DRIP_PERIOD_END, smsBuffer);
        
        // Check if valve was deactivated due to abnormal condition
        if (dripValve2.isAbnormal()) {
          sprintf(smsBuffer, "[%s] ALERT: Drip valve 2 abnormal shutdown. Reason: %s", 
                  timeStr, dripValve2.getShutoffReason().c_str());
          sendAlert(DRIP_ABNORMAL_FLOW, smsBuffer);
        }
      }
      prevDripValve2State = currentDripValve2State;
    }
    
    // Check soil moisture status changes for critical conditions
    String currentMoisture1Status = sensors.getMoisture1Status();
    if (currentMoisture1Status != prevMoisture1Status) {
      if (currentMoisture1Status == "DRY") {
        sprintf(smsBuffer, "[%s] WARNING: Zone 1 soil moisture critically low (DRY).", timeStr);
        sendAlert(SOIL_DRY, smsBuffer);
      } else if (currentMoisture1Status == "WET") {
        sprintf(smsBuffer, "[%s] INFO: Zone 1 soil moisture adequate (WET).", timeStr);
        sendAlert(SOIL_WET, smsBuffer);
      }
      prevMoisture1Status = currentMoisture1Status;
    }
    
    String currentMoisture2Status = sensors.getMoisture2Status();
    if (currentMoisture2Status != prevMoisture2Status) {
      if (currentMoisture2Status == "DRY") {
        sprintf(smsBuffer, "[%s] WARNING: Zone 2 soil moisture critically low (DRY).", timeStr);
        sendAlert(SOIL_DRY, smsBuffer);
      } else if (currentMoisture2Status == "WET") {
        sprintf(smsBuffer, "[%s] INFO: Zone 2 soil moisture adequate (WET).", timeStr);
        sendAlert(SOIL_WET, smsBuffer);
      }
      prevMoisture2Status = currentMoisture2Status;
    }
    
    // Send daily system status (once per day at noon)
    if (now.hour() == 12 && now.minute() == 0 && now.second() < 10) {
      sprintf(smsBuffer, "[%s] DAILY STATUS: Tank: %.1f L, Well: %s, Soil1: %s, Soil2: %s, Temp: %.1f C, Power: %s", 
              timeStr, tankVolume, 
              sensors.isWellWaterLow() ? "LOW" : "OK",
              sensors.getMoisture1Status().c_str(),
              sensors.getMoisture2Status().c_str(),
              sensors.getTemperature(),
              sensors.isMainsOff() ? "BACKUP" : "MAINS");
      sendAlert(SYSTEM_STATUS, smsBuffer);
    }
  }
};


//***********************
// Helper Function to update drip relay
//***********************
void updateDripRelay() {
  bool drip1Active = digitalRead(Pins::DRIP_VALVE1) == LOW; // Active LOW
  bool drip2Active = digitalRead(Pins::DRIP_VALVE2) == LOW; // Active LOW
  if (drip1Active || drip2Active) {
    // If either drip valve is active, enable the drip relay (LOW)
    digitalWrite(Pins::DRIP_RELAY, LOW);
  } else {
    // If both drip valves are inactive, disable the drip relay (HIGH)
    digitalWrite(Pins::DRIP_RELAY, HIGH);
  }
}
//***********************
// Interrupt Service Routines
//***********************
void flowSensor1_ISR() { 
  FlowSensorState::flowCount1++;
}
void flowSensor2_ISR() { 
  FlowSensorState::flowCount2++;
}
//***********************
// Global instances of manager classes
//***********************
SensorManager sensorManager;
PumpManager pumpManager;
DisplayManager displayManager;
DripValve dripValve1(Pins::DRIP_VALVE1);
DripValve dripValve2(Pins::DRIP_VALVE2);
SMSManager smsManager("+919188773337");  // Replace with your actual phone number

//***********************
// Arduino Setup and Loop
//***********************
void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  // Initialize RTC module
  if (!rtc.begin()) {
    Serial.println(F("DS3231 RTC not found!"));
    while (1);
  }
  if(rtc.lostPower()) {
        // this will adjust to the date and time at compilation
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  rtc.disable32K();
  rtc.writeSqwPinMode(DS3231_OFF);
  // Initialize pins for flow sensors and drip relay
  pinMode(Pins::FLOW_SENSOR1, INPUT_PULLUP);
  pinMode(Pins::FLOW_SENSOR2, INPUT_PULLUP);
  pinMode(Pins::DRIP_RELAY, OUTPUT);
  digitalWrite(Pins::DRIP_RELAY, HIGH);  // Initialize to HIGH (disabled)
  // Initialize managers
  sensorManager.init();
  pumpManager.init();
  displayManager.init();
  // Attach interrupt handlers
  attachInterrupt(digitalPinToInterrupt(Pins::FLOW_SENSOR1), flowSensor1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(Pins::FLOW_SENSOR2), flowSensor2_ISR, RISING);
  // Log system startup
  DateTime now = rtc.now();
  char timeStr[20];
  TimeUtils::getTimeString(timeStr, now);
  Serial.println(F("\n--- SYSTEM STARTUP ---"));
  Serial.print(F("["));
  Serial.print(timeStr);
  Serial.print(F("] TEMPERATURE: "));
  Serial.print(sensorManager.getTemperature(), 1);
  Serial.println(F(" C"));
  Serial.print(F("["));
  Serial.print(timeStr);
  Serial.print(F("] WELL WATER: "));
  Serial.println(sensorManager.isWellWaterLow() ? F("LOW") : F("NORMAL"));
  Serial.print(F("["));
  Serial.print(timeStr);
  Serial.print(F("] MAINS POWER: "));
  Serial.println(sensorManager.isMainsOff() ? F("OFF") : F("ON"));
  Serial.print(F("["));
  Serial.print(timeStr);
  Serial.print(F("] DRIP VALVE 1: "));
  Serial.println(digitalRead(Pins::DRIP_VALVE1) == LOW ? F("OPEN") : F("CLOSED"));
  Serial.print(F("["));
  Serial.print(timeStr);
  Serial.print(F("] DRIP VALVE 2: "));
  Serial.println(digitalRead(Pins::DRIP_VALVE2) == LOW ? F("OPEN") : F("CLOSED"));
  Serial.print(F("["));
  Serial.print(timeStr);
  Serial.print(F("] FREE RAM: "));
  Serial.print(sensorManager.getFreeRam());
  Serial.println(F(" bytes"));
  Serial.println(F("--- MONITORING STARTED ---\n"));
  
  Serial.println(F("Initializing GSM module..."));
  smsManager.init();
  // Enable watchdog timer (8-second timeout)
  wdt_enable(WDTO_8S);
}
void loop() {
  // Reset watchdog timer at each loop iteration
  wdt_reset();
  // Update all sensors
  sensorManager.updateAll();
  // Get tank volume
  float tankVolume = sensorManager.getTankVolume();
  // Check for drip period changes
  DateTime now = rtc.now();
  pumpManager.updateDripPeriodTracking(now);
  // Then update pump control with latest state information
  pumpManager.updatePumpControl(tankVolume, sensorManager, now);
  // Update drip valves
  dripValve1.update(sensorManager.getFlowRate1(), tankVolume, now, 
                  sensorManager.getSoilMoisture1(), sensorManager.getMoisture1Status());
  dripValve2.update(sensorManager.getFlowRate2(), tankVolume, now, 
                  sensorManager.getSoilMoisture2(), sensorManager.getMoisture2Status());
  // Update the drip relay based on drip valve states
  updateDripRelay();
  
  // Check and send SMS alerts
  smsManager.checkAlerts(tankVolume, sensorManager, pumpManager, dripValve1, dripValve2);
// Update displays
  displayManager.updateLCD(tankVolume, sensorManager, pumpManager);
  displayManager.updateSerial(tankVolume, sensorManager, pumpManager, dripValve1, dripValve2);
}