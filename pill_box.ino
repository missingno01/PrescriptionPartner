#include "BluetoothSerial.h"
#include <driver/rtc_io.h>
#include <ESP32Time.h>

#define BIN_1 25
#define BIN_2 26
#define SPKR 12
#define BAT 35

ESP32Time rtc;

/* setting PWM properties */
const int freq = 5000;
const int ledChannel_1 = 1;
const int ledChannel_2 = 2;
const int resolution = 8;
int MAX_PWM_VOLTAGE = 255;

int motor_output = 0;
float low_battery = 3.2;
float battery_level = 4.2;

int num_alarms = 3;
RTC_DATA_ATTR int alarm_time[] = {0, 0, 0};
RTC_DATA_ATTR bool alarm_enabled[] = {false, false, false};
RTC_DATA_ATTR bool mute = false;
RTC_DATA_ATTR bool initialized = false;
int current_time = 0;

int sleep_time = 0;
int dt_sleep = 60;
int dt_wake_up_before_alarm = 5;

BluetoothSerial SerialBT;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  if (!initialized) {
    rtc.setTime(0, 0, 0, 1, 1, 2021);  // 1st Jan 2021 00:00:00
    initialized = true;
  }
  SerialBT.begin("Pill Box");
  pinMode(SPKR, OUTPUT);
  pinMode(BAT, INPUT);
  rtc_gpio_pulldown_en(GPIO_NUM_27);
  rtc_gpio_pulldown_en(GPIO_NUM_33);
  /* configure LED PWM functionalitites */
  ledcSetup(ledChannel_1, freq, resolution);
  ledcSetup(ledChannel_2, freq, resolution);

  /* attach the channel to the GPIO to be controlled */
  ledcAttachPin(BIN_1, ledChannel_1);
  ledcAttachPin(BIN_2, ledChannel_2);

  current_time = rtc.getSecond() + 60 * rtc.getMinute() + 3600 * rtc.getHour(true);
  sleep_time = rolloverCorrection(current_time + dt_sleep);
}

void loop() {
  if (SerialBT.available()) {
    sleep_time = rolloverCorrection(current_time + dt_sleep);
    String reading = SerialBT.readStringUntil('\n');  // read from the Serial Monitor
    /* put your code here*/
    char state = reading.charAt(0);
    String value = reading.substring(1, reading.length());
    switch (state) {
      case 'o':
        {
          rtc_gpio_pullup_en(GPIO_NUM_33);
          commandPWM(-100);
          delay(1000);
          commandPWM(0);
          rtc_gpio_pulldown_en(GPIO_NUM_33);
          break;
        }
      case 'c':
        {
          rtc_gpio_pullup_en(GPIO_NUM_33);
          commandPWM(100);
          delay(1000);
          commandPWM(0);
          rtc_gpio_pulldown_en(GPIO_NUM_33);
          break;
        }
      case 'b':
        {
          SerialBT.println(battery_level);
          break;
        }
      case 's':
        {
          goSleep();
          break;
        }
      case 'm':
        {
          mute = !mute;
          break;
        }
      case 't':
        {
          int hour = (getValue(value, ':', 0)).toInt();
          int minute = (getValue(value, ':', 1)).toInt();
          int second = (getValue(value, ':', 2)).toInt();
          rtc.setTime(second, minute, hour, 1, 1, 2021);  // 1st Jan 2021 00:00:00
          //          Serial.print(rtc.getHour(true));   // (String) returns time with specified format
          //          Serial.print(':');
          //          Serial.print(rtc.getMinute());   // (String) returns time with specified format
          //          Serial.print(':');
          //          Serial.println(rtc.getSecond());   // (String) returns time with specified format
          break;
        }
      case 'u':
        {
          Serial.print(rtc.getHour(true));   // (String) returns time with specified format
          Serial.print(':');
          Serial.print(rtc.getMinute());   // (String) returns time with specified format
          Serial.print(':');
          Serial.println(rtc.getSecond());   // (String) returns time with specified format
          break;
        }
      case '1':
        {
          beep(100);
          if (value == "o") {
            alarm_enabled[0] = false;
          }
          else {
            int alarm_hour = (getValue(value, ':', 0)).toInt();
            int alarm_minute = (getValue(value, ':', 1)).toInt();
            alarm_time[0] = 60 * alarm_minute + 3600 * alarm_hour;
            alarm_enabled[0] = true;
          }
          break;
        }
      case '2':
        {
          beep(100);
          if (value == "o") {
            alarm_enabled[1] = false;
          }
          else {
            int alarm_hour = (getValue(value, ':', 0)).toInt();
            int alarm_minute = (getValue(value, ':', 1)).toInt();
            alarm_time[1] = 60 * alarm_minute + 3600 * alarm_hour;
            alarm_enabled[1] = true;
          }
          break;
        }
      case '3':
        {
          beep(100);
          if (value == "o") {
            alarm_enabled[2] = false;
          }
          else {
            int alarm_hour = (getValue(value, ':', 0)).toInt();
            int alarm_minute = (getValue(value, ':', 1)).toInt();
            alarm_time[2] = 60 * alarm_minute + 3600 * alarm_hour;
            alarm_enabled[2] = true;
          }
          break;
        }
    }
    sendFeedback();
    alarmFinish();
  }
  current_time = rtc.getSecond() + 60 * rtc.getMinute() + 3600 * rtc.getHour(true);
  battery_level = float(analogRead(BAT)) / 4095 * 2 * 3.3 * 1.1;
  checkLowBattery(battery_level);
  checkForAlarm();
  checkForSleep();
}

void checkLowBattery(float battery_level) {
  if (battery_level < low_battery) {
    goSleep();
  }
}

void goSleep() {
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 1);
  bool wake_up = false;
  for (int i = 0; i < num_alarms; i++) {
    if (alarm_enabled[i]) {
      wake_up = true;
    }
  }
  if (wake_up) {
    int min_time_to_wake_up = 3600 * 24;
    for (int i = 0; i < num_alarms; i++) {
      if (alarm_enabled[i]) {
        int time_to_wake_up = alarm_time[i] - current_time;
        if (time_to_wake_up < 0) {
          time_to_wake_up += 3600 * 24;
        }
        if (time_to_wake_up < min_time_to_wake_up) {
          min_time_to_wake_up = time_to_wake_up;
        }
      }
    }
    esp_sleep_enable_timer_wakeup(uint64_t(1000000) * uint64_t(max(min_time_to_wake_up - dt_wake_up_before_alarm, 0)));
    Serial.println(min_time_to_wake_up - dt_wake_up_before_alarm);
    Serial.println(alarm_time[0]);
    Serial.println(alarm_time[1]);
    Serial.println(alarm_time[2]);
  }
  beep(1000);
  esp_deep_sleep_start();
}

void commandPWM(int motor_output) {
  if (motor_output > 0) {
    ledcWrite(ledChannel_1, LOW);
    ledcWrite(ledChannel_2, motor_output);
  }
  else {
    ledcWrite(ledChannel_2, LOW);
    ledcWrite(ledChannel_1, -motor_output);
  }
}

void sendFeedback() {
  SerialBT.println(String(int(min(max((battery_level - low_battery) / (4.2 - low_battery), 0.0), 1.0) * 100)) + "," + String(mute));
  //  Serial.println(String(int(min(max((battery_level - low_battery) / (4.2 - low_battery), 0.0), 1.0) * 100)) + "," + String(mute));
}

void checkForAlarm() {
  for (int i = 0; i < num_alarms; i++) {
    if (current_time == alarm_time[i] && alarm_enabled[i]) {
      digitalWrite(SPKR, HIGH * !mute);
    }
  }
}

void checkForSleep() {
  if (current_time == sleep_time) {
    goSleep();
  }
}

void alarmFinish() {
  digitalWrite(SPKR, LOW);
}

int rolloverCorrection(int time_now) {
  if (time_now > 59 + 60 * 59 + 3600 * 23) {
    return time_now - (59 + 60 * 60 + 3600 * 23);
  }
  else {
    return time_now;
  }
}

void beep(int beep_time) {
  digitalWrite(SPKR, HIGH * !mute);
  delay(beep_time);
  digitalWrite(SPKR, LOW);
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
