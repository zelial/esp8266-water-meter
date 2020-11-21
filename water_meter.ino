#include <utils.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "configuration.h"

// we have two sensors, 1 for cold 1 for hot water, hence the array
// [0] == cold , [1] == hot 

// an average value we see on a sensor when moving from 
// dark to light area - we'll use it to recognize one full revolution = 1 liter
int threshold[2] = {40, 40};
// count of how many _consecutive_ values were under/over threshold
// use this to improve consistency / eliminate oscilation
unsigned long over[2] = {0, 0};
unsigned long under[2] = {0, 0};
// measure and upload min/max voltage we see between uploads, for debugging purposes
int voltage_min[2] = {9999, 9999};
int voltage_max[2] = {0, 0};
// actual voltage read from sensor
int voltage[2];
bool in_dark[2] = {false, false};
// how many litres has gone through the meter
// 1 liter == 1 revolution
unsigned int litres[2] = {0, 0};


// when was the last time we uploaded?
unsigned long last_upload = millis();
// when was the last time we counted a liter?
unsigned long last_liter[2] = {0, 0};
// the shortest time to count a liter
// used as a cap to remove false reading dues to fluctuations
unsigned long shortest_interval[2] = {9999999, 9999999};

// pin we use to power IR led (turn it of when not reading it to save power)
// both sensors are driven from the same pin
#define IR_LED_PIN D2

// we need to use multiplexor to read IR lead and battery voltage
// 3rd address pin is not set (fixed to GND) so we have 4 addresses available
#define MUX_A D4
#define MUX_B D3

//battery voltage
float battery = 0;

void setup() {
  init_serial();

  // initialize the multiplexor pins
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);

  wifi_sleep();
  
  // this is the pin we use to power the IR LED
  pinMode(IR_LED_PIN, OUTPUT);
  // initialize state
  for (int i = 0; i<=1 ; i++){
    voltage[i] = read_IR_voltage(i);
    if (voltage[i] > threshold[i]) {
      in_dark[i] = true;
    }  
  }
  
}

void loop() {
   bool should_upload = false;
   
   // get analog pin data
   for (int i = 0;i<=1; i++){
     voltage[i] = read_IR_voltage(i);  
    // store min/max for debugging
    if (voltage_min[i] > voltage[i]){
      voltage_min[i] = voltage[i];
    }
    if (voltage_max[i] < voltage[i]){
      voltage_max[i] = voltage[i];
    }
    //count consecutive voltages over/under threshold
    if (voltage[i] > threshold[i]){
      over[i] += 1;
      under[i] = 0;
    }else{
      under[i] += 1;
      over[i] = 0;
    }
  
    // transitioned dark -> light
    if (in_dark[i] == true && under[i] >= 3){
      // I've measured that with maximum throughput (all faucets opened)
      // there's about 2200 millis betee litres = anything faster
      // is an error and should be ignored
      if (millis() - last_liter[i] > 2000){
        litres[i] += 1;
        if (shortest_interval[i] > (millis() - last_liter[i])){
           shortest_interval[i] = (millis() - last_liter[i]);
        }
        last_liter[i] = millis();
        in_dark[i] = false;
      }else{
        under[i] = 0;
      }
    // transitioned light -> dark
    } else if (in_dark[i] == false && over[i] >= 3){
      in_dark[i] = true;
    }
  
    //update the threshold to be in the middle of min-max interval
    //expcted sane values
    if (voltage_min[i] > 0 && voltage_max[i] < 100){
      // reasonably wide interval
      if ((voltage_max[i] - voltage_min[i]) > 10){
        int new_threshold = (voltage_max[i] + voltage_min[i]) / 2;
        // threshold drifted enough to warrant an update
        if (abs(new_threshold - threshold[i]) >= 3){
          logln("updating threshold:" + String(threshold[i]) + " -> " + String(new_threshold));
          threshold[i] = new_threshold;
        }
      }
    }
    String name = "hot";
    if (i == 0){
      name = "cold";
    }
    logln(name + ": v=" + String(voltage[i]) +
      " (" + String(over[i]) +"/" + String(under[i]) +
      ")->" + String(in_dark[i]) + 
      " l=" + String(litres[i]) +
      " (" + String(voltage_min[i]) + "-" + String(voltage_max[i]) + ")");
   
    // do we have data to upload and has it been longer than refresh_rate_if_data since last upload?
    if (millis() > (last_liter[i] + (refresh_rate_if_data * 1000)) && litres[i] > 0){
      should_upload = true;
      // has it been too long since last upload (even if no data) = heartbeat?
    } else if (millis() > (last_upload + (refresh_rate * 1000))){
      should_upload = true;
    }
  }
  
  int battery_raw = read_battery_voltage();
  battery = (float)battery_raw * 0.00448;
  logln("battery: " + String(battery_raw) + " / " + String(battery));

   if (should_upload){
     upload();
   }
   
  // delay betwen measurements, have to be fast enough to sample the rotation when
  // the wheel is the fastest
  delay(100);
}

int read_IR_voltage(int sensor){
  // power IR LEDs
  digitalWrite(IR_LED_PIN, HIGH);
  delay(5);
  // set multiplexor address to select sensor
  // Option 0 /  pin 13 or Option 1 / pin 14
  if (sensor == 1){
    digitalWrite(MUX_A, LOW);
    digitalWrite(MUX_B, LOW);
  }else{
    digitalWrite(MUX_A, HIGH);
    digitalWrite(MUX_B, LOW);
  }
  int v = analogRead(A0);
  // disable IR LED 
  digitalWrite(IR_LED_PIN, LOW);
  // return the pin to LOW as it is also used for builtin led and 
  // we want that off to save power
  digitalWrite(MUX_A, HIGH);
  digitalWrite(MUX_B, HIGH);
  return v; 
}

int read_battery_voltage(){
  // set multiplexor address to Option 2 /  ping 15
  digitalWrite(MUX_A, LOW);
  digitalWrite(MUX_B, HIGH);
  int v = analogRead(A0);
  // return the pin to LOW as it is also used for builtin led and 
  // we want that off to save power
  digitalWrite(MUX_A, HIGH);
  return v;
}

void upload(){
  wifi_wakeup();
  // our usual reconnect doesn't work after sleep, need to do this one
  WiFi.begin(ssid, wifi_password);
  wifi_reconnect(ssid, wifi_password, ip_last_byte);

  //contact broker
  Broker b = Broker(broker_url);
  b.addProperty("voltage", String(battery));
  b.addProperty("water__type_cold", String(litres[0]));
  b.addProperty("water__type_hot", String(litres[1]));
  b.addProperty("refresh_rate", String(refresh_rate));
  b.addProperty("debug_analog_min__type_cold", String(voltage_min[0]));
  b.addProperty("debug_analog_min__type_hot", String(voltage_min[1]));
  b.addProperty("debug_analog_max__type_cold", String(voltage_max[0]));
  b.addProperty("debug_analog_max__type_hot", String(voltage_max[1]));
  b.addProperty("debug_analog_threshold__type_cold", String(threshold[0]));
  b.addProperty("debug_analog_threshold__type_hot", String(threshold[1]));
  b.addProperty("debug_shortest_interval__type_cold", String(shortest_interval[0]));
  b.addProperty("debug_shortest_interval__type_hot", String(shortest_interval[1]));
  logln("uploading after " + String(int((millis() - last_upload) / 1000)) + "s");
  logln(b.getUrl());
  // broker adds these to a permanent counter 
  // clear counter after successfull upload to not double count
  for (int i = 0; i<=1 ; i++){
    if (b.upload()) {
       litres[i] = 0;
       voltage_min[i] = 999;
       voltage_max[i] = 0;
       shortest_interval[i] = 999999;
    }
  }
  last_upload = millis();
  wifi_sleep(); 
}
