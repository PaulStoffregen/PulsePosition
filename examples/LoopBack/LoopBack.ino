#if defined(__IMXRT1062__)
  #include <PulsePositionIMXRT.h>       //Teensy 4.x and Micromod, __IMXRT1062__, ARDUINO_TEENSY40, ARDUINO_TEENSY41, ARDUINO_MICROMOD
#else
  #include <PulsePosition.h>            //Teensy 3.x and Teensy LC
#endif

// Simple loopback test: create 1 output to transmit
// test pulses, and 1 input to receive the pulses
PulsePositionOutput myOut;
PulsePositionInput myIn;

void setup() {
  myOut.begin(9);  // connect pins 9 and 10 together...
  myIn.begin(10);
  myOut.write(1, 600.03);
  myOut.write(2, 1500);
  myOut.write(3, 759.24);
  // slots 4 and 5 will default to 1500 us
  myOut.write(6, 1234.56);
}

int count=0;

void loop() {
  int i, num;

  // Every time new data arrives, simply print it
  // to the Arduino Serial Monitor.
  num = myIn.available();
  if (num > 0) {
    count = count + 1;
    Serial.print(count);
    Serial.print(" :  ");
    for (i=1; i <= num; i++) {
      float val = myIn.read(i);
      Serial.print(val);
      Serial.print("  ");
    }
    Serial.println();
  }
}
