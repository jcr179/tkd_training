#include <Wire.h>
#include <SPI.h>
#include <SparkFun_ADXL345.h>

const short int gled = 2;
const short int rled = 3;
const short int yled = 4;
const short int wled = 5;
const short int buzzer = 11;

const short int bdelay = 125; // delay for led blinks at reset
const short int tdelay = 2000; // delay for state transition
const short int rdelay = 2000; // indicates when to strike
const short int bzz = 127; // buzz tone for buzzer

boolean hit = false; // hit detect
const short int hitdelay = 100; // read values for this long to determine acceleration
long t0; // time now
long t1; // reaction time
long treact = 1000; // reaction time goal in ms
int kicknum = 1;

// in my basement, adxl at rest with pins going down into breadboard, z = -33
// when flipped over, z = 31
// 1G: z = -33, -1G: z = +31
// so a 2G difference is 64
// and a G difference is 32.
double gforce = 32;

int i = 0; // for loop iteration
double biggest; // to find biggest impact force

// wiring setup is the i2c configuration here:
// https://learn.sparkfun.com/tutorials/adxl345-hookup-guide

ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION
int ix,iy,iz; // initial values at start of loop   
int fx,fy,fz; // values read while waiting for hit

void setup() {
  pinMode(gled, OUTPUT);
  pinMode(rled, OUTPUT);
  pinMode(yled, OUTPUT);
  pinMode(wled, OUTPUT);
  pinMode(buzzer, OUTPUT);

  // adxl345
  Serial.begin(9600);                 // Start the serial terminal
  //Serial.println("SparkFun ADXL345 Accelerometer Hook Up Guide Example");
  //Serial.println();

  adxl.powerOn();                     // Power on the ADXL345
  Serial.println("[DEBUG] ADXL345 powered on");
  adxl.setRangeSetting(16);           // Give the range settings
                                      // Accepted values are 2g, 4g, 8g or 16g
                                      // Higher Values = Wider Measurement Range
                                      // Lower Values = Greater Sensitivity

  adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
                                      // Default: Set to 1
                                      // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 
   
  adxl.setActivityXYZ(1, 0, 0);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
 
  adxl.setInactivityXYZ(1, 0, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?

  adxl.setTapDetectionOnXYZ(0, 0, 1); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
 
  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  adxl.setTapThreshold(50);           // 62.5 mg per increment
  adxl.setTapDuration(15);            // 625 μs per increment
  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment
 
  // Set values for what is considered FREE FALL (0-255)
  adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment
 
  // Setting all interupts to take place on INT1 pin
  //adxl.setImportantInterruptMapping(1, 1, 1, 1, 1);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);" 
                                                        // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
                                                        // This library may have a problem using INT2 pin. Default to INT1 pin.
  
  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  adxl.InactivityINT(1);
  adxl.ActivityINT(1);
  adxl.FreeFallINT(1);
  adxl.doubleTapINT(1);
  adxl.singleTapINT(1);
  
//attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);   // Attach Interrupt
  Serial.println("[DEBUG] Setup complete");
  Serial.print("\nTarget reaction time: ");
  Serial.print(treact);
  Serial.println(" ms");
}

void loop() {
  // reset states
  hit = false;
  digitalWrite(gled, LOW);
  digitalWrite(rled, LOW);
  digitalWrite(yled, LOW);
  digitalWrite(wled, LOW);
  
  adxl.readAccel(&ix, &iy, &iz);         // Read the accelerometer values and store them in variables declared above x,y,z 
  digitalWrite(gled, HIGH);
  digitalWrite(buzzer, HIGH);
  delay(bdelay);
  digitalWrite(gled, LOW);
  digitalWrite(buzzer, LOW);
  delay(bdelay);
  digitalWrite(gled, HIGH);
  digitalWrite(buzzer, HIGH);
  delay(bdelay);
  digitalWrite(gled, LOW);
  digitalWrite(buzzer, LOW);
  delay(tdelay);

  delay(random(0,5)*1000); // delay 0 to 5 seconds after tdelay
  digitalWrite(rled, HIGH);
  digitalWrite(buzzer, HIGH); 
  t0 = millis(); // mark time now to measure reaction time
  
  while (!hit) {
    adxl.readAccel(&fx, &fy, &fz);         // Read the accelerometer values and store them in variables declared above x,y,z
    double disp = sqrt( pow(fx-ix,2.0) + pow(fy-iy, 2.0) + pow(fz-iz, 2.0));  // calculate displacement from rest
    if (disp > 10.0) {
      t1 = millis() - t0;
      biggest = disp;
      
      for (i = 0; i < hitdelay; i++) { // find biggest acceleration experienced in next [hitdelay] ms
        adxl.readAccel(&fx, &fy, &fz);
        disp = sqrt( pow(fx-ix,2.0) + pow(fy-iy, 2.0) + pow(fz-iz, 2.0));
        if (disp > biggest) {
          biggest = disp; 
        } 
      }
      
      Serial.print(kicknum++);
      Serial.print("\t");
      Serial.print(t1);
      Serial.print("ms");
      Serial.print("\t\t");
      Serial.print(biggest/gforce); // debug
      Serial.print("g\t");
      if (t1 < treact) {
        digitalWrite(wled, HIGH);
        Serial.println("X");  
      }
      else {
        Serial.println("-");
      }      
      hit = true;
      digitalWrite(yled, HIGH);
      digitalWrite(rled, LOW);

      delay(tdelay);
    }
  } // end hit detection loop
  



  // Output Results to Serial
  /*
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z); 
  */
  
  //ADXL_ISR();

}

/********************* ISR *********************/
/* Look for Interrupts and Triggered Action    */
void ADXL_ISR() {
  
  // getInterruptSource clears all triggered actions after returning value
  // Do not call again until you need to recheck for triggered actions
  byte interrupts = adxl.getInterruptSource();
  
  // Free Fall Detection
  if(adxl.triggered(interrupts, ADXL345_FREE_FALL)){
    Serial.println("*** FREE FALL ***");
    //add code here to do when free fall is sensed
  } 
  
  // Inactivity
  if(adxl.triggered(interrupts, ADXL345_INACTIVITY)){
    Serial.println("*** INACTIVITY ***");
     //add code here to do when inactivity is sensed
  }
  
  // Activity
  if(adxl.triggered(interrupts, ADXL345_ACTIVITY)){
    Serial.println("*** ACTIVITY ***"); 
     //add code here to do when activity is sensed
  }
  
  // Double Tap Detection
  if(adxl.triggered(interrupts, ADXL345_DOUBLE_TAP)){
    Serial.println("*** DOUBLE TAP ***");
     //add code here to do when a 2X tap is sensed
  }
  
  // Tap Detection
  if(adxl.triggered(interrupts, ADXL345_SINGLE_TAP)){
    Serial.println("*** TAP ***");
     //add code here to do when a tap is sensed
  }  
}
