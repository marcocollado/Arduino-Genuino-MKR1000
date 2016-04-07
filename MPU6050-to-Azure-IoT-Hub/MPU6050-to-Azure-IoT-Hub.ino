// MPU6050-to-Azure-Iot-Hub.ino
// Arduino IDE code for sending Accel/Gyro data to AzureIotHub
// 01-Apr-2016 by Marco Collado
// at https://github.com/marcocollado/Arduino-Genuino-MKR1000/MPU6050-to-Azure-IoT-Hub

// uses **I2C Dev Lib** by Jeff Rowberg (2011), particularly mpu6050_raw
// at https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/Examples/MPU6050_raw

// and **MKR1000 Azure** IoT Hub Interface Using HTTP* by Mohan Palanisamy
// at http://mohanp.com/mkr1000-azure-iot-hub-how-to/

/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2011 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

#include <I2Cdev.h>
#include <MPU6050.h>
#include <SPI.h>
#include <WiFi101.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#endif

//*** MKR1000 variables ***//
#define MKR1000_LED 6 //onboard LED of MKR1000 is at pin 6

//*** MPU6050 variables ***//
#define ACCEL_SENSITIVITY 1 // 0 = +/-2g; 1=4g; 2=8g; 3=16g //  I use sensitivity 1 (+/-4g) because simulated quake measures easily exceeds +/- 2g
#define GYRO_SENSITIVITY 0 // 0 = +/-250 degrees per second; 1=500; 2=1000; 3=2000 // sensitivity 0 is good enough for seismo purposes
#define ACCEL_THRESH 200 // only accel readings beyond +/- 200 (~0.011g, "weak perceived shake" on the Mercalli scale) will be reported
#define GYRO_THRESH 55 // only gyro readings beyond +/- 55 will be reported

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high (if more than 1 sensor
int16_t ax, ay, az;
int16_t gx, gy, gz;

//*** calibration constants and variables ***//
#define CALIB_SAMPLE_SIZE 1000 // No. of samples for calibration
#define CALIB_STEP_SIZE 250 // interval between test offset values (for set...Offset())
#define CALIB_REDUCER 1000.00 // integer that shall prevent overflow of regression numbers
bool needs_calib = true; // determines whether calibration shall be done; becomes false after one reset

//*** Data processing constants and variables ***//
#define REPORTING_INTERVAL 500UL // millis between reports of (statistical; not raw) reading
#define MILLIS_OF_REST 10000UL // millis of "no quake" before sending data // 10 seconds
#define MAX_POST_ATTEMPTS 5 // maximum attempts to send data; otherwise dropped :(
long meas[4][6]; // array to hold 4 data (last reading, max, avg, min) for each of the 6 DOF sensors
int meas_count, i, j, row_index = 0; // counters and indexes
bool quake_detected = false, new_quake = true, post_success;
unsigned long lastMillis, last_quake, first_quake = 0 , millis_offset; // timers
String msgBuff = ""; // contains the data to be sent


//*** WiFi network config ***//
char ssid[] = "WiFi Network Name"; //  your network SSID (name)
char pass[] = "WiFi Network Password";    // your network password (use for WPA, or use as key for WEP)
int WiFi_status = WL_IDLE_STATUS;
WiFiSSLClient client;

//*** Azure IoT Hub config ***//
// for more info, visit http://mohanp.com/mkr1000-azure-iot-hub-how-to/
char hostname[] = "youriothubname.azure-devices.net";    // host name address for your Azure IoT Hub
char azureGET_Uri[] = "/devices/yourdevicename/messages/devicebound?api-version=2016-02-03"; // cloud-to-device URI
char azurePOST_Uri[] = "/devices/yourdevicename/messages/events?api-version=2016-02-03"; // device-to-cloud URI
char authSAS[] = "SharedAccessSignature sr=youriothubname.azure-devices.net%2fdevices%2fyourdevicename&sig=abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRST&se=1234567890";

//===================================================
void setup() {
  // Setup pins
  pinMode(MKR1000_LED, OUTPUT);

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  //setup WiFi and MPU
  while (!setupWIFI() || !setupMPU()) {
    Serial.println("Setup failed...");
    delay (10000);
  }
}

void loop() {

  // calibrate first
  // calibration determines the sensor offsets to make readings start in 0
  // some tutorials would calibrate the Z-axis accel (the axis perpendicular to earth) to read 1g, but I don't think its necessary for this seismo application
  if (needs_calib) {
    calibDevice();
    needs_calib = false;
    delay(5000);
  }
  // after calibration, reporting of readings can start
  else {

    // report statistical readings every interval (default is every 1/2 second; or 2 reports per second)
    if (millis() - lastMillis > REPORTING_INTERVAL) {

      // require "weak shake" before sending data
      // if the sensors readings indicate "no movement" as defined by the sensor, then do not report
      // by not reporting insignificant readings, we are saving data storage in the hub
      if (quake_detected) {
        digitalWrite(MKR1000_LED, HIGH); // light the LED to indicate detected movement
        //Serial.print(meas_count); Serial.print("\t");

        // if this is a new quake--meaning "movement" after some moments of rest, then prepare data headers
        // data headers are column names for the csv (comma-separated values) data
        if (new_quake) {
          msgBuff = ""; // empty the message buffer
          //then put the column names
          msgBuff = "row_index,millis_offset,meas_count,";
          msgBuff = msgBuff + "min_ax,avg_ax,max_ax,";
          msgBuff = msgBuff + "min_ay,avg_ay,max_ay,";
          msgBuff = msgBuff + "min_az,avg_az,max_az,";
          msgBuff = msgBuff + "min_gx,avg_gx,max_gx,";
          msgBuff = msgBuff + "min_gy,avg_gy,max_gy,";
          msgBuff = msgBuff + "min_gz,avg_gz,max_gz,";
          msgBuff = msgBuff + "accel_sensitivity,gyro_sensitivity";
          msgBuff = msgBuff + "\n"; // next line
          // prepend zero readings with added -1 second (for Power BI Adjustment)
          msgBuff = msgBuff + "0,-1000,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0," + ACCEL_SENSITIVITY + "," + GYRO_SENSITIVITY + "\n";
          //msgBuff = msgBuff + "0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0," + ACCEL_SENSITIVITY + "," + GYRO_SENSITIVITY + "\n";

          new_quake = false; // toggle new_quake to indicate that headers have been supplied
        }

        //make sure the millis() value has not reset back to 0 (occurs every ~50 days) or it will scramble the data
        if (millis() > first_quake) {
          row_index++;

          // compute the millis offset from the first quake (first row of data)
          // because this code does not implement it own clock
          // and would rely on the data enqueued date gived by Azure Iot Hub itself
          // the date given by Azure will just be offsetted by the millis_offset so to keep the row of data in order
          millis_offset = millis() - first_quake;

          // write all the data
          msgBuff = msgBuff + row_index + "," + millis_offset + "," + meas_count + ",";
          for (j = 0; j < 6 ; j++) {
            Serial.print("[");
            Serial.print(meas[1][j]); Serial.print(",");
            msgBuff = msgBuff + meas[1][j] +  ",";
            //Serial.print(meas[2][j]); Serial.print(",");
            Serial.print(meas[2][j] / (long) meas_count); Serial.print(",");
            msgBuff = msgBuff + (meas[2][j] / (long) meas_count) + ",";
            Serial.print(meas[3][j]); Serial.print("]");
            msgBuff =  msgBuff + meas[3][j] + ",";
            Serial.print("\t");
          }
          msgBuff =  msgBuff + ACCEL_SENSITIVITY + "," + GYRO_SENSITIVITY + "\n";
          Serial.println(" ");
        }
        // if the millis() has indeed reset, then force to send the contents of queue, if there is any
        else last_quake = 0;

      }
      // we will now send data
      // if (1) there is data to send and;
      // (2.1) it is safe to send (no movement for some time)
      // or (2.2) the message to send has got too large;
      // Azure IoT Hub can accept up to 256Kb of message; in this case, we cut at 160K bytes because MKR1000 memory is 256K only
      // one row of data can take up to 134 bytes, so 160000 bytes can store at least 1195 rows;
      // equiv. to least 10 minutes of continous large quake readings;
      if ((row_index > 0) && ((millis() - last_quake > MILLIS_OF_REST)  || (msgBuff.length() > 160000 ))) {
        // if data is sent due to "no movement", append zero readings plus 1 second (for Power BI adjustment)
        if (msgBuff.length() <= 250000) msgBuff = msgBuff + String (row_index + 1) + "," + String(millis() - first_quake - MILLIS_OF_REST + 1000) + ",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0," + ACCEL_SENSITIVITY + "," + GYRO_SENSITIVITY + "\n";
        Serial.println("Sending to AzureIotHub...");
        Serial.print("Msg: ");
        Serial.println(msgBuff);
        //send data to Azure IoT Hub
        i = 1;
        // try MAX_POST_ATTEMPTS to send data
        while (i <= MAX_POST_ATTEMPTS) {
          post_success = azureHttpPOST(msgBuff);
          if (post_success) break;
          i++;
        }

        // if failed to send, then drop the data
        // ideally it can be stored and resent until success
        if (!post_success) Serial.println("Posting failed; Data dropped...");

        // read the WiFi client until its empty
        // this prevents failed sending... but I can't explain why yet
        while (client.available()) {
          char c = client.read();
          //Serial.print(c);
        }

        // reset indicators
        row_index = 0;
        new_quake = true;

      }
      //reset measurements array
      for (i = 0; i < 4; i++) {
        for (j = 0; j < 6; j++) {
          meas[i][j] = 0;
        }
      }
      // reset counters and timers
      meas_count = 0;
      lastMillis = millis();
      quake_detected = false;
      digitalWrite(MKR1000_LED, LOW); // turn LED off
    }
    else // continue; that is, just read the sensors
    {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      meas_count++;

      // shoot the variables into the array
      meas[0][0] = ax;
      meas[0][1] = ay;
      meas[0][2] = az;
      meas[0][3] = gx;
      meas[0][4] = gy;
      meas[0][5] = gz;

      // calculate stats for accel readings
      for (j = 0; j < 3; j++) {
        if (meas[0][j] < meas[1][j]) meas[1][j] = meas[0][j]; // store the minimum value
        meas[2][j] = meas[2][j] + meas[0][j]; // add the measure to the sum
        if (meas[0][j] > meas[3][j]) meas[3][j] = meas[0][j]; // store the maximum value

        // if the accel readings indicate significant movement, then flag quake_detected
        if ((meas[1][j] < (0 - ACCEL_THRESH)) || (meas[3][j] > ACCEL_THRESH)) quake_detected = true;
      }
      // repeat for gyro
      for (j = 3; j < 6; j++) {
        if (meas[0][j] < meas[1][j]) meas[1][j] = meas[0][j];
        meas[2][j] = meas[2][j] + meas[0][j];
        if (meas[0][j] > meas[3][j]) meas[3][j] = meas[0][j];
        if ((meas[1][j] < (0 - GYRO_THRESH)) || (meas[3][j] > GYRO_THRESH)) quake_detected = true;
      }

      // if it was a new quake ("movement" after some time of "no movement")
      if (quake_detected) {
        if (new_quake) first_quake = millis(); //record the time of first quake
        last_quake = millis();
      }
      digitalWrite(MKR1000_LED, LOW);
      // blink LED to indicate activity
    }
  }
}

bool setupWIFI() {
  //check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println ("WiFi shield not found!");
    return false;
  }

  // attempt to connect to Wifi network:
  while (WiFi_status != WL_CONNECTED) {
    WiFi_status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }
  Serial.print("Connected to SSID: ");
  Serial.println(WiFi.SSID());
  Serial.println(" ");
  return true;
}

bool setupMPU() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println("Initializing I2C devices...");
  mpu.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  if (!mpu.testConnection()) {
    Serial.println("Cannot connect to MPU!");
    return false;
  } else {
    return true;
  }

}
void calibDevice() {

  double rgr[5][6]; //array to contain x, sum_of_x, sum_of_xy, sum_of_x^2, and b for each DOF
  int cc, sc, n; // counters or indexes

  unsigned long resetStart = millis();

  Serial.println("Calibrating the device...");

  // increase the AccelRange first to 8g
  Serial.print("  Temporarily increasing the AccelRange to 2... ");
  mpu.setFullScaleAccelRange(2);
  Serial.println((mpu.getFullScaleAccelRange() == 2) ? "done!" : "failed!");

  for (i = 1; i <= 100; i++) { // read 100 times first
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  }

  for (i = 0; i < 5; i++) { // reset regression array
    for (j = 0; j < 6; j++) {
      rgr[i][j] = 0;
    }
  }

  for (cc = -2; cc <= 2; cc++) { // calibrate using 5 points
    mpu.setXAccelOffset(cc * CALIB_STEP_SIZE); // -4106 -1969 1677
    mpu.setYAccelOffset(cc * CALIB_STEP_SIZE);
    mpu.setZAccelOffset(cc * CALIB_STEP_SIZE);
    mpu.setXGyroOffset(cc * CALIB_STEP_SIZE); // 67 -22 26
    mpu.setYGyroOffset(cc * CALIB_STEP_SIZE);
    mpu.setZGyroOffset(cc * CALIB_STEP_SIZE);

    for (sc = 1; sc <= CALIB_SAMPLE_SIZE; sc++) {
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

      rgr[0][0] = ax;
      rgr[0][1] = ay;
      rgr[0][2] = az;
      rgr[0][3] = gx;
      rgr[0][4] = gy;
      rgr[0][5] = gz;

      for (j = 0; j < 6; j++) {
        rgr[1][j] = rgr[1][j] + rgr [0][j]; // sum_x = sum_x + x
        //Serial.print(rgr[0][j]); Serial.print("\t");
        rgr[2][j] = rgr[2][j] + (rgr[0][j] * cc * CALIB_STEP_SIZE); // sum_xy = sum_xy + x*y
        //Serial.print(rgr[2][j]); Serial.print("\t");
        rgr[3][j] = rgr[3][j] + (rgr [0][j] * rgr [0][j]); // sum_x2 = sum_x2 + x*x
        //Serial.print(rgr[3][j]); Serial.println("\t");
      }
    }
  }

  Serial.print("  New offsets: ");
  for (j = 0; j < 6; j++) {
    n = CALIB_SAMPLE_SIZE * 5;
    // reduce digits size
    rgr[1][j] = rgr[1][j] / CALIB_REDUCER;
    rgr[2][j] = rgr[2][j] / (CALIB_REDUCER * CALIB_REDUCER);
    rgr[3][j] = rgr[3][j] / (CALIB_REDUCER * CALIB_REDUCER);
    rgr[4][j] = CALIB_REDUCER * ((rgr[1][j] * rgr[2][j]) / (( rgr[1][j] * rgr[1][j]) - (n * rgr[3][j]))); // (sum_x*sum_xy)/((sum_x)^2 - n*sumx2)
    Serial.print (rgr[4][j]); Serial.print("\t");

  }

  Serial.println (" ");
  mpu.setXAccelOffset((int) rgr[4][0]); // -4106 -1969 1677
  mpu.setYAccelOffset((int) rgr[4][1]);
  mpu.setZAccelOffset((int) rgr[4][2]);
  mpu.setXGyroOffset((int) rgr[4][3]); // 67 -22 26
  mpu.setYGyroOffset((int) rgr[4][4]);
  mpu.setZGyroOffset((int) rgr[4][5]);

  Serial.print("Calibration finished in ");
  Serial.print((int) (millis() - resetStart));
  Serial.println(" milliseconds.");
  Serial.println(" ");

  Serial.print("Decreasing the AccelRange back to ");
  Serial.print(ACCEL_SENSITIVITY);
  mpu.setFullScaleAccelRange(ACCEL_SENSITIVITY);
  Serial.println((mpu.getFullScaleAccelRange() == ACCEL_SENSITIVITY) ? "...done!" : "...failed!");
  Serial.println(" ");
}

bool azureHttpPOST(String content) {

  // close any connection before send a new request.
  // This will free the socket on the WiFi shield
  //

  // if there’s a successful connection:
  if (client.connected()) {
    digitalWrite(MKR1000_LED, HIGH);
    //make the POST request to the Azure IOT device feed uri
    client.print("POST "); //Do a POST
    client.print(azurePOST_Uri); // azurePOST_Uri
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println(hostname); //with hostname header
    client.print("Authorization: ");
    client.println(authSAS); //Authorization SAS token obtained from Azure IoT device explorer
    client.println("Connection: close");
    client.println("Content-Type: text/plain");
    client.print("Content-Length: ");
    client.println(content.length());
    client.println();
    client.println(content);
    client.println();
    digitalWrite(MKR1000_LED, LOW);
    return true;
  }
  else if (client.connectSSL(hostname, 443)) {
    return azureHttpPOST(content);
  }
  else {
    Serial.println("connection failed");
    client.stop();
    return false;
  }
}
