/*
  This program allows for 2 MAX485s to be used alongside the UART.
  It uses software serial, and allows for daisy-chaining multiple
  devices for sensor and data collection.

  The send and receive both return booleans to verify if things have been
  verified. You will pass a data array pointer into sendData and receiveData.
  Send will convert it into the appropriate packet structure and send it over.
  Receive will also be passed an array pointer, and if the data is valid it will
  fill that array with {array_length, data0, data1, data2....}
  You will get back a verified data stream, but you'll have to be mindeful that
  array[0] will be the length of the array (including itself). This was the easiest
  way I could think of to be able to send somewhat arbitrarily length data streams.

  You will want to tie each of the MAX485 RE(2) & DE(3) pins together, then
  connect those to pins on the Arduino to set SEND/RECEIVE modes. Check
  which pins are availibe for software serial for you Arduino board, and
  use pins that are fine for it for the TX(RO, 1) and RX(DI, 4).

  There uses a very basic packet structure of
  [START][DATA0][INVERSE0][DATA1][INVERSE1]...[END][END]
  Since it's sending 8N1 data, all data is sent with it's inverse
  and END is sent twice. The sender sends the data with checks, the
  receiver replies exactly the same back, and then the sender verifies
  and sends an ACK or NAK back accordingly. This was designed for small
  number of bytes being sent, so something more robust is suggested for
  larger than the current 32 bytes suggested. Tons of diagnostic info is
  being sent over the UART so the data transfer process can be watched.

  This is released under the Unlicense on 10/26/2015 by Jimmie Rodgers

  This is free and unencumbered software released into the public domain.

  Anyone is free to copy, modify, publish, use, compile, sell, or
  distribute this software, either in source code form or as a compiled
  binary, for any purpose, commercial or non-commercial, and by any
  means.

  In jurisdictions that recognize copyright laws, the author or authors
  of this software dedicate any and all copyright interest in the
  software to the public domain. We make this dedication for the benefit
  of the public at large and to the detriment of our heirs and
  successors. We intend this dedication to be an overt act of
  relinquishment in perpetuity of all present and future rights to this
  software under copyright law.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
  OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
  OTHER DEALINGS IN THE SOFTWARE.

  For more information, please refer to <http://unlicense.org/>
*/

#include <SoftwareSerial.h> // Used to expand the number of serial ports.
#define SEND_MODE HIGH    // Enables send
#define RECEIVE_MODE LOW  // Enables receive
#define ACK B00000110     // Acknowledgement
#define NAK B00010101     // Negative Acknowledgement
#define END B1110111      // End of the packet
#define START B01010101   // Start of the packet
#define MAXBYTES 32       // Max suggested data size
                          // Make this smaller if you don't need it.
                          // That will free up memory and time.
/*
  Common baud rates and the needed delay for 2 chars to pass.
  This is needed to make certain a serial buffer is fully clear.
  Anything more or less than these values and it throws off the timing.
  The formula is 2*(10000000/Baud), as it's in uS time scale.
  Baud    Delay
  9600    2083
  19200   1042
  38400   521
  57600   347
  115200  173
*/
#define BAUDRATE 57600 // Slower is generally better, but I get good results here.
unsigned int readWriteDelay = 200000 / (BAUDRATE / 100); // Explained above

// Room for the data, the inverse of the data, and the Start/end bytes.
const byte dataLength = (2 * MAXBYTES) + 3;
byte finalData[dataLength];  // Final data buffer.
unsigned int dataSent = 0;   // Used to track the verified sent packets
unsigned int dataFailed = 0; // Used to track the failed packets

#define PORT1 1        // Used to identify the port in code
#define PORT1_ENABLE 7 // Connected to pins 2&3 of the 1st MAX485
#define PORT1_TX 2     // Connected to pin1 of the 1st MAX485
#define PORT1_RX 4     // Connected to pin4 of the 1st MAX485
#define PORT2 2        // Used to identify the port in code
#define PORT2_ENABLE 8 // Connected to pins 2&3 of the 2nd MAX485
#define PORT2_TX 9     // Connected to pin1 of the 2nd MAX485
#define PORT2_RX 10    // Connected to pin4 of the 2nd MAX485
#define INPUT_PORT 1

// Starts the software serial ports
SoftwareSerial rs485Port1(PORT1_TX, PORT1_RX);
SoftwareSerial rs485Port2(PORT2_TX, PORT2_RX);

// Stuff to change
boolean master = false;      // master receives data only
boolean debug = false;       // enables an ecco test, overides master
boolean verboseMode = true;  // mostly prints the data streams to serial
int sendReceiveTimeout = (dataLength*3)*(100000/BAUDRATE); // timeout before send or receives gives up
int sendDelay = 500;         // Time between the slave sending data

void setup() {
  Serial.begin(BAUDRATE);
  Serial.println(F("Initializing Software Serial"));
  // Sets up the MAX485s in receive mode.
  rs485Port1.begin(BAUDRATE);
  rs485Port2.begin(BAUDRATE);
  pinMode(PORT1_ENABLE, OUTPUT);
  digitalWrite(PORT1_ENABLE, RECEIVE_MODE);
  pinMode(PORT2_ENABLE, OUTPUT);
  digitalWrite(PORT2_ENABLE, RECEIVE_MODE);
}

void loop() {
  if (debug) eccoTest(PORT1); // Loops, just sending data back and forth.
  else if (master) {
    // This just receives data from each port.
    while (!receiveData(finalData, dataLength, PORT1)) {
      // Sits here till there is some verified data
    }
    Serial.println(F("Verified Data: "));
    serialPrintBuffer(finalData, finalData[0]);
    /* Commented out unless needed
      while (!receiveData(finalData, dataLength, PORT2)) {
      // Sits here till there is some verified data
      }
      Serial.println(F("Verified Data: "));
      serialPrintBuffer(finalData, finalData[0]);
    */
    //Serial.flush(); // used to clear the serial port to keep it from drifting.
  }
  else if (!master) {
    // This will generate a data stream of ordered bytes. It will then increment
    // the data and number of bytes being sent up to MAXBYTES.
    Serial.println();
    Serial.println(F("Starting send loop"));
    for (int count = 0; count < MAXBYTES; count++) {
      Serial.println();
      Serial.print(F("Data: "));
      for (int i = 0; i < MAXBYTES; i++) {
        finalData[i] = count + i;
        Serial.print(finalData[i]);
        if (i == MAXBYTES - 1)Serial.println();
        else Serial.print(F(","));
      }
      byte qty = 1 + count % MAXBYTES;
      Serial.print(F("Number of bytes being sent: "));
      Serial.println(qty);
      Serial.print(F("Sending attempt: "));
      Serial.println(count);
      if (sendData(finalData, qty, PORT2)) {
        Serial.println(F("Data Sent"));
        dataSent++;
      }
      else {
        Serial.println(F("Send Failure"));
        dataFailed++;
      }
      /* Commented out unless needed.
        if (sendData(finalData, qty, PORT2)) {
        Serial.println(F("Data Sent"));
        dataSent++;
        }
        else {
        Serial.println(F("Send Failure"));
        dataFailed++;
        }
      */
      Serial.println(F("Ratio sent/failed"));
      Serial.print(dataSent);
      Serial.print(F("/"));
      Serial.println(dataFailed);
      Serial.flush();
      delay(sendDelay);
    }
  }
}

/*
  sendData is used to send data. The arguments are:
  (Array the data is in, number of bytes in the array, port to send on)
  You should use some kind of conditional to catch the boolean, like:
  while(!sendData(data, dataSize, PORT1){//stuff}
  That way it will keep trying till the data goes through and is verified.
*/
boolean sendData(const byte *data, int numBytes, byte port) {
  if (numBytes <= 0) return false;
  Serial.println(F("Starting sendData"));
  Serial.println(F("Constructing data"));
  byte numDataBytes = 3 + (2 * numBytes);
  byte tempData[numDataBytes];
  tempData[0] = START;
  tempData[numDataBytes - 1] = END;
  tempData[numDataBytes - 2] = END;
  // Creates the data/inverse data pairs.
  for (int i = 0; i < numBytes; i++) {
    tempData[(2 * i) + 1] = data[i];
    tempData[(2 * i) + 2] = byte(~data[i]);
  }
  Serial.println(F("Sending Data"));
  if (verboseMode) serialPrintBuffer(tempData, numDataBytes);
  sendBuffer(tempData, numDataBytes, port);
  long timeOut = millis();
  Serial.println(F("Receiving Verification Data"));
  while (timeOut + sendReceiveTimeout > millis()) {
    byte testData[numDataBytes];
    if (checkPortHasData(port)) {
      byte count = readBuffer(testData, numDataBytes, port);
      if (count > 0) {
        Serial.println(F("Data received: "));
        if (verboseMode) serialPrintBuffer(testData, count);
      }
      if (count > 0 && verifyData(tempData, testData, numDataBytes)) {
        sendByte(ACK, port);
        delayMicroseconds(readWriteDelay);
        return true;
      }
      else {
        sendByte(NAK, port);
        return false;
      }
    }
  }
  Serial.println(F("Verification timed out"));
  return false;
}

/*
  receiveData is used to receive data. The arguments are:
  (Pointer to the array you want data in, max expected length, port to receive from)
  Like sendData, you'll want to put this in some kind of conditional statement, like:
  if(receiveData(array, maxBytes, PORT1)){//stuff}
  This passes the pointer of the array into it and writes directly to the memory, as
  there is no way to return an array that I know of with an Arduino.
  data[0] will be the size of the array, including the size value.
*/
boolean receiveData(byte *data, byte maxDataLength, byte port) {
  if (checkPortHasData(port)) {
    Serial.println();
    byte tempData[maxDataLength];
    if (verboseMode) Serial.println(F("Retrieving data: "));
    byte count = readBuffer(tempData, maxDataLength, port);
    if (count > 0) {
      Serial.print(F("Received Count of "));
      Serial.println(count);
    }
    else return false;
    Serial.println(F("Data Recieved:"));
    if (verboseMode) {
      serialPrintBuffer(tempData, count);
    }
    Serial.println(F("Resending Data: "));
    if (verboseMode) {
      serialPrintBuffer(tempData, count);
    }
    sendBuffer(tempData, count, port);
    Serial.println(F("Sent buffer: "));
    serialPrintBuffer(tempData, count);
    delay(1);
    Serial.println(F("Awaiting response"));
    long timeout = millis();
    while (timeout + sendReceiveTimeout > millis()) {
      if (checkPortHasData(port)) {
        byte response = readByte(port);
        //Serial.println(response);
        if (response > 0) {
          Serial.print(F("Response: "));
          Serial.println(response);
          if (response == ACK) {
            Serial.println(F("Data Verified"));
            data[0] = (count - 2) / 2;
            for (int i = 0; i < count - 2; i++) data[i + 1] = tempData[(2 * i) + 1];
            //for(int i = 0; i < count; i++) data[i] = tempData[i];
            return true;
          }
          else if (response == NAK) {
            Serial.println(F("Data Failed Verification"));
            return false;
          }
          else return false;
        }
      }
    }
    Serial.println(F("Timeout/Failout"));
    return false;
  }
  else return false;
}

// Reads and returns the port's buffer till it ends, runs out of time,
// or runs out of space (maxDataLength). It's mostly for internal use.
byte readBuffer(byte *tempData, byte maxDataLength, byte port) {
  long timeOut = millis();
  byte count = 0;
  while (timeOut + sendReceiveTimeout > millis()) {
    if (checkPortHasData(port)) {
      while (count < maxDataLength) {
        boolean endMode = true;
        if (port == PORT1) {
          while (count < maxDataLength + 1 && endMode) {
            if ( rs485Port1.available()) {
              tempData[count++] = rs485Port1.read();
            }
            if ( count > 1 && tempData[count-2] == END && tempData[count-1] == END){
              endMode = false;
            }
          }
        }
        else if (port == PORT2) {
          while (count < maxDataLength + 1 && endMode) {
            if ( rs485Port2.available()) {
              tempData[count++] = rs485Port2.read();
            }
            if ( count > 1 && tempData[count-2] == END && tempData[count-1] == END){
              endMode = false;
            }
          }
        }
        if (verboseMode) {
          Serial.println(F("Received: "));
          serialPrintBuffer(tempData, count);
        }
        if (tempData[0] != START) {
          Serial.println(F("Invalid Data Start"));
          return 0;
        }
        //Serial.println(tempData[count - 1]);
        else if (tempData[count - 1] != END || tempData[count - 2] != END) {
          Serial.println(F("Invalid Data End"));
          Serial.print(tempData[count - 1]);
          Serial.print(F(","));
          Serial.println(tempData[count - 2]);
          return 0;
        }
        else {
          if (verboseMode) {
            Serial.print(F("Count: "));
            Serial.println(count);
          }
          return count;
        }
      }
    }
    else return 0;
  }
  return 0;
}

// Sends the array of numBytes on port. Mostly for internal use.
void sendBuffer(const byte *data, byte numBytes, byte port) {
  if (port == PORT1) {
    digitalWrite(PORT1_ENABLE, SEND_MODE);
    Serial.println(F("Sending Data on Port1"));
    for (int i = 0; i < numBytes; i++){
      rs485Port1.write(data[i]);
      delayMicroseconds(readWriteDelay/32);
    }
    delayMicroseconds(readWriteDelay);
    rs485Port1.flush();
    digitalWrite(PORT1_ENABLE, RECEIVE_MODE);
  }
  else if (port == PORT2) {
    digitalWrite(PORT2_ENABLE, SEND_MODE);
    Serial.println(F("Sending Data on Port2"));
    for (int i = 0; i < numBytes; i++){
      rs485Port2.write(data[i]);
      delayMicroseconds(readWriteDelay/32);
    }
    delayMicroseconds(readWriteDelay);
    rs485Port2.flush();
    digitalWrite(PORT2_ENABLE, RECEIVE_MODE);
  }
}

// Checks that two arrays follow the packet format and returns a true/false.
boolean verifyData(const byte *data1, const byte *data2, byte numDataBytes) {
  Serial.println(F("Starting Verification"));
  // Checks if the first byte is START in each array.
  if (data1[0] == START && data2[0] == START) Serial.println(F("Data Start PASS"));
  else {
    Serial.print(F("Start: "));
    Serial.print(data1[0]);
    Serial.print(F(","));
    Serial.println(data2[0]);
    Serial.println(F("Data Start FAILED"));
    return false;
  }
  // Checks if the last byte is END in each array.
  if (data1[numDataBytes - 1] == END  && data2[numDataBytes - 1] == END)
    Serial.println(F("Data End PASS"));
  else {
    Serial.print(F("End: "));
    Serial.print(data1[numDataBytes - 1]);
    Serial.print(F(","));
    Serial.println(data2[numDataBytes - 1]);
    Serial.println(F("Data End FAILED"));
    return false;
  }
  // Checks if each byte of data is matched with it's inverse.
  for (int i = 1; i < numDataBytes - 1; i += 2) {
    if (data1[i] & data1[i + 1] != 0 && data2[i] & data2[i + 1] != 0 && data1[i] != data2[i]) {
      Serial.print(data1[i] & data1[i + 1]);
      Serial.print(F(","));
      Serial.print(data2[i] & data2[i + 1]);
      Serial.print(F(","));
      Serial.println(data1[i] & data2[i]);
      return false;
    }
  }
  return true;
}

// Checks if a given port has data ready to recieve.
boolean checkPortHasData(byte port) {
  if (port == PORT1) {
    digitalWrite(PORT1_ENABLE, RECEIVE_MODE);
    rs485Port1.listen();
    if (rs485Port1.available()) return true;
  }
  else if (port == PORT2) {
    digitalWrite(PORT2_ENABLE, RECEIVE_MODE);
    rs485Port2.listen();
    if (rs485Port2.available()) return true;
  }
}

// Sends a byte across the port.
void sendByte(byte temp, byte port) {
  if (port == PORT1) {
    digitalWrite(PORT1_ENABLE, SEND_MODE);
    rs485Port1.write(temp);
    rs485Port1.flush();
    digitalWrite(PORT1_ENABLE, RECEIVE_MODE);
  }
  else if (port == PORT2) {
    digitalWrite(PORT2_ENABLE, SEND_MODE);
    rs485Port2.write(temp);
    rs485Port2.flush();
    digitalWrite(PORT2_ENABLE, RECEIVE_MODE);
  }
}

// Reads a single byte from the port.
byte readByte(byte port) {
  byte data = 0;
  if (port == PORT1) {
    rs485Port1.listen();
    if (rs485Port1.available())data = rs485Port1.read();
  }
  else if (port == PORT2) {
    rs485Port2.listen();
    if (rs485Port2.available())data = rs485Port2.read();
  }
  return data;
}

// Prints out an array of numBytes length, useful for testing.
void serialPrintBuffer(byte *data, byte numBytes) {
  for (int i = 0; i < numBytes; i++) {
    Serial.print(data[i]);
    if (i == numBytes - 1)Serial.println();
    else Serial.print(F(","));
  }
}

// Just put this in your loop to test if your physical connection works.
// It simply eccos a number back and forth, incrementing it along the way.
void eccoTest(byte port) {
  byte tempCount = 0;
  boolean sendMode = false;
  while (true) {
    if (checkPortHasData(port) & !sendMode) {
      Serial.print(F("Read:"));
      tempCount = readByte(port);
      Serial.println(tempCount);
      sendMode = true;
    }
    else if (sendMode) {
      Serial.print(F("Sent:"));
      sendByte(++tempCount, port);
      Serial.println(tempCount);
      sendMode = false;
    }
  }
}
