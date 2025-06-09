#include "Xbus_SPI_custom.h"
#include <math.h>
#include <SPI.h>

Xbus::Xbus() {
  for (int i = 0; i < 3; ++i) {
    acc[i]     = NAN;
    freeAcc[i] = NAN;
    gyr[i]     = NAN;
    euler[i]   = NAN;
    mag[i]     = NAN;
    dv[i]   = NAN;
  }
  for (int i = 0; i < 2; ++i) {
    latlon[i] = NAN;
  }
  for (int i = 0; i < 4; ++i) {
    quat[i] = NAN;
    dq[i] = NAN;
  }
  for (int i = 0; i < 9; ++i) {
    rot[i] = NAN;
  }
}


bool Xbus::read(uint8_t nCS) {
  readPipeStatus(nCS);
  if (notificationSize) {                                   //New notification message available to be read
    readPipeNotif(nCS);
    parseNotification(datanotif);
  }
  if (measurementSize) {                                    //New measurement packet available to be read
    readPipeMeas(nCS);
    parseMTData2(datameas, measurementSize);
    return true;                                            //Return true if new measurements were read
  } else {
    return false;
  }
}



void Xbus::readPipeStatus(uint8_t nCS) {
  uint8_t buffer[] = {XSENS_STATUS_PIPE, 0xFF, 0xFF, 0xFF};
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(nCS, LOW);
  SPI.transfer(buffer, sizeof(buffer));
  for (int i = 0; i < 4; i++) {
    status[i] = SPI.transfer(0xFF);
  }
  digitalWrite(nCS, HIGH);
  SPI.endTransaction();

  notificationSize = (uint16_t)status[0] | ((uint16_t)status[1] << 8);
  measurementSize = (uint16_t)status[2] | ((uint16_t)status[3] << 8);
}


void Xbus::readPipeNotif(uint8_t nCS) {
  uint8_t buffer[] = {XSENS_NOTIF_PIPE, 0xFF, 0xFF, 0xFF};
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(nCS, LOW);
  SPI.transfer(buffer, sizeof(buffer));
  for (int i = 0; i < notificationSize; i++) {
    datanotif[i] = SPI.transfer(0xFF);
  }
  digitalWrite(nCS, HIGH);
  SPI.endTransaction();
}



void Xbus::readPipeMeas(uint8_t nCS) {
  uint8_t buffer[] = {XSENS_MEAS_PIPE, 0xFF, 0xFF, 0xFF};
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(nCS, LOW);
  SPI.transfer(buffer, sizeof(buffer));
  for (int i = 0; i < measurementSize; i++) {
    datameas[i] = SPI.transfer(0xFF);
  }
  digitalWrite(nCS, HIGH);
  SPI.endTransaction();
}



void Xbus::parseMTData2(uint8_t* data, uint8_t datalength) {
  if (datalength < 2)                                                           //Reached the end of the MTData2 message
    return;

  uint8_t XDI = data[0] ;                                                       //Xsens Data Identifier
  if (XDI == (uint8_t)MesID::MTDATA2) {                                         //Start of the MTData2 message
    uint8_t length = data[1];
    parseMTData2(data + 2, length);
  } else {
    uint8_t length = data[2];
    switch (((uint16_t)data[1] | ((uint16_t)data[0] << 8)) & (uint16_t)0xFFFF) { //Extract the 2-byte Xsens Data Identifier
      case (uint16_t)DataID::ACCELERATION:
        dataswapendian(data + 3, sizeof(float) * 3);
        memcpy(acc, data + 3, sizeof(float) * 3);
        break;
      case (uint16_t)DataID::FREEACCELERATION:
        dataswapendian(data + 3, sizeof(float) * 3);
        memcpy(freeAcc, data + 3, sizeof(float) * 3);
        break;
      case (uint16_t)DataID::RATEOFTURN:
        dataswapendian(data + 3, sizeof(float) * 3);
        memcpy(gyr, data + 3, sizeof(float) * 3);
        break;
      case (uint16_t)DataID::QUATERNION:
        dataswapendian(data + 3, sizeof(float) * 4);
        memcpy(quat, data + 3, sizeof(float) * 4);
        break;
      case (uint16_t)DataID::ROTATION:
        dataswapendian(data + 3, sizeof(float) * 9);
        memcpy(rot, data + 3, sizeof(float) * 9);
        break;
      case (uint16_t)DataID::EULERANGLES:
        dataswapendian(data + 3, sizeof(float) * 3);
        memcpy(euler, data + 3, sizeof(float) * 3);
        break;
      case (uint16_t)DataID::MAGNETOMETER:
        dataswapendian(data + 3, sizeof(float) * 3);
        memcpy(mag, data + 3, sizeof(float) * 3);
        break;
      case (uint16_t)DataID::LATLON:
        dataswapendian(data + 3, sizeof(float) * 2);
        memcpy(latlon, data + 3, sizeof(float) * 2);
        break;
      case (uint16_t)DataID::DELTAQ:
        dataswapendian(data + 3, sizeof(float) * 4);
        memcpy(dq, data + 3, sizeof(float) * 4);
        break;
      case (uint16_t)DataID::DELTAV:
        dataswapendian(data + 3, sizeof(float) * 3);
        memcpy(dv, data + 3, sizeof(float) * 3);
        break;
      default:
        break;
    }
    parseMTData2(data + length + 3, datalength - length - 3);                     //Move onto next data element within MTData2 packet
  }
}


void Xbus::parseNotification(uint8_t* notif) {                                           //Parse the most common notification messages
  uint8_t notifID = notif[0];
  switch (notifID) {
    case (uint8_t)MesID::WAKEUP: {
        Serial.println("Received WakeUp message.");
        break;
      }
    case (uint8_t)MesID::ERROR: {
        Serial.print("Received an error with code: "); Serial.println(notif[2]);
        break;
      }
    case (uint8_t)MesID::WARNING: {
        uint32_t warn = (uint32_t)notif[5] | ((uint32_t)notif[4] << 8);
        Serial.print("Received a warning with code: "); Serial.println(warn);
        break;
      }
    case (uint8_t)MesID::PRODUCTCODE: {
        Serial.print("Product code is: ");
        for (int i = 2; i < notificationSize - 1; i++) {
          Serial.print(char(notif[i]));
        }
        Serial.println();
        productCode = char(notif[6]);                                               //Store the product code (MTi-#) for later usage
        break;
      }
    case (uint8_t)MesID::FIRMWAREREV: {
        Serial.print("Firmware version is: ");
        Serial.print(notif[2]); Serial.print("."); Serial.print(notif[3]); Serial.print("."); Serial.println(notif[4]);
        break;
      }
    case (uint8_t)MesID::GOTOCONFIGACK: {
        //Serial.println("Received GoToConfigACK.");
        configState = true;
        break;
      }
    case (uint8_t)MesID::GOTOMEASUREMENTACK: {
        //Serial.println("Received GoToMeasurementACK.");
        configState = false;
        break;
      }
    case (uint8_t)MesID::OUTPUTCONFIGURATION: {
        //Serial.println("Received SetOutputConfigurationACK.");
        break;
      }
    case (uint8_t)MesID::REQFILTERPROFILEACK: {
        uint8_t profID = notif[3];
        const char* profName;
        switch (profID) {
          case 14: profName = "General_RTK";         break;
          case 15: profName = "GeneralNoBaro_RTK";   break;
          case 16: profName = "GeneralMag_RTK";      break;
          // fall-backs for the non-RTK ones if you need:
          case 1:  profName = "General";             break;
          case 2:  profName = "GeneralNoBaro";       break;
          case 3:  profName = "GeneralMag";          break;
          default: profName = "UnknownProfile";      break;
        }

        Serial.print("Active filter-profile ID: ");
        Serial.print(profID);
        Serial.print(" → ");
        Serial.println(profName);
        break;
      }
    case (uint8_t)MesID::BAUDRATEACK: {
        // notif[0] = 0x19
        // notif[1] = LEN (should be 0x01 for a request or 0x00 after a set)
        // notif[2] = baudCode (only if LEN == 1)

        uint8_t len = notif[1];
        if (len == 1) {
          uint8_t code = notif[2];
          uint32_t baud;
          switch (code) {
            case 0x09: baud = 9600;   break;
            case 0x07: baud = 19200;  break;
            case 0x05: baud = 38400;  break;
            case 0x04: baud = 57600;  break;
            case 0x02: baud = 115200; break;
            case 0x01: baud = 230400; break;
            case 0x00: baud = 460800; break;
            case 0x0A: baud = 921600; break;
            default:   baud = 0;      break;
          }
          if (baud) {
            Serial.print("Current baudrate: ");
            Serial.println(baud);
          } else {
            Serial.print("Unknown baud-code 0x");
            if (code < 0x10) Serial.print('0');
            Serial.println(code, HEX);
          }
        } else {
          Serial.println("SetBaudRate acknowledged.");
        }
        break;
      }
    default: {
        Serial.print("Received undefined notification: ");
        for (int i = 0; i < notificationSize - 1; i++) {
          Serial.print(notif[i], HEX); Serial.print(" ");
        }
        Serial.println();
        break;
      }
  }
}



void Xbus::dataswapendian(uint8_t* data, uint8_t length) {                          //Swap the endianness of the data such that the float value can be printed
  uint8_t cpy[length];                                                              //Create a copy of the data
  memcpy(cpy, data, length);                                                        //Create a copy of the data
  for (int i = 0; i < length / 4; i++) {
    for (int j = 0; j < 4; j++) {
      data[j + i * 4] = cpy[3 - j + i * 4];                                         //Within each 4-byte (32-bit) float, reverse the order of the individual bytes
    }
  }
}
