#include "MTi_SPI_custom.h"
#include <SPI.h>


bool MTi::detect(uint16_t timeout) {
  //Send goToConfig messages until goToConfigAck is received from the device
  Serial.println("Scanning for MTi.");                                                       //Clear the measurement/notification pipes (without printing) before configuring.
  long int starttime = millis();
  while ((millis() - starttime) < timeout) {
    goToConfig();
    delay(250);

    readMessages();
    if (deviceInConfigMode()) {
      Serial.println("Device detected.");
      return true;
    }
  }
  Serial.println("Failed to detect device.");
  return false;
}

void MTi::requestDeviceInfo() {
  //Request device info from the MTi using Xbus commands. Refer to the MT Low Level Communication Protocol Document for more information on the commands used here:
  //https://mtidocs.xsens.com/mt-low-level-communication-protocol-documentation

  if (!deviceInConfigMode()) {
    Serial.println("Cannot request device info. Device is not in Config Mode.");
    return;
  }
  readMessages();                                                                             //Clear the measurement/notification pipes before configuring.
  Serial.println("Requesting device info...");

  uint8_t reqProductCode[] = {0x1C, 0x00};                                                    //reqProductCode Xbus message
  sendMessage(reqProductCode, sizeof(reqProductCode));
  delay(1000);

  readMessages();

  uint8_t reqFWRev[] = {0x12, 0x00};                                                          //reqFWRev Xbus message
  sendMessage(reqFWRev, sizeof(reqFWRev));
  delay(1000);

  readMessages();
}

void MTi::getFilterProfile() {
  if (!deviceInConfigMode()) {
    Serial.println("Must be in Config Mode to query filter profile.");
    return;
  }

  uint8_t reqFilterProfile[] = { 0x64, 0x00 };
  sendMessage(reqFilterProfile, sizeof(reqFilterProfile));
  delay(1000);

  readMessages();
}

void MTi::getBaudRate() {
  if (!deviceInConfigMode()) {
    Serial.println("Must be in Config Mode to request baudrate.");
    return;
  }

  // MID = BAUDRATE (0x18), LEN = 0x00
  uint8_t getBaud[] = { static_cast<uint8_t>(Xbus::MesID::BAUDRATE), 0x00 };
  sendMessage(getBaud, sizeof(getBaud));
  delay(1000);

  readMessages();
}

void MTi::setBaudRate(uint32_t baudrate) {
  uint8_t baudCode = 0x02;                //Default baudrate is 115200
  switch (baudrate) {
    case 9600:   baudCode = 0x09; break;
    case 19200:  baudCode = 0x07; break;
    case 38400:  baudCode = 0x05; break;
    case 57600:  baudCode = 0x04; break;
    case 115200: baudCode = 0x02; break;
    case 230400: baudCode = 0x01; break;
    case 460800: baudCode = 0x00; break;
    case 921600: baudCode = 0x0A; break;
    default:
      Serial.println("Invalid Baudrate. Defaulting to 115200.");
      baudCode = 0x02;
      break;
  }

  if (!deviceInConfigMode()) {
    Serial.println("Must be in Config Mode to set baudrate.");
    return;
  }

  // MID = BAUDRATE (0x18), LEN = 0x01, payload = baudCode
  uint8_t setBaud[] = {
    static_cast<uint8_t>(Xbus::MesID::BAUDRATE),
    0x01,
    baudCode
  };
  sendMessage(setBaud, sizeof(setBaud));
  delay(1000);

  readMessages();
}

void MTi::configureOutputs() {
  //Configure the outputs of the MTi using Xbus commands. Refer to the MT Low Level Communication Protocol Document for more information on the commands used here:
  //https://mtidocs.xsens.com/mt-low-level-communication-protocol-documentation

  if (!deviceInConfigMode()) {
    Serial.println("Cannot configure device. Device is not in Config Mode.");
    return;
  }
  readMessages();                                                                             //Clear the measurement/notification pipes (without printing) before configuring.

  Serial.println("Configuring Acc, FreeAcc, RateOfTurn & Quaternion @100Hz");
  uint8_t outputConfig[] = {
    // SetOutputConfiguration (0xC0), Data packet size in Hex (N queries x 4 bytes) (0x14 = 20 bytes)
    0xC0, 0x14,
    // Acceleration       (XDI_Acceleration      = 0x4020) @100 Hz
    0x40, 0x20, 0x00, 0x64,
    // Free acceleration  (XDI_FreeAcceleration  = 0x4030) @100 Hz
    0x40, 0x30, 0x00, 0x64,
    // Angular rate       (XDI_RateOfTurn        = 0x8020) @100 Hz
    0x80, 0x20, 0x00, 0x64,
    // EulerAngles        (XDI_Euler             = 0x2030) @100 Hz
    0x20, 0x30, 0x00, 0x64,
    // Quaternion         (XDI_Quaternion        = 0x8030) @100 Hz
    0x20, 0x10, 0x00, 0x64
  };
  sendMessage(outputConfig, sizeof(outputConfig));
  Serial.println("Configured Acc, FreeAcc, RateOfTurn & Quaternion @100Hz");
  delay(1000);
  readMessages();
}



void MTi::configureOutputsAdv(const std::vector<OutputConfigEntry>& outputs) {
  if (!deviceInConfigMode()) {
    Serial.println("Cannot configure device. Device is not in Config Mode.");
    return;
  }
  readMessages();

  Serial.println("Configuring with advanced parameters.");

  // Calculate the length of the payload
  size_t payloadBytes = outputs.size() * 4; // 4 bytes for each entry [ID_MSB, ID_LSB, RATE_MSB, RATE_LSB]
  std::vector<uint8_t> configMsg;
  configMsg.reserve(2 + payloadBytes);

  configMsg.push_back(0xC0); // SetOutputConfiguration command
  configMsg.push_back(static_cast<uint8_t>(payloadBytes));

  for (const auto& entry : outputs) {
    // Append ID and rate in big-endian format
    configMsg.push_back(static_cast<uint8_t>((entry.id >> 8) & 0xFF));
    configMsg.push_back(static_cast<uint8_t>(entry.id & 0xFF));
    configMsg.push_back(static_cast<uint8_t>((entry.rateHz >> 8) & 0xFF));
    configMsg.push_back(static_cast<uint8_t>(entry.rateHz & 0xFF));
  }

  sendMessage(configMsg.data(), configMsg.size());
  Serial.println("Advanced configuration sent.");

  // Let the MTi process it
  delay(1000);
  readMessages();
}

void MTi::configureOutputs(const std::vector<uint16_t>& outputs) {
  if (!deviceInConfigMode()) {
    Serial.println("Cannot configure device. Device is not in Config Mode.");
    return;
  }
  readMessages();

  Serial.println("Configuring");

  // build the Xbus payload:
  //   0xC0 (SetOutputConfiguration), length,
  //   then for each output: [ID_MSB, ID_LSB, RATE_MSB, RATE_LSB]
  const uint16_t rateHz = 0x0064;    // 100 decimal == 0x0064
  size_t       payloadBytes = outputs.size() * 4;
  uint8_t      length       = static_cast<uint8_t>(payloadBytes);

  std::vector<uint8_t> msg;
  msg.reserve(2 + payloadBytes);
  msg.push_back(0xC0);
  msg.push_back(length);

  for (auto id : outputs) {

    // ID big-endian
    msg.push_back(static_cast<uint8_t>( (id >> 8) & 0xFF ));
    msg.push_back(static_cast<uint8_t>(  id        & 0xFF ));
    // rate = 100Hz big-endian
    msg.push_back(static_cast<uint8_t>( (rateHz >> 8) & 0xFF ));
    msg.push_back(static_cast<uint8_t>(  rateHz        & 0xFF ));
  }

  // send it off
  sendMessage(msg.data(), msg.size());
  Serial.println("Configuration sent.");

  // let the MTi process it
  delay(1000);
  readMessages();

}


void MTi::goToConfig() {
  Serial.println("Entering configuration mode.");
  uint8_t goToConfig[] = {0x30, 0x00};                                                        //goToConfig Xbus message
  sendMessage(goToConfig, sizeof(goToConfig));
}


void MTi::goToMeasurement() {
  Serial.println("Entering measurement mode.");
  uint8_t goToMeas[] = {0x10, 0x00};                                                          //goToMeasurement Xbus message
  sendMessage(goToMeas, sizeof(goToMeas));
}


void MTi::sendMessage(uint8_t *message, uint8_t numBytes) {
  //Compute the checksum for the Xbus message to be sent. See https://mtidocs.xsens.com/messages for details.
  uint8_t checksum = 0x01;
  for (int i = 0; i < numBytes; i++) {
    checksum -= message[i];
  }
  message[numBytes] = checksum;                                                                     //Add the checksum at the end of the Xbus message

  uint8_t buffer[] = {XSENS_CONTROL_PIPE, 0xFF, 0xFF, 0xFF};

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(nCS, LOW);
  SPI.transfer(buffer, sizeof(buffer));
  SPI.transfer(message, numBytes + 1);
  digitalWrite(nCS, HIGH);
  SPI.endTransaction();
}


void MTi::readMessages() {                                                                           //read new messages until measurement/notification pipes are empty
  while (digitalRead(drdy)) {
    xbus.read(nCS);
  }
}


void MTi::printData() {
  if (!isnan(getAcceleration()[0])) {                                                                       //Only true if this data has been received once before
    Serial.println("Acceleration [m/s^2]:");
    for (int i = 0 ; i < 3; ++i) {
      Serial.print(getAcceleration()[i]);                                                                   //Print the last read value
      Serial.print(" ");
    }
    Serial.println(" ");
  }

  if (!isnan(getRateOfTurn()[0])) {                                                                         //Only true if this data has been received once before
    Serial.println("Rate Of Turn [deg/s]:");
    for (int i = 0 ; i < 3; ++i) {
      Serial.print(getRateOfTurn()[i] * 180 / PI);                                                          //Print the last read value
      Serial.print(" ");
    }
    Serial.println(" ");
  }

  if (!isnan(getEulerAngles()[0])) {                                                                        //Only true if this data has been received once before
    Serial.println("Euler angles [deg]:");
    for (int i = 0 ; i < 3; ++i) {
      Serial.print(getEulerAngles()[i]);                                                                    //Print the last read value
      Serial.print(" ");
    }
    Serial.println(" ");
  }

  if (xbus.productCode == '7' | xbus.productCode == '8') {//MTi-7 or MTi-8 GNSS/INS
    Serial.println("Lat/Lon [deg]:");
    for (int i = 0 ; i < 2; ++i) {
      Serial.print(getLatLon()[i], 5);                                                                      //Print the last read value. Will be "NaN" as long as there is no GNSS fix
      Serial.print(" ");
    }
    Serial.println(" ");
  }
}
