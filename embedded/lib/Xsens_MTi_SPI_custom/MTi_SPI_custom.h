#ifndef MTI_SPI_CUSTOM_h
#define MTI_SPI_CUSTOM_h

#include <Arduino.h>
#include <vector>
#include "Xbus_SPI_custom.h"


struct OutputConfigEntry {
  uint16_t id;      // Output ID
  uint16_t rateHz;  // Update rate in Hz
};

class MTi {
public:
    MTi(uint8_t x, uint8_t y) : xbus() {
        nCS = x;
        drdy = y;
    }

    uint8_t drdy;
    uint8_t nCS;

    bool detect(uint16_t timeout);
    void requestDeviceInfo();
    void getFilterProfile();
    void getBaudRate();
    void setBaudRate(uint32_t baudrate);
    // Default configuration (100 Hz for common outputs)
    void configureOutputs();
    // Custom configuration: specify XDI outputs and rate (Hz)
    void configureOutputs(const std::vector<uint16_t>& outputs);

    void configureOutputs(const uint8_t& outputConfig);

    void configureOutputsAdv(const std::vector<OutputConfigEntry>& outputs);


    void goToConfig();
    void goToMeasurement();
    void sendMessage(uint8_t *message, uint8_t numBytes);
    void readMessages();
    void printData();

    // Accessors
    float* getAcceleration()       { return xbus.acc; }
    float* getFreeAcceleration()   { return xbus.freeAcc; }
    float* getRateOfTurn()         { return xbus.gyr; }
    float* getQuaternion()         { return xbus.quat; }
    float* getRotation()           { return xbus.rot; }
    float* getEulerAngles()        { return xbus.euler; }
    float* getMagnetometer()       { return xbus.mag; }
    float* getLatLon()             { return xbus.latlon; }
    float* getDeltaQ()             { return xbus.dq; }
    float* getDeltaV()             { return xbus.dv; }

private:
    Xbus xbus;

    bool deviceInConfigMode() {
        return xbus.configState;
    }
};

// XDI message identifiers for configureOutputs()
static constexpr uint16_t XDI_Acceleration      = 0x4020;
static constexpr uint16_t XDI_FreeAcceleration  = 0x4030;
static constexpr uint16_t XDI_RateOfTurn        = 0x8020;
static constexpr uint16_t XDI_Quaternion        = 0x2010;
static constexpr uint16_t XDI_Rotation          = 0x2020;
static constexpr uint16_t XDI_EulerAngles       = 0x2030;
static constexpr uint16_t XDI_Magnetometer      = 0xC020;
static constexpr uint16_t XDI_LatLon            = 0x5040;
static constexpr uint16_t XDI_DeltaQ            = 0x8030;
static constexpr uint16_t XDI_DeltaV            = 0x4010;
static constexpr uint16_t XDI_Velocity          = 0xD010;

#endif // MTI_SPI_CUSTOM_h
