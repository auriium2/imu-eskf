#include "MTi_SPI_custom.h"
#include "ESKF.h"

#include "api/Common.h"


const float GRAV = 9.80665f;
const float SI_TO_G = 1.0f / 9.81f;
const float DEG2RAD = M_PI / 180.0f;
const float RAD2DEG = 180.0f / M_PI;

#define DRDY 3                      //Arduino Digital IO pin used as input for MTi-DRDY
#define nCS 10                      //MTi nCS pin connected to Arduino Digital IO pin 10
static const uint32_t BAUDRATE = 230400;
MTi *mti = NULL;



void setup() {
    Serial.begin(BAUDRATE);
      // while(!Serial);
    SPI.begin();
    pinMode(DRDY, INPUT);             //Data Ready pin, indicates whether data/notifications are available to be read
    pinMode(nCS, OUTPUT);             //nCS pin, used by SPI master to select MTi
    digitalWrite(nCS, HIGH);          //De-select until writing starts

    // ESKF Initialization
    filter.p   = Eigen::Vector3f(0,0,0);
    filter.v   = Eigen::Vector3f(0,0,0);
    filter.q   = Eigen::Quaternionf::Identity();
    filter.a_b = accBiasInit;
    filter.w_b = gyrBiasInit;
    filter.g   = gInit;
    Eigen::Matrix<float,N,N> P0;
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            P0(i,j) = P0_vals[i][j];
        }
    }
    filter.P = P0;
    filter.sigma_an = 70e-6 * GRAV * sqrt(SAMPLE_RATE_HZ);          // accel white noise   [m/s²]
    filter.sigma_wn = 3e-3*DEG2RAD * sqrt(SAMPLE_RATE_HZ);          // gyro  white noise   [rad/s]
    filter.sigma_aw = (40e-6 * GRAV) / sqrt(3600);                  // accel bias RW       [m/s² /√s]
    filter.sigma_ww = (6 * DEG2RAD) / sqrt(3600);                   // gyro  bias RW       [rad/s /√s]

    #if defined(ESKF18)
        filter.sigma_gw = 0.00f;                                      // gravity process noise
    #endif

    // Create MTi object and wait for Ready
    mti = new MTi(nCS, DRDY);
    delay(1000);                    //Allow some time for the MTi to initialize
    if (!mti->detect(1000)) {
        Serial.println("Please check your hardware connections.");
        while(1);
    }
    mti->requestDeviceInfo();
    mti->getFilterProfile();
    mti->setBaudRate(BAUDRATE);
    mti->getBaudRate();
    mti->configureOutputs(
        { XDI_Acceleration,
          XDI_FreeAcceleration,
          XDI_RateOfTurn,
          XDI_Quaternion,
          // XDI_Rotation,
          // XDI_EulerAngles,
          //XDI_DeltaQ,
          //XDI_DeltaV,
        }
    );
    mti->goToMeasurement();
}
void loop() {

}
