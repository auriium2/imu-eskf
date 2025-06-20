#include "Arduino.h"
#include "ArduinoEigen.h"
#include "MTi_SPI_custom.h"
#include "ESKF2.h"
#include "Debias.h"
#include "Prelude.h"
#include "SPI.h"
#include "Zbias.h"
#include <WiFiS3.h>
#include "arduino_secrets.h"
#include "gen/Packet.pb.h"
#include "pb_encode.h"
#include "CBTimer.h"
#include "etl/circular_buffer.h"

#define CONFIG_MODE 0 //1 for debug 2 for real

const float GRAV = 9.80665f;
const float SI_TO_G = 1.0f / 9.81f;
const float DEG2RAD = M_PI / 180.0f;
const float RAD2DEG = 180.0f / M_PI;

#define DRDY 3                      //Arduino Digital IO pin used as input for MTi-DRDY
#define nCS 10                      //MTi nCS pin connected to Arduino Digital IO pin 10
#define N 18

//230400
static const uint32_t BAUDRATE = 460800;

const float sigma_an = 70e-6 * GRAV * sqrt(100);
const float sigma_wn = radians(0.003) * sqrt(100);
const float sigma_aw = (40e-6 * GRAV) / sqrt(3600);
const float sigma_ww = radians(6) / sqrt(3600);

MTi* mti = NULL;
eskf::ESKF filter = eskf::ESKF(eskf::ESKF::Params{sigma_an, sigma_wn, sigma_aw, sigma_ww});
eskf::ZDebiaser debiaser = eskf::ZDebiaser();


void config_mti() {

    //CONFIG THE MTI
    pinMode(DRDY, INPUT);             //Data Ready pin, indicates whether data/notifications are available to be read
    pinMode(nCS, OUTPUT);             //nCS pin, used by SPI master to select MTi
    digitalWrite(nCS, HIGH);          //De-select until writing starts

    mti = new MTi(nCS, DRDY);
    if (!mti->detect(1000)) {
        Serial.println("Please check your hardware connections.");
        while(1);
    }
    mti->requestDeviceInfo();
    mti->getFilterProfile();
    mti->setBaudRate(BAUDRATE);
    mti->getBaudRate();
    mti->configureOutputsAdv(
        {
          {XDI_HR_Acceleration, 200},
          {XDI_HR_Gyro, 200},
          {XDI_Quaternion, 200},

          #if CONFIG_MODE == 1
            {XDI_DeltaQ, 100},
            {XDI_DeltaV, 100},
            {XDI_FreeAcceleration, 100},
            {XDI_RateOfTurn, 100},
          #endif
        }
    );
    mti->goToMeasurement();
}

WiFiUDP udp;
void config_wifi() {
    //CONFIG THE WIFI
    if (WiFi.status() == WL_NO_MODULE) {
      Serial.println("ERROR: Communication with WiFi module failed!");
      while (true); //TODO: this should kill or do something else interesting, but not while forever
    }

    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
      Serial.println("ERROR: bad wifi firmware. Please upgrade the wifi firmware");
    }

    unsigned long startAttemptTime = millis();
    const unsigned long timeout = 30000;

    Serial.print("INFO: Connecting to WiFi SSID: ");
    Serial.println(SECRET_SSID);

    // Attempt to connect to WiFi
    WiFi.begin(SECRET_SSID, SECRET_PASS);

    // Wait until connected or timeout
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < timeout) {
      Serial.print(".");
      delay(500);
    }



    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("");
      Serial.println("INFO: RDY Connected to WiFi!");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());

      udp.begin(RECV_PORT);
    } else {
      Serial.println("Failed to connect to WiFi.");
    }
}

void setup() {
    Serial.begin(BAUDRATE);
    SPI.begin();

    delay(2000);

    config_mti();
    config_wifi();

    delay(1000); //This stops the startup spike
}

uint32_t integrator_called = 0;
uint32_t startup_timer = 0;

const unsigned long OUTER_INTERVAL_US = 10000;  // 100 Hz = 10,000 us = 10 ms
const unsigned long INNER_INTERVAL_US = 5000;   // 1000 Hz = 1,000 us = 1 ms
unsigned long lastOuterTime = 0;
unsigned long lastInnerTime = 0;

etl::circular_buffer<float, 32> ax;
etl::circular_buffer<float, 32> ay;
etl::circular_buffer<float, 32> az;
etl::circular_buffer<float, 32> gx;
etl::circular_buffer<float, 32> gy;
etl::circular_buffer<float, 32> gz;
etl::circular_buffer<long, 32> when;

etl::circular_buffer<float, 32> qw;
etl::circular_buffer<float, 32> qx;
etl::circular_buffer<float, 32> qy;
etl::circular_buffer<float, 32> qz;



void inner_loop(long now_milis) {


    if (!digitalRead(mti->drdy)) {
        //Serial.println("WARNING: Inner loop polled mti for fast message and got none");
        // Something is going wrong here. I'm just going to write a bool that says stale data
        return;
    }

    mti->readMessages();

    float* HR_acc_raw = mti->getHRAcceleration();
    float* HR_gyr_raw = mti->getHRGyro();
    float* quat_raw = mti->getQuaternion();



    ax.push(HR_acc_raw[0]);
    ay.push(HR_acc_raw[1]);
    az.push(HR_acc_raw[2]);
    gx.push(HR_gyr_raw[0]);
    gy.push(HR_gyr_raw[1]);
    gz.push(HR_gyr_raw[2]);
    qw.push(quat_raw[0]);
    qx.push(quat_raw[1]);
    qy.push(quat_raw[2]);
    qz.push(quat_raw[3]);

    when.push(now_milis);
}


void outer_loop() {


}

void log_loop() {
    bool good = true;

    while (!ax.empty()) {
        static imu_Packet pkt = imu_Packet_init_zero;
        pkt.which_payload = imu_Packet_imu_tag;
        pkt.payload.imu.has_a_m = true;
        pkt.payload.imu.has_w_m = true;
        pkt.payload.imu.has_q_m = true;
        pkt.payload.imu.a_m.x = ax.front();
        pkt.payload.imu.a_m.y = ay.front();
        pkt.payload.imu.a_m.z = az.front();
        pkt.payload.imu.w_m.x = gx.front();
        pkt.payload.imu.w_m.y = gy.front();
        pkt.payload.imu.w_m.z = gz.front();
        pkt.payload.imu.q_m.w = qw.front();
        pkt.payload.imu.q_m.x = qx.front();
        pkt.payload.imu.q_m.y = qy.front();
        pkt.payload.imu.q_m.z = qz.front();
        pkt.time_milis = when.front();

        ax.pop(); ay.pop(); az.pop(); gx.pop(); gy.pop(); gz.pop(); when.pop();
        qx.pop(); qy.pop(); qz.pop(); qw.pop();

        uint8_t txBuf[imu_Packet_size];
        pb_ostream_t stream = pb_ostream_from_buffer(txBuf, sizeof(txBuf));
        if (!pb_encode(&stream, imu_Packet_fields, &pkt)) {
            Serial.println("WARNING: Protobuf encode failed");
            continue;
        }

        if (udp.beginPacket(RECV_IP, RECV_PORT)) {
            udp.write(txBuf, stream.bytes_written);
            if (!udp.endPacket()) {
                good = false;
            }
        } else {
            good = false;
        }
    }


    if (!good) Serial.println("WARNING: udp send failed");
}

void loop() {
    unsigned long now_milis = millis();


    unsigned long now = micros();
    if (now - lastInnerTime >= INNER_INTERVAL_US) {
        lastInnerTime = now;;

        // snapshot start of work
        unsigned long startInner = micros();
        inner_loop(now_milis);
        unsigned long innerElapsed = micros() - startInner;

        if (innerElapsed > INNER_INTERVAL_US) {
          Serial.print("WARNING: Inner loop overrun! Took ");
          Serial.print(innerElapsed);
          Serial.print(" us (");
          Serial.print(innerElapsed / 1000);
          Serial.println(" ms)");
        }
    }

    now = micros();
    if (now - lastOuterTime >= OUTER_INTERVAL_US) {
        lastOuterTime += now;

        // snapshot start of work
        unsigned long startOuter = micros();
        outer_loop();
        unsigned long outerElapsed = micros() - startOuter;

        if (outerElapsed > INNER_INTERVAL_US) {
          Serial.print("WARNING: Outer loop overrun! Took ");
          Serial.print(outerElapsed);
          Serial.print(" us (");
          Serial.print(outerElapsed / 1000);
          Serial.println(" ms)");
        }
    }

    log_loop();

}

// void loop() {
//     if (!digitalRead(mti->drdy)) return;

//     //timing
//     uint32_t now = micros();
//     float deltaTime = (now - integrator_called) * 1e-6f;
//     integrator_called = now;

//     mti->readMessages();

//     float* acc      = mti->getAcceleration();      // [ax, ay, az]
//     float* gyr      = mti->getRateOfTurn();        // [gx, gy, gz]
//     float* freeAcc  = mti->getFreeAcceleration();  // [fax, fay, faz]
//     float* quat     = mti->getQuaternion();
//     float* magn = mti->getMagnetometer();
//     float* rot      = mti->getRotation();
//     // float* eul      = mti->getEulerAngles();       // [roll, pitch, yaw]
//     // float* dq       = mti->getDeltaQ();
//     float* dv       = mti->getDeltaV();

//     // Run ESKF
//     Eigen::Vector3f a_m(acc[0], acc[1], acc[2]);
//     Eigen::Vector3f w_m(gyr[0], gyr[1], gyr[2]);
//     Eigen::Vector3f freeAcc_m(freeAcc[0], freeAcc[1], freeAcc[2]);
//     Eigen::Quaternionf q_m(quat[0], quat[1], quat[2], quat[3]);
//     Eigen::Vector3f dv_m(dv[0], dv[1], dv[2]);
//     q_m.normalize();
//     Eigen::Matrix3f rot_m = q_m.toRotationMatrix();

//     Eigen::Vector3f scam(freeAcc[0], freeAcc[1], freeAcc[2]);


//     filter.predict(a_m, w_m, deltaTime);

//     Eigen::Matrix<float,3,1> y {0,0,0};
//     auto h = [](const eskf::ESKF& f)->Eigen::Matrix<float,3,1> {
//         return f.v_;
//     };

//     Eigen::Matrix<float,3,18> Hx = Eigen::Matrix<float,3,18>::Zero();
//     Hx.block<3,3>(0,3) = Eigen::Matrix3f::Identity();

//     Eigen::Matrix<float,3,3> Rvv = Eigen::Matrix<float,3,3>::Zero();

//     float vStdv = 0.02;
//     Eigen::Matrix<float,3,1> RvvDiag {vStdv, vStdv, vStdv};

//     Rvv.diagonal() = RvvDiag;

//     filter.correct(
//         y, h, Hx, Rvv
//     );







//     //   //zero _ update,
//     // Eigen::Matrix<float,3,1> y {0,0,0};
//     // auto h = [](const eskf::ESKF& f)->Eigen::Matrix<float,3,1> {
//     //     Eigen::Matrix<float,3,1> hVec;
//     //     hVec << f.v_;

//     //     //f.p,
//     //     return hVec;
//     // };
//     // Eigen::Matrix<float,3,18> Hx = Eigen::Matrix<float,3,18>::Zero();
//     // Hx.block<3,3>(0,3) = Eigen::Matrix3f::Identity();

//     // Eigen::Matrix<float,3,3> Rvv = Eigen::Matrix<float,3,3>::Zero();

//     // float vStdv = 0.5;
//     // Eigen::Matrix<float,3,1> RvvDiag {vStdv, vStdv, vStdv};

//     // Rvv.diagonal() = RvvDiag;

//     // filter.correct(
//     //     y, h, Hx, Rvv
//     // );



//     auto angles = q_m;


//     //auto cov = filter.P.block<3,3>(9,9).norm();
//     //Pack the data into a structure of uint8s that can be unpacked in python
//     Sample s = {
//       0xA5,
//       0x5A,
//       false,
//       filter.p_.x(), filter.p_.y(), filter.p_.z(),
//       filter.v_.x(), filter.v_.y(), filter.v_.z(),
//       w_m.x(), w_m.y(), w_m.z(),
//       filter.w_b_.x(), filter.w_b_.y(), filter.w_b_.z(),
//       a_m.x(), a_m.y(), a_m.z(),
//       filter.a_b_.x(), filter.a_b_.y(), filter.a_b_.z(),
//       filter.a_w.x(), filter.a_w.y(), filter.a_w.z(), //a_w
//       filter.q_.w(), filter.q_.x(), filter.q_.y(), filter.q_.z(),
//       angles.x(), angles.y(), angles.z(), //angles
//       freeAcc[0], freeAcc[1], freeAcc[2],
//     };


//     const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&s);
//     Serial.write(ptr, sizeof(Sample));
// }
