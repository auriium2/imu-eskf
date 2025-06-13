#include "ArduinoEigen.h"
#include "ArduinoEigen/Eigen/src/Core/Matrix.h"
#include "MTi_SPI_custom.h"
#include "ESKF.h"
#include "Debias.h"
#include "Prelude.h"
#include "SPI.h"


#include <cstdint>
#include <cstdio>
#include "api/Common.h"


const float GRAV = 9.80665f;
const float SI_TO_G = 1.0f / 9.81f;
const float DEG2RAD = M_PI / 180.0f;
const float RAD2DEG = 180.0f / M_PI;

#define DRDY 3                      //Arduino Digital IO pin used as input for MTi-DRDY
#define nCS 10                      //MTi nCS pin connected to Arduino Digital IO pin 10
#define N 18
static const uint32_t BAUDRATE = 230400;


MTi* mti = NULL;
eskf::ESKF filter = eskf::ESKF(
    1,2,3,4
);

eskf::Debiaser debiaser = eskf::Debiaser(Vec3(0,0,-GRAV));


void setup() {
    Serial.begin(BAUDRATE);
    SPI.begin();

    pinMode(DRDY, INPUT);             //Data Ready pin, indicates whether data/notifications are available to be read
    pinMode(nCS, OUTPUT);             //nCS pin, used by SPI master to select MTi
    digitalWrite(nCS, HIGH);          //De-select until writing starts



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
    mti->configureOutputsAdv(
        {
          {XDI_Acceleration, 100},
          {XDI_FreeAcceleration, 100},
          {XDI_RateOfTurn, 100},
          {XDI_Quaternion, 100},
          {XDI_Rotation, 100},
          // {XDI_EulerAngles, 100},
          // {XDI_DeltaQ, 100},
          // {XDI_DeltaV, 100},
        }
    );
    mti->goToMeasurement();

    delay(3000); //Let's see if this stops the startup spike

}

struct __attribute__((packed)) Sample {
  uint8_t sync0, sync1;
  bool isStance;
  float px, py, pz;
  float vx, vy, vz;
  float w_mx, w_my, w_mz;
  float w_bx, w_by, w_bz;
  float a_mx, a_my, a_mz;
  float a_bx, a_by, a_bz;
  float a_wx, a_wy, a_wz;
  float qw, qx, qy, qz;
  float erx, ery, erz;
  float fa_x, fa_y, fa_z;
};


void loop() {
    static uint32_t integrator_called = 0;
    static uint32_t startup_timer = 0;
    if (!digitalRead(mti->drdy)) return;

    //timing
    uint32_t now = micros();
    float deltaTime = (now - integrator_called) * 1e-6f;
    integrator_called = now;

    mti->readMessages();

    float* acc      = mti->getAcceleration();      // [ax, ay, az]
    float* gyr      = mti->getRateOfTurn();        // [gx, gy, gz]
    float* freeAcc  = mti->getFreeAcceleration();  // [fax, fay, faz]
    float* quat     = mti->getQuaternion();
    //float* magn = mti->getMagnetometer();
    float* rot      = mti->getRotation();
    // float* eul      = mti->getEulerAngles();       // [roll, pitch, yaw]
    // float* dq       = mti->getDeltaQ();
    float* dv       = mti->getDeltaV();

    // Run ESKF
    Eigen::Vector3f a_m(acc[0], acc[1], acc[2]);
    Eigen::Vector3f w_m(gyr[0], gyr[1], gyr[2]);
    Eigen::Vector3f freeAcc_m(freeAcc[0], freeAcc[1], freeAcc[2]);
    Eigen::Quaternionf q_m(quat[0], quat[1], quat[2], quat[3]);
    Eigen::Vector3f dv_m(dv[0], dv[1], dv[2]);
    q_m.normalize();
    Eigen::Matrix3f rot_m = q_m.toRotationMatrix();

    Eigen::Vector3f scam(freeAcc[0], freeAcc[1], freeAcc[2]);


    if (startup_timer < 1000) {
        startup_timer++;

        debiaser.writeSample(a_m, w_m, q_m, rot_m, dv_m, deltaTime);
        if (startup_timer == 1000) {
            //Serial.println("WARNING");

            filter.q = q_m;
            debiaser.finalize(filter.P, filter.a_b, filter.w_b, filter.g);

        }

    } else {
        //we have completed the learning experience

        filter.integrate(a_m, w_m, deltaTime);
        filter.predict(a_m, w_m, deltaTime);

    }







    //   // //zero _ update,
    //   // Eigen::Matrix<float,3,1> y {0,0,0};
    //   // auto h = [](const eskf::ESKF& f)->Eigen::Matrix<float,3,1> {
    //   //     Eigen::Matrix<float,3,1> hVec;
    //   //     hVec << f.v;

    //   //     //f.p,
    //   //     return hVec;
    //   // };
    //   // Eigen::Matrix<float,3,18> Hx = Eigen::Matrix<float,3,18>::Zero();
    //   // Hx.block<3,3>(0,3) = Eigen::Matrix3f::Identity();

    //   // Eigen::Matrix<float,3,3> Rvv = Eigen::Matrix<float,3,3>::Zero();

    //   // float vStdv = 0.5;
    //   // Eigen::Matrix<float,3,1> RvvDiag {vStdv, vStdv, vStdv};

    //   // Rvv.diagonal() = RvvDiag;

    //   // filter.correct(
    //   //     y, h, Hx, Rvv
    //   // );
    // } else {


    //   //we know the biases. Let's predict and correct


    //   //filter.predict(a_m, w_m, deltaTime);
    // }


    auto angles = q_m;


    auto cov = filter.P.block<3,3>(9,9).norm();
    //Pack the data into a structure of uint8s that can be unpacked in python
    Sample s = {
      0xA5,
      0x5A,
      false,
      0, 0, 0,
      filter.v.x(), filter.v.y(), filter.v.z(),
      w_m.x(), w_m.y(), w_m.z(),
      filter.w_b.x(), filter.w_b.y(), filter.w_b.z(),
      a_m.x(), a_m.y(), a_m.z(),
      filter.a_b.x(), filter.a_b.y(), filter.a_b.z(),
      filter.a_w.x(), filter.a_w.y(), filter.a_w.z(), //a_w
      filter.q.w(), filter.q.x(), filter.q.y(), filter.q.z(),
      angles.x(), angles.y(), angles.z(), //angles
      freeAcc[0], freeAcc[1], freeAcc[2],
    };


    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&s);
    Serial.write(ptr, sizeof(Sample));
}
