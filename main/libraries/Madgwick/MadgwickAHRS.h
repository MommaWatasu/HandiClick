//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick {
private:
    static float invSqrt(float x);  // 逆平方根計算
    float beta;  // algorithm gain
    float q0, q1, q2, q3;  // quaternion
    float invSampleFreq;
    float roll, pitch, yaw;  // Euler angles
    char anglesComputed;

    // 内部関数: 角度の計算
    void computeAngles();

public:
    // コンストラクタ
    Madgwick();
    
    // 初期化関数
    void begin(float sampleFrequency) {
        invSampleFreq = 1.0f / sampleFrequency;
    }

    // センサデータ更新関数
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

    // ロール・ピッチ・ヨーの取得（度単位）
    float getRoll() {
        if (!anglesComputed) computeAngles();
        return roll * 57.29578f;  // ラジアンから度への変換
    }

    float getPitch() {
        if (!anglesComputed) computeAngles();
        return pitch * 57.29578f;
    }

    float getYaw() {
        if (!anglesComputed) computeAngles();
        return yaw * 57.29578f + 180.0f;
    }

    // ロール・ピッチ・ヨーの取得（ラジアン単位）
    float getRollRadians() {
        if (!anglesComputed) computeAngles();
        return roll;
    }

    float getPitchRadians() {
        if (!anglesComputed) computeAngles();
        return pitch;
    }

    float getYawRadians() {
        if (!anglesComputed) computeAngles();
        return yaw;
    }
};

#endif  // MadgwickAHRS_h

