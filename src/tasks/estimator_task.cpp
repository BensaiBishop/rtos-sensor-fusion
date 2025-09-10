#include "tasks/estimator_task.hpp"
#include "state/state.hpp"
#include "tasks/dashboard_task.hpp"
#include <iostream>
#include <cmath>
#include <array>
#include <random>
#include <mutex>

extern "C"{
    #include "FreeRTOS.h"
    #include "task.h"
}

// Matrix helpers of fixed size for KF
using Vec6 = std::array<float,6>;
using Mat6 = std::array<std::array<float,6>,6>;
using Mat3 = std::array<std::array<float,3>,3>;
using Mat2 = std::array<std::array<float,2>,2>;

// initalize to zero
static Vec6 vec06() {Vec6 v{}; for(int i = 0; i < 6; i++) v[i] = 0.0f; return v;}
static Mat6 mat06() {Mat6 m{}; for(int i = 0; i < 6; i++) for(int j = 0; j < 6; j++) m[i][j] = 0.0f; return m;}

/* multiply small matrices: C = A(6x6) * B(6x6) */
static Mat6 mul6(const Mat6 &A, const Mat6 &B) {
    Mat6 C = mat06();
    for(int i=0;i<6;i++) for(int k=0;k<6;k++) {
        float a = A[i][k];
        if(a==0.0f) continue;
        for(int j=0;j<6;j++) C[i][j] += a * B[k][j];
    }
    return C;
}

/* A*vec (6x6 * 6x1) */
static Vec6 mat6_vec6(const Mat6 &A, const Vec6 &v) {
    Vec6 out = vec06();
    for(int i=0;i<6;i++) {
        float s = 0.0f;
        for(int j=0;j<6;j++) s += A[i][j] * v[j];
        out[i] = s;
    }
    return out;
}

/* Add/sub on Mat6 */
static Mat6 add6(const Mat6 &A, const Mat6 &B) {
    Mat6 C = mat06();
    for(int i=0;i<6;i++) for(int j=0;j<6;j++) C[i][j] = A[i][j] + B[i][j];
    return C;
}
static Mat6 sub6(const Mat6 &A, const Mat6 &B) {
    Mat6 C = mat06();
    for(int i=0;i<6;i++) for(int j=0;j<6;j++) C[i][j] = A[i][j] - B[i][j];
    return C;
}

/* Transpose of 6x6 */
static Mat6 trans6(const Mat6 &A) {
    Mat6 T = mat06();
    for(int i=0;i<6;i++) for(int j=0;j<6;j++) T[i][j] = A[j][i];
    return T;
}

/* Simple inversion for small matrices used in measurement update:
   - For IMU: H is 3x6 => S is 3x3 (we need invert 3x3)
   - For GPS: H is 2x6 => S is 2x2
   - For Baro: H is 1x6 => S is 1x1 (scalar)
   Implement specialized inverses.
*/

/* 1x1 invert */
static float inv1(float a) { return 1.0f / a; }

/* 2x2 invert */
static Mat2 inv2(const Mat2 &m) {
    Mat2 r{};
    float a=m[0][0], b=m[0][1], c=m[1][0], d=m[1][1];
    float det = a*d - b*c;
    if(fabs(det) < 1e-9f) det = 1e-9f;
    float idet = 1.0f/det;
    r[0][0] =  d*idet; r[0][1] = -b*idet;
    r[1][0] = -c*idet; r[1][1] =  a*idet;
    return r;
}

/* 3x3 invert */
static Mat3 inv3(const Mat3 &m) {
    Mat3 inv{};
    float a=m[0][0], b=m[0][1], c=m[0][2];
    float d=m[1][0], e=m[1][1], f=m[1][2];
    float g=m[2][0], h=m[2][1], i=m[2][2];

    float A = (e*i - f*h);
    float B = -(d*i - f*g);
    float C = (d*h - e*g);
    float D = -(b*i - c*h);
    float E = (a*i - c*g);
    float F = -(a*h - b*g);
    float G = (b*f - c*e);
    float H = -(a*f - c*d);
    float I = (a*e - b*d);

    float det = a*A + b*B + c*C;
    if(fabs(det) < 1e-12f) det = (det>=0?1e-12f:-1e-12f);
    float idet = 1.0f/det;

    inv[0][0] = A*idet; inv[0][1] = D*idet; inv[0][2] = G*idet;
    inv[1][0] = B*idet; inv[1][1] = E*idet; inv[1][2] = H*idet;
    inv[2][0] = C*idet; inv[2][1] = F*idet; inv[2][2] = I*idet;
    return inv;
}

/* --- Kalman filter data --- */
static Vec6 x_est = vec06();    // state estimate
static Mat6 P = mat06();         // covariance

void init_filter() {
    // small initial covariance
    for(int i=0;i<6;i++) for(int j=0;j<6;j++) P[i][j] = (i==j) ? 1.0f : 0.0f;
    // small initial estimate (0)
    for(int i=0;i<6;i++) x_est[i] = 0.0f;
}

void EstimatorTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    const float dt = 0.005f; // 200Hz

    // Noise covariances (tunable)
    Mat6 Q = mat06(); // process noise (model)
    for(int i=0;i<6;i++) Q[i][i] = 0.01f; // small process noise

    Mat3 R_imu{}; // IMU measures vx,vy,vz
    R_imu[0][0]=0.04f; R_imu[1][1]=0.04f; R_imu[2][2]=0.04f;

    Mat2 R_gps{}; // GPS measures x,y
    R_gps[0][0]=0.25f; R_gps[1][1]=0.25f;

    float R_baro = 0.05f; // baro measures z (scalar)

    // init
    init_filter();

    // RNG not required here (simulated sensors already noisy), KF consumes noisy meas.
    while(true) {
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5)); // 200Hz

        // --- Prediction step ---
        // State transition F for constant velocity model:
        // [I dt*I]
        // [0  I  ]
        Mat6 F = mat06();
        for(int i=0;i<3;i++) {
            F[i][i] = 1.0f;
            F[i][i+3] = dt;
            F[i+3][i+3] = 1.0f;
        }

        // x = F * x (no control input)
        x_est = mat6_vec6(F, x_est);

        // P = F P F^T + Q
        Mat6 Ft = trans6(F);
        Mat6 FP = mul6(F, P);
        Mat6 FPFt = mul6(FP, Ft);
        P = add6(FPFt, Q);

        // ---- MEASUREMENT UPDATES ----
        TickType_t now = xTaskGetTickCount();

        // IMU (vx,vy,vz) measurement at 200Hz: H_imu (3x6) picks vx,vy,vz
        {
            // Simulated IMU measurement: read trueState.vx/vy/vz under lock
            float z_imu[3];
            {
                std::lock_guard<std::mutex> lock(stateMutex);
                z_imu[0] = trueState.vx;
                z_imu[1] = trueState.vy;
                z_imu[2] = trueState.vz;
                sensorTimestamps.imu = now;
            }

            // H * x_est => predicted measurement: pick entries 3,4,5 (vx,vy,vz)
            float hx[3] = { x_est[3], x_est[4], x_est[5] };

            // Innovation y = z - Hx
            float y[3] = { z_imu[0] - hx[0], z_imu[1] - hx[1], z_imu[2] - hx[2] };

            // S = H P H^T + R  (3x3)
            Mat3 S{}; // compute H P H^T quickly (H selects bottom-right 3x3 block)
            for(int i=0;i<3;i++) for(int j=0;j<3;j++) S[i][j] = P[i+3][j+3];
            // add R_imu
            for(int d=0; d<3; d++) S[d][d] += R_imu[d][d];

            // Compute K = P H^T S^{-1}
            // P H^T -> 6x3 is simply columns 3..5 of P
            std::array<std::array<float,3>,6> PHT{};
            for(int i=0;i<6;i++) for(int j=0;j<3;j++) PHT[i][j] = P[i][j+3];

            Mat3 S_inv = inv3(S);

            // K (6x3) = PHT * S_inv
            std::array<std::array<float,3>,6> K{};
            for(int i=0;i<6;i++) for(int j=0;j<3;j++) {
                float s = 0.0f;
                for(int k=0;k<3;k++) s += PHT[i][k] * S_inv[k][j];
                K[i][j] = s;
            }

            // Update x = x + K*y
            for(int i=0;i<6;i++) {
                float s = 0.0f;
                for(int j=0;j<3;j++) s += K[i][j] * y[j];
                x_est[i] += s;
            }

            // Update P = (I - K H) P //TODO Consider Joseph form to correct numerical instability
            // KH is 6x6 but H has ones at (0,3),(1,4),(2,5)
            Mat6 KH = mat06();
            for(int i=0;i<6;i++) for(int j=0;j<6;j++) {
                // KH[i][j] = sum_k K[i][k] * H[k][j]
                float s = 0.0f;
                for(int k=0;k<3;k++) {
                    int hj = j - 3; // H[k][j] is 1 when j==k+3
                    if(hj == k) s += K[i][k] * 1.0f;
                }
                KH[i][j] = s;
            }
            // I - KH
            Mat6 I_minus_KH = mat06();
            for(int i=0;i<6;i++) for(int j=0;j<6;j++) I_minus_KH[i][j] = (i==j?1.0f:0.0f) - KH[i][j];
            P = mul6(I_minus_KH, P);
        }

        // GPS (px,py) at 1Hz
        if ((now % pdMS_TO_TICKS(1000)) < pdMS_TO_TICKS(5)) {
            float z_gps[2];
            {
                std::lock_guard<std::mutex> lock(stateMutex);
                z_gps[0] = trueState.x;
                z_gps[1] = trueState.y;
                sensorTimestamps.gps = now;
            }
            // H_gps picks px,py => 2x6 measurement
            // Compute innovation y = z - Hx -> 2x1
            float hx0 = x_est[0], hx1 = x_est[1];
            float y0 = z_gps[0] - hx0;
            float y1 = z_gps[1] - hx1;

            // S = H P H^T + R_gps (2x2) -> top-left 2x2 of P + R_gps
            Mat2 Sg{};
            Sg[0][0] = P[0][0] + R_gps[0][0];
            Sg[0][1] = P[0][1];
            Sg[1][0] = P[1][0];
            Sg[1][1] = P[1][1] + R_gps[1][1];

            Mat2 Sg_inv = inv2(Sg);

            // K = P H^T S^{-1}; P H^T is 6x2 using columns 0,1 of P
            std::array<std::array<float,2>,6> K{};
            for(int i=0;i<6;i++) for(int j=0;j<2;j++) {
                // dot P row i with Sg_inv column j across indices [0,1]
                float s = P[i][0] * Sg_inv[0][j] + P[i][1] * Sg_inv[1][j];
                K[i][j] = s;
            }

            // x = x + K * y
            x_est[0] += K[0][0]*y0 + K[0][1]*y1;
            x_est[1] += K[1][0]*y0 + K[1][1]*y1;
            for(int i=2;i<6;i++) x_est[i] += K[i][0]*y0 + K[i][1]*y1;

            // P = (I - K H) P ; H has ones at (0,0) and (1,1)
            Mat6 KH = mat06();
            for(int i=0;i<6;i++){
                // KH[i][0] = K[i][0] * 1  (since H row 0 selects x)
                KH[i][0] = K[i][0];
                // KH[i][1] = K[i][1] * 1
                KH[i][1] = K[i][1];
            }
            Mat6 I_minus_KH = mat06();
            for(int i=0;i<6;i++) for(int j=0;j<6;j++) I_minus_KH[i][j] = (i==j?1.0f:0.0f) - KH[i][j];
            P = mul6(I_minus_KH, P);
        }

        // Barometer (pz) at 10Hz
        if ((now % pdMS_TO_TICKS(100)) < pdMS_TO_TICKS(5)) {
            float z_baro;
            {
                std::lock_guard<std::mutex> lock(stateMutex);
                z_baro = trueState.z;
                sensorTimestamps.baro = now;
            }
            // H is 1x6 selecting index 2
            float hx = x_est[2];
            float y = z_baro - hx;

            // S = P[2][2] + R_baro (scalar)
            float S = P[2][2] + R_baro;
            float K_col[6];
            for(int i=0;i<6;i++) K_col[i] = P[i][2] / S;

            // x = x + K*y
            for(int i=0;i<6;i++) x_est[i] += K_col[i] * y;

            // P = (I - K H) P ; H has 1 at column 2
            Mat6 KH = mat06();
            for(int i=0;i<6;i++) KH[i][2] = K_col[i];
            Mat6 I_minus_KH = mat06();
            for(int i=0;i<6;i++) for(int j=0;j<6;j++) I_minus_KH[i][j] = (i==j?1.0f:0.0f) - KH[i][j];
            P = mul6(I_minus_KH, P);
        }

        // Write estimate back under lock so dashboard/control can read
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            estState.x  = x_est[0];
            estState.y  = x_est[1];
            estState.z  = x_est[2];
            estState.vx = x_est[3];
            estState.vy = x_est[4];
            estState.vz = x_est[5];
        }

        // Dashboard counter 
        dashEstimateCounter++;
    }
}