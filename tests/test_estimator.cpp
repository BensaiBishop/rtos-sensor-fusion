#include <gtest/gtest.h>
#include "tasks/estimator_task.hpp"  // declares init_filter, helpers
#include <cmath>

// --- Matrix helper tests ---

TEST(MatrixHelpers, IdentityMultiplication) {
    Mat6 I = mat06();
    for (int i = 0; i < 6; i++) I[i][i] = 1.0f;

    Mat6 result = mul6(I, I);

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            if (i == j) {
                EXPECT_FLOAT_EQ(result[i][j], 1.0f);
            } else {
                EXPECT_FLOAT_EQ(result[i][j], 0.0f);
            }
        }
    }
}

TEST(MatrixHelpers, VecMultiplication) {
    Mat6 I = mat06();
    for (int i = 0; i < 6; i++) I[i][i] = 1.0f;

    Vec6 v = {1, 2, 3, 4, 5, 6};
    Vec6 result = mat6_vec6(I, v);

    for (int i = 0; i < 6; i++) {
        EXPECT_FLOAT_EQ(result[i], v[i]);
    }
}

TEST(MatrixHelpers, Transpose) {
    Mat6 A = mat06();
    A[0][1] = 5.0f;
    A[2][4] = -2.0f;

    Mat6 T = trans6(A);

    EXPECT_FLOAT_EQ(T[1][0], 5.0f);
    EXPECT_FLOAT_EQ(T[4][2], -2.0f);
}

// --- Inversion tests ---

TEST(Inversion, Invert2x2) {
    Mat2 A{{ {4, 7}, {2, 6} }};
    Mat2 invA = inv2(A);

    // A * invA = I
    float m00 = A[0][0]*invA[0][0] + A[0][1]*invA[1][0];
    float m01 = A[0][0]*invA[0][1] + A[0][1]*invA[1][1];
    float m10 = A[1][0]*invA[0][0] + A[1][1]*invA[1][0];
    float m11 = A[1][0]*invA[0][1] + A[1][1]*invA[1][1];

    EXPECT_NEAR(m00, 1.0f, 1e-5f);
    EXPECT_NEAR(m11, 1.0f, 1e-5f);
    EXPECT_NEAR(m01, 0.0f, 1e-5f);
    EXPECT_NEAR(m10, 0.0f, 1e-5f);
}

// --- Filter initialization ---

TEST(FilterInit, StateAndCovariance) {
    init_filter();

    // check state estimate initialized to 0
    for (int i = 0; i < 6; i++) {
        EXPECT_FLOAT_EQ(x_est[i], 0.0f);
    }

    // covariance should be identity
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            if (i == j) {
                EXPECT_FLOAT_EQ(P[i][j], 1.0f);
            } else {
                EXPECT_FLOAT_EQ(P[i][j], 0.0f);
            }
        }
    }
}
