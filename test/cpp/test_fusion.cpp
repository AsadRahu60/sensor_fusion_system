#include <gtest/gtest.h>
#include <Eigen/Dense>

// Test matrix operations
TEST(FusionTest, EigenMatrixMultiplication) {
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  Eigen::Vector3d b(1, 2, 3);
  
  Eigen::Vector3d c = A * b;
  
  EXPECT_DOUBLE_EQ(c(0), 1.0);
  EXPECT_DOUBLE_EQ(c(1), 2.0);
  EXPECT_DOUBLE_EQ(c(2), 3.0);
}

// Test angle normalization
TEST(FusionTest, AngleNormalization) {
  auto normalize = [](double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  };
  
  EXPECT_NEAR(normalize(4.0), 4.0 - 2*M_PI, 0.01);
  EXPECT_NEAR(normalize(-4.0), -4.0 + 2*M_PI, 0.01);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}