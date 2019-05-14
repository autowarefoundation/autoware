/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "amathutils_lib/kalman_filter.hpp"
#include "amathutils_lib/time_delay_kalman_filter.hpp"

class KalmanFilterTestSuite : public ::testing::Test
{
  public:
    KalmanFilterTestSuite() {}
    ~KalmanFilterTestSuite() {}
};

TEST_F(KalmanFilterTestSuite, updateCase)
{

    KalmanFilter kf;
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd y = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd y_pred = y;

    ASSERT_EQ(false, kf.update(y)) << "uninitialized, false expected";
    ASSERT_EQ(false, kf.update(y, C, R)) << "uninitialized, false expected";
    ASSERT_EQ(false, kf.update(y, y_pred, C, R)) << "uninitialized, false expected";

    kf.init(x, P);
    Eigen::MatrixXd no_initialized;
    ASSERT_EQ(false, kf.update(no_initialized)) << "inappropriate argument, false expected";

    Eigen::MatrixXd R0 = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(3, 3);
    kf.init(x, P0);
    ASSERT_EQ(false, kf.update(y, y_pred, C, R0)) << "R0 inverse problem, false expected";

    kf.init(x, P);
    Eigen::MatrixXd R_bad_dim = Eigen::MatrixXd::Identity(4, 4);
    ASSERT_EQ(false, kf.update(y, y_pred, C, R_bad_dim)) << "R dimension problem, false expected";

    Eigen::MatrixXd y_bad_dim = Eigen::MatrixXd::Identity(4, 1);
    ASSERT_EQ(false, kf.update(y, y_bad_dim, C, R_bad_dim)) << "y_pred dimension problem, false expected";
    ASSERT_EQ(false, kf.update(y_bad_dim, y_pred, C, R_bad_dim)) << "y dimension problem, false expected";

    Eigen::MatrixXd C_bad_dim = Eigen::MatrixXd::Identity(2, 2);
    ASSERT_EQ(false, kf.update(y, y_pred, C_bad_dim, R_bad_dim)) << "C dimension problem, false expected";
    ASSERT_EQ(false, kf.update(y, y_pred, C_bad_dim)) << "C dimension problem, false expected";

    kf.init(x, P);
    ASSERT_EQ(true, kf.update(y, y_pred, C, R)) << "normal process, true expected";
    ASSERT_EQ(true, kf.update(y, C, R)) << "normal process, true expected";

    kf.init(x, P);
    y << 1.0, 1.0, 1.0;
    y_pred << 0.0, 0.0, 0.0;
    kf.update(y, y_pred, C, R);
    Eigen::MatrixXd X_expected(3, 1);
    X_expected << 0.5, 0.5, 0.5;
    Eigen::MatrixXd X_actual;
    kf.getX(X_actual);
    ASSERT_TRUE((X_actual - X_expected).norm() < 1.0E-6) << "X_actual^T : " << X_actual.transpose() << ", X_expected^T : " << X_expected.transpose();
}

TEST_F(KalmanFilterTestSuite, predictCase)
{
    KalmanFilter kf;
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd x_next = x;
    Eigen::MatrixXd u = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 2);
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3, 3);

    ASSERT_EQ(false, kf.predict(u)) << "uninitialized, false expected";
    ASSERT_EQ(false, kf.predict(x_next, A)) << "uninitialized, false expected";

    kf.init(x, P);
    Eigen::MatrixXd no_initialized;
    ASSERT_EQ(false, kf.predict(no_initialized, A, Q)) << "inappropriate argument, false expected";
    ASSERT_EQ(false, kf.predict(x_next, no_initialized, Q)) << "inappropriate argument, false expected";
    ASSERT_EQ(false, kf.predict(x_next, A, no_initialized)) << "inappropriate argument, false expected";
    ASSERT_EQ(false, kf.predict(no_initialized, no_initialized, no_initialized)) << "inappropriate argument, false expected";
    ASSERT_EQ(false, kf.init(x, no_initialized)) << "inappropriate argument, false expected";

    Eigen::MatrixXd A_bad_dim = Eigen::MatrixXd::Zero(4, 4);
    ASSERT_EQ(false, kf.predict(x_next, A_bad_dim, Q)) << "A dimension problem, false expected";
    Eigen::MatrixXd Q_bad_dim = Eigen::MatrixXd::Zero(4, 4);
    ASSERT_EQ(false, kf.predict(x_next, A, Q_bad_dim)) << "Q dimension problem, false expected";
    Eigen::MatrixXd x_bad_dim = Eigen::MatrixXd::Zero(4, 1);
    ASSERT_EQ(false, kf.predict(x_bad_dim, A, Q)) << "x dimension problem, false expected";

    ASSERT_EQ(true, kf.predict(x_next, A, Q)) << "normal process, true expected";
    ASSERT_EQ(true, kf.predict(u, A, B, Q)) << "normal process, true expected";

    kf.init(x, P);
    A = Eigen::MatrixXd::Identity(3, 3);
    Q = Eigen::MatrixXd::Identity(3, 3);
    kf.predict(x_next, A, Q);
    Eigen::MatrixXd P_expected = Eigen::MatrixXd::Identity(3, 3) * 2.0;
    Eigen::MatrixXd P_actual;
    kf.getP(P_actual);
    ASSERT_TRUE((P_actual - P_expected).norm() < 1.0E-6) << "P_actual : " << P_actual << ", P_expected : " << P_expected;
}

TEST_F(KalmanFilterTestSuite, delayedMeasurement)
{
    TimeDelayKalmanFilter tdkf;
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd x_next = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd y = Eigen::MatrixXd::Zero(3, 1);
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(3, 3) * 2.0;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);
    int max_delay_step = 5;

    tdkf.init(x, P, max_delay_step);

    x_next << 1.0, 2.0, 3.0;
    tdkf.predictWithDelay(x_next, A, Q);
    Eigen::MatrixXd x_curr;
    tdkf.getLatestX(x_curr);
    ASSERT_TRUE((x_next - x_curr).norm() < 1.0E-6);

    int delay_step = max_delay_step - 1;
    ASSERT_EQ(true, tdkf.updateWithDelay(y, C, R, delay_step));

    delay_step = max_delay_step;
    ASSERT_EQ(false, tdkf.updateWithDelay(y, C, R, delay_step));

    delay_step = max_delay_step + 1;
    ASSERT_EQ(false, tdkf.updateWithDelay(y, C, R, delay_step));
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "KalmanFilterTestSuite");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}