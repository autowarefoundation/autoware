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

#include <ros/ros.h>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>

#include "mpc_follower/lowpass_filter.h"

class TestSuite : public ::testing::Test
{
public:
    TestSuite() {}
    ~TestSuite() {}
};

TEST_F(TestSuite, TestButterworthFilter)
{
    double dt = 0.01; // sampling time [s]
    double fc = 5.0; // cutoff [hz]
    Butterworth2dFilter butter(dt, fc);

    std::vector<double> coeffs;
    butter.getCoefficients(coeffs);

    const double a0_tmp = coeffs.at(0);
    ASSERT_GT(std::fabs(a0_tmp), 1.0E-5) << "non-zero value is expected";

    // const double a0 = 1.0;
    const double a1 = coeffs.at(1) / a0_tmp;
    const double a2 = coeffs.at(2) / a0_tmp;
    const double b0 = coeffs.at(3) / a0_tmp;
    const double b1 = coeffs.at(4) / a0_tmp;
    const double b2 = coeffs.at(5) / a0_tmp;

    // calculated by butter(2, f_cutoff_hz/(1/dt/2))
    // const double a0_matlab = 1.0;
    const double a1_matlab = -1.5610;
    const double a2_matlab = 0.6414;
    const double b0_matlab = 0.0201;
    const double b1_matlab = 0.0402;
    const double b2_matlab = 0.0201;

    // these values does not match perfectly due to discretization algorithm.
    ASSERT_NEAR(a1, a1_matlab, std::fabs(a1_matlab * 0.1)/* error limit*/) << "a1 value differs more than 10% with matlab, check equation";
    ASSERT_NEAR(a2, a2_matlab, std::fabs(a2_matlab * 0.1)/* error limit*/) << "a2 value differs more than 10% with matlab, check equation";
    ASSERT_NEAR(b0, b0_matlab, std::fabs(b0_matlab * 0.1)/* error limit*/) << "b0 value differs more than 10% with matlab, check equation";
    ASSERT_NEAR(b1, b1_matlab, std::fabs(b1_matlab * 0.1)/* error limit*/) << "b1 value differs more than 10% with matlab, check equation";
    ASSERT_NEAR(b2, b2_matlab, std::fabs(b2_matlab * 0.1)/* error limit*/) << "b2 value differs more than 10% with matlab, check equation";
}



int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "TestNode");
    return RUN_ALL_TESTS();
}