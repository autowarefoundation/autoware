/*
 *  Copyright (c) 2019, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *  THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * test_random_access_bag.cpp
 *
 *  Created on: Dec 3, 2018
 *      Author: sujiwo
 */

#include <string>

#include <gtest/gtest.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include "RandomAccessBag.h"

using namespace std;

class RandomAccessBagTestClass {
public:
  RandomAccessBagTestClass() {
    string mybag = ros::package::getPath("offline_tools") + "/test/test.bag";
    testBag.open(mybag, rosbag::BagMode::Read);
    numberView =
        shared_ptr<RandomAccessBag>(new RandomAccessBag(testBag, "numbers"));
    stringView =
        shared_ptr<RandomAccessBag>(new RandomAccessBag(testBag, "text"));
  }

  int getNumbersLen() { return numberView->size(); }

  int getSum() {
    int s = 0;
    for (int i = 0; i < numberView->size(); ++i) {
      s += numberView->at<std_msgs::Int32>(i)->data;
    }
    return s;
  }

  string getSingleTextMessage() {
    return stringView->at<std_msgs::String>(0)->data;
  }

private:
  rosbag::Bag testBag;
  RandomAccessBag::Ptr numberView, stringView;
};

TEST(TestSuite, BAG_QUERY) {
  RandomAccessBagTestClass rabTest;
  ASSERT_EQ(rabTest.getNumbersLen(), 10) << "Invalid number of messages";
  ASSERT_EQ(rabTest.getSum(), 45) << "Invalid checksum";
  ASSERT_EQ(rabTest.getSingleTextMessage(), "Test Bag") << "Invalid text found";
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
