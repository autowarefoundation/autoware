/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

/*
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
