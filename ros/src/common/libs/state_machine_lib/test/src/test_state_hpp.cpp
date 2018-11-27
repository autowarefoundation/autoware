/*
 * Copyright 2017 Autoware Foundation. All rights reserved.
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
#include <gtest/gtest.h>

#include "state_machine_lib/state.hpp"

class TestSuite: public ::testing::Test {
public:
	TestSuite(){}
	~TestSuite(){}
};

TEST(TestSuite, CheckStateConstructor){

	std::string state_name = "TestState";
	uint64_t state_id = 0;
	state_machine::State st(state_name, state_id);

	ASSERT_EQ(st.getStateID(),state_id) << "_state_id should be " << state_id;
	ASSERT_STREQ(st.getStateName().c_str(), state_name.c_str()) << "state_name should be " << state_name;
	ASSERT_TRUE(st.getChild() == NULL) << "child_state_ pointer should be NULL";
	ASSERT_TRUE(st.getParent() == NULL) << "parent_state_ pointer should be NULL";
	ASSERT_STREQ(st.getEnteredKey().c_str(), "") << "entered_key should be " << "";


	std::ostringstream oss;
	std::streambuf* p_cout_streambuf = std::cout.rdbuf();
	std::cout.rdbuf(oss.rdbuf());

	st.showStateName();

	std::cout.rdbuf(p_cout_streambuf); // restore

	// Test oss content...
	ASSERT_TRUE(oss && oss.str() == state_name + "-") << "Should show" << state_name << "-";

}

TEST(TestSuite, TestParentChild){

	std::string state_name;
	uint64_t state_id;

	state_name = "Start";
	state_id = 0;
	state_machine::State *stateFirst = new state_machine::State(state_name, state_id);
	std::shared_ptr<state_machine::State> stateFirstPtr(stateFirst);

	state_name = "Init";
	state_id = 1;
	state_machine::State *stateSecond = new state_machine::State(state_name, state_id);
	std::shared_ptr<state_machine::State> stateSecondPtr(stateSecond);

	state_name = "Final";
	state_id = 2;
	state_machine::State *stateThird = new state_machine::State(state_name, state_id);
	std::shared_ptr<state_machine::State> stateThirdPtr(stateThird);

	// Set parent
	stateSecondPtr->setParent(stateFirstPtr);
	stateThirdPtr->setParent(stateSecondPtr);

	ASSERT_TRUE(stateFirstPtr->getParent() == NULL) << "Parent should be " << NULL;
	ASSERT_TRUE(stateSecondPtr->getParent() == stateFirstPtr) << "Parent should be " << stateFirstPtr;
	ASSERT_TRUE(stateThirdPtr->getParent() == stateSecondPtr) << "Parent should be " << stateSecondPtr;

	// Set child
	stateFirstPtr->setChild(stateSecondPtr);
	stateSecondPtr->setChild(stateThirdPtr);

	ASSERT_TRUE(stateFirstPtr->getChild() == stateSecondPtr) << "Child should be " << stateSecondPtr;
	ASSERT_TRUE(stateSecondPtr->getChild() == stateThirdPtr) << "Child should be " << stateThirdPtr;
	ASSERT_TRUE(stateThirdPtr->getChild() == NULL) << "Child should be " << NULL;
}

void foo(const std::string&){
	std::cout << "Test output";
};

TEST(TestSuite, SetCallbacks){

	std::string state_name;
	uint64_t state_id;

	state_name = "Start";
	state_id = 0;
	state_machine::State *stateFirst = new state_machine::State(state_name, state_id);
	std::shared_ptr<state_machine::State> stateFirstPtr(stateFirst);

	state_name = "Init";
	state_id = 1;
	state_machine::State *stateSecond = new state_machine::State(state_name, state_id);
	std::shared_ptr<state_machine::State> stateSecondPtr(stateSecond);

	// Set child
	stateFirstPtr->setChild(stateSecondPtr);

	// Set callbacks
	std::function<void(const std::string&)> _f = &foo;
	stateFirstPtr->setCallbackEntry(_f);
	stateFirstPtr->setCallbackExit(_f);
	stateFirstPtr->setCallbackUpdate(_f);

	std::ostringstream oss;
	std::streambuf* p_cout_streambuf = std::cout.rdbuf();
	std::cout.rdbuf(oss.rdbuf());

	stateFirstPtr->onEntry();
	std::cout.rdbuf(p_cout_streambuf); // restore
	ASSERT_TRUE(oss && oss.str() == "Test output") << "onEntry should show Test output";

	stateFirstPtr->onUpdate();
	std::cout.rdbuf(p_cout_streambuf); // restore
	ASSERT_TRUE(oss && oss.str() == "Test output") << "onUpdate should show Test output";

	stateFirstPtr->onExit();
	std::cout.rdbuf(p_cout_streambuf); // restore
	ASSERT_TRUE(oss && oss.str() == "Test output") << "onExit should show Test output";
}

TEST(TestSuite, SetKey){

	std::string state_name;
	uint64_t state_id;

	state_name = "Start";
	state_id = 0;
	state_machine::State *stateFirst = new state_machine::State(state_name, state_id);
	std::shared_ptr<state_machine::State> stateFirstPtr(stateFirst);

	ASSERT_STREQ(stateFirstPtr->getEnteredKey().c_str(), "") << "entered_key should be " << "";
	std::string key = "newKey";
	stateFirstPtr->setEnteredKey(key);
	ASSERT_STREQ(stateFirstPtr->getEnteredKey().c_str(), key.c_str()) << "entered_key should be " << key;
}

TEST(TestSuite, TestTransitionMap){

	std::string state_name;
	uint64_t state_id;
	std::string key;
	uint64_t value;
	std::map<std::string, uint64_t> transition_map;

	state_name = "Start";
	state_id = 0;
	state_machine::State *stateFirst = new state_machine::State(state_name, state_id);
	std::shared_ptr<state_machine::State> stateFirstPtr(stateFirst);

	transition_map = stateFirstPtr->getTransitionMap();
	ASSERT_EQ(transition_map[""], 0) << "Transition value should be" << 0;

	key = "newTransition";
	value = 0;

	stateFirstPtr->addTransition(key, value);
	ASSERT_EQ(stateFirstPtr->getTansitionVal(key), value) << "Transition value for key : " << key << " should be " << value;

	transition_map = stateFirstPtr->getTransitionMap();
	ASSERT_EQ(transition_map[key], value) << "Transition value should be" << value;
}

//int main(int argc, char **argv) {
//	testing::InitGoogleTest(&argc, argv);
//	ros::init(argc, argv, "TestNode");
//	return RUN_ALL_TESTS();
//}

