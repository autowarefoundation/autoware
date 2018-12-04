/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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

#include <fstream>
#include <unistd.h>
#include <dirent.h>

#include "state_machine_lib/state_context.hpp"

class TestSuite: public ::testing::Test {
public:
	TestSuite(){}
	~TestSuite(){}
};

TEST(TestSuite, StateContextConstructor){

	std::string file_name = "testStates.yaml";
	std::string msg_name = "test_states";
	state_machine::StateContext state_context(file_name, msg_name);

	// Check generated dot file
	std::ifstream generated_dot_file("/tmp/"+msg_name+".dot");
	std::stringstream dot_file_string;
	dot_file_string << generated_dot_file.rdbuf();

	std::ifstream reference_dot_file("reference.dot");
	std::stringstream reference_dot_file_string;
	reference_dot_file_string << reference_dot_file.rdbuf();

	ASSERT_STREQ(dot_file_string.str().c_str(), reference_dot_file_string.str().c_str()) << "The generated dot file should be " << reference_dot_file_string.str();

	// Check start state
	std::shared_ptr<state_machine::State> start_state;
	start_state = state_context.getStartState();
	std::string test_string;
	test_string = "Start\n";
	ASSERT_STREQ(start_state->getStateName().c_str(), "Start") << "First state should be Start";
	ASSERT_STREQ(state_context.getStateText().c_str(), test_string.c_str()) << "Text should be " << test_string;
	ASSERT_STREQ(state_context.getAvailableTransition().c_str(), "started:Init,") << "Available transition should be: started:Init,";
}

TEST(TestSuite, ChangeStates){

	std::string file_name = "testStates.yaml";
	std::string msg_name = "test_states";
	state_machine::StateContext state_context(file_name, msg_name);

	state_context.nextState("started");
	ASSERT_STREQ(state_context.getStateText().c_str(), "Init\n") << "Text should be: Init\n";
	ASSERT_STREQ(state_context.getAvailableTransition().c_str(), "init_start:Intermediate,") << "Available transition should be: init_start:Intermediate,";
}

void auxFunc2(const std::string&){
	std::cout << "Test output";
};

TEST(TestSuite, SetCallbacksStateContext){

	std::string file_name = "testStates.yaml";
	std::string msg_name = "test_states";
	state_machine::StateContext state_context(file_name, msg_name);

	// Set callbacks
	std::function<void(const std::string&)> _f = &auxFunc2;
	ASSERT_TRUE(state_context.setCallback(state_machine::CallbackType::UPDATE, "Start", _f));
	ASSERT_TRUE(state_context.setCallback(state_machine::CallbackType::EXIT, "Start", _f));

	std::ostringstream oss;
	std::streambuf* p_cout_streambuf = std::cout.rdbuf();
	std::cout.rdbuf(oss.rdbuf());

	state_context.onUpdate();
	std::cout.rdbuf(p_cout_streambuf); // restore
	ASSERT_TRUE(oss && oss.str() == "Test output") << "onUpdate should show 'Test output'";

	// Set Callback for unexisting state
	ASSERT_TRUE(!state_context.setCallback(state_machine::CallbackType::UPDATE, "NoState", _f)) << "Should be false";
}


