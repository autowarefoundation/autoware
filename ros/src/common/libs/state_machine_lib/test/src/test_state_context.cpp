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
	state_machine::StateContext stateCtx(file_name, msg_name);

	// Check generated dot file
	std::ifstream generatedDotFile("/tmp/"+msg_name+".dot");
	std::stringstream dotFile;
	dotFile << generatedDotFile.rdbuf();

	std::ifstream referenceDotFile("reference.dot");
	std::stringstream referenceDot;
	referenceDot << referenceDotFile.rdbuf();

	ASSERT_STREQ(dotFile.str().c_str(), referenceDot.str().c_str()) << "The generated dot file should be " << referenceDot.str();

	// Check start state
	std::shared_ptr<state_machine::State> start_state;
	start_state = stateCtx.getStartState();
	std::string testStr;
	testStr = "Start\n";
	ASSERT_STREQ(start_state->getStateName().c_str(), "Start") << "First state should be Start";
	ASSERT_STREQ(stateCtx.getStateText().c_str(), testStr.c_str()) << "Text should be " << testStr;
	ASSERT_STREQ(stateCtx.getAvailableTransition().c_str(), "started:Init,") << "Available transition should be: started:Init,";
}

TEST(TestSuite, ChangeStates){

	std::string file_name = "testStates.yaml";
	std::string msg_name = "test_states";
	state_machine::StateContext stateCtx(file_name, msg_name);

	stateCtx.nextState("started");
	ASSERT_STREQ(stateCtx.getStateText().c_str(), "Init\n") << "Text should be: Init\n";
	ASSERT_STREQ(stateCtx.getAvailableTransition().c_str(), "init_start:Intermediate,") << "Available transition should be: init_start:Intermediate,";
}


