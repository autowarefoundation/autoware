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

void foo2(const std::string&){
	std::cout << "Test output";
};

TEST(TestSuite, SetCallbacksStateContext){


	std::string file_name = "/home/autoware/Autoware/ros/src/common/libs/state_machine_lib/testStates.yaml";
	std::string msg_name = "testStates";
	state_machine::StateContext stateCtx(file_name, msg_name);

	// Set callbacks
	std::function<void(const std::string&)> _f = &foo2;
	//	ASSERT_TRUE(stateCtx.setCallback(state_machine::CallbackType::UPDATE, "Start", _f));
	//	ASSERT_TRUE(stateCtx.setCallback(state_machine::CallbackType::ENTRY, "Start", _f));
	//	ASSERT_TRUE(stateCtx.setCallback(state_machine::CallbackType::EXIT, "Start", _f));
	//
	//	std::ostringstream oss;
	//	std::streambuf* p_cout_streambuf = std::cout.rdbuf();
	//	std::cout.rdbuf(oss.rdbuf());

	//	stateCtx.onEntry(0);
	//	std::cout.rdbuf(p_cout_streambuf); // restore
	//	ASSERT_TRUE(oss && oss.str() == "Test output") << "onEntry should show Test output";

	//	stateCtx.onUpdate();
	//	std::cout.rdbuf(p_cout_streambuf); // restore
	//	ASSERT_TRUE(oss && oss.str() == "Test output") << "onUpdate should show Test output";

	//	stateCtx.onExit();
	//	std::cout.rdbuf(p_cout_streambuf); // restore
	//	ASSERT_TRUE(oss && oss.str() == "Test output") << "onExit should show Test output";

	// Set Callback for unexisting state
	ASSERT_TRUE(stateCtx.setCallback(state_machine::CallbackType::UPDATE, "NoState", _f));
}


//TEST(TestSuite, StateContextConstructor){
//
//	std::string file_name = "/home/autoware/Autoware/ros/src/common/libs/state_machine_lib/testStates.yaml";
//	state_machine::StateContext stateCtx(file_name);
//
//	std::ifstream t1("/tmp/a.dot");
//	std::stringstream dotFile;
//	dotFile << t1.rdbuf();
//
//	std::shared_ptr<state_machine::State> start_state;
//	start_state = stateCtx.getStartState();
//	std::string testStr;
//	testStr = "Start\n";
//	ASSERT_STREQ(start_state->getStateName().c_str(), "Start") << "First state should be Start";
//	ASSERT_STREQ(stateCtx.getStateText().c_str(), testStr.c_str()) << "Text should be " << testStr;
//
//
//	std::ifstream t("/home/autoware/Autoware/ros/src/common/libs/state_machine_lib/test/reference.dot");
//	std::stringstream buffer;
//	buffer << t.rdbuf();
//
//	//	std::string getStateText();
//	//	  std::string getAvailableTransition(void);
//	//	  void showStateName();
//	//	  void nextState(const std::string& transition_key);
//
//
//
//	//	root_state->getEnteredKey()
//
//	std::cerr << "***************************************" << std::endl;
//	std::cerr << buffer.str() << std::endl;
//	//	std::cerr << root_state->getStateName() << std::endl;
//	////	std::cerr << state.getStateText() << std::endl;
//	//	std::cerr << state.getAvailableTransition() << std::endl;
//	//	state.nextState("started");
//	//	std::cerr << root_state->getStateID() << std::endl;
//	//	std::cerr << root_state->getStateName() << std::endl;
//	//	std::cerr << state.getAvailableTransition() << std::endl;
//	//	std::cerr << state.nextState("started") << std::endl;
//	std::cerr << "***************************************" << std::endl;
//
//
//	ASSERT_STREQ(buffer.str().c_str(), dotFile.str().c_str()) << "entered_key should be " << "";
//}


