cmake_minimum_required(VERSION 2.8.3)
project(speech_recognition_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  FILES SpeechRecognitionCandidates.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs message_runtime
)
