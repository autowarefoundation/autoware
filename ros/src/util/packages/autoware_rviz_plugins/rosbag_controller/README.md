## Status
**DEVELOPMENT**

## Description
rivz plugin for rosbag recording (*rosbag play module will be added in the future*)

Two ways to set topics parameter for recording:
1. Use .yaml configure file as ***configure_example.yaml*** file.
2. Use ***Topic Refresh*** to real-time scan which topics are existing, then choose those you are interested in by checkboxes.

**Note1**: Configure file and topic from checkboxes can be used at same time, even some topics are repeated (plugin will do duplicate checking when using both of them)  
**Note2**: If you want to clear configure file, you can restart plugin

## Todos
- [ ] Tests

## Steps to Test
1. Start rviz and choose **Panels->Add New Panel->RosbagController**
2. You can use the plugin to record rosbags with specified topics (do not specify any topics means recording all)
