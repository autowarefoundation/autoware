#!/bin/sh
while :
do
    rosservice call /sample_topic_buffer_server/update "/chatter"
    sleep 0.5
done
