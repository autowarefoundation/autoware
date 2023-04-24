#!/bin/bash

topics=$(ros2 topic list | grep localization)

for topic in $topics; do
    info=$(ros2 topic info $topic)
    publishers=$(echo "$info" | grep -oP 'Publisher count:\s+\K\d+')
    subscribers=$(echo "$info" | grep -oP 'Subscription count:\s+\K\d+')
    echo "$topic $publishers $subscribers"

    if [ $publishers -eq 0 ] && [ $subscribers -gt 0 ]; then
        echo "  Topic with subscribers but no publishers: $topic $publishers $subscribers"
        ros2 topic info $topic -v 
    fi
done
