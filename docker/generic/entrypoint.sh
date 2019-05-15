#!/bin/bash

set -e

DEFAULT_USER_ID=1000

#
# Ensure host and container have the same user ID. This is to allow both sides
# to read and write the shared directories.
#
if [ "$USER_ID" != "$DEFAULT_USER_ID" ]; then
    echo "Changing autoware user ID to match your host's user ID ($USER_ID)." 
    echo "This operation can take a while..."

    usermod --uid $USER_ID autoware

    # Ensure all files in the home directory are owned by the new user ID
    find /home/autoware -user $DEFAULT_USER_ID -exec chown -h $USER_ID {} \;
fi

# Start an interactive shell using user 'autoware'
sudo -iu autoware
