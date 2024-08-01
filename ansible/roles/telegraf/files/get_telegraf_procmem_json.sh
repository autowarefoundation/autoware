#!/bin/bash

echo "{"
ps -ax --format "rss command" |
    while read -r rss cmd; do
        if [[ $rss -lt 30000 ]]; then
            continue
        fi
        cmd="${cmd// /_}"
        cmd="${cmd//=/_}"
        cmd="${cmd:0:50}"
        echo "\"${cmd}\":${rss},"
    done
echo '"z":0'
echo "}"
