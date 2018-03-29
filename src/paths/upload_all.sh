#!/usr/bin/env bash
echo "Pinging target ($1)..."
ping -c 4 $1 -q
echo "Uploading center Pathfinder csv files to $1"
scp -r ./finder/center/ lvuser@$1:/home/lvuser/paths_finder/
echo "Uploading left Pathfinder csv files to $1"
scp -r ./finder/left/ lvuser@$1:/home/lvuser/paths_finder/
echo "Uploading right Pathfinder csv files to $1"
scp -r ./finder/right/ lvuser@$1:/home/lvuser/paths_finder/
