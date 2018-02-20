#!/usr/bin/env bash
echo "Pinging target ($1)..."
ping -c 4 $1 -q
echo "Uploading center csv files to $1"
scp -r ./center lvuser@$1:/home/lvuser/paths
echo "Uploading left csv files to $1"
scp -r ./left lvuser@$1:/home/lvuser/paths
echo "Uploading right csv files to $1"
scp -r ./right lvuser@$1:/home/lvuser/paths
