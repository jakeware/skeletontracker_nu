#!/bin/bash

# Define bag and csv filename:
filename=default
input=$1
if [ -n "$input" ] 
then
    filename=$1
fi

echo "Using '${filename}' as filename"

# If default.* exists, move to backup#.*
if [ -a ${filename}.bag ]
then
    num=`ls | grep "${filename}.*bag" | wc -l`
    mv ${filename}.txt ${filename}_backup_"$num".txt
    mv ${filename}.bag ${filename}_backup_"$num".bag
fi

# Wait for a user to press a button:
echo "First get desired users tracking..."
sleep 2
echo "Then press any button to start recording data..."
read -n 1 -s
echo "Beginning recording..."
# Start recording:
rosbag record --quiet -O ${filename}.bag /skeletons &
sleep 1
echo "Now press any button to stop recording..."
read -n 1 -s
echo "Stopping recording, now killing recording process..."
# Stop recording:
killall -2 record
echo "Generating csv file..."
# Generate csv file:
rostopic echo -p -b ${filename}.bag /skeletons > ${filename}.txt

echo "Done creating bag and csv file"
