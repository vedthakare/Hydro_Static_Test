#!/usr/bin/env python3

"""
This script saves each topic in a bagfile as a csv.

Accepts a filename as an optional argument. Operates on all bagfiles in current directory if no argument provided.

Usage1 (for one bag file):
    python bag2csv.py filename.bag
Usage 2 (for all bag files in current directory):
    python bag2csv.py

Written by Nick Speal in May 2013 at McGill University's Aerospace Mechatronics Laboratory.
Bugfixed by Marc Hanheide June 2016.
Updated for Python 3 compatibility by ChatGPT.
"""

import rosbag
import sys
import csv
import time
import os
import shutil

# Verify correct input arguments: 1 or 2
if len(sys.argv) > 2:
    print(f"Invalid number of arguments: {len(sys.argv)}")
    print("Should be 2: 'bag2csv.py' and 'bagName'")
    print("Or just 1  : 'bag2csv.py'")
    sys.exit(1)
elif len(sys.argv) == 2:
    listOfBagFiles = [sys.argv[1]]
    numberOfFiles = 1
    print(f"Reading only 1 bagfile: {listOfBagFiles[0]}")
elif len(sys.argv) == 1:
    listOfBagFiles = [f for f in os.listdir(".") if f.endswith(".bag")]
    numberOfFiles = len(listOfBagFiles)
    print(f"Reading all {numberOfFiles} bagfiles in current directory:\n")
    for f in listOfBagFiles:
        print(f)
    print("\nPress Ctrl+C in the next 10 seconds to cancel\n")
    time.sleep(10)
else:
    print(f"Bad argument(s): {sys.argv}")
    sys.exit(1)

count = 0
for bagFile in listOfBagFiles:
    count += 1
    print(f"Reading file {count} of {numberOfFiles}: {bagFile}")
    
    bag = rosbag.Bag(bagFile)
    bagName = bag.filename
    folder = bagName.rstrip(".bag")

    # Create output directory
    os.makedirs(folder, exist_ok=True)

    # Backup the original bag file
    shutil.copyfile(bagName, os.path.join(folder, bagName))

    # Get list of topics
    listOfTopics = []
    for topic, _, _ in bag.read_messages():
        if topic not in listOfTopics:
            listOfTopics.append(topic)

    for topicName in listOfTopics:
        filename = os.path.join(folder, topicName.replace('/', '_slash_') + '.csv')
        with open(filename, 'w', newline='') as csvfile:
            filewriter = csv.writer(csvfile, delimiter=',')
            firstIteration = True

            for _, msg, t in bag.read_messages(topicName):
                msgString = str(msg)
                msgList = msgString.split('\n')
                instantaneousListOfData = []

                for nameValuePair in msgList:
                    splitPair = nameValuePair.split(':', 1)
                    splitPair = [s.strip() for s in splitPair]
                    instantaneousListOfData.append(splitPair)

                if firstIteration:
                    headers = ["rosbagTimestamp"]
                    headers += [pair[0] for pair in instantaneousListOfData if len(pair) > 1]
                    filewriter.writerow(headers)
                    firstIteration = False

                values = [str(t)]
                values += [pair[1] for pair in instantaneousListOfData if len(pair) > 1]
                filewriter.writerow(values)

    bag.close()

print(f"Done reading all {numberOfFiles} bag file(s).")
