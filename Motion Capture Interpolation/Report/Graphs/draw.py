import matplotlib.pyplot as plt


inputFileName = "input-root-Z-200-500-40.txt"
output1FileName = "BezierEuler-root-Z-200-500-40.txt"
output2FileName = "BezierQuaternion-root-Z-200-500-40.txt"
title = "Graph #4 compares Bezier Euler to Bezier SLERP quaternion"


# read data
with open(inputFileName, 'r') as f:
    inputData = [line.strip().split(',') for line in f.readlines()]

with open(output1FileName, 'r') as f:
    output1Data = [line.strip().split(',') for line in f.readlines()]

with open(output2FileName, 'r') as f:
    output2Data = [line.strip().split(',') for line in f.readlines()]

# prepare data
inputFrames = [int(line[0]) for line in inputData]
inputAngles = [float(line[1]) for line in inputData]
output1Frames = [int(line[0]) for line in output1Data]
output1Angles = [float(line[1]) for line in output1Data]
output2Frames = [int(line[0]) for line in output2Data]
output2Angles = [float(line[1]) for line in output2Data]

# draw data
plt.plot(inputFrames, inputAngles, label=str(inputFileName.split("-")[0]))
plt.plot(output1Frames, output1Angles, label=str(output1FileName.split("-")[0]))
plt.plot(output2Frames, output2Angles, label=str(output2FileName.split("-")[0]))
plt.xlabel('Frame Number')
plt.ylabel('Angle (degrees)')
plt.title(title)
plt.legend();
plt.show()