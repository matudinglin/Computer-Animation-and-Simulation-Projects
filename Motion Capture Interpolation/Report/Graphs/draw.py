import matplotlib.pyplot as plt

# Open the txt file
with open('test.txt', 'r') as f:
    # Read each line and split the values by comma
    lines = [line.strip().split(',') for line in f.readlines()]

# Convert the values to integers and floats
frame_numbers = [int(line[0]) for line in lines]
angles = [float(line[1]) for line in lines]

# Draw the graph
plt.plot(frame_numbers, angles)
plt.xlabel('Frame Number')
plt.ylabel('Angle (degrees)')
plt.title('Angle vs Frame Number')
plt.show()