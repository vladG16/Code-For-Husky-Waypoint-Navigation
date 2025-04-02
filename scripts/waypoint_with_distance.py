import matplotlib.pyplot as plt
import csv

times, distances = [], []
with open("/tmp/husky_distance.csv", "r") as f:
    next(f)  # skip header
    for row in csv.reader(f):
        times.append(float(row[0]))
        distances.append(float(row[1]))

plt.plot(times, distances)
plt.xlabel("Time (s)")
plt.ylabel("Distance (m)")
plt.title("Husky Distance Traveled Over Time")
plt.grid(True)
plt.show()
