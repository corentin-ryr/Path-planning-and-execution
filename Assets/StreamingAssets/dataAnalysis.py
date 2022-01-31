import numpy as np
import matplotlib.pyplot as plt

fileName = "export.csv"

scenario = ""
with open(fileName) as f:
    scenario = f.readline().split()[0]

data = np.genfromtxt(fileName, delimiter=",", skip_header=2)
data = data[data[:, 0] > 2] #Delete the first 2 seconds since we wait for the car to drop on the floor
data[:,0] = data[:,0] - 2
# print(np.var(data[1:,0] - data[:-1, 0])) # Check if the samples are equally separated in time

if scenario == "accelerationProfile":
    plt.title("Speed as a function of time")
    plt.plot(data[:, 0], data[:, 1])
    plt.show()
    
    accel = np.gradient(data[:, 1])
    plt.title("Acceleration as a function of time")
    plt.plot(data[:, 1], accel)
    plt.show()
    
    
if scenario == "speedStability":
    
    data = data[data[:, 0] > 10] #Delete the first 5 seconds because we want to cruising state
    print("Speed variance: " + str(np.var(data[:,1])))
    plt.title("Speed as a function of time")
    plt.plot(data[:, 0], data[:, 1])
    plt.show()

if scenario == "responseTime":
    targetSpeed = ""
    with open(fileName) as f:
        targetSpeed = float(f.readline().split()[1])
        
    responseTime = data[data[:, 1] > (0.9 * targetSpeed), 0][0]
    print("Response time: " + str(responseTime) + "s") 
    plt.title("Speed as a function of time")
    plt.vlines(responseTime, 0, targetSpeed, "r")
    plt.plot(data[:, 0], data[:, 1])
    plt.show()
    
