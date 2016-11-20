import numpy as np
import matplotlib.pyplot as plt
import sys
# sys.path.append("../../steerstats/")
# from util import readCSVToMatrix

import csv
from decimal import _div_nearest

def readCSVToMatrix(csvfile, delimiter=','):
    """
        cvsfile should be an open readable file
    """
    spamreader = csv.reader(csvfile, delimiter=delimiter)
    data=[]
    try:
        for row in spamreader:
            tmp_data=[]
            for item in row:
                # print item
                tmp_data.append(float(item))
            data.append(tmp_data)
        # print "Data: " + str(data)
        return np.array(data, dtype='d')
    except ValueError as err:
        print "item can not be converted: " + str(item)
        print data
        raise err

def plot_optimization(fileName):
    """
    
    """
    f = open(fileName, 'r')
    lables = f.readline()
    # print "Labels: " + str(lables)
    data = readCSVToMatrix(f, delimiter=' ')
    # print data
    
    # plt.plot(range(data.shape[0]), data[:,2])
    # _fig, (_fval, _metric, _diversity) = plt.subplots(3, 1, sharey=False)

    plt.plot(data[:,0], data[:,1], label="metric")
    # plt.plot(range(member.shape[0]), member[:,2], c='r')
    plt.title("Optimization Convergence")
    plt.ylabel("Metric Value")
    # plt.set_xlabel("Iterations")
        
    plt.plot(data[:,0], data[:,3], label="degree: ")
    # plt.set_title("Metric (Degree) Convergence")
    # plt.set_xlabel("Iterations")

    plt.plot(data[:,0], data[:,4], label="depth: ")
    plt.plot(data[:,0], data[:,5], label="entropy: ")
    # plt.plot(data[:,0], data[:,6], label="clearance: ")
    # plt.plot(data[:,0], data[:,7], label="alignment: ")
    # plt.set_title("Diversity Convergence")
    plt.ylabel("Metric")
    # plt.set_xlabel("Iterations")
    # plt.legend(shadow=True, fancybox=True)
    plt.legend(loc="lower right",
       ncol=2, shadow=True, fancybox=True)

if __name__ == "__main__":

    
    plot_optimization(sys.argv[1])
    plt.show()