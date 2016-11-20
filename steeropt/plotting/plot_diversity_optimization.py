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
    
    index_state = data[0][0]
    number_of_members=0
    for row in data:
        if row[0] == index_state:
            number_of_members+=1
        else:
            break
        
    data_=result = [list([]) for _ in xrange(number_of_members)] 
    for row in range(len(data)):
        ind = row % number_of_members
        # print ind
        data_[ind].append(data[row])
        
    # print "New data_ length: " + str(len(data_[0])) + " data length: " + str(len(data))
    # print "New data_: " + str(np.array(data_[0]))
        
    print "Number of Members: " + str(number_of_members)
    # plt.plot(range(data.shape[0]), data[:,2])
    _fig, (_fval, _metric, _diversity) = plt.subplots(3, 1, sharey=False)
    member_index=0
    for member in data_:
        member = np.array(member)
        _fval.plot(range(member.shape[0]), member[:,1], label="member: " + str(member_index))
        # _fval.plot(range(member.shape[0]), member[:,2], c='r')
        _fval.set_title("Diversity Optimization Convergence for " + str(number_of_members) + " Members")
        _fval.set_ylabel("Metric Value")
        # _fval.set_xlabel("Iterations")
        member_index+=1
    _fval.legend(loc="upper right",
       ncol=2, shadow=True, fancybox=True)
    member_index=0
    for member in data_:
        member = np.array(member)    
        _metric.plot(range(member.shape[0]), member[:,3], label="member: " + str(member_index))
        # _metric.set_title("Metric (Degree) Convergence")
        _metric.set_ylabel("Objective")
        # _metric.set_xlabel("Iterations")
        member_index+=1
    _metric.legend(loc="lower right",
       ncol=2, shadow=True, fancybox=True)
    member_index=0
    for member in data_:
        member = np.array(member)  
        _diversity.plot(range(member.shape[0]), member[:,4], label="member: " + str(member_index))
        # _diversity.set_title("Diversity Convergence")
        _diversity.set_ylabel("Diversity")
        # _diversity.set_xlabel("Iterations")
        member_index+=1
    # plt.legend(shadow=True, fancybox=True)
    _diversity.legend(loc="lower right",
       ncol=2, shadow=True, fancybox=True)

if __name__ == "__main__":

    
    plot_optimization(sys.argv[1])
    plt.show()