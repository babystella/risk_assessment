from cProfile import label
from matplotlib import projections
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import csv
import numpy as np

# True
L1_high_mu, L1_high_sigma = 0, 0.5
L1_medium_mu, L1_medium_sigma = 1.7, 0.75
L1_low_mu, L1_low_sigma = 5, 1.8

L2_high_mu, L2_high_sigma = 0, 1.2
L2_medium_mu, L2_medium_sigma = 3, 1
L2_low_mu, L2_low_sigma = 5, 1.4

L3_high_mu, L3_high_sigma = 0, 2.8
L3_medium_mu, L3_medium_sigma = 2.9, 0.6
L3_low_mu, L3_low_sigma = 3.3, 0.2

H_w = 50
M_w = 10
L_w = 5

def gaussian(x, mu, sig):
    return np.exp(-np.power(x - mu, 2.) /(2 * np.power(sig, 2.)))

def dis2Score(Level, disSample):
    if Level == 1:        
        degreeH = gaussian(disSample, L1_high_mu, L1_high_sigma)
        degreeM = gaussian(disSample, L1_medium_mu, L1_medium_sigma)
        degreeL = gaussian(disSample, L1_low_mu, L1_low_sigma)
        singleScore = (degreeH*H_w + degreeM*M_w + degreeL*L_w)*(Level*0.5+0.5)
    elif Level == 2: 
        degreeH = gaussian(disSample, L2_high_mu, L2_high_sigma)
        degreeM = gaussian(disSample, L2_medium_mu, L2_medium_sigma)
        degreeL = gaussian(disSample, L2_low_mu, L2_low_sigma)
        singleScore = (degreeH*H_w + degreeM*M_w + degreeL*L_w)*(Level*0.5+0.5)
    elif Level == 3: 
        degreeH = gaussian(disSample, L3_high_mu, L3_high_sigma)
        degreeM = gaussian(disSample, L3_medium_mu, L3_medium_sigma)
        degreeL = gaussian(disSample, L3_low_mu, L3_low_sigma)
        singleScore = (degreeH*H_w + degreeM*M_w + degreeL*L_w)*(Level*0.5+0.5)
    return singleScore
    
def test():
    #2and3bk
    #markerPosition = (2.36028166744, -1.10185392408)
    #3_best
    markerPosition = (2.34881257286, -0.0303440786647)
    markerPosition2 = (2.8, 0.4)
    # hand setting: 2.639 -0.145
    # field setting: 2.34881257286, -0.0303440786647
    
    x = []
    y = []
    exp_score = []
    
    # Open csv reader
    f_exp = open("./exp_data/experiment_2obj.csv",'r')
    exp_reader = csv.reader(f_exp)
    for row in exp_reader:
        #if float(row[2]) == 0:
        #    continue
        #else:
        x.append(float(row[0]))
        y.append(float(row[1]))
        exp_score.append(float(row[2]))

    # Theoretical score
    disL1Sample_1obj = np.sqrt(np.power((np.array(x) - markerPosition[0]),2) + np.power((np.array(y) - markerPosition[1]),2))
    disL1Sample_2obj = np.sqrt(np.power((np.array(x) - markerPosition[0]),2) + np.power((np.array(y) - markerPosition2[1]),2))
    #print(disL1Sample_1obj)
    singleScore = dis2Score(1, disL1Sample_1obj)
    singleScore2 = dis2Score(2, disL1Sample_2obj)
    singleScore += singleScore2
    
    # Calculate RMSE
    theory_value = np.array(singleScore)
    experiment_value = np.array(exp_score)
    RMSError = np.sqrt(np.average(np.power(theory_value-experiment_value, 2.)))
    print("RMSE:", RMSError,"Range: %f to %f" % (np.min(theory_value), np.max(theory_value)))
    
    ax = plt.axes(projection='3d')
    
    ax.set_xlim(-3, 3)    
    ax.set_ylim(-1, 1)
    xtic = np.linspace(-2.5, 2.5, 5)
    ytic = np.linspace(-1, 1, 5)
    ax.set_xticks(xtic)
    ax.set_yticks(ytic)

    ax.scatter3D(x,y,singleScore, c="orange", depthshade=True, label='theory', s=80, marker='^')
    ax.scatter3D(x,y,exp_score, c="r", depthshade=True, label='experiment',s=80, marker='o')
    ax.plot(xs=x, ys=y, c="g", label='path')
    ax.scatter(2.19935,0.187645, c="g", label='sign1', s=80, marker='*')
    ax.scatter(2.7, 0.3, c="r", label='sign2', s=80, marker='*')
    
    ax.tick_params(labelsize=14) 
    ax.set_xlabel('map X', fontsize=18)
    ax.set_ylabel('map Y', fontsize=18)
    ax.set_zlabel('risk score', fontsize=18)
    ax.set_title('Two HAZMATs sign', fontsize=18)
    
    handles,labels = ax.get_legend_handles_labels()

    handles = [handles[1], handles[2], handles[0],handles[3],handles[4]]
    labels = [labels[1], labels[2], labels[0],labels[3],labels[4]]
    
    ax.legend(handles,labels,fontsize=14, loc="upper right")
    
    plt.show()
    
        
def main():
    test()

if __name__== "__main__":
    main()
