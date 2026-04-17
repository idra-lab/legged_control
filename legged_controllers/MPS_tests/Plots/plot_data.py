import numpy as np
import csv
import matplotlib.pyplot as plt
import os

def plot_one(bins, weights, title, x_label, total_tests, inv):
    print(title)
    percentages = np.zeros(len(weights))
    if inv:
        for i in range(len(weights)):
            percentages[i] = ((total_tests - weights[i])*100)/total_tests
        print(total_tests - weights)
    else:
        for i in range(len(weights)):
            percentages[i] = (weights[i]*100)/total_tests
        print(weights)
    plt.figure()
    plt.hist(bins[:-1], bins, weights = percentages, edgecolor='black')
    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel('Percentage of occurrence (%)')
    plt.ylim([0, 100])
    
def plot_three(bins, comparison, weightsB, weightsC, labelA, labelB, labelC, title, x_label, total_tests, inv):
    print(title)
    print('b',total_tests)
    percentages_comp = np.zeros(len(comparison))
    percentages_B = np.zeros(len(comparison))
    percentages_C = np.zeros(len(comparison))
    if inv:
        for i in range(len(comparison)):
            percentages_comp[i] = ((total_tests - comparison[i])*100)/total_tests
            percentages_B[i]   = ((total_tests - weightsB[i])*100)/total_tests
            percentages_C[i]   = ((total_tests - weightsC[i])*100)/total_tests

        print('total_tests - comparison',total_tests - comparison)
        print('total_tests - weightsB',total_tests - weightsB)
        print('total_tests - weightsC',total_tests - weightsC)
        print('B - comp', (total_tests - weightsB) - (total_tests - comparison))
        print('C - comp', (total_tests - weightsC) - (total_tests - comparison))
        print('total',total_tests - comparison + (total_tests - weightsB) + (total_tests - weightsC))
        
    else:
        for i in range(len(comparison)):
            percentages_comp[i] = (comparison[i]*100)/total_tests
            percentages_B[i]   = (weightsB[i]*100)/total_tests
            percentages_C[i]   = (weightsC[i]*100)/total_tests

        print('comparison',comparison)
        print('weightsB',weightsB)
        print('weightsC',weightsC)
        print('B - comp', (weightsB) - (comparison))
        print('C - comp', (weightsC) - (comparison))
        print('total', comparison + (weightsB-comparison) + (weightsC-comparison))

    plt.figure()
    plot_A = plt.hist(bins[:-1], bins, weights = percentages_comp, label = labelA, edgecolor='black')
    plot_B = plt.hist(bins[:-1], bins, weights = percentages_B - percentages_comp, label = labelB, bottom = plot_A[0], edgecolor='black')
    plt.hist(bins[:-1], bins, weights = percentages_C - percentages_comp, label = labelC, bottom = plot_B[0] + plot_A[0], edgecolor='black')
    plt.legend(prop={'size': 35})
    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel('Percentage of occurrence (%)')
    plt.ylim([0, 100])
    
def plot_two(bins, comparison, weightsB, weightsC, labelB, labelC, title, x_label, total_tests, inv):
    percentages_comp = np.zeros(len(comparison))
    percentages_B = np.zeros(len(comparison))
    percentages_C = np.zeros(len(comparison))
    print(title)
    print('a',total_tests)
    if inv:
        for i in range(len(comparison)):
            percentages_comp[i] = ((total_tests - comparison[i])*100)/total_tests
            percentages_B[i]   = ((total_tests - weightsB[i])*100)/total_tests
            percentages_C[i]   = ((total_tests - weightsC[i])*100)/total_tests
        print('total_tests - comparison',total_tests - comparison)
        print('total_tests - weightsB',total_tests - weightsB)
        print('total_tests - weightsC',total_tests - weightsC)
        print('B - comp', (total_tests - weightsB) - (total_tests - comparison))
        print('C - comp', (total_tests - weightsC) - (total_tests - comparison))
        print('total',total_tests - comparison + (total_tests - weightsB) - (total_tests - weightsC))
    else:
        for i in range(len(comparison)):
            percentages_comp[i] = (comparison[i]*100)/total_tests
            percentages_B[i]   = (weightsB[i]*100)/total_tests
            percentages_C[i]   = (weightsC[i]*100)/total_tests
        print('comparison',comparison)
        print('weightsB',weightsB)
        print('weightsC',weightsC)
        print('B - comp', (weightsB) - (comparison))
        print('C - comp', (weightsC) - (comparison))
        print('total', comparison + (weightsB-comparison) + (weightsC-comparison))

    print('percentages_B - percentages_comp', percentages_B - percentages_comp)
    print('percentages_C - percentages_comp', percentages_C - percentages_comp)

    plt.figure()
    plot_A = plt.hist(bins[:-1], bins, weights = percentages_C, label = 'Backup policy was used and the robot could be stopped', edgecolor='black')
    plot_B = plt.hist(bins[:-1], bins, weights = percentages_B - percentages_C, label = labelB, bottom = plot_A[0], edgecolor='black')
    plt.legend(prop={'size': 35})
    plt.title(title)
    plt.xlabel(x_label)
    plt.ylabel('Percentage of occurrence (%)')
    plt.ylim([0, 100])
    
def list_str_to_int(list_str):
    list_str = list_str.replace('[', '')
    list_str = list_str.replace(']', '')
    list_str = list_str.replace('.', '')
    list_split = list(map(str.strip, list_str.split()))
    list_int = [int(x) for x in list_split]
    return list_int

##### Read Force Data
plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
plt.rcParams["font.size"] = "35"

full_path = os.path.realpath(__file__)
force_file = os.path.dirname(full_path) + "/plot_forces_no_mps_150_200.csv"

N = 1000

force_mag  = [ 150, 200]
stop_force_list = []
backup_force_list = [] 
force_backup_stop_list = []
fall_force_list = []
knee_force_list = []
force_fall_knee_list = []

total_falls = 0
total_backup = 0
total_knees = 0
total_stop = 0
total_backup_stop = 0
no_backup = 0
total_limits = 0
total_limits_or_falls = 0
total_knees_or_falls = 0
total_pos_limits = 0
total_vel_limits = 0

with open(force_file,'r') as csvfile:
    lines = csv.reader(csvfile, delimiter=',')
    for row in lines: 
        stop_force_list.append(list_str_to_int(row[1]))
        backup_force_list.append(list_str_to_int(row[2])) 
        force_backup_stop_list.append(list_str_to_int(row[3]))
        fall_force_list.append(list_str_to_int(row[4]))
        knee_force_list.append(list_str_to_int(row[5]))
        force_fall_knee_list.append(list_str_to_int(row[6]))
csvfile.close()

stop_force = stop_force_list[0]
backup_force = backup_force_list[0] 
force_backup_stop = force_backup_stop_list[0]
fall_force = fall_force_list[0]
knee_force = knee_force_list[0]
force_fall_knee = force_fall_knee_list[0]

for i in range(1, len(stop_force_list)):
    stop_force = np.add(stop_force, stop_force_list[i])
    backup_force = np.add(backup_force, backup_force_list[i])
    force_backup_stop = np.add(force_backup_stop, force_backup_stop_list[i])
    fall_force = np.add(fall_force, fall_force_list[i])
    knee_force = np.add(knee_force, knee_force_list[i])
    force_fall_knee = np.add(force_fall_knee, force_fall_knee_list[i])
    
#### Read Iter Data


total_tests = N #* len(force_mag)
force_iter = [x for x in range(1,91)]



# Histograms force

bins = np.zeros(len(force_mag) + 1)
bins[1:] = [x + 2.5 for x in force_mag]
bins[0] = bins[1] - 5
x_label = 'Force magnitude (N)'

#'''
plot_one(bins, stop_force, 'Times the robot stopped', x_label, total_tests, True)

plot_one(bins, backup_force, 'Times the backup policy was used', x_label, total_tests, True)

labelB = 'Backup policy was used'
labelC = 'The robot stopped'
title = 'Comparison between the times the backup policy was used and the robot stopped'
plot_two(bins, force_backup_stop, backup_force, stop_force, labelB, labelC, title, x_label, total_tests, True)#'''

#'''
plot_one(bins, fall_force, 'Times the robot fell', x_label, total_tests, False)

plot_one(bins, knee_force, 'Times at least one knee touched the floor', x_label, total_tests, False)

labelA = 'Robot fell and at least one knee touched the floor'
labelB = 'Robot fell'
labelC = 'At least one knee touched the floor'
title = 'Comparison between the times the robot fell and at least one knee touched the floor'
plot_three(bins, force_fall_knee, fall_force, knee_force, labelA, labelB, labelC, title, x_label, total_tests, False)


# Histograms random iteration

bins = np.zeros(len(force_iter) + 1)
bins[1:] = [x + 0.5 for x in force_iter]#force_iter
bins[0] = bins[1] - 0.5
x_label = 'Iteration in which the force was applied'


for i in fall_force:
    total_falls += i

for i in backup_force:
    total_backup += (N - i)
    no_backup += i

for i in stop_force:
    total_stop += (N - i)

for i in force_backup_stop:
    total_backup_stop += (N - i)

for i in knee_force:
    total_knees += i


total_limits_or_falls = total_limits
total_limits = total_limits + (total_vel_limits - total_limits) + (total_pos_limits - total_limits)

for i in force_fall_knee:
    total_knees_or_falls += i



knees_and_falls = total_knees_or_falls
total_knees_or_falls = total_knees_or_falls + (total_falls - total_knees_or_falls) + (total_knees - total_knees_or_falls)

print('total_falls',total_falls, 'percentage', total_falls*100/(N*len(force_mag)), '%')
print('total_limits',total_limits, 'percentage', total_limits*100/(N*len(force_mag)), '%')
print('total_backup',total_backup, 'percentage', total_backup*100/(N*len(force_mag)), '%')
print('total_stop',total_stop, 'percentage', total_stop*100/(N*len(force_mag)), '%')
print('total_knees',total_knees, 'percentage', total_knees*100/(N*len(force_mag)), '%')
print('knees_and_falls',knees_and_falls, 'percentage', knees_and_falls*100/(N*len(force_mag)), '%')
print('total_limits_or_falls',total_limits_or_falls, 'percentage', total_limits_or_falls*100/(N*len(force_mag)), '%')

print('total_knees_or_falls',total_knees_or_falls, 'percentage', total_knees_or_falls*100/(N*len(force_mag)), '%')
print('total_pos_limits',total_pos_limits, 'percentage', total_pos_limits*100/(N*len(force_mag)), '%')
print('total_vel_limits',total_vel_limits, 'percentage', total_vel_limits*100/(N*len(force_mag)), '%')
print('no_backup', no_backup)
print('no_backup', no_backup)
print('total - no_backup', N*len(force_mag) - no_backup)
print(N*len(force_mag))


plt.show()