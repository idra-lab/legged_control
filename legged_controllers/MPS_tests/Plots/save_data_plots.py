import numpy as np
import csv
import matplotlib.pyplot as plt
import os

def read_limit_violation(file, max_joint, min_joint):
    # Read data about joint limit violation
    try:
        dir_file = open(file)
        dir_file.close()
        
    except FileNotFoundError:
        pass
        
    else:
        with open(file,'r') as csvfile:
            lines = csv.reader(csvfile, delimiter=',')
            for row in lines:
                max_joint[int(row[2]) - 1] = list_str_to_float(row[3])
                min_joint[int(row[2]) - 1] = list_str_to_float(row[4])
    return max_joint, min_joint

def count_data(file, force_mag, force_iter, i_init, force_compare, iter_compare):
    data_force = np.zeros(len(force_mag))
    data_iter  = np.zeros(len(force_iter))
    
    force_compare_sum = np.zeros(len(force_mag))
    iter_compare_sum  = np.zeros(len(force_iter))

    try:
        dir_file = open(file)
        dir_file.close()
        
    except FileNotFoundError:
        pass
        
    else:
        with open(file,'r') as csvfile:
            lines = csv.reader(csvfile, delimiter=',')
            for row in lines:
                # Read force magnitude
                force_file = int(row[i_init])
                force_index = force_mag.index(force_file)
                data_force[force_index] += 1

                # Read test number
                iter_file = test_num.index(int(row[i_init + 1]))
                iter_index = force_iter.index(rand_iter[iter_file])
                data_iter[iter_index] += 1
                #print(iter_index)
                
                if iter_file in force_compare[force_index]:
                    force_compare_sum[force_index] += 1
                    
                    if [force_file, iter_file] in iter_compare[iter_index]:
                        iter_compare_sum[iter_index] += 1

            
    return data_force, data_iter, force_compare_sum, iter_compare_sum
    
def count_data2(file, force_mag, force_iter, i_init, force_compare, iter_compare, lim, min_real, max_real):
    data_force = np.zeros(len(force_mag))
    data_iter  = np.zeros(len(force_iter))
    
    force_compare_sum = np.zeros(len(force_mag))
    iter_compare_sum  = np.zeros(len(force_iter))
    
    force_compare_2 = []
    iter_compare_2 = []
    for i in range(len(force_mag)):
        force_compare_2.append([])
    for i in range(len(force_iter)):
        iter_compare_2.append([])
        
    try:
        dir_file = open(file)
        dir_file.close()
        
    except FileNotFoundError:
        pass
        
    else:
        with open(file,'r') as csvfile:
            lines = csv.reader(csvfile, delimiter=',')
            for row in lines:
                
                force_file = int(row[i_init])
                force_index = force_mag.index(force_file)
                iter_file = test_num.index(int(row[2]))
       #         print('int(row[2])',int(row[2]))
        #        print('iter_file',iter_file)
                max_compare = max_real[iter_file]
                min_compare = min_real[iter_file]
                out_lim = False
                for j in range(len(max_compare)):
                    if max_compare[j] != 0 and max_compare[j] > lim and not out_lim:
                        out_lim = True
                    if min_compare[j] != 0 and min_compare[j] < -lim and not out_lim:
                        out_lim = True
                        
                if out_lim:
                    data_force[force_index] += 1
          #          print('force_index',force_index)
           #         print(data_force)
                    #iter_file = int(row[i_init + 1])
                    iter_index = force_iter.index(rand_iter[iter_file])
                    data_iter[iter_index] += 1
            #        print(force_compare)
                    if iter_file in force_compare[force_index]:
                        force_compare_sum[force_index] += 1
                        
                        if [force_file, iter_file] in iter_compare[iter_index]:
                            iter_compare_sum[iter_index] += 1
                    force_compare_2[force_index].append(iter_file)
                    iter_compare_2[iter_index].append([force_file, iter_file])
            
    return data_force, data_iter, force_compare_sum, iter_compare_sum, force_compare_2, iter_compare_2
    
    
def count_data_save(file, force_mag, force_iter):
    # Arrays to save data depending on the force and iteration when it is applied
    data_force = np.zeros(len(force_mag))
    data_iter  = np.zeros(len(force_iter))
    
    force_compare = []
    iter_compare = []
    for i in range(len(force_mag)):
        force_compare.append([])
    for i in range(len(force_iter)):
        iter_compare.append([])
        
    # Count data if file exists
    try:
        dir_file = open(file)
        dir_file.close()
        
    except FileNotFoundError:
        pass
        
    else:
        with open(file,'r') as csvfile:
            lines = csv.reader(csvfile, delimiter=',')
            for row in lines:
                # Read force magnitude
                force_file = int(row[0])
                force_index = force_mag.index(force_file)
                data_force[force_index] += 1

                # Read test number
                iter_file = test_num.index(int(row[1]))
                force_compare[force_index].append(iter_file)
                iter_index = force_iter.index(rand_iter[iter_file])
                data_iter[iter_index] += 1
                iter_compare[iter_index].append([force_file, iter_file])   
            
    return data_force, data_iter, force_compare, iter_compare
    
def count_data_save2(file, force_mag, force_iter, i_init, min_lim, max_lim, min_real, max_real):
    data_force = np.zeros(len(force_mag))
    data_iter  = np.zeros(len(force_iter))
    
    force_compare = []
    iter_compare = []
    for i in range(len(force_mag)):
        force_compare.append([])
    for i in range(len(force_iter)):
        iter_compare.append([])
        
    try:
        dir_file = open(file)
        dir_file.close()
        
    except FileNotFoundError:
        pass
        
    else:   
        with open(file,'r') as csvfile:
            lines = csv.reader(csvfile, delimiter=',')
            for row in lines:
                
                force_file = int(row[i_init])
                force_index = force_mag.index(force_file)
            #    print('int(row[2]) Pos',int(row[2]))
            #    print('force_index Pos', force_index)
                iter_file = test_num.index(int(row[2]))
            #    print('iter_file Pos', iter_file)
                max_compare = max_real[iter_file]
                min_compare = min_real[iter_file]
                out_lim = False
                for j in range(len(max_compare)):
                    if max_compare[j] != 0 and max_compare[j] > max_lim[j] and not out_lim:
             #           print('out lim max', max_compare[j], '>', max_lim[j])
                        out_lim = True
              #      else:
               #         print(' lim max', max_compare[j], '>', max_lim[j])
                    if min_compare[j] != 0 and min_compare[j] < min_lim[j] and not out_lim:
                #        print('out lim min', min_compare[j] , '<', min_lim[j])
                        out_lim = True

                 #   else:
                  #      print(' lim min', min_compare[j], '<', min_lim[j])
                    
                if out_lim:
                    data_force[force_index] += 1
                    force_compare[force_index].append(iter_file)
                    iter_index = force_iter.index(rand_iter[iter_file])
                    data_iter[iter_index] += 1
                    iter_compare[iter_index].append([force_file, iter_file])

            
    return data_force, data_iter, force_compare, iter_compare
    
def list_str_to_float(list_str):
    list_str = list_str.replace('[', '')
    list_str = list_str.replace(']', '')
    list_split = list(map(str.strip, list_str.split()))
    list_float = [float(x) for x in list_split]
    return list_float

force_mag = [200, 210, 220, 230, 240, 250, 260, 270]
full_path = os.path.realpath(__file__)

for force_data in force_mag:
    row_num = 0

    dir_path = os.path.dirname(full_path) + "/../No_MPS_frequencies/" + str(force_data)
    N = 1000

    f60 = dir_path + "/data_sim" + str(force_data) + ".csv"
    
    # Array to save iterations when the robot was pushed
    rand_iter = np.zeros(N, dtype = np.int8)

    # Arrays to save data about joint limit violation
    
    empty = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]

    # Array to save test numbers
    test_num  = []

    with open(f60,'r') as csvfile:
        lines = csv.reader(csvfile, delimiter=',')
        for row in lines:
            rand_iter[row_num] = int(row[0][2:-2])
            test_num.append(int(row[3]))
            row_num += 1
              
    stop_files = dir_path + "/save_stop" + str(force_data) + ".csv"
    backup_files = dir_path + "/save_backup" + str(force_data) + ".csv"
    fall_files = dir_path + "/save_fall" + str(force_data) + ".csv"
    knee_files = dir_path + "/save_knee" + str(force_data) + ".csv"
       

    force_iter = [x for x in range(1,91)]


    stop_force, stop_iter, force_compare, iter_compare = count_data_save(stop_files, force_mag, force_iter)

    backup_force, backup_iter, force_backup_stop, iter_backup_stop = count_data(backup_files, force_mag, force_iter, 0, force_compare, iter_compare)

    fall_force, fall_iter, force_compare, iter_compare = count_data_save(fall_files, force_mag, force_iter)

    knee_force, knee_iter, force_fall_knee, iter_fall_knee = count_data(knee_files, force_mag, force_iter, 0, force_compare, iter_compare)

    


    data_forces_save = [[force_data, stop_force, backup_force, force_backup_stop, fall_force, knee_force, force_fall_knee]]
    nameFile = os.path.dirname(full_path) + "/plot_forces_no_mps.csv"
    with open(nameFile, 'a', encoding="ISO-8859-1", newline='') as myfile:
        wr = csv.writer(myfile)
        wr.writerows(data_forces_save)
    myfile.close()


    '''data_iter_save = [[force_data, stop_iter, backup_iter, iter_backup_stop, fall_iter, knee_iter, iter_fall_knee, vel_iter, pos_iter, iter_pos_vel, outside_limits_iter, iter_outside_fall]]
    nameFile = "plot_iter_no_mps_full.csv"
    #nameFile = "plot_iter_sensor_075.csv"
    #nameFile = "plot_iter_state_099.csv"
    #nameFile = "plot_iter_vf_mujoco_095_full.csv"
    with open(nameFile, 'a', encoding="ISO-8859-1", newline='') as myfile:
        wr = csv.writer(myfile)
        wr.writerows(data_iter_save)
    myfile.close()#'''
    