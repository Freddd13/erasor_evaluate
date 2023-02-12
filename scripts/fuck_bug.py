'''
Date: 2023-02-12 18:33:39
LastEditors: yxt
LastEditTime: 2023-02-12 19:04:48
Description: 
'''
import numpy as np

url = "/mnt/d/dataset/dynamic_self/Jan_success_out_machi_inside_to_outside/slam_kitti_withtime.txt"
with open(url) as f:
    lines = [line[:-1] for line in f]

new_url = "./slam_kitti_withtime.txt"
with open(new_url, 'w') as f:
    for line in lines:
        elements = line.split(" ")
        print(len(elements))
        err_data = elements[8]# 8 9| 10 11 12
        correct_data = [err_data.split(".")[0] +"."+ err_data.split(".")[1][:6], err_data.split(".")[1][6:] + "."+ err_data.split(".")[2]]
        new_line = []
        for i in range(8):
            new_line.append(elements[i])
        new_line.extend(correct_data)
        for i in range(9,12):
            new_line.append(elements[i])
        print(line)
        print(new_line)
        print(" ".join(new_line))
        f.write("%s\n" % " ".join(new_line))
# print(content)


