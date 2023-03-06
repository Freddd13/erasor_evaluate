# -*- coding: utf-8 -*-
import pypcd
from tqdm import tqdm
import numpy as np
from sklearn.neighbors import NearestNeighbors
from tabulate import tabulate
DYNAMIC_CLASSES = [254]

def intensity2labels(intensity_np):
    label = intensity_np.astype(np.uint32)
    sem_label = label & 0xFFFF  # semantic label in lower half
    inst_label = label >> 16  # instance id in upper half
    return sem_label, inst_label

def fetch_dynamic_objects_ids(intensity_np):
    sem_label, inst_label = intensity2labels(intensity_np)

    num_total = 0
    # Count class
    for class_id in DYNAMIC_CLASSES:
        unique_ids = np.unique(inst_label[sem_label == class_id])
        print(class_id, unique_ids)


def count_static_and_dynamic(intensity_np, verbose=False):
    NUM_CLASS_ON_MAP = {252: 0, 253: 0, 254: 0, 255: 0,
                        256: 0, 257: 0, 258: 0, 259: 0,
                        'dynamic': 0, 'static': 0, 'percentage': 0, 'total': 0}
    sem_label, inst_label = intensity2labels(intensity_np)

    if verbose: print("[Debug]: total: {}".format(sem_label.shape[0]))
    num_total = 0
    # Count class
    for class_id in DYNAMIC_CLASSES:
        num_class = np.count_nonzero(sem_label == class_id)
        NUM_CLASS_ON_MAP[class_id] = num_class
        num_total = num_total + num_class
    
    nd = num_total
    ns = sem_label.shape[0] - nd
    NUM_CLASS_ON_MAP['dynamic'] = nd
    NUM_CLASS_ON_MAP['static'] = ns
    NUM_CLASS_ON_MAP['total'] = ns + nd
    NUM_CLASS_ON_MAP['percentage'] = float(nd)/float(ns)*100
    display = "# Total - {} => Static : {} | # Dynamic: {} | percentage: {}".format(ns + nd, ns, nd,
                                                                                    float(nd)/float(ns) * 100)
    if verbose: print(display)

    return NUM_CLASS_ON_MAP

def data2xyz_np(pc):
    xs = np.reshape(pc.pc_data['x'], (-1, 1))
    ys = np.reshape(pc.pc_data['y'], (-1, 1))
    zs = np.reshape(pc.pc_data['z'], (-1, 1))
    return np.concatenate((xs, ys, zs), axis=1)

def show_num_static_dynamc(path):
    print("On loading data...")
    data = pypcd.PointCloud.from_path(path)
    print("Complete to load data")
    count_static_and_dynamic(data.pc_data['intensity'])

def calc_metrics(gt, estimate, dists, indices, voxelsize):
    num_pc = gt.pc_data['x'].shape[0]
    assert num_pc == dists.shape[0]
    assert num_pc == indices.shape[0]

    # Metrics
    num_geometric_inliers = 0
    num_inlier_dynamic = 0
    num_inlier_static = 0
    num_remainder_dynamic = 0

    num_preserved = 0
    num_semantically_preserved = 0

    # num_deterministic_inliers = {'inliers': 0, 'num_remainder': 0, 'num_preserved': 0}

    DETERMINISTIC_INLIER_THR = 0.01

    gt_sem_label, gt_inst_label = intensity2labels(gt.pc_data['intensity'])
    # By means of indices, the est values are correspond to gt!
    est_sem_label, est_inst_label = intensity2labels(estimate.pc_data['intensity'][indices])

    # 1. deterministic_inliers - no need to spatial comparison
    is_inliers = dists < DETERMINISTIC_INLIER_THR
    num_preserved += np.count_nonzero(is_inliers)
    gt_inliers_semantic = gt_sem_label[is_inliers]
    est_inliers_semantic = gt_sem_label[is_inliers]
    print(num_preserved)

    for gt_sem, est_sem in tqdm(zip(gt_inliers_semantic, est_inliers_semantic)):
        if gt_sem in DYNAMIC_CLASSES:  # dynamic
            pass
        else:
            if est_sem in DYNAMIC_CLASSES:  # dynamic <-> dynamic
                pass
            else:  # static <-> static
                num_semantically_preserved += 1

    print(num_preserved, num_semantically_preserved)

    # over_the_voxelsize = dists > voxelsize * np.sqrt(3)/2  # Absolutely far from the each point
    # within_the_voxelsize = dists < voxelsize * np.sqrt(3)/2
    #
    # num_geometric_inliers = num_pc - np.count_nonzero(over_the_voxelsize)
    # print(num_geometric_inliers)
    #
    # gt_filtered_xyz = data2xyz_np(gt)[within_the_voxelsize]
    # est_corresponding_xyz = data2xyz_np(estimate)[indices]  # Corresponding
    # dists_filtered = dists[within_the_voxelsize]
    #
    # # Those are filtered
    # gt_sem_label, gt_inst_label = intensity2labels(gt.pc_data['intensity'][within_the_voxelsize])
    # est_sem_label, est_inst_label = intensity2labels(estimate.pc_data['intensity'][indices])
    #
    # for i in range(10):
    #     print(dists_filtered[i])
    #     print(gt_filtered_xyz[i, 0])
    #     print(est_corresponding_xyz[i, 0])
    #
    # print(np.amax(dists_filtered))


def calc_naive_preservation(gt, estimate, dists, indices, voxelsize):
    num_pc = gt.pc_data['x'].shape[0]
    assert num_pc == dists.shape[0]
    assert num_pc == indices.shape[0]

    num_preserved = 0
    num_static_preserved = 0
    num_dynamic_preserved = 0

    DETERMINISTIC_INLIER_THR = voxelsize * np.sqrt(3) / 2
    # DETERMINISTIC_INLIER_THR = voxelsize / 2

    gt_sem_label, gt_inst_label = intensity2labels(gt.pc_data['intensity'])
    # By means of indices, the est values are correspond to gt!
    est_sem_label, est_inst_label = intensity2labels(estimate.pc_data['intensity'][indices])

    # 1. deterministic_inliers - no need to spatial comparison
    is_inliers = dists < DETERMINISTIC_INLIER_THR
    num_preserved += np.count_nonzero(is_inliers)
    gt_inliers = gt_sem_label[is_inliers]
    est_inliers = est_sem_label[is_inliers]

    for gt_sem, est_sem in tqdm(zip(gt_inliers, est_inliers)):
        if (gt_sem not in DYNAMIC_CLASSES) and (est_sem not in DYNAMIC_CLASSES):  # both are static
            num_static_preserved += 1

        elif (gt_sem in DYNAMIC_CLASSES) and (est_sem in DYNAMIC_CLASSES):  # both are dynamic
            num_dynamic_preserved += 1
        else:
            pass

    return num_preserved, num_static_preserved, num_dynamic_preserved


def evaluate(gt, estimate, voxelsize=0.2):
    # 0. print current state
    num_gt = count_static_and_dynamic(gt.pc_data['intensity'])
    num_estimate = count_static_and_dynamic(estimate.pc_data['intensity'])

    contents = []
    # Print class-wise rejection rate
    for dyn_class in DYNAMIC_CLASSES:
        if num_gt[dyn_class] == 0:
            line = [dyn_class, "N/A", "N/A", "N/A"]
        else:
            line = [dyn_class, float(num_gt[dyn_class] - num_estimate[dyn_class]) / float(num_gt[dyn_class]) * 100, num_estimate[dyn_class], num_gt[dyn_class]]
        contents.append(line)
    print(tabulate(contents, headers=["Class ID", "R. R", "# remain pts", "# all"], tablefmt="github"))
        
    # 1. Set data to xyz
    gt_xyz = data2xyz_np(gt)
    estimate_xyz = data2xyz_np(estimate)

    nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(estimate_xyz)
    # indices: gt -> estimates
    dists, indices = nbrs.kneighbors(gt_xyz)
    num_preserved, num_static_preserved, num_dynamic_preserved = calc_naive_preservation(gt, estimate, np.reshape(dists, -1), np.reshape(indices, -1), voxelsize)
    # Print metrics

    gt_data = [num_gt['static'], num_gt['dynamic'], num_gt['percentage']]
    est_data = [num_estimate['static'], num_estimate['dynamic'], num_estimate['percentage']]

    precision = float(num_gt['dynamic'] - num_estimate['dynamic']) / float(num_gt['total'] - num_estimate['total'] + 0.00000000001)
    recall = float(num_gt['dynamic'] - num_estimate['dynamic']) / float(num_gt['dynamic'])

    pr = float(num_static_preserved) / float(num_gt['static']) * 100
    rr = float(num_gt['dynamic'] - num_dynamic_preserved) / float(num_gt['dynamic']) * 100
    printed_data = gt_data + est_data + [pr, rr, 2 * (pr/100) * (rr/100) / ((pr/100) + (rr/100))]

    print(tabulate([printed_data], headers=['# stat. pts', '# dyn. pts', '%', '# est. stat. pts', '# est. dyn. pts',
                                            '%', 'Preservation', 'rejection', 'F1'], tablefmt='github'))
def load_pcd(path):
    print("On loading data...")
    data = pypcd.PointCloud.from_path(path)
    print("Complete to load data")
    return data

if __name__ == "__main__":
    import argparse
    import os

    parser = argparse.ArgumentParser(description='Analysis of static map')
    parser.add_argument('--cgt', default='/mnt/d/dataset/dynamic_self/test/machi_FPGT_res_corner__to__voxel_0.200000.pcd', type=str)
    parser.add_argument('--cest', default='/mnt/d/dataset/dynamic_self/test/machi_no_distort_featurori_config1_full_filter_res_corner__to__voxel_0.200000.pcd', type=str)

    parser.add_argument('--sgt', default='/mnt/d/dataset/dynamic_self/test/machi_FPGT_res_surf__to__voxel_0.400000.pcd', type=str)
    parser.add_argument('--sest', default='/mnt/d/dataset/dynamic_self/test/machi_no_distort_featurori_config1_full_filter_res_surf__to__voxel_0.400000.pcd', type=str)

    parser.add_argument('--fullgt', default='/mnt/d/dataset/dynamic_self/test/machi_FPGT_res_full__to__voxel_0.200000.pcd', type=str)
    parser.add_argument('--fullest', default='/mnt/d/dataset/dynamic_self/test/machi_no_distort_featurori_config1_full_filter_res_full__to__voxel_0.200000.pcd', type=str)    
    args = parser.parse_args()

    ########################### corner ################################
    print("corner GT Path: " + args.cgt)
    print("corner Estimate Path: " + args.cest)

    assert os.path.isfile(args.cgt), "Corner GT path does not exist"
    assert os.path.isfile(args.cest), "Corner Est path does not exist"

    cgt_data = load_pcd(args.cgt)
    ctarget_data = load_pcd(args.cest)

    # 评估
    corner_voxelsize = 0.2
    print("\n =================== corner map {} ====================".format(corner_voxelsize))
    evaluate(cgt_data, ctarget_data, voxelsize=corner_voxelsize)
    


    ########################### surf #############################
    print("surf GT Path: " + args.sgt)
    print("surf Estimate Path: " + args.sest)

    assert os.path.isfile(args.sgt), "Surf GT path does not exist"
    assert os.path.isfile(args.sest), "Surf Est path does not exist"
    sgt_data = load_pcd(args.sgt)
    starget_data = load_pcd(args.sest)

    # 评估
    surf_voxelsize = 0.4
    print("\n =================== surf map ====================".format(surf_voxelsize))
    evaluate(sgt_data, starget_data, voxelsize=surf_voxelsize)



    ########################### full #############################
    print("full GT Path: " + args.fullgt)
    print("full Estimate Path: " + args.fullest)

    assert os.path.isfile(args.fullgt), "full GT path does not exist"
    assert os.path.isfile(args.fullest), "full Est path does not exist"
    fullgt_data = load_pcd(args.fullgt)
    fulltarget_data = load_pcd(args.fullest)

    # 评估
    full_voxelsize = 0.2
    print("\n =================== full map ====================".format(full_voxelsize))
    evaluate(fullgt_data, fulltarget_data, voxelsize=full_voxelsize)