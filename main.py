from __future__ import print_function
import matplotlib
import os, numpy as np, time, sys
from model import MOT2D
from xinshuo_io import load_list_from_folder, fileparts, mkdir_if_missing


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('错误')
        sys.exit(1)

    dataset = sys.argv[1]
    save_root = './results'


    seq_file_list, num_seq = load_list_from_folder(os.path.join('data', dataset))   #seq_file_list 是一个列表 里面有'data/Car_test/0000.txt', 'data/Car_test/0001.txt', 'data/Car_test/0002.txt',
    total_time, total_frames = 0.0, 0
    save_dir = os.path.join(save_root, dataset)   #保存在./results/Car_test
    mkdir_if_missing(save_dir)
    eval_dir = os.path.join(save_dir, 'data')   #./results/Car_test/data
    mkdir_if_missing(eval_dir)
    seq_count = 0
    for seq_file in seq_file_list:     #seq_file 是一个个字符串str  data/Car_test/0000.txt    共有好几个seq序列一个seq里面有上百帧
        _, seq_name, _ = fileparts(seq_file)     #seq_name 是0000，0001 这个
        eval_file = os.path.join(eval_dir, seq_name + '.txt')   #./results/Car_test/0000.txt
        eval_file = open(eval_file, 'w')
        save_trk_dir = os.path.join(save_dir, 'track_result', seq_name)   #./results/Car_test/trk_withid/0000
        mkdir_if_missing(save_trk_dir)

        mottracker = MOT2D()
        seq_dets = np.loadtxt(seq_file, delimiter=',')    #load detections  N*6d       seq_dets 是二维数组

        if len(seq_dets.shape) == 1:
            seq_dets = np.expend_dims(seq_dets, axis=0) #变成二维
        if seq_dets.shape[1]==0:
            eval_file.close()
            continue


        min_frame, max_frame = int(seq_dets[:, 0].min()), int(seq_dets[:, 0].max())
        for frame in range(min_frame, max_frame + 1):
            print_str='processing %s: %d %d, %d %d  \r' % (seq_name, seq_count, num_seq, frame, max_frame)
            sys.stdout.write(print_str)  #在屏幕上显示
            sys.stdout.flush()   #在屏幕上连续显示
            #print(print_str)
            save_trk_file = os.path.join(save_trk_dir, '%d.txt' % frame)   #./results/Car_test/trk_withid/0000/(frame.txt)
            save_trk_file = open(save_trk_file, 'w')

            info = seq_dets[seq_dets[:,0] == frame, 1].reshape((-1,1))   #info 是6个输入中的type


            dets = seq_dets[seq_dets[:,0] == frame, 2:6]   #一个帧可能有N个det，N*4 矩阵， x1,y1,x2,y2     从seq_dets数组中取一部分为所要的。格式不变
            dets_all = {'dets': dets, 'info': info}

            start_time = time.time()

            trackers = mottracker.update(dets_all)
            cycle_time = time.time() - start_time
            total_time += cycle_time

            #saving results
            for d in trackers:
                bbox2D_tmp = d[0:4]
                id_tmp = d[4]
                type_tmp = d[5]

                str_to_write = '%d %f %f %f %f %d\n' % (frame, bbox2D_tmp[0],bbox2D_tmp[1], bbox2D_tmp[2],
                                                       bbox2D_tmp[3], id_tmp )
                save_trk_file.write(str_to_write)

            total_frames += 1
            save_trk_file.close()
        seq_count += 1
    print('全部用时： %.3f 秒 共 %d 帧，或者是 %.1f FPS' % (total_time, total_frames, total_frames/ total_time))




