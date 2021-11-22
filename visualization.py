import os.path
import time
import matplotlib.pyplot as plt
from xinshuo_io import load_list_from_folder, fileparts, mkdir_if_missing
if __name__ =='__main__':
    start_time= time.time()
    visualization_path='/Users/jiayansong/Desktop/MOT2D/visualization'
    mkdir_if_missing(visualization_path)
    track_result_path='/Users/jiayansong/Desktop/MOT2D/results/Car_test/track_result'
    seq_file_list, num_seq = load_list_from_folder(track_result_path)#seq_file_list:track_result/0000, ....
    for seq_file in seq_file_list:      #seq_file:track_result/0000(文件夹)
        _,seq_name,_=fileparts(seq_file)
        print(seq_name)
        seq_file_frames,_=load_list_from_folder(seq_file)  #seq_file_frames:track_result/0000/0.txt ...... a list
        seq_save_path=os.path.join(visualization_path,seq_name)
        mkdir_if_missing(seq_save_path)
        for frame_name in seq_file_frames:
            _,frame,_ = fileparts(frame_name)   #frame:0, 或者 1,3 是帧数
            frame_path=os.path.join((frame_name))
            with open(frame_path,'r')as f:
                files=f.readlines()
            plt.figure()
            plt.xlim(-200,1400)
            plt.ylim(50,450)
            plt.title('Multi-Object Tracking sequence=%s Author: Jia Yansong'%seq_name)
            plt.xlabel('frame=%s'%frame)
            for temp in files:     #temp是数据集每一行数据的字符串
                tempp=temp.split(' ')
                x1=float(tempp[1])
                y1=float(tempp[2])
                x3=float(tempp[3])
                y3=float(tempp[4])
                x2=x3
                y2=y1
                x4=x1
                y4=y3
                ID=int(tempp[5])
                color=['b','g','r','c','m','y']
                id_color=color[ID%6]
                xall = [x1,x2,x3,x4,x1] # x的坐标
                yall = [y1,y2,y3,y4,y1] # y的坐标
                plt.plot(xall, yall,color=id_color)
                plt.text((0.25*x3+0.75*x1),y3,'ID=%d'%ID,fontsize=10,color=id_color)
            save_path=os.path.join(visualization_path,seq_name,frame)
            plt.savefig(save_path)
    end_time=time.time()
    total_time=end_time-start_time
    print('程序一共用时：%.2f秒'%total_time)


