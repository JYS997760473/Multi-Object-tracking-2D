# Author :Jia Yansong

import numpy as np
from scipy.optimize import linear_sum_assignment
from bbox import iou_2d,convert_2dbox_to_4conrner
from MOT2D_Kalman_Filter import KalmanTracker


def associate_detections_to_trackers(detections,trackers,iou_threshold=0.01):
    """
    detections: N*4*2  dets_4corners
    trackers: N*4*2    三维数组，N个4个点的x,y坐标值  trks_4corners
    return 3 lists of matches, unmatched_detections and unmatched_trackers
    """
    if(len(trackers)==0):

        return np.empty((0,2),dtype=int), np.arange(len(detections)),np.empty((0,4,2),dtype=int)
    iou_matrix=np.zeros((len(detections),len(trackers)),dtype=np.float32)    #len(detections)=N

    for d,det in enumerate(detections):         # detections是三维数组，d表示第几个物体，det是二维数组，表示第几个物体的四个坐标
        for t, trk in enumerate(trackers):

            iou_matrix[d, t] = iou_2d(det, trk)


    row_ind, col_ind = linear_sum_assignment(-iou_matrix)
    matched_indices = np.stack((row_ind, col_ind), axis=1)   #得到的是坐标索引 第一列是d物体标号，第二列是t物体标号 axis=1在行上进行堆叠

    unmatched_detections = []
    for d, det in enumerate(detections):
        if d not in matched_indices[:, 0]:
            unmatched_detections.append(d)   #进去的d是物体的标号

    unmatched_trackers=[]
    for t, trk in enumerate(trackers):
        if t not in matched_indices[:, 1]:
            unmatched_trackers.append(t)  #进去的t是物体的标号

    matches =[]
    for m in matched_indices:                         #m是一个一行两列的数组
        if (iou_matrix[m[0], m[1]] < iou_threshold):
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else: matches.append(m.reshape(1,2))    #一行两列
    if len(matches)==0:
        matches = np.empty((0,2),dtype=int)
    else:
        matches = np.concatenate(matches, axis=0)
    return matches, np.array(unmatched_detections), np.array(unmatched_trackers)



class MOT2D(object):
    def __init__(self,max_age=2, min_hits=3):
        self.max_age=max_age
        self.min_hits= min_hits
        self.trackers = []
        self.frame_count = 0


    def update(self, dets_all):
        dets, info = dets_all['dets'], dets_all['info']
        self.frame_count += 1

        #Predict next state (prior) using the Kalman filter state propagation
        #equations.


        trks = np.zeros((len(self.trackers),4))

        to_del = []
        ret =[]
        for t,trk in enumerate(trks):
            pos=self.trackers[t].predict().reshape((-1,1))     #predict阶段 predict next state
            trk[:]=[pos[0],pos[1],pos[2],pos[3]]
            if(np.any(np.isnan(pos))):
                to_del.append(t)
        trks=np.ma.compress_rows(np.ma.masked_invalid(trks))    #其实在这个数据集中predict 后的结果和之前的位置是一样的
        for t in reversed(to_del):    #反转列表to_del
            self.trackers.pop(t)

        #得到matched等
        dets_4corner = [convert_2dbox_to_4conrner(det_tmp) for det_tmp in dets]   #dets_4corner: 一个list 一个个二维数组构成一个list dets是一帧内的物体
        if len(dets_4corner) > 0:
            dets_4corner = np.stack(dets_4corner, axis=0)     #将列表中元素按行堆叠，又将它变成标准数组的格式，现在是一个数组

        trks_4corner = [convert_2dbox_to_4conrner(trk_tmp) for trk_tmp in trks]
        if len(trks_4corner) > 0:
            trks_4corner = np.stack(trks_4corner,axis=0)

        matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(dets_4corner,trks_4corner)

        ###updata 阶段， 上面的predict和data association 分别提供了 下一阶段的跟踪器的位置和每个跟踪器物体的预测值应该对应的测量值（多目标跟踪体现在这个地方了！！！（data association））
        #update matched trackers with assigned detections 更新跟踪器
        for t, trk in enumerate(self.trackers):
            if t not in unmatched_trks:     #找匹配到测量值的tracker的索引
                d=matched[np.where(matched[:, 1] == t)[0], 0]   #找到测量值的索引
                trk.update(dets[d,:][0], info[d, :][0])    #add the dets[d,:][0] to the trk(KalmanTracker object) 将前验估计变成后验估计

        # create and initialise new trackers for unmatched detections
        for i in unmatched_dets:
            trk = KalmanTracker(dets[i, :], info[i])       #第一帧的trackers从这里开始进入KalmanTracker类    所以trackers 和trk 都是KalmanTracker的类
            self.trackers.append(trk)



        i=len(self.trackers)

        for trk in reversed(self.trackers):
            d=trk.get_state()    #bbox2D location
            #选择未被最新测量值更新次数少于2次的和已经连续有2次以上跟踪次数的跟踪器  而且只能出现一次
            if((trk.time_since_update < self.max_age) and (trk.hits >= self.min_hits or self.frame_count <=self.min_hits)):
                ret.append(np.concatenate((d,[trk.id + 1], trk.info)).reshape(1,-1))  #转化成一行
            i-=1

            #remove dead tracklet
            if (trk.time_since_update >= self.max_age):
                self.trackers.pop(i)
        if(len(ret)>0):
            return np.concatenate(ret)
        return np.empty((0,6))







