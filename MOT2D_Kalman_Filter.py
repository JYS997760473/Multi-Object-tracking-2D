import numpy as np
from filterpy.kalman import KalmanFilter


class KalmanTracker(object):

    count=0

    def __init__(self, bbox2D, info):
        self.kf = KalmanFilter(dim_x=8,dim_z=4)   #state vector: x1,y1,x2,y2,vx1,vy1,vx2,vy2

        self.kf.F= np.array([[1,0,0,0,1,0,0,0],
                             [0,1,0,0,0,1,0,0],
                             [0,0,1,0,0,0,1,0],
                             [0,0,0,1,0,0,0,1],
                             [0,0,0,0,1,0,0,0],
                             [0,0,0,0,0,1,0,0],
                             [0,0,0,0,0,0,1,0],
                             [0,0,0,0,0,0,0,1]])

        self.kf.H = np.array([[1,0,0,0,0,0,0,0],
                              [0,1,0,0,0,0,0,0],
                              [0,0,1,0,0,0,0,0],
                              [0,0,0,1,0,0,0,0]])

        self.kf.P[4:,4:] *= 1000 #速度不确定， 初始值设大   但发现并没有卵用
        self.kf.P *= 10   #总体改进  estimate uncertainty

        self.kf.Q[4:,4:] *= 0.01     #process noise uncertainty
        self.kf.x[:4] = bbox2D.reshape((4,1))    #bbox2D=dets

        self.time_since_update = 0
        self.id = KalmanTracker.count
        KalmanTracker.count += 1        #ID
        self.history = []               #记录这个跟踪器predict的state
        self.hits = 1  # number of total hits including the first detection
        self.hit_streak = 1  # number of continuing hit considering the first detection
        self.first_continuing_hit = 1
        self.still_first = True     #给新进来的tracker初始化的标志
        self.age = 0
        self.info = info  #other info associated

    def update(self,bbox2D, info):     #dim_bbox=dim_z=4   H用到了，R=None H观测矩阵
        #update the state vector with observed bbox

        self.time_since_update = 0    #归为0 更新了就归位0
        self.history=[]             #history清零
        self.hits += 1              #记录连续跟踪次数
        self.hit_streak += 1    #number of continuing hit
        if self.still_first:
            self.first_continuing_hit += 1  #number of continuing hit in the first time

        #flip
        self.kf.update(bbox2D)    #将前验估计变成后验估计

        self.info = info

    def predict(self):
        #Advances the state vector and returns the predicted bounding box estimate.

        self.kf.predict()
        self.age += 1       #预测次数
        if (self.time_since_update > 0):     # 已经预测过了
            self.hit_streak = 0             #hit_streak 归位0  （判断只更新不预测的情况）
            self.still_first = False     #将初始化的tracker告知已经开始预测了
        self.time_since_update += 1      #预测次数（和update配套使用，为了去除只预测而不得到更新的跟踪器）
        self.history.append(self.kf.x)     #往history里添加现在state
        return self.history[-1]   #倒数第一行     #返回现在的state矩阵（就是前验估计）

    def get_state(self):
        # return the current bounding box estimate

        return self.kf.x[:4].reshape((4,))



