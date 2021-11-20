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

        self.kf.P[4:,4:] *= 1000 #速度不确定， 初始值设大
        self.kf.P *= 10

        self.kf.Q[4:,4:] *= 0.01
        self.kf.x[:4] = bbox2D.reshape((4,1))    #bbox2D=dets

        self.time_since_update = 0
        self.id = KalmanTracker.count
        KalmanTracker.count += 1
        self.history = []
        self.hits = 1  # number of total hits including the first detection
        self.hit_streak = 1  # number of continuing hit considering the first detection
        self.first_continuing_hit = 1
        self.still_first = True
        self.age = 0
        self.info = info  #other info associated

    def update(self,bbox2D, info):
        #update the state vector with observed bbox

        self.time_since_update = 0
        self.history=[]
        self.hits += 1
        self.hit_streak += 1    #number of continuing hit
        if self.still_first:
            self.first_continuing_hit += 1  #number of continuing hit in the first time

        #flip
        self.kf.update(bbox2D)

        self.info = info

    def predict(self):
        #Advances the state vector and returns the predicted bounding box estimate.

        self.kf.predict()
        self.age += 1
        if (self.time_since_update > 0):
            self.hit_streak = 0
            self.still_first = False
        self.time_since_update += 1
        self.history.append(self.kf.x)
        return self.history[-1]   #倒数第一行

    def get_state(self):
        # return the current bounding box estimate

        return self.kf.x[:4].reshape((4,))



