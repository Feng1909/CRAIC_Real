'''
'''

import math
import rospy 
import threading
from sensor_msgs.msg  import LaserScan
import numpy as np
import copy


class ObjectDetected:
    def __init__(self,objs_dist, offset_ang, angle_max=math.pi,angle_min = -math.pi):
        '''
            
            
        '''
        self.drone_type = 0.15  
        self.lidar_res = 0.391  
        self.lidar_range =[angle_min,angle_max] 
        self.lidar_increment = 0
        self.resolution = 0.391
        self.lidar_data = [] 
        self.lidar_data_mtx = threading.Lock()
        self.isSimulate = False 
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.zero_idx = 0
        self.angles = []
        self.isInit = False
        self.objs_dist = objs_dist 
        self.nan = 200 
        self_offset_ang = offset_ang 
    def InputLidarData(self,data):
        if(isinstance(data,LaserScan)):
            #
            self.lidar_data_mtx.acquire()
            if(self.zero_idx == 0):
                self.angle_min = data.angle_min
                self.angle_max = data.angle_max
                self.lidar_increment = data.angle_increment
                self.resolution = (self.angle_max-self.angle_min)/len(list(data.ranges))
                self.zero_idx = int(len(data.ranges) /2)
                self.angles = np.linspace(self.angle_min,self.angle_max,len(data.ranges))
                self.cos_t = np.cos(self.angles)
                self.sin_t = np.sin(self.angles)
                self.isInit = True
                self.range_max = data.range_max
                self.range_min = data.range_min
                
            self.lidar_data  = np.array(list(data.ranges))

            self.lidar_data = self.lidar_data.tolist()
            self.lidar_data_mtx.release()

        pass
        
    def GetYawObjDistance(self,yaw):
        if(len(self.lidar_data) == 0 ):
            print("lidar_data is empty")
            return -1
        offset  = 0
        if(yaw < 0):
            offset = int((-math.pi - yaw)/self.resolution) 
        if(yaw > 0):
            offset  = int((math.pi - yaw)/self.resolution) 
        
        self.lidar_data_mtx.acquire()
        idx = int(self.zero_idx + offset)
        if(idx < 0): idx = 0
        if(idx >= len(self.lidar_data)): idx = len(self.lidar_data) -1
        dist = self.lidar_data[idx]
        self.lidar_data_mtx.release()
        return dist

    def GetObjMsg(self):
        if(len(self.lidar_data) == 0):
            print("lidar data is empty")
            return[]
        self.lidar_data_mtx.acquire()
        range_array = np.array(self.lidar_data)
        range_array[np.where(range_array == np.inf)] = self.nan
        range_array[np.where(range_array == -np.inf)] = self.nan
        
        self.lidar_data_mtx.release()
        
        pc = np.array([-np.multiply(self.cos_t ,range_array),np.multiply(self.sin_t,range_array)]) 
        
        pc[np.where(pc == np.inf)] = self.nan
        pc[np.where(pc == -np.inf)] = self.nan
        range_array[np.where(range_array == np.inf)] = self.nan

        diff_x = np.diff(pc[0])  
        diff_y = np.diff(pc[1])   
        diff_xx = np.power(diff_x,2)
        diff_yy = np.power(diff_y,2)
        dists = np.sqrt(diff_xx+diff_yy)
        
        dists_diff = np.diff(dists) 
         
        idxs = np.argwhere(np.abs(dists_diff) > self.objs_dist) 
        obj_start = []
        obj_end = []
        start_idx = 0
        if(range_array[0] < self.nan):
            obj_start = [pc[0][0],pc[1][0]]

        idxs_nan = np.argwhere(range_array == self.nan) #
        lines = []
        idxs_pair= []
        
        for i in range(len(idxs)):
            if len(obj_start) == 0 and len(idxs_nan[np.isin(idxs_nan,idxs[i]+1)]) == 0:
                obj_start = [pc[0][idxs[i]+1],pc[1][idxs[i]+1]]
                start_idx = idxs[i]+1
                continue
            elif(len(obj_start) > 0) :
                obj_end = [pc[0][idxs[i]],pc[1][idxs[i]]]
                lines += [[copy.deepcopy(obj_start),copy.deepcopy(obj_end)]]
                idxs_pair += [[start_idx,idxs[i]]]
                obj_start = []
                obj_end = []
                if(not np.isin(idxs_nan,idxs[i]+1).all()):
                    
                    obj_start = [pc[0][idxs[i]+1],pc[1][idxs[i]+1]]
                    start_idx = idxs[i] +1
                else:
                    obj_start = []
                
        if(len(obj_start) > 0 and range_array[-1] < self.objs_dist): 
            obj_end = [pc[0][-1],pc[1][-1]]
            lines += [[copy.deepcopy(obj_start),copy.deepcopy(obj_end)]]
            idxs_pair += [[start_idx,len(range_array)-1]]

        
        
        
        line_set = []
        for i in range(len(idxs_pair)):
            start_idx = idxs_pair[i][0]
            end_idx = idxs_pair[i][1]
            if(start_idx == end_idx):
                line_set +=[[lines[i][0],lines[i][0]]]
                continue
            anchor = copy.deepcopy(lines[i][0])
            start = copy.deepcopy(anchor)
            r_anchor_index = end_idx
            end = copy.deepcopy(lines[i][1])
            stack = [] 
            while True:
                
                A = end[1] - anchor[1]
                B = anchor[0] - end[0]
                if(A == 0 and B== 0):
                    break
                C = end[0] * anchor[1] - anchor[0] * end[1]
                mid_idx = int((start_idx + end_idx)/2)
                mid = [pc[0][mid_idx],pc[1][mid_idx]]
                d = np.abs((A*mid[0] + B *mid[1] + C)/(math.sqrt(A*A + B*B)))
                if(d > self.objs_dist):#
                    stack.append(end)
                    end = copy.deepcopy(mid)
                    r_anchor_index = end_idx
                    end_idx = mid_idx
                    continue
                else: 
                    if(np.linalg.norm(np.array(end) - np.array(lines[i][1])) < self.objs_dist):
                        line_set +=[[anchor,end]]
                        
                        break
                    if(r_anchor_index - start_idx < 3):
                        
                        line_set += [[copy.deepcopy(anchor),copy.deepcopy(end)]]
                        anchor = copy.deepcopy(end)
                        end = copy.deepcopy(lines[i][1])
                        r_anchor_index = idxs_pair[i][1]
                        stack = [lines[i][1]]
                    if(end_idx - start_idx > 2):
                        
                        start_idx = mid_idx
                        continue
                    else:
                        start_idx = end_idx
                        end = stack[-1]
            
        return line_set


           
       
