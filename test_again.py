
import os.path
import time
import matplotlib.pyplot as plt
from pyquaternion import Quaternion
import numpy as np
from nuscenes.nuscenes import NuScenes, NuScenesExplorer
import cv2
from nuscenes.utils.data_classes import LidarPointCloud, RadarPointCloud, Box
from nuscenes.utils.geometry_utils import view_points, box_in_image, BoxVisibility, transform_matrix
nusc = NuScenes(version='v1.0-mini', dataroot='/Users/jiayansong/Desktop/nuscenes', verbose=True)
nuscexplore: NuScenesExplorer = NuScenesExplorer(nusc)
result_path = '/Users/jiayansong/Desktop/MOT2DwithNuscenes/results/Car_nuscenes/track_result/0000'
if __name__ =='__main__':

    my_scene = nusc.scene[4]

    first_sample_token = my_scene['first_sample_token']
    k = 0

    while first_sample_token != '' :
        first_sample = nusc.get('sample', first_sample_token)
        result = []
        result_token = []
        for annoation in first_sample['anns']:   #遍历这一个sample里面的所有annotations标注token
            anno = nusc.get('sample_annotation', annoation)   #通过token得到annotation的具体信息
            #print(anno)
            num_radar_pts_of_ann = anno['num_radar_pts']
            num_lidar_pts_of_ann = anno['num_lidar_pts']
            if num_radar_pts_of_ann > 1:
                result.append(anno)   #得到通过雷达点标注的物体信息
                result_token.append(anno['token'])
        #print(result_token)
        ans = []
        for j in result :
            ans.append(j['translation'])

        ann = result[0]
        center = np.array(ann['translation'])
        orientation = np.array(ann['rotation'])
        #nusc.render_egoposes_on_map(log_location='boston-seaport', out_path='/Users/jiayansong/Desktop/3')
        #lidar_file = nusc.get('sample_data', first_sample['data']['LIDAR_TOP'])
        #calib_data = nusc.get('calibrated_sensor', lidar_file['calibrated_sensor_token'])
        #ego_data = nusc.get('ego_pose', lidar_file['ego_pose_token'])
        _,boxes,_ = nusc.get_sample_data(first_sample['data']['LIDAR_TOP'], selected_anntokens=result_token, use_flat_vehicle_coordinates= True)
        _, ax = plt.subplots(1, 1, figsize=(9, 9))

###绘制anno的box
        for box in boxes:
            #print(box)
            #print(type(box))
            box.render(ax, view=np.eye(4), colors=('r', 'r', 'r'))


        ax.plot(0, 0, 'x', color='red')

        ax.set_xlim(-40,40)
        ax.set_ylim(-40,40)

###
        ###下面是绘制雷达点云

        first_sample_data = nusc.get('sample_data', first_sample['data']['LIDAR_TOP'])
        cs_record = nusc.get('calibrated_sensor', first_sample_data['calibrated_sensor_token'])
        pose_record = nusc.get('ego_pose', first_sample_data['ego_pose_token'])
        ref_to_ego = transform_matrix(translation=cs_record['translation'],
                                      rotation=Quaternion(cs_record["rotation"]))

        # Compute rotation between 3D vehicle pose and "flat" vehicle pose (parallel to global z plane).
        ego_yaw = Quaternion(pose_record['rotation']).yaw_pitch_roll[0]
        rotation_vehicle_flat_from_vehicle = np.dot(
            Quaternion(scalar=np.cos(ego_yaw / 2), vector=[0, 0, np.sin(ego_yaw / 2)]).rotation_matrix,
            Quaternion(pose_record['rotation']).inverse.rotation_matrix)
        vehicle_flat_from_vehicle = np.eye(4)
        vehicle_flat_from_vehicle[:3, :3] = rotation_vehicle_flat_from_vehicle
        viewpoint = np.dot(vehicle_flat_from_vehicle, ref_to_ego)

        ###
        pc1, times1 = RadarPointCloud.from_file_multisweep(nusc, first_sample, 'RADAR_FRONT', 'LIDAR_TOP', nsweeps=5)
        points1 = view_points(pc1.points[:3, :], viewpoint, normalize=False)
        dists = np.sqrt(np.sum(pc1.points[:2, :] ** 2, axis=0))
        colors = np.minimum(1, dists / 40 / np.sqrt(2))
        scatter1 = ax.scatter(points1[0, :], points1[1, :], c=colors, s=3.0)
        ####
        velocities1 = pc1.points[8:10, :]  # Compensated velocity
        velocities1 = np.vstack((velocities1, np.zeros(pc1.points.shape[1])))
        velocities1 = np.dot(Quaternion(cs_record['rotation']).rotation_matrix, velocities1)
        velocities1 = np.dot(Quaternion(cs_record['rotation']).rotation_matrix.T, velocities1)
        velocities1[2, :] = np.zeros(pc1.points.shape[1])
        ###
        points_vel = view_points(pc1.points[:3, :] + velocities1, viewpoint, normalize=False)
        deltas_vel = points_vel - points1
        deltas_vel = 6 * deltas_vel  # Arbitrary scaling
        max_delta = 20
        deltas_vel = np.clip(deltas_vel, -max_delta, max_delta)  # Arbitrary clipping
        colors_rgba = scatter1.to_rgba(colors)
        for i in range(points1.shape[1]):
            ax.arrow(points1[0, i], points1[1, i], deltas_vel[0, i], deltas_vel[1, i], color=colors_rgba[i])
        ####
        pc2, times2 = RadarPointCloud.from_file_multisweep(nusc, first_sample, 'RADAR_BACK_LEFT', 'LIDAR_TOP', nsweeps=5)
        points2 = view_points(pc2.points[:3, :], viewpoint, normalize=False)
        dists = np.sqrt(np.sum(pc2.points[:2, :] ** 2, axis=0))
        colors = np.minimum(1, dists / 40 / np.sqrt(2))
        scatter2 = ax.scatter(points2[0, :], points2[1, :], c=colors, s=3.0)
        ####
        velocities = pc2.points[8:10, :]  # Compensated velocity
        velocities = np.vstack((velocities, np.zeros(pc2.points.shape[1])))
        velocities = np.dot(Quaternion(cs_record['rotation']).rotation_matrix, velocities)
        velocities = np.dot(Quaternion(cs_record['rotation']).rotation_matrix.T, velocities)
        velocities[2, :] = np.zeros(pc2.points.shape[1])
        ###
        points_vel = view_points(pc2.points[:3, :] + velocities, viewpoint, normalize=False)
        deltas_vel = points_vel - points2
        deltas_vel = 6 * deltas_vel  # Arbitrary scaling
        max_delta = 20
        deltas_vel = np.clip(deltas_vel, -max_delta, max_delta)  # Arbitrary clipping
        colors_rgba = scatter2.to_rgba(colors)
        for i in range(points2.shape[1]):
            ax.arrow(points2[0, i], points2[1, i], deltas_vel[0, i], deltas_vel[1, i], color=colors_rgba[i])
        ####
        ###
        pc3, times3 = RadarPointCloud.from_file_multisweep(nusc, first_sample, 'RADAR_BACK_RIGHT', 'LIDAR_TOP', nsweeps=5)
        points3 = view_points(pc3.points[:3, :], viewpoint, normalize=False)
        dists = np.sqrt(np.sum(pc3.points[:2, :] ** 2, axis=0))
        colors = np.minimum(1, dists / 40 / np.sqrt(2))
        scatter3 = ax.scatter(points3[0, :], points3[1, :], c=colors, s=3.0)
        ####
        velocities = pc3.points[8:10, :]  # Compensated velocity
        velocities = np.vstack((velocities, np.zeros(pc3.points.shape[1])))
        velocities = np.dot(Quaternion(cs_record['rotation']).rotation_matrix, velocities)
        velocities = np.dot(Quaternion(cs_record['rotation']).rotation_matrix.T, velocities)
        velocities[2, :] = np.zeros(pc3.points.shape[1])
        ###
        points_vel = view_points(pc3.points[:3, :] + velocities, viewpoint, normalize=False)
        deltas_vel = points_vel - points3
        deltas_vel = 6 * deltas_vel  # Arbitrary scaling
        max_delta = 20
        deltas_vel = np.clip(deltas_vel, -max_delta, max_delta)  # Arbitrary clipping
        colors_rgba = scatter3.to_rgba(colors)
        for i in range(points3.shape[1]):
            ax.arrow(points3[0, i], points3[1, i], deltas_vel[0, i], deltas_vel[1, i], color=colors_rgba[i])
        ####
        ###
        pc4, times4 = RadarPointCloud.from_file_multisweep(nusc, first_sample, 'RADAR_FRONT_RIGHT', 'LIDAR_TOP', nsweeps=5)
        points4 = view_points(pc4.points[:3, :], viewpoint, normalize=False)
        dists = np.sqrt(np.sum(pc4.points[:2, :] ** 2, axis=0))
        colors = np.minimum(1, dists / 40 / np.sqrt(2))
        scatter4 = ax.scatter(points4[0, :], points4[1, :], c=colors, s=3.0)
        ####
        velocities = pc4.points[8:10, :]  # Compensated velocity
        velocities = np.vstack((velocities, np.zeros(pc4.points.shape[1])))
        velocities = np.dot(Quaternion(cs_record['rotation']).rotation_matrix, velocities)
        velocities = np.dot(Quaternion(cs_record['rotation']).rotation_matrix.T, velocities)
        velocities[2, :] = np.zeros(pc4.points.shape[1])
        ###
        points_vel = view_points(pc4.points[:3, :] + velocities, viewpoint, normalize=False)
        deltas_vel = points_vel - points4
        deltas_vel = 6 * deltas_vel  # Arbitrary scaling
        max_delta = 20
        deltas_vel = np.clip(deltas_vel, -max_delta, max_delta)  # Arbitrary clipping
        colors_rgba = scatter4.to_rgba(colors)
        for i in range(points4.shape[1]):
            ax.arrow(points4[0, i], points4[1, i], deltas_vel[0, i], deltas_vel[1, i], color=colors_rgba[i])
        ####
        ###
        pc5, times5 = RadarPointCloud.from_file_multisweep(nusc, first_sample, 'RADAR_FRONT_LEFT', 'LIDAR_TOP', nsweeps=5)
        points5 = view_points(pc5.points[:3, :], viewpoint, normalize=False)
        dists = np.sqrt(np.sum(pc5.points[:2, :] ** 2, axis=0))
        colors = np.minimum(1, dists / 40 / np.sqrt(2))
        scatter5 = ax.scatter(points5[0, :], points5[1, :], c=colors, s=10.0)
        ####
        velocities = pc5.points[8:10, :]  # Compensated velocity
        velocities = np.vstack((velocities, np.zeros(pc5.points.shape[1])))
        velocities = np.dot(Quaternion(cs_record['rotation']).rotation_matrix, velocities)
        velocities = np.dot(Quaternion(cs_record['rotation']).rotation_matrix.T, velocities)
        velocities[2, :] = np.zeros(pc5.points.shape[1])
        ###
        points_vel = view_points(pc5.points[:3, :] + velocities, viewpoint, normalize=False)
        deltas_vel = points_vel - points5
        deltas_vel = 6 * deltas_vel  # Arbitrary scaling
        max_delta = 20
        deltas_vel = np.clip(deltas_vel, -max_delta, max_delta)  # Arbitrary clipping
        colors_rgba = scatter5.to_rgba(colors)
        for i in range(points5.shape[1]):
            ax.arrow(points5[0, i], points5[1, i], deltas_vel[0, i], deltas_vel[1, i], color=colors_rgba[i])
        ####
        ###

        ### 绘制地图（道路）
        nuscexplore.render_ego_centric_map(first_sample['data']['LIDAR_TOP'], 40, ax)

        ###关闭x y轴
        ax.axis('off')

        ###  results可视化
        result_path_ = result_path + '/' + str(k) + '.txt'
        file = open(result_path_, 'r')
        files = file.readlines()
        for temp in files:
            tempp = temp.split(' ')
            x1 = float(tempp[1])
            y1 = float(tempp[2])
            x3 = float(tempp[3])
            y3 = float(tempp[4])
            x2 = x1
            y2 = y3
            x4 = x3
            y4 = y1
            ax.plot([x1,x2],[y1,y2],color = 'b')
            ax.plot([x1,x3],[y1,y3],color = 'b')
            ax.plot([x1,x4],[y1,y4],color = 'b')
            ax.plot([x2,x3],[y2,y3],color = 'b')
            ax.plot([x2,x4],[y2,y4],color = 'b')
            ax.plot([x3,x4],[y3,y4],color = 'b')
        k += 1
        print(k)
        plt.savefig('/Users/jiayansong/Desktop/pic_1_1/'+str(k), bbox_inches='tight', pad_inches=0, dpi=200)

        first_sample_token = first_sample['next']


