import io, numpy as np, sys, cv2
from PIL import Image
from xinshuo_io import is_path_exists, mkdir_if_missing, load_list_from_folder, fileparts
from xinshuo_io import load_list_from_folder, fileparts, mkdir_if_missing
import time
import matplotlib.pyplot as plt
import os

if __name__ =='__main__':
    image_collection_path = '/Users/jiayansong/Desktop/MOT2D/0028'
    whole_coordinate_path = '/Users/jiayansong/Desktop/MOT2D/results/Car_test/track_result/0028'
    coordinate_lists = [whole_coordinate_path + '/' + str(i) + '.txt' for i in range(0, 175)]    #这个地方要随时改
    #coordinate_lists, num_of_lists = load_list_from_folder(whole_coordinate_path)
    pictureLists, num_of_frames = load_list_from_folder(image_collection_path)
    #print(coordinate_lists)


    i = 0
    for (everyPicture, everycoordinate) in zip(pictureLists, coordinate_lists):    #使用zip函数   遍历所有frames
        print("现在是第%d帧", i)
        i += 1
        image_path = os.path.join(image_collection_path, everyPicture)
        coordinate_path = os.path.join(whole_coordinate_path, everycoordinate)
        with open(coordinate_path, 'r') as f:
            files = f.readlines()
        for temp in files :           #在一张图一个frame里面循环画物体框
            tempp = temp.split(' ')
            x1 = int(float(tempp[1]))
            y1 = int(float(tempp[2]))
            x2 = int(float(tempp[3]))
            y2 = int(float(tempp[4]))
            ID = int(tempp[5])
            image = cv2.imread(image_path)
            first_point = (x1, y1)
            last_point = (x2, y2)
            purple = (255, 0, 0)
            golden = (255, 204, 0)
            green = (0, 255, 0)
            blue = (0, 204, 255)
            red = (255, 0, 255)
            anotherRed = (153, 51, 102)
            colour = [purple, golden, green, blue, red, anotherRed]
            id_colour = colour[ID % 6]
            cv2.rectangle(image, first_point, last_point, id_colour, 2)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image, 'ID = %d'%ID, (int(0.25 * x2 + 0.75 * x1), y1 - 5), font, 0.4, id_colour, None, None, None)
            cv2.imwrite(image_path, image)


