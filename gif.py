import imageio
from xinshuo_io import mkdir_if_missing
import time
def create_gif(image_list, gif_name, duration=0.35):
    frames = []
    for image_name in image_list:
        frames.append(imageio.imread(image_name))
    imageio.mimsave(gif_name, frames, 'GIF', duration=duration)
    return


def main():
    picture_path='/Users/jiayansong/Desktop/MOT2D/visualization/0028'
    image_list=[picture_path+'/'+str(i)+'.png' for i in range(0,175)]

    gif_name = 'cat/cat28.gif'
    mkdir_if_missing(gif_name)
    duration = 0.10
    create_gif(image_list, gif_name, duration)


if __name__ == '__main__':
    start_time=time.time()
    main()
    total_time=time.time()-start_time
    print('共用时%.2f秒'%total_time)


"""
from moviepy.editor import ImageSequenceClip
img_names = ['./imgs/'+str(i)+'.png' for i in range(1,11)]
img_names.reverse()
clip = ImageSequenceClip(img_names,fps=1)
clip.write_gif('demo.gif')
"""