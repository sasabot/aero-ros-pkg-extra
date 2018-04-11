#!/usr/bin/env python

##  convert EgoHands Dataset (http://vision.soic.indiana.edu/projects/egohands/) to
##  dataset for trainer in aero_object_detector.

from __future__ import print_function
import scipy.io
import os
import shutil
import glob
import sys

## === params === ##
label = "hand"
## ============== ##

def main(argv):
    if len(argv) < 3:
        print('provide path to egohands_data root directory and output directory')
        return
    print('\033[33mchecking directory\033[0m')
    try:
        isdir = os.path.isdir(argv[1]) and os.path.isdir(argv[2])
        if not isdir:
            raise ValueError('\033[31mdata directory not found\033[0m')
    except ValueError, e:
        print(e)
        raise SystemExit

    root_dir = argv[1]
    save_dir = argv[2]
    root_dir = root_dir + "/_LABELLED_SAMPLES/"
    cnt = 0;
    for dname in ("/images/", "/labels/"):
        if not os.path.isdir(save_dir + dname):
            os.mkdir(save_dir + dname)

    video_list = sorted(os.listdir(root_dir))
    # for each video
    for video_path in video_list:
        matdata = scipy.io.loadmat(root_dir + video_path  + "/polygons.mat")
        image_list = sorted(glob.glob(root_dir + video_path + "/*.jpg"))
        # for each frame
        for (image_path, frame) in zip(image_list, matdata["polygons"][0]):
            # yourright, yourleft, myright, myleft
            print("Reading {0}  iteration : {1}".format(image_path, cnt))
            suffix_name = str(cnt).zfill(6)
            save_text_name = save_dir + '/labels/' + suffix_name + '.txt'
            save_image_name = save_dir + '/images/' + suffix_name + '.jpg'
            text_file = open(save_text_name, 'w')
            valid_cnt = 0
            for polygon in frame:
                # polygon to rect (for valid polygon only)
                if len(polygon) > 1:
                    x_min = min(polygon[:, 0:1])[0] - min(polygon[:, 0:1])[0] % 1.0
                    x_max = max(polygon[:, 0:1])[0] - max(polygon[:, 0:1])[0] % 1.0
                    y_min = min(polygon[:, 1:2])[0] - min(polygon[:, 1:2])[0] % 1.0
                    y_max = max(polygon[:, 1:2])[0] - max(polygon[:, 1:2])[0] % 1.0
                    valid_cnt += 1
                    text_file.write('%s 0.0 0 0.0 %s %s %s %s 0.0 0.0 0.0 0.0 0.0 0.0 0.0\n' %
                                    (label, x_min, y_min, x_max, y_max))
            text_file.close()
            if valid_cnt > 0:
                shutil.copy(image_path, save_image_name)
                cnt += 1
    class_list_name = save_dir + '/class_list.txt'
    class_list_file = open(class_list_name, 'w')
    class_list_file.write("dontcare 0\n")
    class_list_file.write("%s 1\n" % label)
    class_list_file.close()

if __name__ == "__main__":
    main(sys.argv)
