#!/usr/bin/env python

import argparse
import multiprocessing
import os.path as osp
import os.path as osp
import yaml
import rospkg
import jsk_data

import numpy as np
import chainer
from detection_dataset import DetectionDataset, get_class_list
from train_utils import train
from chainercv.visualizations import vis_bbox
import matplotlib.pyplot as plot


def download_data(*args, **kwargs):
    p = multiprocessing.Process(
            target=jsk_data.download_data,
            args=args,
            kwargs=kwargs)
    p.start()
    p.join()

def install_sample(PKG):

    ## //=== Sample SSD training data === ##
    download_data(
        pkg_name=PKG,
        path="data/sample/final_demo.tar.gz",
        url='https://drive.google.com/uc?id=1zR5h2Yu9SzdsrWH20rCuyT0CX7iGoH-d',
        md5='a1b594bc77f4f331f01eab47fe4a5f1a',
        quiet=False,
        extract=True
    )

def main():
    ## download train data ##
    print("Downloading train data.")
    PKG = 'aero_object_detector'
    rp = rospkg.RosPack()
    pkg_path = rp.get_path(PKG)
    data_path = pkg_path + "/data/sample/final_demo/"
    install_sample(PKG)

    ## set default args ##
    print("\n Start training. Result will be saved in result/sample/")
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--data', type=str, default=data_path,
        help='The root directory of the dataset')
    parser.add_argument(
        '--class_list', type=str, default=data_path + "class_list.txt",
        help='The path to the txt file with label names')
    parser.add_argument(
        '--iteration', type=int, default=120000,
        help='The number of iterations to run until finishing the train loop')
    parser.add_argument(
        '--lr', type=float, default=1e-4, help='Initial learning rate')
    parser.add_argument(
        '--step_size', type=int, default=10000,
        help='The number of iterations to run before '
        'dropping the learning rate by 0.1')
    parser.add_argument(
        '--batchsize', type=int, default=10,
        help='The size of batch')
    parser.add_argument(
        '--gpu', type=int, default=0, help='GPU ID')
    parser.add_argument('--out', default='result/sample',
                        help='The directory in which logs are saved')
    parser.add_argument(
        '--val_iteration', type=int, default=1000,
        help='The number of iterations between every validation.')
    parser.add_argument(
        '--log_iteration', type=int, default=10,
        help='The number of iterations between every logging.')
    parser.add_argument(
        '--loaderjob', type=int, default=4,
        help='The number of processes to launch for MultiprocessIterator.')
    parser.add_argument(
        '--resume',
        help='The path to the trainer snapshot to resume from. '
        'If unspecified, no snapshot will be resumed')
    args = parser.parse_args()

    label_names = get_class_list(args.class_list)
    train_data = DetectionDataset(args.data  + "/train/", label_names)
    val_data = DetectionDataset(args.data + "/val/", label_names)

    step_points = [args.step_size]
    train(
        train_data,
        val_data,
        label_names,
        args.iteration,
        args.lr,
        step_points,
        args.batchsize,
        args.gpu,
        args.out,
        args.val_iteration,
        args.log_iteration,
        args.loaderjob,
        args.resume)

if __name__ == '__main__':
    main()
