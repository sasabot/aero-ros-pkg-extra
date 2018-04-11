#!/usr/bin/env python

import argparse
import multiprocessing
import os.path as osp
import os
import yaml
import rospkg

import jsk_data

def download_data(*args, **kwargs):
    p = multiprocessing.Process(
            target=jsk_data.download_data,
            args=args,
            kwargs=kwargs)
    p.start()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet

    PKG = 'aero_object_detector'
    rp = rospkg.RosPack()
    pkg_path = rp.get_path(PKG)

    if not osp.exists(osp.join(pkg_path, "models/chainer")):
        os.makedirs(osp.join(pkg_path, "models/chainer"))

    ## //=== Pretrained SSD models === ##

    # Generate model list yaml
    list_file = open(osp.join(pkg_path, "models/chainer/model_list.yaml"), "w")
    model_list = {}

    # conveni 22 class detector
    model_name = "full"
    model_path = "models/chainer/full_detector/full_model.npz"
    list_path = 'models/chainer/full_detector/full_class_list.txt'
    download_data(
        pkg_name=PKG,
        path=model_path,
        url='https://drive.google.com/uc?id=0B09VRnpQxd6PbmZwbVV0bGVDZWc',
        md5='11cf0914aad17a3c865baa3627bb2fe7',
        quiet=quiet,
    )
    download_data(
        pkg_name=PKG,
        path=list_path,
        url='https://drive.google.com/uc?id=0B09VRnpQxd6PSGUxdlJtbzRzc2c',
        md5='b365e4885a2a59b5bd4e926bdfd7b1cb',
        quiet=quiet,
    )
    model_list[model_name] = {}
    model_list[model_name]["model"] = osp.join(pkg_path, model_path)
    model_list[model_name]["class_list"] = osp.join(pkg_path, list_path)

    # fcsc 4 class detector for objected in container and sandwich
    model_name = "fcsc"
    model_path = "models/chainer/fcsc_detector/fcsc_model.npz"
    list_path = 'models/chainer/fcsc_detector/fcsc_class_list.txt'
    download_data(
        pkg_name=PKG,
        path=model_path,
        url='https://drive.google.com/uc?id=0B09VRnpQxd6PaVRNaDdYRHNTeVk',
        md5='4c303db516ef5a68b53c4ca7b8dc24e7',
        quiet=quiet,
    )
    download_data(
        pkg_name=PKG,
        path=list_path,
        url='https://drive.google.com/uc?id=0B09VRnpQxd6PdHh5N1QxU1NPZ00',
        md5='176980e0093d331528c3d5a567e65ff7',
        quiet=quiet,
    )
    model_list[model_name] = {}
    model_list[model_name]["model"] = osp.join(pkg_path, model_path)
    model_list[model_name]["class_list"] = osp.join(pkg_path, list_path)

    # fcsc 3 class detector for objects in container (contains item hodleras negative data)
    model_name = "container"
    model_path = "models/chainer/container_detector/container_model.npz"
    list_path = 'models/chainer/container_detector/container_class_list.txt'
    download_data(
        pkg_name=PKG,
        path=model_path,
        url='https://drive.google.com/uc?id=1AEji5PvOlGmyS1zZJ3ZsYsUlLfpuGMWA',
        md5='ead1e692178f397b2af22a9b69e2b9a1',
        quiet=quiet,
    )
    download_data(
        pkg_name=PKG,
        path=list_path,
        url='https://drive.google.com/uc?id=1vChBEJ3tmxW0LhfeV-6w8Pu9PIHJGSsc',
        md5='22eb846024bd0c1a1d4f766b4a3b5430',
        quiet=quiet,
    )
    model_list[model_name] = {}
    model_list[model_name]["model"] = osp.join(pkg_path, model_path)
    model_list[model_name]["class_list"] = osp.join(pkg_path, list_path)

    # fcsc 3 class detector for objects in container
    model_name = "container_old"
    model_path = "models/chainer/container_old_detector/container_old_model.npz"
    list_path = 'models/chainer/container_old_detector/container_old_class_list.txt'
    download_data(
        pkg_name=PKG,
        path=model_path,
        url='https://drive.google.com/uc?id=18OtQCmc-t5CsKfPa-I5_SPIHb4y-AYZa',
        md5='42cac9b4c16bd703fa5dfcf8571508fc',
        quiet=quiet,
    )
    download_data(
        pkg_name=PKG,
        path=list_path,
        url='https://drive.google.com/uc?id=1vChBEJ3tmxW0LhfeV-6w8Pu9PIHJGSsc',
        md5='22eb846024bd0c1a1d4f766b4a3b5430',
        quiet=quiet,
    )
    model_list[model_name] = {}
    model_list[model_name]["model"] = osp.join(pkg_path, model_path)
    model_list[model_name]["class_list"] = osp.join(pkg_path, list_path)

    # fcsc sandwich detector
    model_name = "sandwich"
    model_path = "models/chainer/sandwich_detector/sandwich_model.npz"
    list_path = 'models/chainer/sandwich_detector/sandwich_class_list.txt'
    download_data(
        pkg_name=PKG,
        path=model_path,
        url='https://drive.google.com/uc?id=1IeAd-lAxHUUvZRxL_eb4IlpzIIP8ZOjU',
        md5='65e47fb1a1fb3a5021f84f20a123d979',
        quiet=quiet,
    )
    download_data(
        pkg_name=PKG,
        path=list_path,
        url='https://drive.google.com/uc?id=1NQJhJ14Qx0fi3ZnyvsVaBna1FAQrGhTm',
        md5='a8f21bb6e046c0d20dbbcf12bf5ddd75',
        quiet=quiet,
    )
    model_list[model_name] = {}
    model_list[model_name]["model"] = osp.join(pkg_path, model_path)
    model_list[model_name]["class_list"] = osp.join(pkg_path, list_path)

    # hand detector
    model_name = "final"
    model_path = "models/chainer/final_detector/final_model.npz"
    list_path = 'models/chainer/final_detector/final_class_list.txt'
    download_data(
        pkg_name=PKG,
        path=model_path,
        url='https://drive.google.com/uc?id=1dX9sQxntSu9WdabKrK6LSvvTCuvuWWlJ',
        md5='7e4a8f367742d8c599f4292ce75085f1',
        quiet=quiet,
    )
    download_data(
        pkg_name=PKG,
        path=list_path,
        url='https://drive.google.com/uc?id=10Xa2wGR3MhZ5bVZAbaYqlU3z9UurXdMP',
        md5='b5ec511cf04244e0fa6c7e4b2453e15d',
        quiet=quiet,
    )
    model_list[model_name] = {}
    model_list[model_name]["model"] = osp.join(pkg_path, model_path)
    model_list[model_name]["class_list"] = osp.join(pkg_path, list_path)

    # hand detector
    model_name = "hand"
    model_path = "models/chainer/hand_detector/hand_model.npz"
    list_path = 'models/chainer/hand_detector/hand_class_list.txt'
    download_data(
        pkg_name=PKG,
        path=model_path,
        url='https://drive.google.com/uc?id=1NZSpDJKK6rrA7vgELSo35oJMCjLxZyw2',
        md5='b56551c02c8d3257133c79133751c5ab',
        quiet=quiet,
    )
    download_data(
        pkg_name=PKG,
        path=list_path,
        url='https://drive.google.com/uc?id=1-WDI3slMcPx3qijso-9J6ujHS_MbMgNy',
        md5='09a4e12777e2c6469cd66f27ee391d8e',
        quiet=quiet,
    )
    model_list[model_name] = {}
    model_list[model_name]["model"] = osp.join(pkg_path, model_path)
    model_list[model_name]["class_list"] = osp.join(pkg_path, list_path)

    # empty detector (for test)
    model_name = "test"
    model_path = "models/chainer/test/test_model.npz"
    list_path = 'models/chainer/test/test_class_list.txt'
    model_list[model_name] = {}
    model_list[model_name]["model"] = osp.join(pkg_path, model_path)
    model_list[model_name]["class_list"] = osp.join(pkg_path, list_path)

    list_file.write(yaml.dump(model_list, default_flow_style=False))
    list_file.close()

    ## === Pretrained SSD models ===// ##
    ## =======================
    ## =======================
    ## //=== Rotation detector templates === ##

    download_data(
        pkg_name=PKG,
        path='template/sandwich/tamago_template.tar.gz',
        url='https://drive.google.com/uc?id=1UpqEQEIEBxUq7flROnqOLxxkzZhVF2YS',
        md5='24826dc925a9796eb229259c32746055',
        quiet=quiet,
        extract=True
    )

    ## === Rotation detector templates ===// ##

if __name__ == '__main__':
    main()


