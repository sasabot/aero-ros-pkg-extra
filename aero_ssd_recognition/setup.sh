#!/bin/bash

yamlfile=$1/models/chainer/model_list.yaml

dirs=$(ls -l --time-style="long-iso" $1/models/chainer | egrep '^d' | awk '{print $8}')
num_dirs=$(echo $dirs | awk '{print NF}')

> $yamlfile

for (( i = 1; i <= $num_dirs; i++))
do
    dir=$(echo $dirs | awk '{print $i}')
    name=$(ls $1/models/chainer/$dir | grep .npz | cut -d_ -f1)
    echo -e "${name}:" >> $yamlfile
    echo -e "  class_list: $1/models/chainer/${dir}/${name}_class_list.txt" >> $yamlfile
    echo -e "  model: $1/models/chainer/${dir}/${name}_model.npz" >> $yamlfile
done
