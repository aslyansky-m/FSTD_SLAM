#!/bin/bash
mkdir -p backup
if [[ ! -e 'backup/yolov3-tiny_3l-cones_new_last.weights' ]]; then
	../../darknet/darknet detector train ../../data/cones_full.data yolov3-tiny_3l-cones_new.cfg ../model_darknet/darknet53.conv.74
else
	echo "restored!"
	../../darknet/darknet detector train ../../data/cones_full.data yolov3-tiny_3l-cones_new.cfg backup/yolov3-tiny_3l-cones_new_last.weights
fi
