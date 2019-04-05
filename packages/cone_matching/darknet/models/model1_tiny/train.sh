#!/bin/bash
mkdir -p backup
if [[ ! -e 'backup/yolov3-tiny_3l-cones_last.weights' ]]; then
	../../darknet/darknet detector train ../../data/cones_full.data yolov3-tiny-cones.cfg ../model_darknet/darknet53.conv.74
else
	echo "restored!"
	../../darknet/darknet detector train ../../data/cones_full.data yolov3-tiny-cones.cfg backup/yolov3-tiny-cones_last.weights
fi
