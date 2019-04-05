#!/bin/bash
mkdir -p backup
if [[ ! -e 'backup/yolov3-tiny_3l-cones_updated_last.weights' ]]; then
	../../darknet/darknet detector train ../../data/cones_updated_gray.data yolov3-tiny_3l-cones_updated.cfg ../model_darknet/darknet53.conv.74 -map
else
	echo "restored!"
	../../darknet/darknet detector train ../../data/cones_updated_gray.data yolov3-tiny_3l-cones_updated.cfg backup/yolov3-tiny_3l-cones_updated_last.weights -map
fi


for i in `seq 1 10`;
do
        echo "TRY $i"
	cp chart.png "chart$i.png" 
	echo "restored!"
	../../darknet/darknet detector train ../../data/cones_updated_gray.data yolov3-tiny_3l-cones_updated.cfg backup/yolov3-tiny_3l-cones_updated_last.weights -map
done

