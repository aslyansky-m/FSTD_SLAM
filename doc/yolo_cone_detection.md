# Cone Detection using YOLOv3-tiny

## Installation
follow the instructions in darknet [repository](https://github.com/AlexeyAB/darknet)

## Architecture search
| Number | Modification            | FPS          | Accuracy |
| :----- | :-------------          | :----------: |:----------: |
| 1      | yolov3-tiny             | ~100   |    ?  |
| 2      | **1** + routes, +1 yolo layer | ?   |    ?  |
| 3      | **2** + resolution 608x416 | ?   |    ?  |
| 4      | **2** + resolution 608x608 | ?   |    ?  |
| 5      | **3** + 15 anchors         | ?   |    ?  |
| 6      | **2** : gray               | ?   |    ?  |

 
## Misc
to train:
```sh
./darknet detector train cfg_/cones.data cfg_/yolov3-tiny_3l-cones.cfg ../yolo_weights/darknet53.conv.74 -map    
```
