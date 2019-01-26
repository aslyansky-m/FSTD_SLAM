# Cone Detection using YOLOv3-tiny

## Architecture search
| Number | Modification            | FPS          | Accuracy |
| :----- | :-------------          | :----------: |:----------: |
| 1      | yolov3-tiny             | ~100   |    ?  |
| 2      | **1** + routes, +1 yolo layer | ?   |    ?  |
| 3      | **2** + resolution 608x416 | ?   |    ?  |
| 4      | **2** + resolution 608x608 | ?   |    ?  |
| 4      | **3** + 15 anchors         | ?   |    ?  |

  

to train:
```sh
./darknet detector train cfg_/cones.data cfg_/yolov3-tiny_3l-cones.cfg ../yolo_weights/darknet53.conv.74 -map    
```
