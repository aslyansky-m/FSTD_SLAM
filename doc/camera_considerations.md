
# Camera Considerations

## Camera should preferably:
  - be ROS integrated
  - have built-in IMU
  - be industrial: high dynamic range, global shutter, precise calibration
  - RGB is better than monochromatic although it depends on detector perforamce with grayscale images
  - have wide FOV
  - be stereo
  - have good SDK
## Available options:
  - [optor](https://github.com/optor-vis/optor_vi-stereo-v1/blob/master/optor_VI_Sensor_SDK_V1.0/Optor%20User%20Manual.pdf), [Tara](https://www.e-consystems.com/3D-USB-stereo-camera.asp), [loitor](https://www.seeedstudio.com/Optor-Cam2pc-VisualInertial-SLAM-p-2873.html) **149$**
    - pros: cheapest
    - cons: bad sync accordingly to **5**
  - [ZED mini](https://www.stereolabs.com/zed-mini/), **449$** 
    - pros: color
    - cons: rolling shutter, expensive
  - [DUO mlx](https://duo3d.com/product/duo-minilx-lv1#tab=overview), **695$**
    - pros: very wide FOV, accurate
    - cons: expensive
  - [mynt](https://mynteyeai.com/products/mynt-eye-stereo-camera), **249$**
    - pros: seems OK accordingly to **5**
  - [intel realsense d435i](https://click.intel.com/intel-realsense-depth-camera-d435i-imu.html): **199$**
    - pros:  integrated depth processor, color
    - cons: narrow baseline
## References:
  - [1 - rovio wiki](https://github.com/ethz-asl/mav_tools_public/wiki/Visual-Inertial-Sensors)
  - [2 - rovio wiki](https://github.com/ethz-asl/mav_dji_ros_interface/wiki/Visual-inertial-sensor)
  - [3 - rovio issues](https://github.com/ethz-asl/rovio/issues/84)
  - [4 - rovio issues](https://github.com/ethz-asl/rovio/issues/192#issuecomment-408401898)
  - [5 - survey of ROS supported cameras](https://rosindustrial.org/3d-camera-survey/)

## Conclusion 
- Mynt or Realsense (if for free) 
    

