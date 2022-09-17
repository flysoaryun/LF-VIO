# LF-VIO

### LF-VIO: A Visual-Inertial-Odometry Framework for Large Field-of-View Cameras with Negative Plane [[PDF]](https://arxiv.org/pdf/2202.12613.pdf)

Ze Wang, [Kailun Yang](https://yangkailun.com/), Hao Shi, [Peng Li](https://person.zju.edu.cn/en/lipeng), [Fei Gao](http://zju-fast.com/fei-gao/), [Kaiwei Wang](http://wangkaiwei.org/indexeg.html).

## News 
**LF-VIO** is accepted to **IROS2022**.

## Download PALVIO Dataset
  ID01, ID06, ID10: [**Google Drive**](https://drive.google.com/drive/folders/1RdnUtMulDuhWBfAgq_CLp18EgDvTrZ89?usp=sharing)
  
  ID01~ID10: [**Baidu Yun**](https://pan.baidu.com/s/1o6TgcDwfcDIFl6n9dzsysA), Code: d7wq 

<img src="figure\LF-VIO-hardware.png" alt="Hardware" style="zoom: 10%;" />

## How to run LF-VIO
1、Build LF-VIO on ROS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/flysoaryun/LF-VIO.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
---

2、Run
```
    roslaunch vins_estimator mindvision.launch
```
---
3、Use your own PAL camera

Change config/mindvision/mindvision.yaml 

mask parameters: `(CENTER_X、CENTER_Y、MIN_R、MAX_R)`

---

4、Dataset camera parameters
Dataset include two Panoramic Annular cameras, Realsense 430, IMU(CUAV Nora) and ground truth captured by vicon.

Pal_camera(each):
```
Fov: 360°x(40°~120°)

Resolution ratio: 1280x960

Lens: Designed by Hangzhou HuanJun Technology.

Sensor: Mindvision stereo module.

Frequency: 15Hz
```

Pal_camera (up):
```
model_type: scaramuzza
camera_name: pal
image_width: 1280
image_height: 960
poly_parameters:
   p0: -2.445239e+02
   p1: 0.000000e+00
   p2: 1.748610e-03
   p3: -1.757770e-06
   p4: 4.475965e-09 
inv_poly_parameters:
   p0: 376.845565
   p1: 246.746504
   p2: 19.035187
   p3: 23.840497
   p4: 18.991943
   p5: 6.066253
   p6: 1.560387
   p7: 5.854280
   p8: 3.458320
   p9: -1.995166
   p10: -1.509264
   p11: 1.089614
   p12: 1.340245
   p13: 0.255323 
   p14: 0.0
   p15: 0.0
   p16: 0.0
   p17: 0.0
   p18: 0.0 
   p19: 0.0 
affine_parameters:
   ac: 1.000000
   ad: 0.000000
   ae: 0.000000
   cx: 645.107791
   cy: 486.025172
```

Pal_camera (down): 
```
model_type: scaramuzza
camera_name: pal
image_width: 1280
image_height: 960
poly_parameters:
   p0: -2.391784e+02
   p1: 0.000000e+00
   p2: 1.552040e-03
   p3: 9.486697e-07
   p4: 3.552717e-09 
inv_poly_parameters:
   p0: 375.457776
   p1: 247.588593
   p2: 22.263634
   p3: 28.123722
   p4: 19.186776
   p5: 5.725551
   p6: 3.414701
   p7: 5.912720
   p8: 2.4223882
   p9: -1.807970
   p10: -0.892402
   p11: 1.194095
   p12: 1.021028
   p13: 0.221771 
   p14: 0.0
   p15: 0.0
   p16: 0.0
   p17: 0.0
   p18: 0.0 
   p19: 0.0 
affine_parameters:
   ac: 1.000000
   ad: 0.000000
   ae: 0.000000
   cx: 655.565207
   cy: 479.232243

```

Realsense 430:
```
Fov: 360°x(40°~120°)

Resolution ratio: 640x480

Lens and Sensor: Designed by Intel.

Frequency: 15Hz
```

Realsense (left and right)：
```
model_type: PINHOLE
camera_name: camera
image_width: 640
image_height: 480
distortion_parameters:
   k1: 0.0
   k2: 0.0
   p1: 0.0
   p2: 0.0
projection_parameters:
   fx: 385.7544860839844
   fy: 385.7544860839844
   cx: 323.1204833984375
   cy: 236.7432098388672
```

IMU(CUAV Nora):
```
Frequency: 200Hz
acc_n: 0.02          # accelerometer measurement noise standard deviation.
gyr_n: 0.01         # gyroscope measurement noise standard deviation.    
acc_w: 0.04         # accelerometer bias random work noise standard deviation.  
gyr_w: 0.001      # gyroscope bias random work noise standard deviation.    
```

The extrinsic parameter between IMU and pal Camera(up and down):
```
body_T_cam0: !!opencv-matrix
rows: 4
cols: 4
dt: d
data: [ -1.7258304783318579e-04, -5.1284405923863796e-02,
9.9868407413161842e-01, 9.2090173104221407e-02,
-9.9977604501626516e-01, 2.1142996716590590e-02,
9.1296302071691704e-04, 3.6525428847530695e-05,
-2.1161994866424005e-02, -9.9846025629409940e-01,
-5.1276569448382769e-02, -5.0330263804913365e-02, 0., 0., 0., 1. ]
body_T_cam1: !!opencv-matrix
rows: 4
cols: 4
dt: d
data: [ 2.2687450079617033e-04, -5.1445642937752611e-02,
9.9867577038330202e-01, 8.9336321406793776e-02,
-9.9977658549267567e-01, 2.1096275328565728e-02,
1.3138751339761834e-03, -4.9743508219684610e-02,
-2.1135932166980842e-02, -9.9845294981285038e-01,
-5.1429363027847952e-02, -5.1796263625809504e-02, 0., 0., 0., 1. ]
```

The extrinsic parameter between IMU and Realsense 430:
```
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ -5.7586305857286746e-03, -4.0463318787729019e-03,
       9.9997523237933461e-01, 2.0329267950355900e-02,
       -9.9998287214160420e-01, -1.0224590553211677e-03,
       -5.7628118925283633e-03, 7.9325209639615653e-03,
       1.0457519809151661e-03, -9.9999129084997906e-01,
       -4.0403746097850135e-03, 2.8559824645148020e-03, 0., 0., 0., 1. ]

body_T_cam1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ -1.0021770212322867e-03, 3.6313480322730518e-04,
       9.9999943188700535e-01, 1.5285779565991807e-02,
       -9.9999216342926500e-01, -3.8303422615924010e-03,
       -1.0007788055728661e-03, -5.2435791444330505e-02,
       3.8299766679101843e-03, -9.9999259827824449e-01,
       3.6697063849344680e-04, 8.6931302450199057e-03, 0., 0., 0., 1. ]
```

[![Video](demo.jpg?raw=true)](https://yangkailun.com/videos/lfvio.mp4 "Video Demo")

## Publication
If you use our code or dataset, please consider referencing the following paper:

**LF-VIO: A Visual-Inertial-Odometry Framework for Large Field-of-View Cameras with Negative Plane.**
Z. Wang, K. Yang, H. Shi, P. Li, F. Gao, K. Wang. 
In 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Kyoto, Japan, October 2022.
[[**PDF**](https://arxiv.org/pdf/2202.12613)]

```
@inproceedings{wang2022lfvio,
  title={LF-VIO: A Visual-Inertial-Odometry Framework for Large Field-of-View Cameras with Negative Plane},
  author={Wang, Ze and Yang, Kailun and Shi, Hao and Li, Peng and Gao, Fei and Wang, Kaiwei},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2022}
}
```


# Acknowledgements

- We thank the authors of [VINS-MONO](https://github.com/HKUST-Aerial-Robotics/VINS-Mono), [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM) for their great jobs.
