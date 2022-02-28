# LF-VIO

# Download Dataset
  ID01, ID06, ID10: [**Google Drive**](https://drive.google.com/drive/folders/1RdnUtMulDuhWBfAgq_CLp18EgDvTrZ89)
  
  ID01~ID10: [**Baidu Yun**](https://pan.baidu.com/s/1o6TgcDwfcDIFl6n9dzsysA), Code: d7wq 


# How to run LF-VIO
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

mask parameters:(CENTER_X、CENTER_Y、MIN_R、MAX_R)

---

4、Dataset camera parameters

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
