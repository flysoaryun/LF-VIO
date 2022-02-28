# LF-VIO

Dataset：
```
  ID01、ID06、ID10：https://drive.google.com/drive/folders/1RdnUtMulDuhWBfAgq_CLp18EgDvTrZ89
  ID01~ID10:链接: https://pan.baidu.com/s/1o6TgcDwfcDIFl6n9dzsysA 提取码: d7wq 
```

Cancel changes
1、Build LF-VIO on ROS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/flysoaryun/LF-VIO.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

2、Run
```
    roslaunch vins_estimator mindvision.launch
```
