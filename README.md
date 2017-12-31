save_cloud

Kinect V2を使い、ポイントクラウドと画像を表示、保存するコード。
PLCVisualizerのtutorialと以下のサイトを参考に作成した。
https://github.com/kanezaki/ssii2016_tutorial/blob/master/save_pcd.cpp
 
環境 
Ubuntu16.04 
ROS Kinetic 

コンパイル 
$ cd ~/catkin_ws
$ catkin_make
 
実行方法  
$ roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true 
$ rosrun save_cloud save_cloud 
 
使いかた 
マウスカーソルをポイントクラウドのウインドウにおく。 
sキーを押すたびにポイントクラウドのpcdファイルと画像のpngファイルが保存さる。 
zキーで終了。 
 
以上