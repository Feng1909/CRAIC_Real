#!/usr/bin/expect
 set home_ip [lindex $argv 0]
 set host_ip [lindex $argv 1]
 set password [lindex $argv 3]
 set host_name [lindex $argv 2]
 set timeout 10

 spawn ssh $host_name@$host_ip         
 expect {                 
 "*yes/no" { send "yes\r"; exp_continue}    
 "*password:" { send "$password\r"; exp_continue }
 "*Last login:" { send "export ROS_MASTER_URI=http://$home_ip:11311\r"; exp_continue }
 "ROS_MASTER_URI" { send "export ROS_IP=$host_ip\r";exp_continue }
 "ROS_IP" { send "source ~/vinsfusiongpu_ws/devel/setup.bash\r";exp_continue }
 "source" { send "rosrun vins vins_node ~/vinsfusiongpu_ws/src/vins-fusion-gpu-speed_opti/config/maxidroneyee_realsense_d435i/realsense_stereo_imu_config.yaml\r";}
 }  
 #"ROS_IP" { send "rosrun vins vins_node ~/catkin_vins/src/VINS-Fusion/config/mynteye-d/mynt_stereo_imu_config.yaml\r"; 

interact


