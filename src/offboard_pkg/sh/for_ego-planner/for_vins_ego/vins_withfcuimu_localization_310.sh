gnome-terminal --window -e 'bash -c "source ~/realsense_ws/devel/setup.bash;roslaunch realsense2_camera rs_camera.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; ./ttyTHS0.sh; exec bash"' \
--tab -e 'bash -c "sleep 15; source ~/multifunc_ws/devel/setup.bash;roslaunch mavros px4.launch; exec bash"' \
--tab -e 'bash -c "sleep 30; rosrun vins vins_node ~/vinsfusiongpu_ws/src/vins-fusion-gpu-speed_opti/config/maxidroneyee_realsense_d435i_fcu_imu/realsense_stereo_imu_config.yaml; exec bash"' \
--tab -e 'bash -c "sleep 40; source ~/multifunc_ws/devel/setup.bash;rosrun offboard_pkg vinsfusion_withfcuimu_to_mavros; exec bash"' \
--window -e 'bash -c "sleep 45; rostopic echo /vins_estimator/odometry; exec bash"' \
--window -e 'bash -c "sleep 45; rostopic echo /mavros/local_position/pose; exec bash"' \
--window -e 'bash -c "sleep 45; rostopic echo /mavros/state; exec bash"' \


