source /opt/ros/humble/setup.bash
source /root/usc_lqr_gazebo_test/ros2_ws/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/root/usc_lqr_gazebo_test/ros2_ws/install/crazyflie_lqr_gz_plugin/lib
export GZ_SIM_RESOURCE_PATH=/root/usc_lqr_gazebo_test/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models:/root/usc_lqr_gazebo_test/ros2_ws/install/ros_gz_crazyflie_gazebo/share/ros_gz_crazyflie_gazebo

ros2 launch ros_gz_crazyflie_bringup crazyflie_two_simulation.launch.py
  -------------------------------------------------------------------------------------
  cd /root/usc_lqr_gazebo_test/IRL
python3 -u gazebo_irl_hover21.py --namespace cf_leader --x 0.0 \
  --k-file cdc/forward_out/K_star_task2.csv \
  --eta-roll-deg 0.8 --eta-pitch-deg 0.8 --eta-delta-T-scale-n 0.035 \
  --eta-white 1.0 --eta-seed 2 --train-seconds 21 --land-seconds 6 \
  --training-csv cdc/data/task2_run1.csv

  -------------------------------------------------------
cd /root/usc_lqr_gazebo_test/IRL
python3 demo_hover.py leader 3 cf_leader 0.0
________________________________________________
cd /root/usc_lqr_gazebo_test/IRL
python3 demo_hover.py follower 3 cf_follower 1.0
______________________________________________________________
#tasks
cd /root/usc_lqr_gazebo_test/IRL
source /opt/ros/humble/setup.bash
source /root/usc_lqr_gazebo_test/ros2_ws/install/setup.bash
unset USC_PYTHON_DIR
rm -f cdc_csv/hover_test/velC2n_task1.csv
PERT1_Z=0.1 PERT2_Z=0.1 timeout -s INT 50 python3 gazebo_irl_hover21.py \
  --namespace cf_leader --x 0.0 --k-file gains/K_cdc_hover_vel_task1.csv \
  --eta-roll-deg 0 --eta-pitch-deg 0 --eta-delta-T-scale-n 0.01 --eta-white 1.0 --eta-seed 1 \
  --train-seconds 20 --training-csv cdc_csv/hover_test/velC2n_task1.csv 2>&1 | tail -30
echo "lines: $(wc -l < cdc_csv/hover_test/velC2n_task1.csv 2>/dev/null || echo 0)"
---------------------------------------------------------------------------
cd /root/usc_lqr_gazebo_test/IRL
source /opt/ros/humble/setup.bash
source /root/usc_lqr_gazebo_test/ros2_ws/install/setup.bash
unset USC_PYTHON_DIR
rm -f cdc_csv/hover_test/velC2n_task2.csv
PERT1_Z=0.2 PERT2_Z=0.2 timeout -s INT 50 python3 gazebo_irl_hover21.py \
  --namespace cf_leader --x 0.0 --k-file gains/K_cdc_hover_vel_task2.csv \
  --eta-roll-deg 0 --eta-pitch-deg 0 --eta-delta-T-scale-n 0.01 --eta-white 1.0 --eta-seed 2 \
  --train-seconds 20 --training-csv cdc_csv/hover_test/velC2n_task2.csv 2>&1 | tail -30
echo "lines: $(wc -l < cdc_csv/hover_test/velC2n_task2.csv 2>/dev/null || echo 0)"
---------------------------------------------------------------
cd /root/usc_lqr_gazebo_test/IRL
source /opt/ros/humble/setup.bash
source /root/usc_lqr_gazebo_test/ros2_ws/install/setup.bash
unset USC_PYTHON_DIR
export PERT1_Z=0.3 PERT2_Z=0.1
rm -f cdc_csv/hover_test/lf_leader_task3.csv cdc_csv/hover_test/lf_follower_task3.csv

# follower = recovered K, at its spawn x=1.0, background
timeout -s INT 50 python3 gazebo_irl_hover21.py \
  --namespace cf_follower --x 1.0 --y 0.0 --k-file gains/K_cdc_hover_rec_task3.csv \
  --eta-roll-deg 0 --eta-pitch-deg 0 --eta-delta-T-scale-n 0 --eta-white 0 \
  --train-seconds 20 --training-csv cdc_csv/hover_test/lf_follower_task3.csv \
  > cdc_csv/logs/lf_follower_task3.log 2>&1 &
FPID=$!
sleep 2

# leader = true K, at its spawn x=0.0, foreground
timeout -s INT 50 python3 gazebo_irl_hover21.py \
  --namespace cf_leader --x 0.0 --y 0.0 --k-file gains/K_cdc_hover_vel_task3.csv \
  --eta-roll-deg 0 --eta-pitch-deg 0 --eta-delta-T-scale-n 0 --eta-white 0 \
  --train-seconds 20 --training-csv cdc_csv/hover_test/lf_leader_task3.csv \
  > cdc_csv/logs/lf_leader_task3.log 2>&1

wait $FPID
echo "leader   -> $(wc -l < cdc_csv/hover_test/lf_leader_task3.csv 2>/dev/null || echo 0) lines"
echo "follower -> $(wc -l < cdc_csv/hover_test/lf_follower_task3.csv 2>/dev/null || echo 0) lines"
------------------------------------------------------------------------------------------------------------

