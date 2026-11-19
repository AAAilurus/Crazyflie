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
Task 1

bash
cd ~/crazyflie_ws/UTA_lighthouse/cflib_python
python3 UTA_vicon_cdc_hover.py --uri radio://0/80/2M/E7E7E7E7E7 --vicon-host 10.88.1.1 --vicon-object Crazyflie2_2 --heading-deg -90 --x 0.0 --y 0.0 --z 1.1 --yaw 0.0 --takeoff-seconds 4 --run-seconds 90 --land-z 0.05 --land-seconds 6 --loop-hz 200 --ext-pos-std 0.002 --hover-slope $(cat IRL_hw/gains/hover_slope.txt) --hover-trim $(cat IRL_hw/gains/hover_trim.txt) --kaug-file IRL_hw/gains/K_hwlagF_task1.csv --rw-z 0.04 --rw-seed 1 --eta-std 0.01 --eta-seed 1 --training-csv IRL_hw/cdc_csv/hover_test/hwlagF_task1.csv 2>&1 | tee hwlagF_task1.txt

Task 2

bash
cd ~/crazyflie_ws/UTA_lighthouse/cflib_python
python3 UTA_vicon_cdc_hover.py --uri radio://0/80/2M/E7E7E7E7E7 --vicon-host 10.88.1.1 --vicon-object Crazyflie2_2 --heading-deg -90 --x 0.0 --y 0.0 --z 1.1 --yaw 0.0 --takeoff-seconds 4 --run-seconds 90 --land-z 0.05 --land-seconds 6 --loop-hz 200 --ext-pos-std 0.002 --hover-slope $(cat IRL_hw/gains/hover_slope.txt) --hover-trim $(cat IRL_hw/gains/hover_trim.txt) --kaug-file IRL_hw/gains/K_hwlagF_task2.csv --rw-z 0.05 --rw-seed 2 --eta-std 0.01 --eta-seed 2 --training-csv IRL_hw/cdc_csv/hover_test/hwlagF_task2.csv 2>&1 | tee hwlagF_task2.txt

Task 3

bash
cd ~/crazyflie_ws/UTA_lighthouse/cflib_python
python3 UTA_vicon_cdc_hover.py --uri radio://0/80/2M/E7E7E7E7E7 --vicon-host 10.88.1.1 --vicon-object Crazyflie2_2 --heading-deg -90 --x 0.0 --y 0.0 --z 1.1 --yaw 0.0 --takeoff-seconds 4 --run-seconds 90 --land-z 0.05 --land-seconds 6 --loop-hz 200 --ext-pos-std 0.002 --hover-slope $(cat IRL_hw/gains/hover_slope.txt) --hover-trim $(cat IRL_hw/gains/hover_trim.txt) --kaug-file IRL_hw/gains/K_hwlagF_task3.csv --rw-z 0.06 --rw-seed 3 --eta-std 0.01 --eta-seed 3 --training-csv IRL_hw/cdc_csv/hover_test/hwlagF_task3.csv 2>&1 | tee hwlagF_task3.txt

Task 4

bash
cd ~/crazyflie_ws/UTA_lighthouse/cflib_python
python3 UTA_vicon_cdc_hover.py --uri radio://0/80/2M/E7E7E7E7E7 --vicon-host 10.88.1.1 --vicon-object Crazyflie2_2 --heading-deg -90 --x 0.0 --y 0.0 --z 1.1 --yaw 0.0 --takeoff-seconds 4 --run-seconds 90 --land-z 0.05 --land-seconds 6 --loop-hz 200 --ext-pos-std 0.002 --hover-slope $(cat IRL_hw/gains/hover_slope.txt) --hover-trim $(cat IRL_hw/gains/hover_trim.txt) --kaug-file IRL_hw/gains/K_hwlagF_task4.csv --rw-z 0.07 --rw-seed 4 --eta-std 0.01 --eta-seed 4 --training-csv IRL_hw/cdc_csv/hover_test/hwlagF_task4.csv 2>&1 | tee hwlagF_task4.txt

Task 5

bash
cd ~/crazyflie_ws/UTA_lighthouse/cflib_python
python3 UTA_vicon_cdc_hover.py --uri radio://0/80/2M/E7E7E7E7E7 --vicon-host 10.88.1.1 --vicon-object Crazyflie2_2 --heading-deg -90 --x 0.0 --y 0.0 --z 1.1 --yaw 0.0 --takeoff-seconds 4 --run-seconds 90 --land-z 0.05 --land-seconds 6 --loop-hz 200 --ext-pos-std 0.002 --hover-slope $(cat IRL_hw/gains/hover_slope.txt) --hover-trim $(cat IRL_hw/gains/hover_trim.txt) --kaug-file IRL_hw/gains/K_hwlagF_task5.csv --rw-z 0.08 --rw-seed 5 --eta-std 0.01 --eta-seed 5 --training-csv IRL_hw/cdc_csv/hover_test/hwlagF_task5.csv 2>&1 | tee hwlagF_task5.txt

Task 6

bash
cd ~/crazyflie_ws/UTA_lighthouse/cflib_python
python3 UTA_vicon_cdc_hover.py --uri radio://0/80/2M/E7E7E7E7E7 --vicon-host 10.88.1.1 --vicon-object Crazyflie2_2 --heading-deg -90 --x 0.0 --y 0.0 --z 1.1 --yaw 0.0 --takeoff-seconds 4 --run-seconds 90 --land-z 0.05 --land-seconds 6 --loop-hz 200 --ext-pos-std 0.002 --hover-slope $(cat IRL_hw/gains/hover_slope.txt) --hover-trim $(cat IRL_hw/gains/hover_trim.txt) --kaug-file IRL_hw/gains/K_hwlagF_task6.csv --rw-z 0.09 --rw-seed 6 --eta-std 0.01 --eta-seed 6 --training-csv IRL_hw/cdc_csv/hover_test/hwlagF_task6.csv 2>&1 | tee hwlagF_task6.txt

------------------------------------------------------------------------------------------------------------------

cd ~/crazyflie_ws/UTA_lighthouse/cflib_python/IRL_hw/cdc
python3 hw2_inverse.py --prefix ../cdc_csv/hover_test/hwlagE_task --truth ../gains/truth_hwlag.json --nv 2 --mem 6 --shared-phi --bias 2>&1 | tee hwlagE_inverse_out.txt | grep -E "K_bar=|shared|Q0_hat - Q0_true|^ +[1-6]  \[|mean \|\|"

-------------------------------------------------------------

