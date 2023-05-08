#!/bin/bash
#gnome-terminal -t "start_docker_vircam" -x bash -c "./StartDocker_VirCam.sh;exec bash;"
#sleep 1s
#gnome-terminal -t "pvnet" -x bash -c "cd ~/clean-pvnet; echo '123456' | sudo -S sudo /home/ros/anaconda3/envs/pvnet/bin/python kinetic_client.py ;exec bash;"
./StartDocker_VirCam.sh
cd ~/clean-pvnet
sudo /home/iwata/anaconda3/envs/pvnet/bin/python kinetic_client.py
#sleep 5s
#gnome-terminal -t "ros_docker" -x bash -c "echo '123456' | sudo -S apt-get update;sudo docker exec -it 50592d313dad bash /root/Desktop/ROS-Unity-OpenCR1.0/runHead_hand.sh;exec bash;"
