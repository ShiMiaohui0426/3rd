#!/bin/bash
containerName=a8cb0ed66e05
exist=`echo 'jubiparty' | sudo -S docker inspect --format '{{.State.Running}}' ${containerName}`
echo " "
if [ "${exist}" != "true" ]; then
		echo "Docker_VirCam starting"
		echo "jubiparty" | sudo -S docker start ${containerName}
		echo "jubiparty" | sudo -S modprobe v4l2loopback devices=1
		echo "Docker_VirCam started"
else
		echo "Docker_VirCam started"
fi
