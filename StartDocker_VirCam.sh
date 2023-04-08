#!/bin/bash
containerName=50592d313dad
exist=`echo '123456' | sudo -S docker inspect --format '{{.State.Running}}' ${containerName}`
echo " "
if [ "${exist}" != "true" ]; then
		echo "Docker_VirCam starting"
		echo "123456" | sudo -S docker start ${containerName}
		echo "123456" | sudo -S modprobe v4l2loopback devices=1
		echo "Docker_VirCam started"
else
		echo "Docker_VirCam started"
fi
