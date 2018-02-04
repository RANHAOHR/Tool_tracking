for (( i = 1; i < 51; i++ )); do
	cd ~/ros_ws/src/cwru_davinci/cwru_playfile_reader/play/jsp/joint1
	rosrun cwru_playfile_reader playfile_jointspace "$i".jsp 
	echo "---Moved the tool, ready for tracking--- "
	sleep 5
	rosrun tool_tracking tracking_particle
	sleep 2

	echo "---Finished $i-th configuration---"
done