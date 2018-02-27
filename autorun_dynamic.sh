for (( i = 21; i < 51; i++ )); do
	cd ~/ros_ws/src/cwru_davinci/cwru_playfile_reader/play/jsp
	rosrun cwru_playfile_reader playfile_jointspace "test_$i".jsp &
	rosrun tool_tracking tracking_particle &
	wait
	sleep 5

	echo "---Finished $i-th configuration---"
done