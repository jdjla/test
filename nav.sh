#colcon build 
cmds=(
		#"ros2 run tree_test tree_test"
		"ros2 launch rm_bringup startup.launch.py"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
