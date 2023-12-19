add_files:
	git add *
	git rm *.jpg

avg_commit:
	git add *
	git commit -m "Update"
	git push

build:
	colcon build --symlink-install --packages-select robot_move robot_description robot_bringup
	source install/setup.bash

