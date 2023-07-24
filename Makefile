.PHONY: clean build dependencies source-dirs sim

dependencies:
	rosdep install --from-paths src --ignore-src -r -y

clean:
	rm -r ./build/ ./install/ ./log/

build:
	colcon build --symlink-install --cmake-args " -DCMAKE_BUILD_TYPE=RelWithDebInfo"

