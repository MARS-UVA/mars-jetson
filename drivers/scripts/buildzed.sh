cd ..
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
cd ..
sudo apt update
rosdep update
rosdep install --from-paths drivers --ignore-src -r -y # install dependencies
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc) # build the workspace
source ~/.bashrc