FROM tiryoh/ros2-desktop-vnc:humble

RUN apt-get update \
    && apt-get install -y sudo \
    && apt-get install -y nano && rm -rf /var/lib/apt/lists

RUN mkdir -p /home/ubuntu/hr_ws/src && cd /home/ubuntu/hr_ws/src \
    # Download crane_x7 repositories
    && git clone -b ros2 https://github.com/rt-net/crane_x7_ros.git \
    && git clone -b ros2 https://github.com/rt-net/crane_x7_description.git\
    # Install dependencies
    && rosdep -y update && apt-get update \
    && rosdep install -r -y -i --from-paths .

# copy some source files
COPY src/crane_x7_hr_edu/ /home/ubuntu/hr_ws/src/crane_x7_hr_edu/

# build all the file
RUN cd /home/ubuntu/hr_ws \
    && source /opt/ros/humble/setup.bash \
    && colcon build

RUN echo 'source /home/ubuntu/hr_ws/install/local_setup.bash' >> ~/.bashrc
