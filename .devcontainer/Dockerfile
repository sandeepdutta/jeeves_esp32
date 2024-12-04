FROM espressif/idf:v5.3
RUN /opt/esp/python_env/idf5.3_py3.10_env/bin/pip3 install catkin_pkg lark-parser colcon-common-extensions empy==3.3.4 
RUN apt update &&\
    apt install -y udev
RUN echo "source /opt/esp/idf/export.sh" >> ~/.bashrc
