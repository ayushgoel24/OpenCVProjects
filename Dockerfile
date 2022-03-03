FROM ubuntu:latest

MAINTAINER Behnam Asadi behnam.asadi@gmail.com

#Configuring tzdata
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Berlin


# Update aptitude 
RUN apt update && apt install cmake git build-essential  gdb libeigen3-dev libgtk2.0-dev locales x11-apps -y

VOLUME /home/opencv-src

RUN cd /home/opencv-src

RUN git clone https://github.com/gflags/gflags.git
RUN git clone https://github.com/google/glog.git
RUN git clone https://github.com/ceres-solver/ceres-solver.git
RUN git clone https://github.com/opencv/opencv.git 
RUN git clone https://github.com/opencv/opencv_contrib.git


# gflags
RUN rm -rf gflags/build && mkdir gflags/build &&  cd gflags/build && cmake ../   -DCMAKE_CXX_FLAGS=-std=c++11 -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON   && make -j8 all install && cd ../../


# glog
RUN rm -rf glog/build && mkdir glog/build && cd glog/build && cmake -DCMAKE_CXX_FLAGS=-std=c++11 -DCMAKE_BUILD_TYPE=Release  -DBUILD_SHARED_LIBS=ON  -DGFLAGS_NAMESPACE=ON   .. && make -j8 all install && cd ../../


# ceres
RUN rm -rf ceres-solver/build && mkdir ceres-solver/build &&  cd ceres-solver/build && cmake -DCMAKE_CXX_FLAGS=-std=c++11 -DCMAKE_BUILD_TYPE=Release  -DBUILD_SHARED_LIBS=ON  -Dglog_DIR=/usr/lib/cmake/glog/  .. && make -j8 all install  &&  cd ../../


# opencv
RUN rm -rf opencv/build && mkdir opencv/build cd opencv/mkdir && cd opencv/build && cmake  -DCMAKE_CXX_FLAGS=-std=c++11 -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DCMAKE_BUILD_TYPE=Release -DOPENCV_ENABLED_NONFREE=True  .. && make -j8 all install


VOLUME /home/OpenCVProjects
RUN cd /home/
#RUN git clone https://github.com/behnamasadi/OpenCVProjects







