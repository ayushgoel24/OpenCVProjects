# ubunutu is the base image

FROM ubuntu:20.04


MAINTAINER Behnam Asadi behnam.asadi@gmail.com


# this is for timezone config
ENV DEBIAN_FRONTEND=noninteractive 
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update
#-y is for accepting yes when the system asked us for installing the package
RUN apt update && apt install -y build-essential cmake git openssh-server gdb pkg-config libeigen3-dev libgtk2.0-dev locales x11-apps libsuitesparse-dev -y




# 1) gflags
RUN echo "************************ gflags ************************"
RUN git clone https://github.com/gflags/gflags
RUN mkdir -p gflags/build &&  cd gflags/build
WORKDIR "gflags/build"
RUN pwd
RUN cmake -DCMAKE_CXX_FLAGS=-std=c++11 -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON   ../ && make -j8 all install 
RUN cd ../../
RUN rm -rf gflags


# 2) glog
RUN echo "************************ glog ************************"
RUN git clone https://github.com/google/glog
WORKDIR "glog/build"
RUN mkdir -p  glog/build && cd glog/build
RUN cmake -DCMAKE_CXX_FLAGS=-std=c++11 -DCMAKE_BUILD_TYPE=Release  -DBUILD_SHARED_LIBS=ON   ../ && make -j8 all install 
WORKDIR "/"
RUN rm -rf glog


RUN echo "************************ googletest ************************"
# 3) googletest
RUN git clone https://github.com/google/googletest
RUN mkdir -p  googletest/build && cd googletest/build
WORKDIR "googletest/build"
RUN cmake -DCMAKE_CXX_FLAGS=-std=c++1z -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON ../ && make -j8 all install 
WORKDIR "/"
RUN rm -rf googletest




RUN echo "************************ ceres ************************"
# 4) ceres
RUN git clone https://github.com/ceres-solver/ceres-solver.git
RUN mkdir -p  ceres-solver/build && cd ceres-solver/build
WORKDIR "ceres-solver/build"
RUN cmake -DCMAKE_CXX_FLAGS=-std=c++1z -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON ../ && make -j8 all install 
WORKDIR "/"
RUN rm -rf ceres-solver






RUN echo "************************ opencv ************************"
# 5) opencv
RUN git clone https://github.com/opencv/opencv.git 
RUN mkdir -p  opencv/build && cd opencv/build
WORKDIR "opencv/build"
RUN cmake -DCMAKE_CXX_FLAGS=-std=c++1z -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON ../ && make -j8 all install 








RUN echo "************************ opencv_contrib ************************"
# 6) opencv_contrib
WORKDIR "/opencv"
RUN git clone https://github.com/opencv/opencv_contrib.git
RUN mkdir -p  opencv_contrib/build && cd opencv_contrib/build
WORKDIR "opencv_contrib/build"
RUN cmake -DCMAKE_CXX_FLAGS=-std=c++1z -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON ../ && make -j8 all install
