FROM ubuntu:18.04

# Install needed libraries
RUN apt-get update && \
    apt-get install -y wget git cmake vim \
                       clang libclang-dev pkg-config \
                       libc6-dbg gdb valgrind \
                       libgtk2.0-dev

# Install rust
RUN wget https://raw.githubusercontent.com/rust-lang/rustup/master/rustup-init.sh && \
    chmod +x rustup-init.sh && \
    ./rustup-init.sh -y

# Grab opencv (3.4) and contrib modules (for SIFT, etc)
RUN git clone https://github.com/opencv/opencv.git && \
    cd opencv && git checkout tags/4.5.4 && cd .. && \
    git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv_contrib && git checkout tags/4.5.4 && cd .. 

# Build opencv
RUN cd opencv && mkdir build && cd build && \
    # cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DBUILD_opencv_xfeatures2d=ON -DOPENCV_ENABLE_NONFREE=ON .. && \
    cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DBUILD_opencv_xfeatures2d=ON -DOPENCV_ENABLE_NONFREE=ON -DWITH_GTK=ON .. && \
    make -j4 install

ENV LD_LIBRARY_PATH=/usr/local/lib/
