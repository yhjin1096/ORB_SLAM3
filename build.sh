# Get the absolute path of the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Configuring and building OpenCV 4.4 ..."

OPENCV_INSTALL_DIR="$SCRIPT_DIR/Thirdparty/opencv/install"
OPENCV_CONFIG_FILE="$OPENCV_INSTALL_DIR/lib/cmake/opencv4/OpenCVConfig.cmake"

# Check if OpenCV is already built and installed
if [ ! -f "$OPENCV_CONFIG_FILE" ]; then
    echo "OpenCV not found or not installed, building..."
    
    if [ ! -d "Thirdparty/opencv" ]; then
        mkdir -p Thirdparty/opencv
    fi
    
    cd Thirdparty/opencv
    
    if [ ! -d "opencv" ]; then
        echo "Cloning OpenCV 4.4.0 ..."
        git clone --depth 1 --branch 4.4.0 https://github.com/opencv/opencv.git
        if [ $? -ne 0 ]; then
            echo "Error: Failed to clone OpenCV"
            exit 1
        fi
    fi
    
    if [ ! -d "opencv_contrib" ]; then
        echo "Cloning OpenCV contrib 4.4.0 ..."
        git clone --depth 1 --branch 4.4.0 https://github.com/opencv/opencv_contrib.git
        if [ $? -ne 0 ]; then
            echo "Error: Failed to clone OpenCV contrib"
            exit 1
        fi
    fi
    
    cd opencv
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_INSTALL_PREFIX="$OPENCV_INSTALL_DIR" \
             -DOPENCV_EXTRA_MODULES_PATH="$SCRIPT_DIR/Thirdparty/opencv/opencv_contrib/modules" \
             -DBUILD_EXAMPLES=OFF \
             -DBUILD_DOCS=OFF \
             -DBUILD_TESTS=OFF \
             -DBUILD_PERF_TESTS=OFF
    if [ $? -ne 0 ]; then
        echo "Error: Failed to configure OpenCV"
        exit 1
    fi
    make -j$(nproc)
    if [ $? -ne 0 ]; then
        echo "Error: Failed to build OpenCV"
        exit 1
    fi
    make install
    if [ $? -ne 0 ]; then
        echo "Error: Failed to install OpenCV"
        exit 1
    fi
    cd "$SCRIPT_DIR"
else
    echo "OpenCV already built and installed, skipping build..."
fi

export OpenCV_DIR="$OPENCV_INSTALL_DIR/lib/cmake/opencv4"

cd "$SCRIPT_DIR"

echo "Configuring and building Pangolin ..."

PANGOLIN_INSTALL_DIR="$SCRIPT_DIR/Thirdparty/pangolin/install"
PANGOLIN_CONFIG_FILE="$PANGOLIN_INSTALL_DIR/lib/cmake/Pangolin/PangolinConfig.cmake"

# Check if Pangolin is already built and installed
if [ ! -f "$PANGOLIN_CONFIG_FILE" ]; then
    echo "Pangolin not found or not installed, building..."
    
    if [ ! -d "Thirdparty/pangolin" ]; then
        mkdir -p Thirdparty/pangolin
    fi
    
    cd Thirdparty/pangolin
    
    if [ ! -d "Pangolin" ]; then
        echo "Cloning Pangolin ..."
        git clone https://github.com/stevenlovegrove/Pangolin.git --recursive
        if [ $? -ne 0 ]; then
            echo "Error: Failed to clone Pangolin"
            exit 1
        fi
    fi
    
    cd Pangolin
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_INSTALL_PREFIX="$PANGOLIN_INSTALL_DIR" \
             -DBUILD_EXAMPLES=OFF \
             -DBUILD_TESTS=OFF \
             -DBUILD_PANGOLIN_PYTHON=OFF \
             -DBUILD_PANGOLIN_LIBOPENEXR=OFF
    if [ $? -ne 0 ]; then
        echo "Error: Failed to configure Pangolin"
        exit 1
    fi
    cmake --build . -j$(nproc)
    if [ $? -ne 0 ]; then
        echo "Error: Failed to build Pangolin"
        exit 1
    fi
    cmake --install .
    if [ $? -ne 0 ]; then
        echo "Error: Failed to install Pangolin"
        exit 1
    fi
    cd "$SCRIPT_DIR"
else
    echo "Pangolin already built and installed, skipping build..."
fi

export Pangolin_DIR="$PANGOLIN_INSTALL_DIR/lib/cmake/Pangolin"
export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:$PANGOLIN_INSTALL_DIR"

cd "$SCRIPT_DIR"

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd "$SCRIPT_DIR"

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

cd "$SCRIPT_DIR"

echo "Configuring and building ORB_SLAM3 ..."

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DOpenCV_DIR="$OpenCV_DIR" \
         -DPangolin_DIR="$Pangolin_DIR"
if [ $? -ne 0 ]; then
    echo "Error: Failed to configure ORB_SLAM3"
    exit 1
fi
make -j4
if [ $? -ne 0 ]; then
    echo "Error: Failed to build ORB_SLAM3"
    exit 1
fi

echo ""
echo "Build completed successfully!"
echo ""
echo "Setting LD_LIBRARY_PATH for this session..."
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$PANGOLIN_INSTALL_DIR/lib:$OPENCV_INSTALL_DIR/lib"
echo "  LD_LIBRARY_PATH has been set to: $LD_LIBRARY_PATH"
echo ""
echo "Adding LD_LIBRARY_PATH to ~/.bashrc for permanent setup..."

# Check if the setting already exists in .bashrc
BASHRC_PATH="$HOME/.bashrc"
LIB_PATH_SETTING="export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$PANGOLIN_INSTALL_DIR/lib:$OPENCV_INSTALL_DIR/lib"

if [ -f "$BASHRC_PATH" ]; then
    if grep -qF "$PANGOLIN_INSTALL_DIR/lib" "$BASHRC_PATH" && grep -qF "$OPENCV_INSTALL_DIR/lib" "$BASHRC_PATH"; then
        echo "  LD_LIBRARY_PATH setting already exists in ~/.bashrc, skipping..."
    else
        echo "" >> "$BASHRC_PATH"
        echo "# ORB_SLAM3 library paths" >> "$BASHRC_PATH"
        echo "$LIB_PATH_SETTING" >> "$BASHRC_PATH"
        echo "  Added LD_LIBRARY_PATH setting to ~/.bashrc"
    fi
else
    echo "$LIB_PATH_SETTING" > "$BASHRC_PATH"
    echo "  Created ~/.bashrc with LD_LIBRARY_PATH setting"
fi

# Source .bashrc to apply changes
echo "  Applying changes by sourcing ~/.bashrc..."
source "$BASHRC_PATH"
echo "  LD_LIBRARY_PATH is now permanently configured!"
echo ""
echo "Example executables are located in:"
echo "  - Examples/Monocular/"
echo "  - Examples/RGB-D/"
echo "  - Examples/Stereo/"
echo "  - Examples/Monocular-Inertial/"
echo "  - Examples/Stereo-Inertial/"
