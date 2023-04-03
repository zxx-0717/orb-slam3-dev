// 部分操作docker中已经修改过
下载 orb-slam3  git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3

1、 find_package(OpenCV 4.4) 改为 find_package(OpenCV 4.2)

2、 CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
    修改成c++14版本    
    CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11) 
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")   // 注意这里只改-std=c++14,其对应的名字不要改，避免需要修改源程序Examples。

3、拷贝orbbec sdk1.57及对应的ros2包到src目录下，修改cmake文件，添加 include 和 lib

set(OrbbecSDK_ROOT_DIR "/workspaces/capella_orb_slam3_dev/src/Orbbec_SDK/SDK")
set(OrbbecSDK_LIBRARY_DIRS ${OrbbecSDK_ROOT_DIR}/lib)
set(OrbbecSDK_INCLUDE_DIR ${OrbbecSDK_ROOT_DIR}/include)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/CameraModels
    ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${OrbbecSDK_INCLUDE_DIR}  //添加这一句
    )

    link_directories(${OrbbecSDK_LIBRARY_DIRS}) // 添加

4、进入orb-slam3目录，执行 chmod +x build_docker.sh && ./build_docker.sh

5、Example中 *.sh 文件不存在，从 https://github.com/electech6/ORB_SLAM3_detailed_comments/tree/master中拷过来。

6、若测试Examples中数据集，要根据github上提供的地址手动下载。*.sh 中数据集文件目录对应修改

7、orb-slam 添加 package.xml,添加编译依赖关系，修改camke里projetc name 为orbslam3,与ament_cmake命名规范一致。

8、Exmaples下新增两个demo(stereo_tx1, rgbd_dabaiDCW),cmake里添加相应代码

    add_executable(stereo_tx1
        Examples/Stereo/stereo_tx1.cc)
    target_link_libraries(stereo_tx1 ${PROJECT_NAME})

    add_executable(rgbd_dabaiDCW
            Examples/RGB-D/rgbd_dabaiDCW.cc)
    target_link_libraries(rgbd_dabaiDCW ${PROJECT_NAME} OrbbecSDK)

9、 如果不需要跑D435相机demo可删除libreasen2库


------------------- 依赖库 opencv -------------------
1、Dockerfile中添加 sudo apt-get install libopencv-dev

------------------- 依赖库 eigen3 -------------------
1、在Dockerfile中添加 sudo apt-get install libeigen3-dev

------------------- 依赖库 boost -------------------
1、在Dockerfile中添加 sudo apt-get install libboost-all-dev


/*暂时源码安装的库*/

------------------- 依赖库 librealsense -------------------
    1、git clone https://github.com/IntelRealSense/librealsense.git -b master // 不要用ros2debian, rgbd_realsense_d435 编译不过，缺少变量定义。

    2、使用 colcon build 编译（Terminal->tasks->（install dependencies/build）

------------------- 依赖库 pangolin -------------------
    1、git clone --recursive https://github.com/stevenlovegrove/Pangolin.git

    2、使用 colcon build 编译（Terminal->tasks-> （install dependencies/build））

    // 新建一个文件夹存放所有需要copy的文件，避免 mv 后出现更新提示
    libreasense package.xml 更改catkin为ament_cmake

    librealsense 编译依赖 https://blog.csdn.net/qq_45779334/article/details/124304087
    libreasese 需要 x11_xrandr sudo apt-get install libxrandr-dev
    sudo apt-get install libxinerama-dev
    sudo apt-get install libsdl2-dev

