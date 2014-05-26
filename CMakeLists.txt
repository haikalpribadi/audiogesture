cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories (include/audiogesture)
include_directories (/usr/include/libxml2)
rosbuild_add_executable(audio_input src/AudioInput.cpp)
rosbuild_add_executable(audio_gesture_server src/AudioGestureServer.cpp src/Sample.cpp)
rosbuild_add_executable(audio_gesture_trainer src/AudioGestureTrainer.cpp)
rosbuild_add_executable(collection_generator src/CollectionGenerator.cpp)
rosbuild_add_executable(data_transformer src/DataTransformer.cpp)
rosbuild_add_executable(data_visualizer src/DataVisualizer.cpp)
rosbuild_add_executable(feature_normalizer src/FeatureNormalizer.cpp)
rosbuild_add_executable(function_extractor src/FunctionExtractor.cpp)
rosbuild_add_executable(gesture_kinect_input src/GestureKinectInput.cpp)
rosbuild_add_executable(gesture_output src/GestureOutput.cpp)
rosbuild_add_executable(gesture_publisher src/GesturePublisher.cpp)
rosbuild_add_executable(gesture_receiver src/GestureReceiver.cpp)
rosbuild_add_executable(gesture_test src/GestureTest.cpp)
rosbuild_add_executable(gesture_visualizer src/GestureVisualizer.cpp)
rosbuild_add_executable(music_extractor_buffer src/MusicExtractorBuffer.cpp)
rosbuild_add_executable(music_extractor src/MusicExtractor.cpp src/Bextract.cpp src/WekaSink.cpp)
rosbuild_add_executable(pca_extractor src/PCAExtractor.cpp)
rosbuild_add_executable(pca_regression src/PCARegression.cpp)
rosbuild_add_executable(sample_listener src/SampleListener.cpp)
rosbuild_add_executable(sample_player src/SamplePlayer.cpp)
target_link_libraries(music_extractor marsyas)
target_link_libraries(pca_extractor pca armadillo)
target_link_libraries(pca_regression pca mlpack armadillo)
