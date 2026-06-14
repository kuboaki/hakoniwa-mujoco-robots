cmake_minimum_required(VERSION 3.20)

add_executable(
    spikehat_sim
    main.cpp
    sim_spikehat.c
)
target_compile_features(spikehat_sim PRIVATE cxx_std_20)
hako_configure_target_warnings(spikehat_sim)

target_include_directories(spikehat_sim
    PRIVATE ${PROJECT_ROOT_DIR}/src/mujoco-common
    PRIVATE ${PROJECT_ROOT_DIR}/src
    PRIVATE ${PROJECT_ROOT_DIR}/include
    PRIVATE ${GLFW_DIR}
    PRIVATE ${HAKO_CORE_INCLUDE_BASE}
    PRIVATE ${HAKO_CORE_INCLUDE_ROOT}
    PRIVATE ${HAKO_PDU_TYPES_INCLUDE_DIR}
)
target_include_directories(spikehat_sim SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
    ${PROJECT_ROOT_DIR}/thirdparty/nolman/single_include
    ${PROJECT_ROOT_DIR}/thirdparty/hakoniwa-core-pro/hakoniwa-pdu-registry/pdu/types
)

target_link_libraries(spikehat_sim
    msensors
    ${LIBMUJOCO}
    mujoco-common
    ${HAKO_ASSETS_LINK_TARGET}
    ${HAKO_CONDUCTOR_LINK_TARGET}
    ${HAKO_PDU_ENDPOINT_LINK_TARGET}
)
target_compile_definitions(spikehat_sim PRIVATE USE_VIEWER=$<BOOL:${USE_VIEWER}>)
hako_configure_windows_runtime(spikehat_sim)

# libspikehat_sim: sonar_radar (Python/ctypes) 等から使う共有ライブラリ。
# 実機の libspikehat と同じAPI(spikehat.h)をMuJoCo上で模倣する。
add_library(
    spikehat_sim_shared SHARED
    sim_spikehat.c
)
target_include_directories(spikehat_sim_shared
    PRIVATE ${PROJECT_ROOT_DIR}/include
)
target_include_directories(spikehat_sim_shared SYSTEM PRIVATE
    ${MUJOCO_SOURCE_DIR}
)
target_link_libraries(spikehat_sim_shared
    ${LIBMUJOCO}
)
set_target_properties(spikehat_sim_shared PROPERTIES
    OUTPUT_NAME spikehat_sim
    PREFIX lib
)
