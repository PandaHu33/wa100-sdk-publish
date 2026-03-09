cmake_minimum_required(VERSION 3.10)
project(WA100_Examples)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ========== 目录约定 ==========
# 本仓库仅提供 SDK 头文件 + 预编译库 + 示例工程。
set(SDK_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(EXAMPLES_DIR ${SDK_ROOT_DIR}/examples)
set(WA100_EXAMPLE_CONFIG ${EXAMPLES_DIR}/serial_config.json)

include_directories(${SDK_ROOT_DIR}/include)

# ========== 平台检测 ==========
if(WIN32)
    if(MSVC)
        set(PLATFORM_DIR "win-msvc")
        set(DLL_SUFFIX ".dll")
    elseif(MINGW)
        set(PLATFORM_DIR "win-wingw32")
        set(DLL_SUFFIX ".dll")
    else()
        message(FATAL_ERROR "Windows 平台仅支持 MSVC 或 MinGW")
    endif()
elseif(UNIX)
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(PLATFORM_DIR "linux-arm_64")
    else()
        set(PLATFORM_DIR "linux-x86_64")
    endif()
else()
    message(FATAL_ERROR "不支持的平台")
endif()

# ========== 路径配置 ==========
set(LIB_DIR ${SDK_ROOT_DIR}/lib/${PLATFORM_DIR})
link_directories(${LIB_DIR})

# ========== 查找库文件 ==========
find_library(WA100_SDK_LIB
    NAMES wa100_sdk
    PATHS ${LIB_DIR}
    NO_DEFAULT_PATH
)

if(WIN32)
    set(WA100_SDK_DLL ${LIB_DIR}/wa100_sdk${DLL_SUFFIX})
endif()

if(NOT WA100_SDK_LIB)
    message(FATAL_ERROR "找不到WA100 SDK库文件！\n"
                        "平台: ${PLATFORM_DIR}\n"
                        "搜索路径: ${LIB_DIR}\n"
                        "请确保库文件存在于: ${SDK_ROOT_DIR}/lib/${PLATFORM_DIR}/")
endif()

message(STATUS "========================================")
message(STATUS "WA100 SDK Examples 配置")
message(STATUS "========================================")
message(STATUS "平台目录: ${PLATFORM_DIR}")
message(STATUS "编译器: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "C++标准: ${CMAKE_CXX_STANDARD}")
message(STATUS "SDK根目录: ${SDK_ROOT_DIR}")
message(STATUS "示例目录: ${EXAMPLES_DIR}")
message(STATUS "库目录: ${LIB_DIR}")
message(STATUS "SDK库文件: ${WA100_SDK_LIB}")
if(WIN32)
    message(STATUS "DLL文件: ${WA100_SDK_DLL}")
endif()
message(STATUS "========================================")

# ========== 编译选项 ==========
if(MSVC)
    add_compile_options(/utf-8)
    add_compile_options(/W3)
    add_compile_options(
        /wd4251
        /wd4275
        /wd4819
        /wd4459
    )
    add_definitions(-DUNICODE -D_UNICODE)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    add_compile_options(-finput-charset=UTF-8 -fexec-charset=UTF-8)
endif()

set(WA100_RUNTIME_DIR ${CMAKE_BINARY_DIR}/bin)

function(wa100_add_demo target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} ${WA100_SDK_LIB})

    set_target_properties(${target_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${WA100_RUNTIME_DIR}
        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${WA100_RUNTIME_DIR}
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${WA100_RUNTIME_DIR}
    )

    if(WIN32 AND EXISTS ${WA100_SDK_DLL})
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${WA100_SDK_DLL}
            ${WA100_RUNTIME_DIR}
            COMMENT "复制 wa100_sdk.dll"
        )
    endif()

    if(EXISTS ${WA100_EXAMPLE_CONFIG})
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${WA100_EXAMPLE_CONFIG}
            ${WA100_RUNTIME_DIR}/serial_config.json
            COMMENT "复制 serial_config.json"
        )
    endif()

    if(UNIX)
        set_target_properties(${target_name} PROPERTIES
            INSTALL_RPATH "$ORIGIN/../lib"
            BUILD_WITH_INSTALL_RPATH TRUE
        )
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${WA100_SDK_LIB}
            ${WA100_RUNTIME_DIR}/
            COMMENT "复制 libwa100_sdk.so"
        )
    endif()
endfunction()

# ========== 示例程序 ==========
wa100_add_demo(basic_demo ${EXAMPLES_DIR}/basic_demo.cpp)
wa100_add_demo(advanced_demo ${EXAMPLES_DIR}/advanced_demo.cpp)
wa100_add_demo(quickstart_demo ${EXAMPLES_DIR}/quickstart_demo.cpp)

# 新增 UDP 控制程序
wa100_add_demo(udp_receiver ${EXAMPLES_DIR}/udp_receiver.cpp)
if(WIN32)
    # Winsock 库链接
    target_link_libraries(udp_receiver ws2_32)
endif()

# ========== 安装规则 ==========
install(TARGETS basic_demo advanced_demo quickstart_demo udp_receiver
    RUNTIME DESTINATION bin
)

if(EXISTS ${WA100_EXAMPLE_CONFIG})
    install(FILES ${WA100_EXAMPLE_CONFIG} DESTINATION bin)
endif()

if(WIN32 AND EXISTS ${WA100_SDK_DLL})
    install(FILES ${WA100_SDK_DLL} DESTINATION bin)
elseif(UNIX)
    install(FILES ${WA100_SDK_LIB} DESTINATION lib)
endif()

message(STATUS "")
message(STATUS "构建目标: basic_demo, advanced_demo, quickstart_demo")
message(STATUS "输出目录: ${WA100_RUNTIME_DIR}")
message(STATUS "========================================")