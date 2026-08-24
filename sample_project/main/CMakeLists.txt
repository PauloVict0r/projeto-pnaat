
set(MODEL_FOLDER ../components) 
set(EI_SDK_FOLDER ../components/edge-impulse-sdk) 

set(include_dirs 
    ${MODEL_FOLDER} 
    ${MODEL_FOLDER}/tflite-model 
    ${MODEL_FOLDER}/model-parameters 
    ${EI_SDK_FOLDER} 
    ${EI_SDK_FOLDER}/porting/espressif/esp-dsp/modules/fft/include 
    ${EI_SDK_FOLDER}/porting/espressif/esp-dsp/modules/common/include 
) 

if(NOT CMAKE_BUILD_EARLY_EXPANSION) 
    add_definitions(-DEI_CLASSIFIER_TFLITE_ENABLE_ESP_NN=1) 
    add_definitions(-DEIDSP_USE_ESP_DSP=1) 
    if(${IDF_TARGET} STREQUAL "esp32s3") 
        add_definitions(-DEI_CLASSIFIER_TFLITE_ENABLE_ESP_NN_S3=1) 
    endif() 
endif() 

OPTION(DEFINE_DEBUG 
    "Build the project using debugging code" 
    OFF) 
IF(DEFINE_DEBUG) 
    MESSAGE("Adding Debug flag...") 
    ADD_DEFINITIONS(-DDEBUG) 
    SET(CMAKE_BUILD_TYPE Debug) 
    MESSAGE("Build type is " ${CMAKE_BUILD_TYPE}) 
ENDIF(DEFINE_DEBUG) 

include(${EI_SDK_FOLDER}/cmake/utils.cmake) 
 
RECURSIVE_FIND_FILE_EXCLUDE_DIR(SOURCE_FILES "../components/edge-impulse-sdk" "CMSIS" 
"*.cpp") 
RECURSIVE_FIND_FILE_EXCLUDE_DIR(MODEL_FILES "../components/tflite-model" "CMSIS" "*.cpp") 
RECURSIVE_FIND_FILE_EXCLUDE_DIR(CC_FILES "../components/edge-impulse-sdk" "CMSIS" "*.cc") 
RECURSIVE_FIND_FILE_EXCLUDE_DIR(C_FILES "../components/edge-impulse-sdk" "CMSIS" "*.c") 
 
RECURSIVE_FIND_FILE_EXCLUDE_DIR(S_FILES_ESP_NN "${EI_SDK_FOLDER}/porting/espressif/ESP-NN" 
"CMSIS" "*.s") 
RECURSIVE_FIND_FILE_EXCLUDE_DIR(S_FILES_ESP_DSP "${EI_SDK_FOLDER}/porting/espressif/esp-dsp/modules/fft" "CMSIS" "*.s") 
 
list(APPEND SOURCE_FILES ${C_FILES}) 
list(APPEND SOURCE_FILES ${CC_FILES}) 
list(APPEND SOURCE_FILES ${MODEL_FILES}) 
list(APPEND SOURCE_FILES ${S_FILES_ESP_NN}) 
list(APPEND SOURCE_FILES ${S_FILES_ESP_DSP}) 

idf_component_register(SRCS "main.cpp" "${SOURCE_FILES}" 
                       INCLUDE_DIRS "${include_dirs}") 
 
target_compile_options(${COMPONENT_LIB} PRIVATE  
    -std=gnu++17  
    -Wno-maybe-uninitialized  
    -Wno-missing-field-initializers 
) 