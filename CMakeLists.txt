# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# 鍥轰欢鐗堟湰鍙凤細鍐欒繘 esp_app_desc_t锛孫TA 鍗囩骇鍚庡彲浠ョ敤瀹冪‘璁?鏂板浐浠剁湡鐨勮窇璧锋潵浜?銆?
# 姣忔鍑嗗鍙戠増灏辨敼杩欓噷锛坴1.0.1銆乿1.1.0鈥︼級锛岄厤鍚?`idf.py -p COMx coredump-info` 涓€璧风敤銆?
set(PROJECT_VER "1.5.3")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sample_project)
