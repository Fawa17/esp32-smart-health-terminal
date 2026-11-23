# OTA 组件：HTTP 下载 + 写备用分区 + 回滚保护
#
# 头文件只 include <stdbool.h> 和 esp_err.h，所以 esp_http_client / app_update
# 这些都是**私有**依赖，没必要污染上层。
idf_component_register(
    SRCS         "ota_app.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES esp_http_client app_update esp_partition
)