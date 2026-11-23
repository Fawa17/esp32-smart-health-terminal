# 屏幕 UI 层（LVGL）。
#
# 依赖关系：
#   display —— 提供帧缓冲（PSRAM）与"把若干行推给屏"的能力
#   touch   —— 提供触摸状态（LVGL 的输入设备从这里读）
#   lvgl    —— 由 idf_component.yml 声明，构建时自动下载
idf_component_register(
    SRCS         "font_num_48.c" "font_num_28.c" "font_num_20.c" "ui_lvgl.c" "font_cjk_16.c"
    INCLUDE_DIRS "include"
    REQUIRES     lvgl display touch sys_pm
    PRIV_REQUIRES esp_timer esp_app_format
)

# font_cjk_16.c 是 C:\ESP32\fontgen\gen.js 用 lv_font_conv 从「等线」生成的
# 16px / 4bpp 中文子集，只含界面用到的 170 来个字 + ASCII，约 137 KB。
# 要加新汉字：改 gen.js 里的 SYMBOLS，重跑 node gen.js，再重新 build。