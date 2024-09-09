cmake_minimum_required(VERSION 3.16)

idf_component_register(
  SRCS
    "atk_spilcd_st7789s.c"
  INCLUDE_DIRS
    "include"
  REQUIRES
    driver
    esp_lcd
)

