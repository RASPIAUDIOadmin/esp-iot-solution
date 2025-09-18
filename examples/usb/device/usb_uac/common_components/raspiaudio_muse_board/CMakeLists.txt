idf_component_register(
    SRCS "src/raspiaudio_muse_board.c"
    INCLUDE_DIRS "include"
    REQUIRES "driver" "esp_codec_dev"
)
