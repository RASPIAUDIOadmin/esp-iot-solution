file(GLOB_RECURSE SRCS src/*.c)

set(priv_requires "raspiaudio_muse_board")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "include"
    PRIV_REQUIRES ${priv_requires})
