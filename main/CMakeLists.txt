# file(GLOB_RECURSE GUI_SRCS "gui/*.c")
file(GLOB GUI_SRCS "gui_music/*.c")

idf_component_register(SRCS "m5stack-tab5-lvgl.c" "mp3player.c" ${GUI_SRCS}
                    INCLUDE_DIRS "." "gui_music")
