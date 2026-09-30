# Main component: face recognition application
set(SOURCES "main.cpp"
    "drivers/led/led.c"
    "drivers/lcd/spilcd.c"
    "drivers/camera/camera.c"
    "drivers/face_recognition/who_ai_utils.cpp"
    "drivers/face_recognition/who_human_face_recognition.cpp"
    "drivers/fb/fb_gfx.c"
)

set(INCLUDE_DIRS "."
            "drivers"
            "drivers/led"
            "drivers/lcd"
            "drivers/camera"
            "drivers/face_recognition"
            "drivers/fb"
)

idf_component_register(SRCS ${SOURCES}
                INCLUDE_DIRS ${INCLUDE_DIRS}
                EMBED_FILES "canon.pcm"
                REQUIRES esp-dl esp32-camera nvs_flash spi_flash driver
                )
