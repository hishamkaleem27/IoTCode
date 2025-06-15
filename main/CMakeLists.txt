idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES display RFID joystick codec LED tactile SPI)
