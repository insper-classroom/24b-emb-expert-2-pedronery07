add_executable(main
    main.c 
)

target_link_libraries(main
    pico_stdlib
    hardware_adc
    ili9341 
    gfx
    # bme280_spi 
    hardware_spi
    hardware_dma
)