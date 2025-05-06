idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES arduino-audio-tools EthernetESP32
                 )
project(main)
