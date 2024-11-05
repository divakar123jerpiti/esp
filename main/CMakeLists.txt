idf_component_register(
    SRCS  "main.c" 
          "LCD_Driver/ST7701S.c" 
          "Touch/GT911.c"
          "Touch/esp_lcd_touch/esp_lcd_touch.c" 
          "TCA9554PWR/TCA9554PWR.c"
          "test2.c"
    
    INCLUDE_DIRS "./LCD_Driver"
                 "./Touch" 
                 "./Touch/esp_lcd_touch"  
                 "TCA9554PWR"
                 "."
    
    REQUIRES freertos esp_timer esp_wifi nvs_flash esp_event esp_netif driver  esp_lcd
)
# Add the partition table
set(PARTITION_TABLE_CSV "partitions.csv")