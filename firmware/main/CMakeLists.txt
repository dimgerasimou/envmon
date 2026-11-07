idf_component_register(SRCS "main.c" "sensors.c" "settings.c" "console.c" "net.c"
	PRIV_REQUIRES board bmp280 aht10 ens160 sht40 stcc4 sen66 nvs_flash console esp_timer
	             esp_wifi esp_netif esp_event)
