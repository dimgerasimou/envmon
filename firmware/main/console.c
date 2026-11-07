#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_idf_version.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_console.h"
#include "console.h"
#include "net.h"
#include "sensors.h"
#include "settings.h"

static const char *TAG = "console";

static int cmd_show(int argc, char **argv)
{
	settings_t cfg;

	settings_get(&cfg);
	printf("wifi_ssid  %s\n", cfg.wifi_ssid[0] ? cfg.wifi_ssid : "(not set)");
	printf("wifi_pass  %s\n", cfg.wifi_pass[0] ? "********" : "(not set)");
	printf("period     %lu s\n", (unsigned long)(cfg.period_ms / 1000));
	printf("node_name  %s\n", cfg.node_name);
	printf("mqtt_uri   %s\n", cfg.mqtt_uri[0] ? cfg.mqtt_uri : "(not set)");
	printf("mqtt_user  %s\n", cfg.mqtt_user[0] ? cfg.mqtt_user : "(anonymous)");
	printf("mqtt_pass  %s\n", cfg.mqtt_pass[0] ? "********" : "(not set)");
	return 0;
}

static int cmd_set(int argc, char **argv)
{
	if (argc != 3) {
		printf("usage: set <name> <value>   (\"\" clears a value)\n"
		       "names: wifi_ssid, wifi_pass, period, node_name, mqtt_uri, mqtt_user, "
		       "mqtt_pass\n");
		return 1;
	}

	esp_err_t err;

	if (strcmp(argv[1], "wifi_ssid") == 0) {
		err = settings_set_wifi_ssid(argv[2]);
	} else if (strcmp(argv[1], "wifi_pass") == 0) {
		err = settings_set_wifi_pass(argv[2]);
	} else if (strcmp(argv[1], "node_name") == 0) {
		err = settings_set_node_name(argv[2]);
	} else if (strcmp(argv[1], "mqtt_uri") == 0) {
		err = settings_set_mqtt_uri(argv[2]);
	} else if (strcmp(argv[1], "mqtt_user") == 0) {
		err = settings_set_mqtt_user(argv[2]);
	} else if (strcmp(argv[1], "mqtt_pass") == 0) {
		err = settings_set_mqtt_pass(argv[2]);
	} else if (strcmp(argv[1], "period") == 0) {
		char *end;
		unsigned long s = strtoul(argv[2], &end, 10);

		if (*end != '\0' || s > SENSORS_PERIOD_MAX_MS / 1000) {
			printf("period: whole seconds, %d to %d\n", SENSORS_PERIOD_MIN_MS / 1000,
			       SENSORS_PERIOD_MAX_MS / 1000);
			return 1;
		}
		err = settings_set_period(s * 1000);
		if (err == ESP_OK)
			err = sensors_set_period(s * 1000); /* apply now, not just at next boot */
	} else {
		printf("unknown setting '%s'\n", argv[1]);
		return 1;
	}

	if (err != ESP_OK) {
		printf("failed: %s\n", esp_err_to_name(err));
		return 1;
	}
	printf("saved\n");
	if (strcmp(argv[1], "period") != 0) /* only the period applies right away */
		printf("reboot to apply\n");
	return 0;
}

static void print_header(void)
{
	printf("%-12s %9s  %-6s %s\n", "quantity", "value", "unit", "source");
}

static void print_row(sensors_qty_t q, bool valid, float value, const char *src, bool chosen)
{
	const sensors_qty_info_t *info = sensors_qty_info(q);

	if (valid)
		printf("%-12s %9.2f  %-6s %-7s%s\n", info->name, value, info->unit, src,
		       chosen ? " *" : "");
	else
		printf("%-12s %9s  %-6s\n", info->name, "-", info->unit);
}

/* Chosen value per quantity. Quantities this board cannot measure are hidden. */
static void print_best(const sensors_data_t *d)
{
	print_header();
	for (sensors_qty_t q = 0; q < SENSORS_QTY_COUNT; q++) {
		if (sensors_qty_supported(q))
			print_row(q, d->qty[q].valid, d->qty[q].value, d->qty[q].src, false);
	}
}

/* Every sensor's reading, grouped by quantity; '*' marks the one chosen. */
static void print_all(const sensors_data_t *d, const sensors_readings_t *all)
{
	print_header();
	for (sensors_qty_t q = 0; q < SENSORS_QTY_COUNT; q++) {
		if (!sensors_qty_supported(q))
			continue;

		bool any = false;

		for (size_t i = 0; i < all->count; i++) {
			const sensors_reading_t *r = &all->r[i];

			if (r->qty != q)
				continue;
			/* same string pointer = same sensor that won this quantity */
			print_row(q, true, r->value, r->src, r->src == d->qty[q].src);
			any = true;
		}
		if (!any)
			print_row(q, false, 0, NULL, false);
	}
}

static int cmd_sensors(int argc, char **argv)
{
	static sensors_data_t best; /* static: keep ~450 bytes off the console stack */
	static sensors_readings_t all;
	bool show_all = argc == 2 && strcmp(argv[1], "all") == 0;

	if (argc > 2 || (argc == 2 && !show_all)) {
		printf("usage: sensors [all]\n");
		return 1;
	}
	if (sensors_get_latest(&best, &all) != ESP_OK) {
		printf("no sample yet\n");
		return 1;
	}

	if (show_all)
		print_all(&best, &all);
	else
		print_best(&best);
	return 0;
}

static void restart(void)
{
	printf("rebooting\n");
	fflush(stdout); /* let the text leave before the chip resets */
	vTaskDelay(pdMS_TO_TICKS(100));
	esp_restart();
}

static int cmd_reboot(int argc, char **argv)
{
	restart();
	return 0; /* never reached */
}

static int cmd_factory_reset(int argc, char **argv)
{
	if (argc != 2 || strcmp(argv[1], "yes") != 0) {
		printf("erases all settings; type 'factory-reset yes'\n");
		return 1;
	}

	esp_err_t err = settings_factory_reset();

	if (err != ESP_OK) {
		printf("failed: %s\n", esp_err_to_name(err));
		return 1;
	}
	restart();
	return 0;
}

static const char *reset_reason(void)
{
	switch (esp_reset_reason()) {
	case ESP_RST_POWERON:
		return "power on";
	case ESP_RST_SW:
		return "software";
	case ESP_RST_PANIC:
		return "panic";
	case ESP_RST_INT_WDT:
	case ESP_RST_TASK_WDT:
	case ESP_RST_WDT:
		return "watchdog";
	case ESP_RST_BROWNOUT:
		return "brownout";
	case ESP_RST_USB:
		return "usb";
	default:
		return "other";
	}
}

static int cmd_status(int argc, char **argv)
{
	unsigned long s = (unsigned long)(esp_timer_get_time() / 1000000); /* us since boot */

	printf("uptime     %lud %02lu:%02lu:%02lu\n", s / 86400, s / 3600 % 24, s / 60 % 60,
	       s % 60);
	printf("reset      %s\n", reset_reason());
	printf("heap       %lu free, %lu lowest\n", (unsigned long)esp_get_free_heap_size(),
	       (unsigned long)esp_get_minimum_free_heap_size());
	printf("idf        %s\n", esp_get_idf_version());

	net_status_t net;

	net_get_status(&net);
	switch (net.state) {
	case NET_OFF:
		printf("wifi       off (no wifi_ssid saved)\n");
		break;
	case NET_CONNECTING:
		printf("wifi       connecting\n");
		break;
	case NET_CONNECTED:
		printf("wifi       connected, channel %u, %d dBm\n", net.channel, net.rssi);
		printf("ip         %s\n", net.ip);
		break;
	}
	if (net.disconnects)
		printf("drops      %lu, last: %s (%u)\n", (unsigned long)net.disconnects,
		       net_reason_str(net.last_reason), net.last_reason);
	return 0;
}

esp_err_t console_start(void)
{
	esp_console_repl_t *repl = NULL;
	esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
	esp_console_dev_usb_serial_jtag_config_t dev_cfg =
	        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();

	repl_cfg.prompt = "envmon> ";
	ESP_RETURN_ON_ERROR(esp_console_new_repl_usb_serial_jtag(&dev_cfg, &repl_cfg, &repl), TAG,
	                    "new repl");

	/* esp_console_cmd_register() copies each entry, so a local array is fine */
	const esp_console_cmd_t cmds[] = {
		{ .command = "show", .help = "Show settings", .func = cmd_show },
		{ .command = "set",
		  .help = "Save a setting; run 'set' alone for the list. \"\" clears a value",
		  .hint = "<name> <value>",
		  .func = cmd_set },
		{ .command = "sensors",
		  .help = "Print the latest sample; 'all' also shows lower-priority sensors",
		  .hint = "[all]",
		  .func = cmd_sensors },
		{ .command = "status",
		  .help = "Uptime, reset reason, memory, WiFi",
		  .func = cmd_status },
		{ .command = "reboot", .help = "Restart the node", .func = cmd_reboot },
		{ .command = "factory-reset",
		  .help = "Erase all settings and restart",
		  .hint = "yes",
		  .func = cmd_factory_reset },
	};

	ESP_RETURN_ON_ERROR(esp_console_register_help_command(), TAG, "help");
	for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++)
		ESP_RETURN_ON_ERROR(esp_console_cmd_register(&cmds[i]), TAG, "register %s",
		                    cmds[i].command);

	return esp_console_start_repl(repl);
}
