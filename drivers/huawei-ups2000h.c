/*
 * huawei-ups2000h.c - Driver for Huawei UPS2000-H (6kVA-10kVA)
 *
 * The document describing the protocol implemented by this driver
 * is not open to public, you'll need an account with registered product
 * to view it at:
 *
 *   https://support.huawei.com/enterprise/en/doc/EDOC1100376943 (English)
 *   https://support.huawei.com/enterprise/zh/doc/EDOC1100216924 (Simplified Chinese)
*
 * Huawei UPS2000 driver implemented by
 *   Copyright (C) 2020, 2021 Yifeng Li <tomli@tomli.me>
 *   The author is not affiliated with Huawei or other manufacturers.
 * 
 * Huawei UPS2000-H driver implemented by
 *   Copyright (C) 2025 Nya Candy <dev@candinya.com>
 *   The author is not affiliated with Huawei or other manufacturers.
 *
 * This model is NOT compatible with the existing UPS2000 driver,
 * so we need to fork it with modification to add additional support.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "config.h"  /* must be the first header */

#include <stdbool.h>
#include <modbus.h>
#include "main.h"
#include "serial.h"
#include "nut_stdint.h"
#include "timehead.h"   /* fallback gmtime_r() variants if needed (e.g. some WIN32) */

#define DRIVER_NAME	"NUT Huawei UPS2000-H (6kVA-10kVA) RS-232 Modbus driver"
#define DRIVER_VERSION	"0.01"

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define MODBUS_SLAVE_ID 1

/*
 * Known UPS models. We only attempt to load the driver if
 * the initial communication indicates the UPS is a known
 * model of the UPS2000-H series.
 */
static const char *supported_model[] = {
	"UPS2000",
	NULL
};

/*
 * UPS2000 device identification. The information is obtained during
 * initial communication using Modbus command 0x2B (read device identi-
 * fication) to read the object 0x87 (device list). The object contains
 * a list of fields, each with a type, length, and value. The object is
 * parsed by ups2000_device_identification() and filled into the array
 * of struct ups2000_ident.
 *
 * Fields of interest are:
 *
 *     0x87, int32 (Device Count): Only one UPS unit is supported,
 *     the driver aborts if more than one device is detected.
 *
 *     0x88, string (Device Description of the 1st unit): This is a
 *     ASCII string that contains information about the 1st UPS unit.
 *     This string, again, contains a list of fields. They are parsed
 *     further into the array ups2000_desc.
 *
 */
#define UPS2000_IDENT_MAX_FIELDS 3
#define UPS2000_IDENT_MAX_LEN 128
#define UPS2000_IDENT_OFFSET
static struct {
	uint8_t type;
	uint8_t len;
	uint8_t val[UPS2000_IDENT_MAX_LEN];
} ups2000_ident[UPS2000_IDENT_MAX_FIELDS];

/*
 * UPS2000 device description. The information is initially obtained
 * as field 0x88 in the UPS2000 device identification. This field is
 * a semicolon seperated ASCII string that contains multiple fields.
 * It is parsed again by ups2000_device_identification() and filled
 * into the ups2000_desc[] 2D array. The first dimension is used as
 * a key to select the wanted field (defined in the following enmu,
 * the second dimension is a NULL-terminated ASCII string.
 *
 * Note that ups2000_desc[0] is deliberately unused, the array begins
 * at one, allowing mapping from UPS2000_DESC_* to ups2000_desc[]
 * directly without using offsets.
 */
#define UPS2000_DESC_MAX_FIELDS 9
#define UPS2000_DESC_MAX_LEN 128
enum {
	UPS2000_DESC_MANUFACTURER = 0,
	UPS2000_DESC_MODEL,
	UPS2000_DESC_REVISION,
};
static char ups2000_desc[UPS2000_DESC_MAX_FIELDS][UPS2000_DESC_MAX_LEN] = { { 0 } };

/* global variable for modbus communication */
static modbus_t *modbus_ctx = NULL;

/*
 * How many seconds to wait before switching off/on/reboot the UPS?
 *
 * This can be set at startup time via a command-line argument,
 * or at runtime by writing to RW variables "ups.delay.shutdown"
 * and "ups.delay.start". See ups2000_delay_get/set.
 */
#define UPS2000_DELAY_INVALID 0xFFFF
static uint16_t ups2000_offdelay = UPS2000_DELAY_INVALID;
static uint16_t ups2000_ondelay  = UPS2000_DELAY_INVALID;
static uint16_t ups2000_rebootdelay = UPS2000_DELAY_INVALID;

/*
 * Time when the current shutdown/reboot request is expected
 * to complete. This is used to calculate the ETA, See
 * ups2000_update_timers().
 */
static time_t shutdown_at = 0;
static time_t reboot_at = 0;
static time_t start_at = 0;

/*
 * Is it safe to enter bypass mode? It's checked by ups2000_update_alarm()
 * and used by ups2000_instcmd_bypass_start().
 */
static bool bypass_available = 0;

/* function prototypes */
static int ups2000_update_info(void);
static int ups2000_update_status(void);
static int ups2000_update_alarm(void);
static int ups2000_update_timers(void);
static void ups2000_device_identification(void);
static size_t ups2000_read_serial(uint8_t *buf, size_t buf_len);
static int ups2000_read_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest);
static int ups2000_write_register(modbus_t *ctx, int addr, uint16_t val);
static int ups2000_write_registers(modbus_t *ctx, int addr, int nb, uint16_t *src);
static uint16_t crc16(uint8_t *buffer, size_t buffer_length);
static time_t time_seek(time_t t, int seconds);

/* rw variables function prototypes */
static int ups2000_update_rw_var(void);
static int setvar(const char *name, const char *val);
static int ups2000_autostart_set(const uint16_t reg, const char *string);
static int ups2000_autostart_get(const uint16_t reg);
static int ups2000_beeper_set(const uint16_t reg, const char *string);
static int ups2000_beeper_get(const uint16_t reg);
static void ups2000_delay_get(void);
static int ups2000_delay_set(const char *var, const char *string);

/* instant command function prototypes */
static void ups2000_init_instcmd(void);
static int instcmd(const char *cmd, const char *extra);
static int ups2000_instcmd_load_on(const uint16_t reg);
static int ups2000_instcmd_bypass_start(const uint16_t reg);
static int ups2000_instcmd_beeper_toggle(const uint16_t reg);
static int ups2000_instcmd_shutdown_stayoff(const uint16_t reg);
static int ups2000_instcmd_shutdown_return(const uint16_t reg);
static int ups2000_instcmd_shutdown_reboot(const uint16_t reg);
static int ups2000_instcmd_shutdown_reboot_graceful(const uint16_t reg);

/* driver description structure */
upsdrv_info_t upsdrv_info = {
	DRIVER_NAME,
	DRIVER_VERSION,
	"Nya Candy <dev@candinya.com>\n",
	DRV_EXPERIMENTAL,
	{ NULL }
};


void upsdrv_initups(void)
{
	int r;

	upsdebugx(2, "upsdrv_initups");

	/*
	 * This is an ugly workaround to a serious problem: libmodbus doesn't
	 * support device identification. Although there's a function called
	 * modbus_send_raw_request() for custom commands, but modbus_receive_
	 * confirmation() assumes a message length in the header, which is
	 * incompatible with device identification - It simply stops reading
	 * in the middle of the message and cannot receive our message. Worse,
	 * there's no public API to receive a raw response.
	 *
	 * See: https://github.com/stephane/libmodbus/issues/231
	 *
	 * Thus, the only thing we could do is opening it as a serial device
	 * for device identification, and reopen it via libmodbus for other
	 * commands as usual. We also have to copy the CRC-16 function from
	 * the libmodbus source code since there's no public API to use that...
	 */
	upsfd = ser_open(device_path);
	ser_set_speed(upsfd, device_path, B115200);
	ser_set_rts(upsfd, 0);
	ser_set_dtr(upsfd, 0);

	modbus_ctx = modbus_new_rtu(device_path, 115200, 'N', 8, 1);
	if (modbus_ctx == NULL)
		fatalx(EXIT_FAILURE, "Unable to create the libmodbus context");

#if LIBMODBUS_VERSION_CHECK(3, 1, 2)
	/*
	 * Although it rarely occurs, it can take as slow as 2 sec. for the
	 * UPS to respond a read and finish transmitting the message.
	 */
	modbus_set_response_timeout(modbus_ctx, 2, 0);
#else
	{
		struct timeval timeout;
		timeout.tv_sec = 2;
		timeout.tv_usec = 0;
		modbus_set_response_timeout(modbus_ctx, &timeout);
	}
#endif

	r = modbus_set_slave(modbus_ctx, MODBUS_SLAVE_ID);
	if (r < 0) {
		modbus_free(modbus_ctx);
		fatalx(EXIT_FAILURE, "Invalid slave ID %d", MODBUS_SLAVE_ID);
	}

	if (modbus_connect(modbus_ctx) == -1) {
		modbus_free(modbus_ctx);
		fatalx(EXIT_FAILURE, "modbus_connect: unable to connect: %s", modbus_strerror(errno));
	}
}


#define IDENT_REQUEST_LEN 7
#define IDENT_RESPONSE_MAX_LEN 128
#define IDENT_RESPONSE_HEADER_LEN 8
#define IDENT_RESPONSE_CRC_LEN 2
#define IDENT_FIELD_HEADER_LEN 2
static void ups2000_device_identification(void)
{
	static const uint8_t ident_req[IDENT_REQUEST_LEN] = {
		MODBUS_SLAVE_ID,  /* addr */
		0x2B,             /* command: device identification */
		0x0E,             /* MEI type */
		0x01,             /* ReadDevID: extended identification */
		0x00,             /* Object ID: device list */
		0x70, 0x77        /* CRC-16 */
	};

	/*
	 * Response header:
	 * 	0x01, 0x2B, 0x0E, 0x01, 0x01, 0x00, 0x00, 0x03
	 *
	 * Response fields:
	 * 	header: 0x00, 0x06  // type (厂商名称), length
	 * 	data:   ASCII string
	 * 	(e.g. HUAWEI)
	 *
	 * 	header: 0x01, 0x07  // type (产品代码), length
	 * 	data:   ASCII string
	 * 	(e.g. UPS2000)
	 *
	 * 	header: 0x02, 0x13  // type (主要修订本), length
	 * 	data:   ASCII string
	 * 	(e.g. UPS2000 V100R021C10)
	 *
	 * CRC-16:
	 * 	0xA9, 0x9A
	 */
	static const uint8_t expected_header[IDENT_RESPONSE_HEADER_LEN] = {
		MODBUS_SLAVE_ID,
		0x2B, 0x0E, 0x01, 0x01, 0x00, 0x00, 0x03,
	};

	bool serial_fail = 0;  /* unable to read from serial */
	uint16_t crc16_recv, crc16_calc;  /* resp CRC */
	bool crc16_fail = 0;  /* resp CRC failure */
	uint8_t ident_response[IDENT_RESPONSE_MAX_LEN];  /* resp buf */
	size_t ident_response_len;    /* buf len */
	uint8_t *ident_response_end = NULL;  /* buf end marker (excluding CRC) */
	uint8_t *ptr = NULL;  /* buf iteratior */

	/* a desc string copied from ups2000_ident[] */
	char *ups2000_ident_desc = NULL;
	int i;
	ssize_t r;

	/* attempt to obtain a response header with valid CRC. */
	for (i = 0; i < 3; i++) {
		/* step 1: record response length and initialize ptr */
		upsdebugx(2, "ser_send_buf");

		ser_flush_in(upsfd, "", nut_debug_level);
		r = ser_send_buf(upsfd, ident_req, IDENT_REQUEST_LEN);
		if (r != IDENT_REQUEST_LEN) {
			fatalx(EXIT_FAILURE, "unable to send request!");
		}

		ident_response_len = ups2000_read_serial(ident_response, IDENT_RESPONSE_MAX_LEN);
		ptr = ident_response;
		ident_response_end = ptr + ident_response_len - IDENT_RESPONSE_CRC_LEN;

		/* step 2: check response length */
		if (ident_response_len == 0) {
			upslogx(LOG_ERR, "unable to read from serial port %s, retry...", device_path);
			serial_fail = 1;
			continue;
		}
		else
			serial_fail = 0;

		upsdebug_hex(2, "ups2000_read_serial() received", ptr, ident_response_len);

		if (ptr + IDENT_RESPONSE_HEADER_LEN > ident_response_end) {
			fatalx(EXIT_FAILURE, "response header too short! "
				"expected %d, received %" PRIuSIZE ".",
				IDENT_RESPONSE_HEADER_LEN, ident_response_len);
		}

		/* step 3: check response CRC-16 */
		crc16_recv = (uint16_t) ident_response_end[0] << 8 | ident_response_end[1];
		crc16_calc = crc16(ident_response, ident_response_len - IDENT_RESPONSE_CRC_LEN);
		if (crc16_recv == crc16_calc) {
			crc16_fail = 0;
			break;
		}
		crc16_fail = 1;
	}

	/* step 4: check serial & CRC-16 verification status */
	if (serial_fail)
		fatalx(EXIT_FAILURE, "unable to read from serial port %s!", device_path);

	if (crc16_fail)
		fatalx(EXIT_FAILURE, "response CRC verification failed!");

	/* step 5: check response header */
	if (memcmp(expected_header, ident_response, IDENT_RESPONSE_HEADER_LEN))
		fatalx(EXIT_FAILURE, "unexpected response header!");

	ptr += IDENT_RESPONSE_HEADER_LEN;

	/* step 6: extract ident fields */
	memset(ups2000_ident, 0x00, sizeof(ups2000_ident));
	for (i = 0; i < UPS2000_IDENT_MAX_FIELDS; i++) {
		uint8_t type, len;

		if (ptr + 2 > ident_response_end)
			break;

		type = *ptr++;
		len = *ptr++;

		if (len + 1 > UPS2000_IDENT_MAX_LEN)
			fatalx(EXIT_FAILURE, "response field too long!");

		ups2000_ident[i].type = type;
		ups2000_ident[i].len = len;
		/*
		 * Always zero-terminate the bytes, in case the data
		 * is an ASCII string (i.e. device desc string), libc
		 * string functions can be used.
		 */
		ups2000_ident[i].val[len] = '\0';

		if (ptr + len > ident_response_end)
			fatalx(EXIT_FAILURE, "response field too short!");

		memcpy(ups2000_ident[i].val, ptr, len);
		ptr += len;
	}

	/* step 7: validate device identification field 0x87 and 0x88 */
	for (i = 0; i < UPS2000_IDENT_MAX_FIELDS; i++) {
		ups2000_ident_desc = strdup((char *) ups2000_ident[i].val);

		switch (ups2000_ident[i].type) {
		case UPS2000_DESC_MANUFACTURER:
		case UPS2000_DESC_MODEL:
		case UPS2000_DESC_REVISION:
			break;
		
		default:
			// Unknown property
			continue;
		}

		memcpy(ups2000_desc[ups2000_ident[i].type], ups2000_ident_desc, strlen(ups2000_ident_desc) + 1);
	}
	free(ups2000_ident_desc);

	/*
	 * step 9: Validate desc fields that we are going to use are valid.
	 *
	 * Note: UPS2000_DESC_DEVICE_ID and UPS2000_DESC_PARALLEL_ID are
	 * currently unused and unchecked.
	 */
	for (i = UPS2000_DESC_MANUFACTURER; i <= UPS2000_DESC_REVISION; i++) {
		if (strlen(ups2000_desc[i]) == 0)
			fatalx(EXIT_FAILURE, "desc field %d is missing!", i);
	}
}


void upsdrv_initinfo(void)
{
	bool in_list = 0;
	int i = 0;

	upsdebugx(2, "upsdrv_initinfo");

	ups2000_device_identification();

	/* check whether the UPS is a known model */
	for (i = 0; supported_model[i] != NULL; i++) {
		if (!strcmp(supported_model[i],
			ups2000_desc[UPS2000_DESC_MODEL])
		) {
			in_list = 1;
		}
	}
	if (!in_list) {
		fatalx(EXIT_FAILURE, "Unknown UPS model %s",
			ups2000_desc[UPS2000_DESC_MODEL]);
	}

	dstate_setinfo("device.mfr", "%s",
		ups2000_desc[UPS2000_DESC_MANUFACTURER]);
	dstate_setinfo("device.type", "ups");
	dstate_setinfo("device.model", "%s",
		ups2000_desc[UPS2000_DESC_MODEL]);
	// dstate_setinfo("device.serial", "%s",
	// 	ups2000_desc[UPS2000_DESC_ESN]);

	dstate_setinfo("ups.mfr", "%s",
		ups2000_desc[UPS2000_DESC_MANUFACTURER]);
	dstate_setinfo("ups.model", "%s",
		ups2000_desc[UPS2000_DESC_MODEL]);
	dstate_setinfo("ups.firmware", "%s",
		ups2000_desc[UPS2000_DESC_REVISION]);
	// dstate_setinfo("ups.firmware.aux", "%s",
	// 	ups2000_desc[UPS2000_DESC_PROTOCOL_REV]);
	// dstate_setinfo("ups.serial", "%s",
	// 	ups2000_desc[UPS2000_DESC_ESN]);
	dstate_setinfo("ups.type", "online");

	dstate_setinfo("input.phases", "3");
	dstate_setinfo("output.phases", "1");

	/* RW variables */
	upsh.setvar = setvar;

	/* instant commands */
	ups2000_init_instcmd();
	upsh.instcmd = instcmd;
}


/*
 * All registers are uint16_t. But the data they represent can
 * be either an integer or a float. This information is used for
 * error checking (int and float have different invalid values).
 */
enum {
	REG_UINT16, // Unsigned
	REG_INT16, // Signed
	REG_UINT32,	/* occupies two registers */
};
#define REG_UINT16_INVALID 0xFFFFU
#define REG_INT16_INVALID 0x7FFF
#define REG_UINT32_INVALID 0xFFFFFFFFU


/*
 * Declare UPS attribute variables, format strings, registers,
 * and their scaling factors in a lookup table to avoid spaghetti
 * code.
 */
static struct {
	const char *name;
	const char *fmt;
	const uint16_t reg;
	const int datatype;    /* only UINT32 occupies 2 regs */
	const float scaling;   /* scale it down to get the original */
} ups2000_var[] =
{
	{ "input.L1.voltage",             "%.0f", 1000, REG_UINT16,  10.0  },
	{ "input.L2.voltage",             "%.0f", 1001, REG_UINT16,  10.0  },
	{ "input.L3.voltage",             "%.0f", 1002, REG_UINT16,  10.0  },
	{ "input.frequency",              "%.0f", 1003, REG_UINT16,  10.0  },
	{ "input.bypass.L1.voltage",      "%.0f", 1004, REG_UINT16,  10.0  },
	{ "input.bypass.L2.voltage",      "%.0f", 1005, REG_UINT16,  10.0  },
	{ "input.bypass.L3.voltage",      "%.0f", 1006, REG_UINT16,  10.0  },
	{ "input.bypass.frequency",       "%.0f", 1007, REG_UINT16,  10.0  },
	{ "output.voltage",            "%.0f", 1008, REG_UINT16,  10.0  },
	// { "output.L2.voltage",            "%.0f", 1009, REG_UINT16,  10.0  },
	// { "output.L3.voltage",            "%.0f", 1010, REG_UINT16,  10.0  },
	{ "output.current",            "%.0f", 1011, REG_UINT16,  10.0  },
	// { "output.L2.current",            "%.0f", 1012, REG_UINT16,  10.0  },
	// { "output.L3.current",            "%.0f", 1013, REG_UINT16,  10.0  },
	{ "output.frequency",             "%.0f", 1014, REG_UINT16,  10.0  },
	{ "output.realpower",          "%.0f", 1015, REG_UINT16,  10.0  },
	// { "output.L2.realpower",          "%.0f", 1016, REG_UINT16,  10.0  },
	// { "output.L3.realpower",          "%.0f", 1017, REG_UINT16,  10.0  },
	{ "output.power",              "%.0f", 1018, REG_UINT16,  10.0  },
	// { "output.L2.power",              "%.0f", 1019, REG_UINT16,  10.0  },
	// { "output.L3.power",              "%.0f", 1020, REG_UINT16,  10.0  },
	{ "output.load",               "%.0f", 1021, REG_UINT16,  10.0  }, // %
	// { "output.L2.load",               "%.0f", 1022, REG_UINT16,  10.0  }, // %
	// { "output.L3.load",               "%.0f", 1023, REG_UINT16,  10.0  }, // %
	{ "ups.temperature",              "%.0f", 1027, REG_INT16,   10.0  },
	{ "output.inverter.voltage",   "%.0f", 1038, REG_UINT16,  10.0  },
	// { "output.inverter.L2.voltage",   "%.0f", 1039, REG_UINT16,  10.0  },
	// { "output.inverter.L3.voltage",   "%.0f", 1040, REG_UINT16,  10.0  },
	{ "input.L1-L2.voltage",          "%.0f", 1055, REG_UINT16,  10.0  },
	{ "input.L2-L3.voltage",          "%.0f", 1056, REG_UINT16,  10.0  },
	{ "input.L3-L1.voltage",          "%.0f", 1057, REG_UINT16,  10.0  },
	{ "output.inverter.current",   "%.0f", 1058, REG_UINT16,  10.0  },
	// { "output.inverter.L2.current",   "%.0f", 1059, REG_UINT16,  10.0  },
	// { "output.inverter.L3.current",   "%.0f", 1060, REG_UINT16,  10.0  },
	{ "output.inverter.frequency",    "%.0f", 1063, REG_UINT16,  10.0  },
	// { "ambient.1.temperature",        "%.0f", 1064, REG_INT16,    10.0  }, // 环境温度
	// { "ambient.1.humidity",           "%.0f", 1065, REG_UINT16,   10.0  }, // 环境湿度
	// { "旁路运行时间",                "%.0f", 1066, REG_UINT32,   3600.0  }, // Hour
	// { "逆变运行时间",                "%.0f", 1068, REG_UINT32,   3600.0  }, // Hour
	// { "风扇寿命",                   "%.0f", 1047, REG_UINT32,   10.0  }, // Year
	// { "母线电容寿命",                "%.0f", 1048, REG_UINT32,   10.0  }, // Year
	{ "ambient.2.temperature",        "%.0f", 1347, REG_INT16,    10.0  }, // 输入线缆端子温度
	{ "device.uptime",                "%.0f", 1348, REG_UINT32,   1.0  },
	{ "ambient.3.temperature",        "%.0f", 1350, REG_INT16,    10.0  }, // 输出线缆端子温度
	{ "input.L1.current",             "%.0f", 1360, REG_UINT16,  10.0  },
	{ "input.L2.current",             "%.0f", 1361, REG_UINT16,  10.0  },
	{ "input.L3.current",             "%.0f", 1362, REG_UINT16,  10.0  },
	{ "battery.voltage",              "%.0f", 2000, REG_UINT16,  10.0  },
	{ "battery.current",              "%.0f", 2001, REG_INT16,  10.0  },
	{ "battery.charge",               "%.0f", 2003, REG_UINT16,  1.0  },
	{ "battery.runtime",              "%.0f", 2004, REG_UINT32,  1.0  },
	// { "battery.temperature",          "%.0f", 2006, REG_INT16,    10.0  },
	// { "正组电池电压",                "%.0f", 2024, REG_UINT16,  10.0  },
	// { "负组电池电压",                "%.0f", 2025, REG_UINT16,  10.0  },
	// { "正组电池电流",                "%.0f", 2026, REG_INT16,  10.0  },
	// { "负组电池电流",                "%.0f", 2027, REG_INT16,  10.0  },
	// { "电池运行时间",                "%.0f", 2088, REG_UINT32,  3600.0  }, // Hour
	{ "battery.packs",                "%.0f",   2096, REG_UINT16,  1.0  },
	{ "battery.capacity",             "%.0f",   2097, REG_UINT16,  1.0  },
	// { "电池SOH",                    "%.0f",   2098, REG_UINT16,  1.0  }, // %
	{ "battery.temperature.cell.max",  "%.0f", 2101, REG_INT16,    10.0  },
	{ "battery.temperature.cell.min",  "%.0f", 2102, REG_INT16,    10.0  },
	// { "锂电在位状态",                 "%.0f",   2103, REG_UINT32,  1.0  }, // %
	{ "battery.voltage.cell.max",      "%.0f", 2105, REG_UINT16,  10.0  }, // 注1：x1363 == 0或者1时，增益为10，x1363 == 2时，增益为1000
	{ "battery.voltage.cell.min",      "%.0f", 2106, REG_UINT16,  10.0  }, // 注1：x1363 == 0或者1时，增益为10，x1363 == 2时，增益为1000
	// { "A相并机输出有功功率",           "%.0f", 4000, REG_UINT16,  10.0  },
	// { "B相并机输出有功功率",           "%.0f", 4001, REG_UINT16,  10.0  },
	// { "C相并机输出有功功率",           "%.0f", 4002, REG_UINT16,  10.0  },
	// { "A相并机输出视在功率",           "%.0f", 4003, REG_UINT16,  10.0  },
	// { "B相并机输出视在功率",           "%.0f", 4004, REG_UINT16,  10.0  },
	// { "C相并机输出视在功率",           "%.0f", 4005, REG_UINT16,  10.0  },
	// { "A相并机负载率",                "%.0f", 4006, REG_UINT16,  10.0  },
	// { "B相并机负载率",                "%.0f", 4007, REG_UINT16,  10.0  },
	// { "C相并机负载率",                "%.0f", 4008, REG_UINT16,  10.0  },
	// { "UPS并机在线状态",              "%.0f", 4014, REG_UINT16,  1.0  },
	// { "活动告警流水号",               "%.0f", 9000, REG_UINT32,  1.0  },
	// { "历史告警流水号",               "%.0f", 9002, REG_UINT32,  1.0  },
	// { "设备列表变化流水号",            "%.0f", 9004, REG_UINT16,  1.0  },
	// { "配置信号变化流水号",            "%.0f", 9005, REG_UINT16,  1.0  },
	{ "ups.power.nominal",             "%.0f", 9009, REG_UINT16,  0.01  }, // 10 / K

	{ NULL, NULL, 0, 0, 0 },
};


static int ups2000_update_info(void)
{
	int i;

	upsdebugx(2, "ups2000_update_info");

	/*
	 * All status registers have an offset of 10000 * ups_number.
	 * We only support 1 UPS, thus it's always 10000. Register
	 * 1000 becomes 11000.
	 */

	for (i = 0; ups2000_var[i].name != NULL; i++) {
		uint16_t reg[2];
		uint16_t reg_id = 10000 + ups2000_var[i].reg;
		uint32_t val;
		uint32_t val_offset = 0;
		bool invalid = 0;

		switch (ups2000_var[i].datatype) {
		case REG_UINT16:
			if (ups2000_read_registers(modbus_ctx, reg_id, 1, &reg[0]) != 1) {
				return 1;
			}
			val = reg[0];
			if (val == REG_UINT16_INVALID)
				invalid = 1;
			break;
		case REG_INT16:
			if (ups2000_read_registers(modbus_ctx, reg_id, 1, &reg[0]) != 1) {
				return 1;
			}
			val = reg[0];
			val_offset = 0x8000; // Convert to signed
			if (val == REG_INT16_INVALID)
				invalid = 1;
			break;
		case REG_UINT32:
			if (ups2000_read_registers(modbus_ctx, reg_id, 2, &reg[0]) != 2) {
				return 1;
			}
			val  = (uint32_t)(reg[0]) << 16;
			val |= (uint32_t)(reg[1]);
			if (val == REG_UINT32_INVALID)
				invalid = 1;
			break;
		default:
			fatalx(EXIT_FAILURE, "invalid data type in register table!");
		}

		if (invalid) {
			upslogx(LOG_ERR, "register %04d has invalid value %04x,", reg_id, val);
			return 1;
		}

#ifdef HAVE_PRAGMAS_FOR_GCC_DIAGNOSTIC_IGNORED_FORMAT_NONLITERAL
#pragma GCC diagnostic push
#endif
#ifdef HAVE_PRAGMA_GCC_DIAGNOSTIC_IGNORED_FORMAT_NONLITERAL
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
#endif
#ifdef HAVE_PRAGMA_GCC_DIAGNOSTIC_IGNORED_FORMAT_SECURITY
#pragma GCC diagnostic ignored "-Wformat-security"
#endif
		dstate_setinfo(ups2000_var[i].name, ups2000_var[i].fmt,
			(float) (val - val_offset) / ups2000_var[i].scaling);
#ifdef HAVE_PRAGMAS_FOR_GCC_DIAGNOSTIC_IGNORED_FORMAT_NONLITERAL
#pragma GCC diagnostic pop
#endif
	}
	return 0;
}


/*
 * A lookup table of all the status registers and the list of
 * corresponding flags they represent. A register may set multiple
 * status flags, represented by an array of flags_t.
 *
 * There are two types of flags. If the flag is a "status flag"
 * for status_set(), for example, "OL" or "OB", the field
 * "status_name" is used. If the flag is a "data variable" for
 * dstate_setinfo(), the variable name and value is written in
 * "var_name" and "var_val" fields.
 *
 * For each flag, if it's indicated by a specific value in a
 * register, the "val" field is used. If a flag is indicated by
 * a bit, the "bit" field should be used. Fields "val" and "bit"
 * cannot be used at the same time, at least one must be "-1".
 *
 * Also, some important registers indicate basic system status
 * (e.g. whether the UPS is on line power or battery), this info
 * must always be available, and they are always expected to set
 * at least one flag. If the important register does not set any
 * flag, it means we've received an invalid or unknown value,
 * and we must report an error. The "must_set_flag" field is used
 * for this purpose.
 */

// { "供电模式",               1024,  }, // 0 均不供电, 1 旁路供电, 2 主路供电, 3 电池供电, 5 主路 ECO, 6 电池 ECO, 7 联合供电
// { "输入制式",               1025,  }, // 0 单相, 1 三相
// { "输出制式",               1026,  }, // 0 单相, 1 三相
// { "开机状态",               1028,  }, // 二进制：00 关机（可开机）, 01 开机中（中间状态）, 10 开机失败（可开机）, 11 开机完成（可关机）
// { "工作模式",               1061,  }, // 0 变频器模式, 1 ECO模式, 2 自老化模式, 3 正常模式, 4 智能在线模式
// { "选配卡类型",              1070,  }, // 0 无, 1 SNMP卡, 2 MODBUS卡, 3 干接点卡
// { "SNMP卡IP地址自动分配结果", 1057,  }, // 0 未分配, 1 分配中, 2 分配成功, 3 分配失败
// { "蜂鸣器状态",              1305,  }, // 0 未蜂鸣, 1 蜂鸣
// { "切换提示",               1306,  }, // 二进制：000 无提示, 001 关机会导致系统过载提示, 010 关机会导致系统切旁路提示, 011 关机会导致系统间断旁路提示, 100 关机会导致系统掉电提示
// { "并机开机状态",            1307,  }, // 二进制：00 关机（可开机）, 01 开机中（中间状态）, 10 开机失败（可开机）, 11 开机完成（可关机）
// { "并机切换提示",            1308,  }, // 二进制：000 无提示, 001 关机会导致系统过载提示, 010 关机会导致系统切旁路提示, 011 关机会导致系统间断旁路提示, 100 关机会导致系统掉电提示
// { "并机关机状态",            1309,  }, // 二进制：00 关机完成（可开机）, 01 关机中（中间状态）, 10 关机失败（可开机）, 11 开机（可关机）
// { "机型信息",               1344,  }, // 硬件版本： 0 6k, 1 10k, 2 15k~20k
// { "ESN写入状态",            1346,  }, // 0 未写入, 1 已写入
// { "首次上电状态",            1354,  }, // 1 首次上电, 2 非首次上电
// { "UPS系列",               1363,  }, // 0 UPS2000-G, 1 UPS2000-H-Li, 2 UPS2000-H-L
// { "品牌类型",               1372,  }, // 0 标准版, 1 双品牌, 2 品牌替换, 3 白牌
// { "电池状态",               2002,  }, // 0 未接入, 1 非充非放, 2 休眠, 3 浮充, 4 均充, 5 放电
// { "电池是否可以浮充转均充",    2011,  }, // 0 可以, 非0 不可以
// { "电池是否可以均充转浮充",    2015,  }, // 0 可以, 非0 不可以
// { "电池是否可测试状态",       2019,  }, // 0：浅放电、核对容量测试都可以；Bit15=1,且Bit14~8=0：仅浅放电测试可以 Bit15~1=0且Bit0=1（1）：核对容量测试中 Bit8=1且其余为0（0x100）：浅放电测试中 其它：不可以
// { "电池是否可结束测试",       2022,  }, // 0 可以, 非0 不可以
// { "电池是否可测试状态2",      2107,  }, // 0:分组核对性容量测试不允许 1:分组核对性容量测试允许 2:分组核对性容量测试中 其他：非法值(不允许）
// { "电池测试状态",            2108,  }, // 0:非测试状态，不显示 1:强制均充 2:浅放电测试 3:定时浅放电测试 4:核对性容量测试 5:分组核对性容量测试
// { "UPS硬件功率等级",         9007,  }, // 16 6k, 32 10k, 64 20k
// { "UPS设备连接状态",         9008,  }, // 0 断连, 1 正常, 2 通讯正常，业务功能无效（不通过此信号，判断UPS是否在线）

static struct {
	const uint16_t reg;
	bool must_set_flag;
	struct flags_t {
		const char *status_name;
		const int16_t val;
		const int bit;
		const char *var_name, *var_val;
	} flags[10];
} ups2000_status_reg[] =
{
	{ 1024, 1, {
		{ "OFF",      0, -1, NULL, NULL },
		{ "BYPASS",   1, -1, NULL, NULL },
		{ "OL",       2, -1, NULL, NULL },
		{ "OB",       3, -1, NULL, NULL },
		{ "OL ECO",   5, -1, NULL, NULL },
		{ "OB ECO",   6, -1, NULL, NULL },
		{ NULL,      -1, -1, NULL, NULL },
	}},
	/*
	 * Note: 3 = float charging, 4 = equalization charging, but
	 * both of them are reported as "charging", not "floating".
	 * The definition of "floating" in NUT is: "battery has
	 * completed its charge cycle, and waiting to go to resting
	 * mode", which is not true for UPS2000.
	 */
	{ 2002, 1, {
		{ "",         2, -1, "battery.charger.status", "resting"     },
		{ "CHRG",     3, -1, "battery.charger.status", "charging"    },
		{ "CHRG",     4, -1, "battery.charger.status", "charging"    },
		{ "DISCHRG",  5, -1, "battery.charger.status", "discharging" },
		{ NULL,      -1, -1, NULL, NULL },
	}},
	{ 2108, 0, {
		{ "CAL",     -1,  2, NULL, NULL }, // 浅放电测试
		{ "CAL",     -1,  3, NULL, NULL }, // 定时浅放电测试
		{ "CAL",     -1,  4, NULL, NULL }, // 核对性容量测试
		{ "CAL",     -1,  5, NULL, NULL }, // 分组核对性容量测试
		{ NULL,      -1, -1, NULL, NULL },
	}},
	{ 0, 0, { { NULL, -1, -1, NULL, NULL } } }
};


static int ups2000_update_status(void)
{
	int i, j;
	int r;

	upsdebugx(2, "ups2000_update_status");

	for (i = 0; ups2000_status_reg[i].reg != 0; i++) {
		uint16_t reg, val;
		struct flags_t *flag;
		int flag_count = 0;

		reg = ups2000_status_reg[i].reg;
		r = ups2000_read_registers(modbus_ctx, reg + 10000, 1, &val);
		if (r != 1)
			return 1;

		if (val == REG_UINT16_INVALID) {
			upslogx(LOG_ERR, "register %04d has invalid value %04x,", reg, val);
			return 1;
		}

		flag = ups2000_status_reg[i].flags;
		for (j = 0; flag[j].status_name != NULL; j++) {
			/*
			 * if the register is equal to the "val" we are looking
			 * for, or if register has its n-th "bit" set...
			 */
			if ((flag[j].val != -1 && flag[j].val == val) ||
			    (flag[j].bit != -1 && CHECK_BIT(val, flag[j].bit))
			) {
				/* if it has a corresponding status flag */
				if (strlen(flag[j].status_name) != 0)
					status_set(flag[j].status_name);
				/* or if it has a corresponding dstate variable (or both) */
				if (flag[j].var_name && flag[j].var_val)
					dstate_setinfo(flag[j].var_name, "%s", flag[j].var_val);
				flag_count++;
			}
		}
		if (ups2000_status_reg[i].must_set_flag && flag_count == 0) {
			upslogx(LOG_ERR, "register %04d has invalid value %04x,", reg, val);
			return 1;
		}
	}

	return 0;
}


/*
 * A lookup table of all the alarm registers and the list of
 * corresponding alarms they represent. Each alarm condition is
 * listed by its register base address "reg" and its "bit"
 * position.
 *
 * Each alarm condition has an "alarm_id", "alarm_cause_id",
 * and "alarm_name". In addition, a few alarms conditions also
 * indicates conditions related to batteries that is needed to
 * be set via status_set(), those are listed in "status_name".
 * Unused "status_name" is set to NULL.
 *
 * After an alarm is reported/cleared by the UPS, the "active"
 * flag is changed to reflect its status. The error logging
 * code uses this variable to issue warnings only when needed
 * (i.e. only after a change, avoid issuing the same warning
 * repeatedly).
 */
#define ALARM_CLEAR_AUTO      1
#define ALARM_CLEAR_MANUAL    2
#define ALARM_CLEAR_DEPENDING 3

static struct {
	bool active;             /* runtime: is this alarm currently active? */
	const uint16_t reg;      /* alarm register to check */
	const int bit;           /* alarm bit to check */
	const int alarm_clear;   /* auto or manual clear */
	const int loglevel;      /* warning or error */
	const int alarm_id, alarm_cause_id;
	const char *status_name; /* corresponding NUT status word */
	const char *alarm_name;  /* alarm string */
	const char *alarm_desc;  /* brief explanation */
} ups2000_alarm[] =
{
	{
		false, 40156, 3, ALARM_CLEAR_AUTO, LOG_ALERT,
		30, 1, NULL, "UPS internal overtemperature",
		"The ambient temperature is over 50-degree C. "
		"Startup from standby mode is prohibited.",
	},
	{
		false, 40161, 1, ALARM_CLEAR_AUTO, LOG_WARNING,
		10, 1, NULL, "Abnormal bypass voltage",
		"Bypass input is unavailable or out-of-range. Wait for "
		"bypass input to recover, or change acceptable bypass "
		"range via front panel.",
	},
	{
		false, 40161, 2, ALARM_CLEAR_AUTO, LOG_WARNING,
		10, 2, NULL, "Abnormal bypass voltage",
		"Bypass input is unavailable or out-of-range. Wait for "
		"bypass input to recover, or change acceptable bypass "
		"range via front panel.",
	},
	{
		false, 40163, 3, ALARM_CLEAR_DEPENDING, LOG_WARNING,
		25, 1, NULL, "Battery overvoltage",
		"When the UPS is started, voltage of each battery exceeds 15 V. "
		"Or: current battery voltage exceeds 14.7 V.",
	},
	{
		false, 40164, 1, ALARM_CLEAR_AUTO, LOG_WARNING,
		29, 1, "RB", "Battery needs maintenance",
		"During the last battery self-check, the battery voltage "
		"was lower than the replacement threshold (11 V).",
	},
	{
		false, 40164, 3, ALARM_CLEAR_AUTO, LOG_WARNING,
		26, 1, NULL, "Battery undervoltage",
		NULL,
	},
	{
		false, 40170, 4, ALARM_CLEAR_AUTO, LOG_ALERT,
		22, 1, NULL, "Battery disconnected",
		"Battery is not connected, has loose connection, or faulty.",
	},
	{
		false, 40173, 5, ALARM_CLEAR_AUTO, LOG_ALERT,
		66, 1, "OVER", "Output overload (105%-110%)",
		"UPS will shut down or transfer to bypass mode in 5-10 minutes.",
	},
	{
		false, 40173, 3, ALARM_CLEAR_AUTO, LOG_ALERT,
		66, 2, "OVER", "Output overload (110%-130%)",
		"UPS will shut down or transfer to bypass mode in 30-60 seconds.",
	},
	{
		false, 40174, 0, ALARM_CLEAR_DEPENDING, LOG_ALERT,
		14, 1, NULL, "UPS startup timeout",
		"The inverter output voltage is not within +/- 2 V of the "
		"rated output. Or: battery is overdischarged.",
	},
	{
		false, 40179, 14, ALARM_CLEAR_MANUAL, LOG_ALERT,
		42, 15, NULL, "Rectifier fault (internal fault)",
		"Bus voltage is lower than 320 V.",
	},
	{
		false, 40179, 15, ALARM_CLEAR_MANUAL, LOG_ALERT,
		42, 17, NULL, "Rectifier fault (internal fault)",
		"Bus voltage is higher than 450 V.",
	},
	{
		false, 40180, 1, ALARM_CLEAR_MANUAL, LOG_ALERT,
		42, 18, NULL, "Rectifier fault (internal fault)",
		"Bus voltage is lower than 260 V.",
	},
	{
		false, 40180, 5, ALARM_CLEAR_AUTO, LOG_ALERT,
		42, 24, NULL, "EEPROM fault (internal fault)",
		"Faulty EEPROM. All settings are restored to "
		"factory default and cannot be saved.",
	},
	{
		false, 40180, 6, ALARM_CLEAR_MANUAL, LOG_ALERT,
		42, 27, NULL, "Inverter fault (internal fault)",
		"Inverter output overvoltage, undervoltage or "
		"undercurrent.",
	},
	{
		false, 40180, 7, ALARM_CLEAR_DEPENDING, LOG_ALERT,
		42, 28, NULL, "Inverter fault (internal fault)",
		"The inverter output voltage is lower than 100 V.",
	},
	{
		false, 40180, 10, ALARM_CLEAR_MANUAL, LOG_ALERT,
		42, 31, NULL, "Inverter fault (internal fault)",
		"The difference between the absolute value of the positive bus "
		"voltage and that of the negative bus voltage is 100 V.",
	},
	{
		false, 40180, 11, ALARM_CLEAR_DEPENDING, LOG_ALERT,
		42, 32, NULL, "UPS internal overtemperature",
		"The ambient temperature is over 50 degree C, "
		"switching to bypass mode.",
	},
	{
		false, 40180, 13, ALARM_CLEAR_MANUAL, LOG_ALERT,
		42, 36, NULL, "Charger fault (internal fault)",
		"The charger has no output. Faulty internal connections.",
	},
	{
		false, 40182, 4, ALARM_CLEAR_MANUAL, LOG_ALERT,
		42, 42, NULL, "Charger fault (internal fault)",
		"The charger has no output while the inverter is on, "
		"battery undervoltage. Faulty switching transistor.",
	},
	{
		false, 40182, 13, ALARM_CLEAR_MANUAL, LOG_ALERT,
		66, 3, "OVER", "Output overload shutdown",
		"UPS has shutdown or transferred to bypass mode.",
	},
	{
		false, 40182, 14, ALARM_CLEAR_MANUAL, LOG_ALERT,
		66, 4, "OVER", "Bypass output overload shutdown",
		"UPS has shutdown, bypass output was overload and exceeded "
		"time limit.",
	},
	{
		false, 40182, 5, ALARM_CLEAR_MANUAL, LOG_ALERT,
		85, 1, "EPO", "Emergency power off",
		"UPS has been shutdown by EPO switch.",
	},
	{ false, 0, -1, -1, -1, -1, -1, NULL, NULL, NULL }
};


/* don't spam the syslog */
static time_t alarm_logged_since = 0;
#define UPS2000_LOG_INTERVAL 600  /* 10 minutes */


static int ups2000_update_alarm(void)
{
	uint16_t val[27];
	int i;
	int r;

	char alarm_buf[128];
	size_t all_alarms_len = 0;

	int alarm_count = 0;
	bool alarm_logged = 0;
	bool alarm_rtfm = 0;
	time_t now = time(NULL);

	upsdebugx(2, "ups2000_update_alarm");

	/*
	 * All alarm registers have an offset of 1024 * ups_number.
	 * We only support 1 UPS, it's always 1024.
	 */
	r = ups2000_read_registers(modbus_ctx, ups2000_alarm[0].reg + 1024, 27, val);
	if (r != 27)
		return 1;

	bypass_available = 1;  /* register 40161 hack, see comments below */

	for (i = 0; ups2000_alarm[i].alarm_id != -1; i++) {
		int idx = ups2000_alarm[i].reg - ups2000_alarm[0].reg;
		if (idx > 26 || idx < 0)
			fatalx(EXIT_FAILURE, "register calculation overflow!");

		if (CHECK_BIT(val[idx], ups2000_alarm[i].bit)) {
			int gotlen;
			if (ups2000_alarm[i].reg == 40161)
				/*
				 * HACK: special treatment for register 40161. If this
				 * register indicates an alarm, we need to lock the
				 * "bypass.on" command as a software foolproof mechanism.
				 * It's written to the global "bypass_available" flag.
				 */
				bypass_available = 0;

			alarm_count++;

			gotlen = snprintf(alarm_buf, 128, "(ID %02d/%02d): %s!",
				ups2000_alarm[i].alarm_id,
				ups2000_alarm[i].alarm_cause_id,
				ups2000_alarm[i].alarm_name);

			if (gotlen < 0 || (uintmax_t)gotlen > SIZE_MAX) {
				fatalx(EXIT_FAILURE, "alarm_buf preparation over/under-flow!");
			}

			all_alarms_len += (size_t)gotlen;
			alarm_set(alarm_buf);

			if (ups2000_alarm[i].status_name)
				status_set(ups2000_alarm[i].status_name);

			/*
			 * Log the warning only if it's a new alarm, or if a long time
			 * has paseed since we first warned it.
			 */
			if (!ups2000_alarm[i].active ||
			    difftime(now, alarm_logged_since) >= UPS2000_LOG_INTERVAL
			) {
				int loglevel;
				const char *alarm_word;

				/*
				 * Most text editors have syntax highlighting, adding an
				 * alarm word makes the log more readable
				 */
				loglevel = ups2000_alarm[i].loglevel;
				if (loglevel <= LOG_ERR) {
					alarm_word = "ERROR";
					/*
					 * If at least one error is serious, suggest reading
					 * manual.
					 */
					alarm_rtfm = 1;
				}
				else {
					alarm_word = "WARNING";
				}

				upslogx(loglevel, "%s: alarm %02d, Cause %02d: %s!",
					alarm_word,
					ups2000_alarm[i].alarm_id,
					ups2000_alarm[i].alarm_cause_id,
					ups2000_alarm[i].alarm_name);

				if (ups2000_alarm[i].alarm_desc)
					upslogx(loglevel, "%s", ups2000_alarm[i].alarm_desc);

				switch (ups2000_alarm[i].alarm_clear) {
				case ALARM_CLEAR_AUTO:
					upslogx(loglevel, "This alarm can be auto cleared.");
					break;
				case ALARM_CLEAR_MANUAL:
					upslogx(loglevel,
						"This alarm can only be manually cleared "
						"via front panel.");
					break;
				case ALARM_CLEAR_DEPENDING:
					upslogx(loglevel,
						"This alarm is auto or manually cleared "
						"depending on the specific problem.");
					break;
				default:
					break;
				}

				ups2000_alarm[i].active = 1;
				alarm_logged = 1;
			}

		}
		else {
			if (ups2000_alarm[i].active) {
				upslogx(LOG_WARNING,
					"Cleared alarm %02d, Cause %02d: %s",
					ups2000_alarm[i].alarm_id,
					ups2000_alarm[i].alarm_cause_id,
					ups2000_alarm[i].alarm_name);
				ups2000_alarm[i].active = 0;
				alarm_logged = 1;
			}
		}

	}

	if (alarm_count > 0) {
		/* append this to the alarm string as a friendly reminder */
		int gotlen = snprintf(alarm_buf, 128, "Check log for details!");

		if (gotlen < 0 || (uintmax_t)gotlen > SIZE_MAX) {
			fatalx(EXIT_FAILURE, "alarm_buf preparation over/under-flow!");
		}

		all_alarms_len += (size_t)gotlen;
		alarm_set(alarm_buf);

		/* if the alarm string is too long, replace it with this */
		if (all_alarms_len + 1 > ST_MAX_VALUE_LEN) {
			alarm_init();  /* discard all original alarms */
			snprintf(alarm_buf, 128, "UPS has %d alarms in effect, "
						 "check log for details!", alarm_count);
			alarm_set(alarm_buf);
		}

		/*
		 * If we are doing a syslog, write the final message and refresh the
		 * do-not-spam-the-log timer "alarm_logged_since".
		 */
		if (alarm_logged) {
			upslogx(LOG_WARNING, "UPS has %d alarms in effect.", alarm_count);
			if (alarm_rtfm)
				upslogx(LOG_WARNING, "Read Huawei User Manual for "
					"troubleshooting information.");
			alarm_logged_since = time(NULL);
		}
	}
	else {
		upsdebugx(2, "UPS has 0 alarms in effect.");

		if (alarm_logged) {
			upslogx(LOG_WARNING, "UPS has cleared all alarms.");
			alarm_logged_since = time(NULL);
		}
	}
	return 0;
}


void upsdrv_updateinfo(void)
{
	int err = 0;

	upsdebugx(2, "upsdrv_updateinfo");
	status_init();
	alarm_init();

	err += ups2000_update_timers();
	err += ups2000_update_alarm();
	err += ups2000_update_info();
	err += ups2000_update_status();
	err += ups2000_update_rw_var();

	if (err > 0) {
		upsdebugx(2, "upsdrv_updateinfo failed, data stale.");
		dstate_datastale();
		return;
	}

	alarm_commit();
	status_commit();
	dstate_dataok();
	upsdebugx(2, "upsdrv_updateinfo done");
}


/*
 * A lookup table of simple RW (configurable) variable "name", and their
 * "getter" and "setter". A "getter" function reads the variable from
 * the UPS, and a "setter" overwrites it.
 *
 * This struct only handles simple variables, delays are handled in another
 * table.
 */
static struct {
	const char *name;
	const uint16_t reg;
	int (*const getter)(const uint16_t);
	int (*const setter)(const uint16_t, const char *);
} ups2000_rw_var[] =
{
	{ "ups.start.auto",     1050, ups2000_autostart_get, ups2000_autostart_set },
	{ "ups.beeper.status",  1062, ups2000_beeper_get,    ups2000_beeper_set    },
	{ NULL, 0, NULL, NULL },
};


/*
 * A specialized lookup table of startup, reboot and shutdown delays,
 * represented by RW variables.
 */
static struct ups2000_delay_t {
	const char *name;             /* RW variable name */
	uint16_t *const global_var;   /* its corresponding global variable */
	const char *varname_cmdline;  /* cmdline argument passed to us */
	const uint16_t min;           /* minimum value allowed (seconds) */
	const uint16_t max;           /* maximum value allowed (seconds) */
	const uint8_t step;           /* can only be set in discrete steps */
	const uint16_t dfault;        /* default value */
} ups2000_rw_delay[] =
{
	/* 5940 = 99 min. */
	{ "ups.delay.shutdown", &ups2000_offdelay,    "offdelay",     6, 5940,  6, 60 },
	{ "ups.delay.reboot",   &ups2000_rebootdelay, "rebootdelay",  6, 5940,  6, 60 },
	{ "ups.delay.start",    &ups2000_ondelay,     "ondelay",     60, 5940, 60, 60 },
	{ NULL, NULL, NULL, 0, 0, 0, 0 },
};
enum {
	SHUTDOWN,
	REBOOT,
	START
};


static int ups2000_update_rw_var(void)
{
	int i;
	int r;

	upsdebugx(2, "ups2000_update_rw_var");

	for (i = 0; ups2000_rw_var[i].name != NULL; i++) {
		r = ups2000_rw_var[i].getter(ups2000_rw_var[i].reg);
		if (r != 0)
			return 1;
	}

	ups2000_delay_get();

	return 0;
}


static int setvar(const char *name, const char *val)
{
	int i;
	int r;

	for (i = 0; ups2000_rw_var[i].name != NULL; i++) {
		if (!strcasecmp(ups2000_rw_var[i].name, name)) {
			r = ups2000_rw_var[i].setter(ups2000_rw_var[i].reg, val);
			goto found;
		}
	}

	for (i = 0; ups2000_rw_delay[i].name != NULL; i++) {
		if (!strcasecmp(ups2000_rw_delay[i].name, name)) {
			r = ups2000_rw_var[i].setter(ups2000_rw_var[i].reg, val);
			goto found;
		}
	}

	return STAT_SET_UNKNOWN;

found:
	if (r == STAT_SET_FAILED)
		upslogx(LOG_ERR, "setvar: setting variable [%s] to [%s] failed", name, val);
	else if (r == STAT_SET_INVALID)
		upslogx(LOG_WARNING, "setvar: [%s] is not valid for variable [%s]", val, name);
	return r;
}


static int ups2000_autostart_get(const uint16_t reg)
{
	/*
	 * "ups.start.auto" is not supported because it overcomplicates
	 * the logic. The driver changes "ups.start.auto" internally to
	 * allow shutdown and reboot commands to do their jobs. If we make
	 * "ups.start.auto" an user configuration, it means we must (1)
	 * watch for UPS front panel updates and apply the user setting to
	 * the driver, and (2) save the restart setting temporally before
	 * restarting, track the UPS restart process, and program the value
	 * back later.
	 *
	 * Not supporting it greatly simplifies the logic - upsdrv_shutdown
	 * always put the UPS in a restartable mode, following the standard
	 * NUT behavior. Worse is better. (To prevent user confusion, we
	 * don't even report this variable, otherwise the user may attempt
	 * to change it using the front panel.
	 */
	NUT_UNUSED_VARIABLE(reg);
	return 0;
}


/*
 * Currently for internal use only, see comments above.
 */
static int ups2000_autostart_set(const uint16_t reg, const char *string)
{
	uint16_t val;
	int r;

	if (!strcasecmp(string, "yes"))
		val = 1;
	else if (!strcasecmp(string, "no"))
		val = 0;
	else
		return STAT_SET_INVALID;

	r = ups2000_write_register(modbus_ctx, reg + 10000, val);
	if (r != 1)
		return STAT_SET_FAILED;

	return STAT_SET_HANDLED;
}


static int ups2000_beeper_get(const uint16_t reg)
{
	uint16_t val;
	int r;

	r = ups2000_read_registers(modbus_ctx, reg + 10000, 1, &val);
	if (r != 1)
		return -1;

	if (val != 0 && val != 1)
		return -1;

	/*
	 * The register is "beeper disable", but we need to report whether it's
	 * enabled, thus we invert the boolean.
	 */
	if (val == 0)
		dstate_setinfo("ups.beeper.status", "enabled");
	else
		dstate_setinfo("ups.beeper.status", "disabled");

	dstate_setflags("ups.beeper.status", ST_FLAG_RW);
	dstate_addenum("ups.beeper.status", "enabled");
	dstate_addenum("ups.beeper.status", "disabled");

	return 0;
}


static int ups2000_beeper_set(const uint16_t reg, const char *string)
{
	uint16_t val;
	int r;

	if (!strcasecmp(string, "disabled") || !strcasecmp(string, "muted")) {
		/*
		 * Temporary "muted" is not supported. Only permanent "disabled"
		 * is. This is why we only support "beeper.disable" as an instant
		 * command, not "beeper.muted". But when setting it as a variable,
		 * we try to be robust here and treat both as synonyms.
		 */
		val = 1;
	}
	else if (!strcasecmp(string, "enabled"))
		val = 0;
	else
		return STAT_SET_INVALID;

	r = ups2000_write_register(modbus_ctx, reg + 10000, val);
	if (r != 1)
		return STAT_SET_FAILED;

	return STAT_SET_HANDLED;
}


/*
 * Note: variables "ups.delay.{shutdown,start,reboot}" are software-
 * only variables. We only get the user settings, validate its value
 * and store them as global variables. The actual hardware register
 * are only programmed when a shutdown/reboot is issued.
 */
static void ups2000_delay_get(void)
{
	char *cmdline;
	int i;
	int r;

	for (i = 0; ups2000_rw_delay[i].name != NULL; i++) {
		struct ups2000_delay_t *delay;

		delay = &ups2000_rw_delay[i];
		if (*delay->global_var == UPS2000_DELAY_INVALID) {
			cmdline = getval(delay->varname_cmdline);
			if (cmdline) {
				r = ups2000_delay_set(delay->name, cmdline);
				if (r != STAT_SET_HANDLED) {
					upslogx(LOG_ERR, "servar: %s is invalid. "
							 "Reverting to default %s %d seconds",
							 delay->varname_cmdline,
							 delay->varname_cmdline,
							 delay->dfault);
					*delay->global_var = delay->dfault;
				}
			}
			else {
				*delay->global_var = delay->dfault;
				upslogx(LOG_INFO, "setvar: use default %s %d seconds",
					delay->varname_cmdline, delay->dfault);
			}
		}

		dstate_setinfo(delay->name, "%d", *delay->global_var);
		dstate_setflags(delay->name, ST_FLAG_RW);
		dstate_addrange(delay->name, delay->min, delay->max);
	}
}


static int ups2000_delay_set(const char *var, const char *string)
{
	struct ups2000_delay_t *delay_schema = NULL;
	uint16_t delay, delay_rounded;
	int i;
	int r;

	r = str_to_ushort_strict(string, &delay, 10);
	if (!r)
		return STAT_SET_INVALID;

	for (i = 0; ups2000_rw_delay[i].name != NULL; i++) {
		if (!strcmp(ups2000_rw_delay[i].name, var)) {
			delay_schema = &ups2000_rw_delay[i];
			break;
		}
	}

	if (!delay_schema)
		return STAT_SET_UNKNOWN;

	if (delay > delay_schema->max)
		return STAT_SET_INVALID;
	if (delay < delay_schema->min) {
		upslogx(LOG_NOTICE, "setvar: %s [%u] is too low, "
			"it has been set to %u seconds",
			delay_schema->varname_cmdline, delay,
			delay_schema->min);
		delay = delay_schema->min;
	}

	if (delay % delay_schema->step != 0) {
		delay_rounded = delay + delay_schema->step - delay % delay_schema->step;
		upslogx(LOG_NOTICE, "setvar: %s [%u] is not a multiple of %d, "
			"it has been rounded up to %u seconds",
			delay_schema->varname_cmdline, delay,
			delay_schema->step, delay_rounded);
		delay = delay_rounded;
	}

	*delay_schema->global_var = delay;
	return STAT_SET_HANDLED;
}


/*
 * A lookup table of all instant commands "cmd" and their
 * corresponding registers "reg". For each instant command,
 * it's handled by...
 *
 * 1. One register write, by writing "val1" to "reg1", the
 * simplest case.
 *
 * 2. Two register writes, by writing "val1" to "reg1", and
 * writing "val2" to "reg2". One after another.
 *
 * 3. Calling "*handler_func" and passing "reg1". This is
 * used to handle commands that needs additional processing.
 * If "reg1" is not necessary or unsuitable, "-1" is used.
 */
#define REG_NULL  -1, -1
#define FUNC_NULL NULL

static struct ups2000_cmd_t {
	const char *cmd;
	const int16_t reg1, val1, reg2, val2;
	int (*const handler_func)(const uint16_t);
} ups2000_cmd[] =
{
	{ "test.battery.start.quick", 2020,  1, REG_NULL, FUNC_NULL },
	{ "test.battery.start.deep",  2021,  1, REG_NULL, FUNC_NULL },
	{ "test.battery.stop",        2023,  1, REG_NULL, FUNC_NULL },
	{ "beeper.enable",            1062,  0, REG_NULL, FUNC_NULL },
	{ "beeper.disable",           1062,  1, REG_NULL, FUNC_NULL },
	{ "load.off",                 1030,  1, REG_NULL, FUNC_NULL },
	{ "bypass.stop",              1029,  1, REG_NULL, FUNC_NULL },
	{ "load.on",                  1029, -1, REG_NULL, ups2000_instcmd_load_on                  },
	{ "bypass.start",             REG_NULL, REG_NULL, ups2000_instcmd_bypass_start             },
	{ "beeper.toggle",            1062, -1, REG_NULL, ups2000_instcmd_beeper_toggle            },
	{ "shutdown.stayoff",         2077, -1, REG_NULL, ups2000_instcmd_shutdown_stayoff         },
	{ "shutdown.return",          REG_NULL, REG_NULL, ups2000_instcmd_shutdown_return          },
	{ "shutdown.reboot",          REG_NULL, REG_NULL, ups2000_instcmd_shutdown_reboot          },
	{ "shutdown.reboot.graceful", REG_NULL, REG_NULL, ups2000_instcmd_shutdown_reboot_graceful },
	{ NULL, -1, -1, -1, -1, NULL },
};


static void ups2000_init_instcmd(void)
{
	int i;

	for (i = 0; ups2000_cmd[i].cmd != NULL; i++) {
		dstate_addcmd(ups2000_cmd[i].cmd);
	}
}


static int instcmd(const char *cmd, const char *extra)
{
	int i;
	int status;
	struct ups2000_cmd_t *cmd_action = NULL;
	NUT_UNUSED_VARIABLE(extra);

	for (i = 0; ups2000_cmd[i].cmd != NULL; i++) {
		if (!strcasecmp(cmd, ups2000_cmd[i].cmd)) {
			cmd_action = &ups2000_cmd[i];
		}
	}

	if (!cmd_action) {
		upslogx(LOG_WARNING, "instcmd: command [%s] unknown", cmd);
		return STAT_INSTCMD_UNKNOWN;
	}

	if (cmd_action->handler_func) {
		/* handled by a function */
		if (cmd_action->reg1 < 0) {
			upslogx(LOG_WARNING, "instcmd: command [%s] reg1 is negative", cmd);
			return STAT_INSTCMD_UNKNOWN;
		} else {
			status = cmd_action->handler_func((uint16_t)cmd_action->reg1);
		}
	}
	else if (cmd_action->reg1 >= 0 && cmd_action->val1 >= 0) {
		/* handled by a register write */
		int r = ups2000_write_register(modbus_ctx,
			10000 + cmd_action->reg1,
			(uint16_t)cmd_action->val1);
		if (r == 1)
			status = STAT_INSTCMD_HANDLED;
		else
			status = STAT_INSTCMD_FAILED;

		/*
		 * if the previous write succeeds and there is an additional
		 * register to write.
		 */
		if (r == 1 && cmd_action->reg2 >= 0 && cmd_action->val2 >= 0) {
			r = ups2000_write_register(modbus_ctx,
				10000 + cmd_action->reg2,
				(uint16_t)cmd_action->val2);
			if (r == 1)
				status = STAT_INSTCMD_HANDLED;
			else
				status = STAT_INSTCMD_FAILED;
		}
	}
	else {
		fatalx(EXIT_FAILURE, "invalid ups2000_cmd table!");
	}

	if (status == STAT_INSTCMD_FAILED)
		upslogx(LOG_ERR, "instcmd: command [%s] failed", cmd);
	else if (status == STAT_INSTCMD_HANDLED)
		upslogx(LOG_INFO, "instcmd: command [%s] handled", cmd);
	return status;
}


static int ups2000_instcmd_load_on(const uint16_t reg)
{
	int r;
	const char *status;

	/* force refresh UPS status */
	status_init();
	r = ups2000_update_status();
	if (r != 0) {
		/*
		 * When the UPS status is updated, the code must set either OL, OB, OL ECO,
		 * BYPASS, or OFF. These five options are mutually exclusive. If the register
		 * value is invalid and set none of these flags, failure code 1 is returned.
		 */
		dstate_datastale();
		return STAT_INSTCMD_FAILED;
	}
	status_commit();

	status = dstate_getinfo("ups.status");
	if (strstr(status, "OFF")) {
		/* no warning needed, continue at ups2000_write_register() below */
	}
	else if (strstr(status, "OL") || strstr(status, "OB")) {
		/*
		 * "Turning it on" has no effect if it's already on. Log a warning
		 * while still accepting and executing the command.
		 */
		upslogx(LOG_WARNING, "load.on: UPS is already on.");
		upslogx(LOG_WARNING, "load.on: still executing command anyway.");
	}
	else if (strstr(status, "BYPASS")) {
		/*
		 * If it's in bypass mode, reject this command. The UPS would otherwise
		 * enter normal mode, but "load.on" is not supposed to affect the
		 * normal/bypass status. Also log an error and suggest "bypass.stop".
		 */
		upslogx(LOG_ERR, "load.on error: UPS is already on, and is in bypass mode. "
				 "To enter normal mode, use bypass.stop");
		return STAT_INSTCMD_FAILED;
	}
	else {
		/* unreachable, see comments for r != 0 at the beginning */
		upslogx(LOG_ERR, "load.on error: invalid ups.status (%s) detected. "
				 "Please file a bug report!", status);
		return STAT_INSTCMD_FAILED;
	}

	r = ups2000_write_register(modbus_ctx, 10000 + reg, 1);
	if (r != 1)
		return STAT_INSTCMD_FAILED;
	return STAT_INSTCMD_HANDLED;
}


static int ups2000_instcmd_bypass_start(const uint16_t reg)
{
	int r;
	NUT_UNUSED_VARIABLE(reg);

	/* force update alarms */
	alarm_init();
	r = ups2000_update_alarm();
	if (r != 0)
		return STAT_INSTCMD_FAILED;
	alarm_commit();

	/* bypass input has a power failure, refuse to bypass */
	if (!bypass_available) {
		upslogx(LOG_ERR, "bypass input is abnormal, refuse to enter bypass mode.");
		return STAT_INSTCMD_FAILED;
	}

	/* enable "bypass on shutdown" */
	r = ups2000_write_register(modbus_ctx, 10000 + 1045, 1);
	if (r != 1)
		return STAT_INSTCMD_FAILED;

	/* shutdown */
	r = ups2000_write_register(modbus_ctx, 10000 + 1030, 1);
	if (r != 1)
		return STAT_INSTCMD_FAILED;

	return STAT_INSTCMD_HANDLED;
}


static int ups2000_instcmd_beeper_toggle(const uint16_t reg)
{
	int r;
	const char *string;

	r = ups2000_beeper_get(reg);
	if (r != 0)
		return STAT_INSTCMD_FAILED;

	string = dstate_getinfo("ups.beeper.status");
	if (!strcasecmp(string, "enabled"))
		r = ups2000_beeper_set(reg, "disabled");
	else if (!strcasecmp(string, "disabled"))
		r = ups2000_beeper_set(reg, "enabled");
	else
		return STAT_INSTCMD_FAILED;

	if (r != STAT_SET_HANDLED)
		return STAT_INSTCMD_FAILED;

	return STAT_INSTCMD_HANDLED;
}


/*
 * "ups.shutdown.stayoff": wait an optional offdelay and shutdown.
 * When the grid power returns, stay off.
 */
static int ups2000_instcmd_shutdown_stayoff(const uint16_t reg)
{
	uint16_t val;
	int r;

	r = setvar("ups.start.auto", "no");
	if (r != STAT_SET_HANDLED)
		return STAT_INSTCMD_FAILED;

	val = ups2000_offdelay * 10;  /* scaling factor */
	val /= 60;                    /* convert to minutes */

	r = ups2000_write_register(modbus_ctx, 10000 + reg, val);
	if (r != 1)
		return STAT_INSTCMD_FAILED;

	return STAT_INSTCMD_HANDLED;
}


/*
 * Wait for "offdelay" second, turn off the load. Then, wait
 * for "ondelay" seconds. If the grid power still exists or
 * has returned after the timer, turn on the load. Otherwise,
 * shutdown the UPS. When combined with "ups.start.auto", it
 * guarantees the server can always be restarted even if there
 * is a power race.
 *
 * "shutdown.return", "shutdown.reboot" and "shutdown.reboot.
 * graceful" all rely on this function.
 */
static int ups2000_shutdown_guaranteed_return(uint16_t offdelay, uint16_t ondelay)
{
	int r;
	uint16_t val[2];

	r = setvar("ups.start.auto", "yes");
	if (r != STAT_SET_HANDLED)
		return STAT_INSTCMD_FAILED;

	val[0] = (offdelay * 10) / 60;
	val[1] = ondelay / 60;

	r = ups2000_write_registers(modbus_ctx, 1047 + 10000, 2, val);
	if (r != 2)
		return STAT_INSTCMD_FAILED;

	return STAT_INSTCMD_HANDLED;
}


/*
 * "ups.shutdown.return": wait an optional "offdelay" and shutdown.
 * When the grid power returns, power on the load.
 */
static int ups2000_instcmd_shutdown_return(const uint16_t reg)
{
	int r;
	NUT_UNUSED_VARIABLE(reg);

	r = ups2000_shutdown_guaranteed_return(
		ups2000_offdelay,
		ups2000_ondelay);
	if (r == STAT_INSTCMD_HANDLED) {
		shutdown_at = time_seek(time(NULL), ups2000_offdelay);
	}
	return r;
}


/*
 * "ups.shutdown.reboot": shutdown as soon as possible using the
 * smallest "rebootdelay" (inside the UPS, it's the same "ondelay"
 * timer), restart after an "ondelay".
 *
 * In our implementation, it's like "ups.shutdown.return", just
 * with a minimal "ondelay".
 */
static int ups2000_instcmd_shutdown_reboot(const uint16_t reg)
{
	int r;
	NUT_UNUSED_VARIABLE(reg);

	r = ups2000_shutdown_guaranteed_return(
		ups2000_rw_delay[REBOOT].min,
		ups2000_ondelay);
	if (r == STAT_INSTCMD_HANDLED) {
		reboot_at = time_seek(time(NULL), ups2000_rw_delay[REBOOT].min);
		start_at = time_seek(reboot_at, ups2000_ondelay);
	}
	return r;
}


/*
 * "ups.shutdown.reboot.graceful": shutdown after a "rebootdelay"
 * (inside the UPS, it's the same "ondelay" timer), restart after
 * an "ondelay".
 *
 * In our implementation, it's like "ups.shutdown.return", just
 * with a "rebootdelay" instead of an "ondelay".
 */
static int ups2000_instcmd_shutdown_reboot_graceful(const uint16_t reg)
{
	int r;
	NUT_UNUSED_VARIABLE(reg);

	r = ups2000_shutdown_guaranteed_return(
		ups2000_rebootdelay,
		ups2000_ondelay);
	if (r == STAT_INSTCMD_HANDLED) {
		reboot_at = time_seek(time(NULL), ups2000_rebootdelay);
		start_at = time_seek(reboot_at, ups2000_ondelay);
	}
	return r;
}


/*
 * List of countdown timers and pointers to their corresponding
 * global variables "at_time". They record estimated timestamps
 * when the actions are supposed to be performed.
 */
static struct {
	const char *name;
	time_t *const at_time;
} ups2000_timers[] = {
	{ "ups.timer.reboot",   &reboot_at   },
	{ "ups.timer.shutdown", &shutdown_at },
	{ "ups.timer.start",    &start_at    },
	{ NULL, NULL },
};


static int ups2000_update_timers(void)
{
	time_t now;
	int eta;
	int i;

	now = time(NULL);

	for (i = 0; ups2000_timers[i].name != NULL; i++) {
		if (*ups2000_timers[i].at_time) {
			eta = difftime(*ups2000_timers[i].at_time, now);
			if (eta < 0)
				eta = 0;
			dstate_setinfo(ups2000_timers[i].name, "%d", eta);
		}
		else {
			dstate_setinfo(ups2000_timers[i].name, "%d", -1);
		}
	}
	return 0;
}


void upsdrv_shutdown(void)
{
	/* Only implement "shutdown.default"; do not invoke
	 * general handling of other `sdcommands` here */

	int ret = do_loop_shutdown_commands("shutdown.reboot", NULL);
	if (ret != STAT_INSTCMD_HANDLED) {
		upslogx(LOG_ERR, "upsdrv_shutdown failed!");
		if (handling_upsdrv_shutdown > 0)
			set_exit_flag(EF_EXIT_FAILURE);
	}
}


void upsdrv_help(void)
{
}


/* list flags and values that you want to receive via -x */
void upsdrv_makevartable(void)
{
	char msg[64];

	snprintf(msg, 64, "Set shutdown delay, in seconds, 6-second step"
		" (default=%d)", ups2000_rw_delay[SHUTDOWN].dfault);
	addvar(VAR_VALUE, "offdelay", msg);

	snprintf(msg, 64, "Set reboot delay, in seconds, 6-second step"
		" (default=%d).", ups2000_rw_delay[REBOOT].dfault);
	addvar(VAR_VALUE, "rebootdelay", msg);

	snprintf(msg, 64, "Set start delay, in seconds, 60-second step"
		" (default=%d).", ups2000_rw_delay[START].dfault);
	addvar(VAR_VALUE, "ondelay", msg);
}


void upsdrv_cleanup(void)
{
	if (modbus_ctx != NULL) {
		modbus_close(modbus_ctx);
		modbus_free(modbus_ctx);
	}
	ser_close(upsfd, device_path);
}


/*
 * Seek time "t" forward or backward by n "seconds" without assuming
 * the underlying type and format of "time_t". This ensures maximum
 * portability. Although on POSIX and many other systems, "time_t"
 * is guaranteed to be in seconds.
 *
 * On error, abort the program.
 */
static time_t time_seek(time_t t, int seconds)
{
	struct tm time_tm;
	time_t time_output;

	if (!t)
		fatalx(EXIT_FAILURE, "time_seek() failed!");

	if (gmtime_r(&t, &time_tm) == NULL)
		fatalx(EXIT_FAILURE, "time_seek() failed!");

	time_tm.tm_sec += seconds;
	time_output = mktime(&time_tm);

	if (time_output == (time_t) -1)
		fatalx(EXIT_FAILURE, "time_seek() failed!");

	return time_output;
}


/*
 * Read bytes from the UPS2000 serial port, until the buffer has
 * nothing left to read (after ser_get_buf() times out). The buffer
 * size is limited to buf_len (inclusive).
 *
 * On error, return 0.
 *
 * In the serial library, ser_get_buf() can be a short read, and
 * ser_get_buf_let() requires a precalculated length, necessiates
 * our own read function.
 */
static size_t ups2000_read_serial(uint8_t *buf, size_t buf_len)
{
	ssize_t bytes = 0;
	size_t total = 0;

	/* wait 400 ms for the device to process our command */
	usleep(400 * 1000);

	while (buf_len > 0) {
		bytes = ser_get_buf(upsfd, buf, buf_len, 1, 0);
		if (bytes < 0)
			return 0;      /* read failure */
		else if (bytes == 0)
			return total;  /* nothing to read */

		if ((size_t) bytes > buf_len) {
			/*
			 * Assertion: This should never happen. bytes is always less or equal
			 * to buf_len, and buf_len will never underflow under any circumstances.
			 */
			fatalx(EXIT_FAILURE, "ups2000_read_serial() reads too much!");
		}

		total += (size_t)bytes;        /* increment byte counter */
		buf += bytes;                  /* advance buffer position */
		buf_len -= (size_t)bytes;      /* decrement limiter */
	}
	return 0;  /* buffer exhaustion */
}


/*
 * Retry control. By default, we are in RETRY_ENABLE mode. For each
 * register read, we retry three times (1 sec. between each attempt),
 * before raising a fatal error and giving up. So far so good, but,
 * if the link went down, all operation would fail and the program
 * would become unresponsive due to excessive retrys.
 *
 * To prevent this problem, after the first fatal error, we stop all
 * retry attempts by entering RETRY_DISABLE_TEMPORARY mode, allowing
 * subsequent operation to fail without retry. Later, after the first
 * success is encountered, we move back to RETRY_ENABLE mode.
 */
enum {
	RETRY_ENABLE,
	RETRY_DISABLE_TEMPORARY
};
static int retry_status = RETRY_ENABLE;


/*
 * Read one or more registers using libmodbus.
 *
 * This is simply a wrapper for libmodbus's modbus_read_registers() with
 * retry and workaround logic. When an error has occured, we retry 3 times
 * before giving up, allowing us to recover from non-fatal failures without
 * triggering a data stale.
 */
static int ups2000_read_registers(modbus_t *ctx, int addr, int nb, uint16_t *dest)
{
	int i;
	int r = -1;

	if (addr < 10000)
		upslogx(LOG_ERR, "Invalid register read from %04d detected. "
				 "Please file a bug report!", addr);

	for (i = 0; i < 3; i++) {
		/*
		 * If the previous read failed with a timeout, often there
		 * are still unprocessed bytes in the serial buffer and they
		 * would be mixed with the new data, creating invalid messages,
		 * making all subsequent reads to fail as well.
		 *
		 * Flush read buffer first to avoid it.
		 */
		modbus_flush(ctx);

		r = modbus_read_registers(ctx, addr, nb, dest);

		/* generic retry for modbus read failures. */
		if (retry_status == RETRY_ENABLE && r != nb) {
			upslogx(LOG_WARNING, "modbus_read_registers() failed (%d, errno %d): %s",
				r, errno, modbus_strerror(errno));
			upslogx(LOG_WARNING, "Register %04d has a read failure. Retrying...", addr);
			sleep(1);
			continue;
		}
		else if (r == nb)
			retry_status = RETRY_ENABLE;

		/*
		 * Workaround for buggy register 2002 (battery status). Sometimes
		 * this register returns invalid values. This is a known problem
		 * and it's not fatal, so we use LOG_INFO.
		 */
		if (retry_status == RETRY_ENABLE &&
		    addr == 12002 && (dest[0] < 2 || dest[0] > 5)
		) {
			upslogx(LOG_INFO, "Battery status has a non-fatal read failure, it's usually harmless. Retrying... ");
			sleep(1);
			continue;
		}
		else if (addr == 12002 && dest[0] >= 2 && dest[0] <= 5)
			retry_status = RETRY_ENABLE;

		return r;
	}

	/* Give up */
	upslogx(LOG_ERR, "modbus_read_registers() failed (%d, errno %d): %s",
		r, errno, modbus_strerror(errno));
	upslogx(LOG_ERR, "Register %04d has a fatal read failure.", addr);
	retry_status = RETRY_DISABLE_TEMPORARY;
	return r;
}


static int ups2000_write_registers(modbus_t *ctx, int addr, int nb, uint16_t *src)
{
	int i;
	int r = -1;

	if (addr < 10000)
		upslogx(LOG_ERR, "Invalid register write to %04d detected. "
				 "Please file a bug report!", addr);

	for (i = 0; i < 3; i++) {
		r = modbus_write_registers(ctx, addr, nb, src);

		/* generic retry for modbus write failures. */
		if (retry_status == RETRY_ENABLE && r != nb) {
			upslogx(LOG_WARNING, "modbus_write_registers() failed (%d, errno %d): %s",
				r, errno, modbus_strerror(errno));
			upslogx(LOG_WARNING, "Register %04d has a write failure. Retrying...", addr);
			sleep(1);
			continue;
		}
		else if (r == nb)
			retry_status = RETRY_ENABLE;

		return r;
	}

	/* Give up */
	upslogx(LOG_ERR, "modbus_write_registers() failed (%d, errno %d): %s",
		r, errno, modbus_strerror(errno));
	upslogx(LOG_ERR, "Register %04d has a fatal write failure.", addr);
	retry_status = RETRY_DISABLE_TEMPORARY;
	return r;
}


static int ups2000_write_register(modbus_t *ctx, int addr, uint16_t val)
{
	return ups2000_write_registers(ctx, addr, 1, &val);
}


/*
 * The following CRC-16 code was copied from libmodbus.
 *
 * Copyright (C) 2001-2011 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */

/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
	0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
	0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
	0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
	0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
	0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
	0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
	0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
	0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
	0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
	0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
	0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
	0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
	0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
	0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};


static uint16_t crc16(uint8_t * buffer, size_t buffer_length)
{
	uint8_t crc_hi = 0xFF;	/* high CRC byte initialized */
	uint8_t crc_lo = 0xFF;	/* low CRC byte initialized */
	unsigned int i;		/* will index into CRC lookup */

	/* pass through message buffer */
	while (buffer_length--) {
		i = crc_hi ^ *buffer++;	/* calculate the CRC  */
		crc_hi = crc_lo ^ table_crc_hi[i];
		crc_lo = table_crc_lo[i];
	}

	return (uint16_t) crc_hi << 8 | crc_lo;
}
