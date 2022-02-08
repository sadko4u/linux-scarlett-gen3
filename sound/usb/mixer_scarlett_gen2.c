// SPDX-License-Identifier: GPL-2.0
/*
 *   Focusrite Scarlett Gen 2/3 Driver for ALSA
 *
 *   Supported models:
 *   - 6i6/18i8/18i20 Gen 2
 *   - Solo/2i2/4i4/8i6/18i8/18i20 Gen 3
 *
 *   Copyright (c) 2018-2021 by Geoffrey D. Bennett <g at b4.vu>
 *   Copyright (c) 2020-2021 by Vladimir Sadovnikov <sadko4u@gmail.com>
 *
 *   Based on the Scarlett (Gen 1) Driver for ALSA:
 *
 *   Copyright (c) 2013 by Tobias Hoffmann
 *   Copyright (c) 2013 by Robin Gareus <robin at gareus.org>
 *   Copyright (c) 2002 by Takashi Iwai <tiwai at suse.de>
 *   Copyright (c) 2014 by Chris J Arges <chris.j.arges at canonical.com>
 *
 *   Many codes borrowed from audio.c by
 *     Alan Cox (alan at lxorguk.ukuu.org.uk)
 *     Thomas Sailer (sailer at ife.ee.ethz.ch)
 *
 *   Code cleanup:
 *   David Henningsson <david.henningsson at canonical.com>
 */

/* The protocol was reverse engineered by looking at the communication
 * between Focusrite Control 2.3.4 and the Focusrite(R) Scarlett 18i20
 * (firmware 1083) using usbmon in July-August 2018.
 *
 * Scarlett 18i8 support added in April 2019.
 *
 * Scarlett 6i6 support added in June 2019 (thanks to Martin Wittmann
 * for providing usbmon output and testing).
 *
 * Scarlett 4i4/8i6 Gen 3 support added in May 2020 (thanks to Laurent
 * Debricon for donating a 4i4 and to Fredrik Unger for providing 8i6
 * usbmon output and testing).
 *
 * Scarlett 18i8/18i20 Gen 3 support added in June 2020 (thanks to
 * Darren Jaeckel, Alex Sedlack, and Clovis Lunel for providing usbmon
 * output, protocol traces and testing).
 *
 * Support for loading mixer volume and mux configuration from the
 * interface during driver initialisation added in May 2021 (thanks to
 * Vladimir Sadovnikov for figuring out how).
 *
 * Support for Solo/2i2 Gen 3 added in May 2021 (thanks to Alexander
 * Vorona for 2i2 protocol traces).
 *
 * Support for phantom power, direct monitoring, speaker switching,
 * and talkback added in May-June 2021.
 *
 * This ALSA mixer gives access to (model-dependent):
 *  - input, output, mixer-matrix muxes
 *  - mixer-matrix gain stages
 *  - gain/volume/mute controls
 *  - level meters
 *  - line/inst level, pad, and air controls
 *  - phantom power, direct monitor, speaker switching, and talkback
 *    controls
 *  - disable/enable MSD mode
 *
 * <ditaa>
 *    /--------------\    18chn            20chn     /--------------\
 *    | Hardware  in +--+------\    /-------------+--+ ALSA PCM out |
 *    \--------------/  |      |    |             |  \--------------/
 *                      |      |    |    /-----\  |
 *                      |      |    |    |     |  |
 *                      |      v    v    v     |  |
 *                      |   +---------------+  |  |
 *                      |    \ Matrix  Mux /   |  |
 *                      |     +-----+-----+    |  |
 *                      |           |          |  |
 *                      |           |18chn     |  |
 *                      |           |          |  |
 *                      |           |     10chn|  |
 *                      |           v          |  |
 *                      |     +------------+   |  |
 *                      |     | Mixer      |   |  |
 *                      |     |     Matrix |   |  |
 *                      |     |            |   |  |
 *                      |     | 18x10 Gain |   |  |
 *                      |     |   stages   |   |  |
 *                      |     +-----+------+   |  |
 *                      |           |          |  |
 *                      |18chn      |10chn     |  |20chn
 *                      |           |          |  |
 *                      |           +----------/  |
 *                      |           |             |
 *                      v           v             v
 *                      ===========================
 *               +---------------+       +--—------------+
 *                \ Output  Mux /         \ Capture Mux /
 *                 +---+---+---+           +-----+-----+
 *                     |   |                     |
 *                10chn|   |                     |18chn
 *                     |   |                     |
 *  /--------------\   |   |                     |   /--------------\
 *  | S/PDIF, ADAT |<--/   |10chn                \-->| ALSA PCM in  |
 *  | Hardware out |       |                         \--------------/
 *  \--------------/       |
 *                         v
 *                  +-------------+    Software gain per channel.
 *                  | Master Gain |<-- 18i20 only: Switch per channel
 *                  +------+------+    to select HW or SW gain control.
 *                         |
 *                         |10chn
 *  /--------------\       |
 *  | Analogue     |<------/
 *  | Hardware out |
 *  \--------------/
 * </ditaa>
 *
 * Gen 3 devices have a Mass Storage Device (MSD) mode where a small
 * disk with registration and driver download information is presented
 * to the host. To access the full functionality of the device without
 * proprietary software, MSD mode can be disabled by:
 * - holding down the 48V button for five seconds while powering on
 *   the device, or
 * - using this driver and alsamixer to change the "MSD Mode" setting
 *   to Off and power-cycling the device
 */

#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/moduleparam.h>

#include <sound/control.h>
#include <sound/tlv.h>

#include "usbaudio.h"
#include "mixer.h"
#include "helper.h"

#include "mixer_scarlett_gen2.h"

/* device_setup value to enable */
#define SCARLETT2_ENABLE 0x01

/* device_setup value to allow turning MSD mode back on */
#define SCARLETT2_MSD_ENABLE 0x02

/* some gui mixers can't handle negative ctl values */
#define SCARLETT2_VOLUME_BIAS 127

/* mixer range from -80dB to +6dB in 0.5dB steps */
#define SCARLETT2_MIXER_MIN_DB -80
#define SCARLETT2_MIXER_BIAS (-SCARLETT2_MIXER_MIN_DB * 2)
#define SCARLETT2_MIXER_MAX_DB 6
#define SCARLETT2_MIXER_MAX_VALUE \
	((SCARLETT2_MIXER_MAX_DB - SCARLETT2_MIXER_MIN_DB) * 2)
#define SCARLETT2_MIXER_VALUE_COUNT (SCARLETT2_MIXER_MAX_VALUE + 1)

/* map from (dB + 80) * 2 to mixer value
 * for dB in 0 .. 172: int(8192 * pow(10, ((dB - 160) / 2 / 20)))
 */
static const u16 scarlett2_mixer_values[SCARLETT2_MIXER_VALUE_COUNT] = {
	0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2,
	2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 6, 6, 6, 7, 7, 8, 8,
	9, 9, 10, 10, 11, 12, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
	23, 24, 25, 27, 29, 30, 32, 34, 36, 38, 41, 43, 46, 48, 51,
	54, 57, 61, 65, 68, 73, 77, 81, 86, 91, 97, 103, 109, 115,
	122, 129, 137, 145, 154, 163, 173, 183, 194, 205, 217, 230,
	244, 259, 274, 290, 307, 326, 345, 365, 387, 410, 434, 460,
	487, 516, 547, 579, 614, 650, 689, 730, 773, 819, 867, 919,
	973, 1031, 1092, 1157, 1225, 1298, 1375, 1456, 1543, 1634,
	1731, 1833, 1942, 2057, 2179, 2308, 2445, 2590, 2744, 2906,
	3078, 3261, 3454, 3659, 3876, 4105, 4349, 4606, 4879, 5168,
	5475, 5799, 6143, 6507, 6892, 7301, 7733, 8192, 8677, 9191,
	9736, 10313, 10924, 11571, 12257, 12983, 13752, 14567, 15430,
	16345
};

/* Maximum number of analogue inputs */
#define SCARLETT2_ANALOGUE_IN_MAX                8

/* Maximum number of analogue outputs */
#define SCARLETT2_ANALOGUE_OUT_MAX               10

/* Maximum number of all possible inputs for any device */
#define SCARLETT2_ALL_IN_MAX                     42

/* Maximum number of all outputs for any device */
#define SCARLETT2_ALL_OUT_MAX                    26

/* Maximum length of the text label associated with name of input */
#define SCARLETT2_IN_NAME_LEN                    12

/* Maximum length of the text label associated with name of output */
#define SCARLETT2_OUT_NAME_LEN                   12

/* Maximum number of level and pad switches */
#define SCARLETT2_LEVEL_SWITCH_MAX               2
#define SCARLETT2_PAD_SWITCH_MAX                 8
#define SCARLETT2_AIR_SWITCH_MAX                 8
#define SCARLETT2_PHANTOM_SWITCH_MAX             2

/* Maximum number of inputs to the mixer */
#define SCARLETT2_INPUT_MIX_MAX 25

/* Maximum number of outputs from the mixer */
#define SCARLETT2_OUTPUT_MIX_MAX 12

/* Maximum size of the data in the USB mux assignment message:
 * 20 inputs, 20 outputs, 25 matrix inputs, 12 spare
 */
#define SCARLETT2_MUX_MAX 77

/* Maximum number of meters (sum of output port counts) */
#define SCARLETT2_MAX_METERS 65

/* The maximum packet size used to transfer data */
#define SCARLETT2_SW_CONFIG_BASE                 0xec

/* The maximum packet size used to transfer data */
#define SCARLETT2_SW_CONFIG_PACKET_SIZE          1024

/* Maximum 30 inputs per one mixer in config */
#define SCARLETT2_SW_CONFIG_MIXER_INPUTS         30
/* Maximum 12 mixer outputs in config */
#define SCARLETT2_SW_CONFIG_MIXER_OUTPUTS        12
/* Maximum number of outputs */
#define SCARLETT2_SW_CONFIG_OUTPUT_MAX           26

/* Hardware port types:
 * - None (no input to mux)
 * - Analogue I/O
 * - S/PDIF I/O
 * - ADAT I/O
 * - Mixer I/O
 * - PCM I/O
 */
enum {
	SCARLETT2_PORT_TYPE_NONE     = 0,
	SCARLETT2_PORT_TYPE_ANALOGUE = 1,
	SCARLETT2_PORT_TYPE_SPDIF    = 2,
	SCARLETT2_PORT_TYPE_ADAT     = 3,
	SCARLETT2_PORT_TYPE_MIX      = 4,
	SCARLETT2_PORT_TYPE_PCM      = 5,
	SCARLETT2_PORT_TYPE_COUNT    = 6,
};

/* I/O count of each port type kept in struct scarlett2_ports */
enum {
	SCARLETT2_PORT_IN    = 0,
	SCARLETT2_PORT_OUT   = 1,
	SCARLETT2_PORT_DIRNS = 2,
};

/* Dim/Mute buttons on the 18i20 */
enum {
	SCARLETT2_BUTTON_MUTE    = 0,
	SCARLETT2_BUTTON_DIM     = 1,
	SCARLETT2_DIM_MUTE_COUNT = 2,
};

static const char *const scarlett2_dim_mute_names[SCARLETT2_DIM_MUTE_COUNT] = {
	"Mute Playback Switch", "Dim Playback Switch"
};

/* Description of each hardware port type:
 * - id: hardware ID of this port type
 * - src_descr: printf format string for mux input selections
 * - src_num_offset: added to channel number for the fprintf
 * - dst_descr: printf format string for mixer controls
 */
struct scarlett2_port {
	u16 id;
	const char * const src_descr;
	int src_num_offset;
	const char * const dst_descr;
	const char * const dst_stereo_descr;
};

static const struct scarlett2_port scarlett2_ports[SCARLETT2_PORT_TYPE_COUNT] = {
	[SCARLETT2_PORT_TYPE_NONE] = {
		.id = 0x000,
		.src_descr = "Off"
	},
	[SCARLETT2_PORT_TYPE_ANALOGUE] = {
		.id = 0x080,
		.src_descr = "Analogue %d",
		.src_num_offset = 1,
		.dst_descr = "Analogue Output %02d Playback",
		.dst_stereo_descr = "Analogue Output %02d-%02d"
	},
	[SCARLETT2_PORT_TYPE_SPDIF] = {
		.id = 0x180,
		.src_descr = "S/PDIF %d",
		.src_num_offset = 1,
		.dst_descr = "S/PDIF Output %d Playback",
		.dst_stereo_descr = "S/PDIF Output %d-%d"
	},
	[SCARLETT2_PORT_TYPE_ADAT] = {
		.id = 0x200,
		.src_descr = "ADAT %d",
		.src_num_offset = 1,
		.dst_descr = "ADAT Output %d Playback",
		.dst_stereo_descr = "ADAT Output %d-%d"
	},
	[SCARLETT2_PORT_TYPE_MIX] = {
		.id = 0x300,
		.src_descr = "Mix %c",
		.src_num_offset = 'A',
		.dst_descr = "Mixer Input %02d Capture",
		.dst_stereo_descr = "Mixer Input %02d-%02d"
	},
	[SCARLETT2_PORT_TYPE_PCM] = {
		.id = 0x600,
		.src_descr = "PCM %d",
		.src_num_offset = 1,
		.dst_descr = "PCM %02d Capture",
		.dst_stereo_descr = "PCM %02d-%02d Capture"
	},
};

/* Number of mux tables: one for each band of sample rates
 * (44.1/48kHz, 88.2/96kHz, and 176.4/176kHz)
 */
#define SCARLETT2_MUX_TABLES 3

/* Maximum number of entries in a mux table */
#define SCARLETT2_MAX_MUX_ENTRIES 10

/* One entry within mux_assignment defines the port type and range of
 * ports to add to the set_mux message. The end of the list is marked
 * with count == 0.
 */
struct scarlett2_mux_entry {
	u8 port_type;
	u8 start;
	u8 count;
};

struct scarlett2_sw_port_mapping {
	u8 type;      /* The type of the port */
	u8 direction; /* The direction of the port */
	u8 hw_offset; /* The number of first port in hardware settings */
	u8 count;     /* The actual number of ports */
	u8 sw_offset; /* The number of first port in software settings */
};

struct scarlett2_device_info {
	u32 usb_id; /* USB device identifier */

	/* Gen 3 devices have an internal MSD mode switch that needs
	 * to be disabled in order to access the full functionality of
	 * the device.
	 */
	u8 has_msd_mode;

	/* Gen 3 devices without a mixer have a different
	 * configuration set
	 */
	u8 has_mixer;
    
	/* Software configuration area in the device is supported */
	u8 has_sw_config;
    
	/* Configuration space size for device, 0 for large configs */
	u8 config_size;

	/* line out hw volume is sw controlled */
	u8 line_out_hw_vol;

	/* support for main/alt speaker switching */
	u8 has_speaker_switching;

	/* support for talkback microphone */
	u8 has_talkback;

	/* the number of analogue inputs with a software switchable
	 * level control that can be set to line or instrument
	 */
	u8 level_input_count;

	/* the first input with a level control (0-based) */
	u8 level_input_first;

	/* the number of analogue inputs with a software switchable
	 * 10dB pad control
	 */
	u8 pad_input_count;

	/* the number of analogue inputs with a software switchable
	 * "air" control
	 */
	u8 air_input_count;

	/* the number of phantom (48V) software switchable controls */
	u8 phantom_count;

	/* the number of inputs each phantom switch controls */
	u8 inputs_per_phantom;

	/* the number of direct monitor options
	 * (0 = none, 1 = mono only, 2 = mono/stereo)
	 */
	u8 direct_monitor;

	/* remap analogue outputs; 18i8 Gen 3 has "line 3/4" connected
	 * internally to the analogue 7/8 outputs
	 */
	u8 line_out_remap_enable;
	u8 line_out_remap[SCARLETT2_ANALOGUE_OUT_MAX];

	/* additional description for the line out volume controls */
	const char * const line_out_descrs[SCARLETT2_ANALOGUE_OUT_MAX];

	/* number of sources/destinations of each port type */
	const int port_count[SCARLETT2_PORT_TYPE_COUNT][SCARLETT2_PORT_DIRNS];
    
	/* layout/order of the entries in the set_mux message */
	struct scarlett2_mux_entry mux_assignment[SCARLETT2_MUX_TABLES]
						 [SCARLETT2_MAX_MUX_ENTRIES];

	/* The port mapping for software configuration area */
	const struct scarlett2_sw_port_mapping *sw_port_mapping;
};

/* The desciption of the volume for volume settings */
struct scarlett2_sw_cfg_volume {
	__le16 volume;  /* volume */
	u8 changed;     /* change flag */
	u8 flags;       /* some flags? */
};

/* Software configuration of the scarlett device */
struct scarlett2_sw_cfg {
	__le16 all_size;                                                    /* +0x0000: overall size, 0x1990 for all devices */
	__le16 magic1;                                                      /* +0x0002: magic number: 0x3006 */
	__le32 version;                                                     /* +0x0004: probably version */
	__le16 szof;                                                        /* +0x0008: the overall size, 0x1984 for all devices  */
	__le16 __pad0;                                                      /* +0x000a: ???????? */
	u8 out_mux[SCARLETT2_SW_CONFIG_OUTPUT_MAX];                         /* +0x00f8: output routing */
	u8 __pad1[0x0066];                                                  /* +0x0112: ???????? */
	u8 mixer_in_mux[SCARLETT2_SW_CONFIG_MIXER_INPUTS];                  /* +0x008c: 'custom mix' input routing                    */
	u8 mixer_in_map[SCARLETT2_SW_CONFIG_MIXER_INPUTS];                  /* +0x00aa: 'custom mix' input mapping                    */
	__le32 stereo_sw;                                                   /* +0x01b4: stereo configuration for each port (bit mask) */
	__le32 mute_sw;                                                     /* +0x01b8: mute switches (bit mask) */
	struct scarlett2_sw_cfg_volume volume[SCARLETT2_ANALOGUE_OUT_MAX];  /* +0x01bc: Volume settings of each output */
	u8 __pad2[0x01dc];                                                  /* +0x01e4: ???????? */
	u8 in_alias[SCARLETT2_ALL_IN_MAX][SCARLETT2_IN_NAME_LEN];           /* +0x03c0: Symbolic names of inputs */
	u8 __pad3[0x0438];                                                  /* +0x0420: ???????? */
	u8 out_alias[SCARLETT2_ALL_OUT_MAX][SCARLETT2_OUT_NAME_LEN];        /* +0x09f0: Symbolic names of outputs */
	u8 __pad4[0x04c8];                                                  /* +0x0b28: ???????? */
	__le32 mixer[SCARLETT2_SW_CONFIG_MIXER_OUTPUTS][SCARLETT2_SW_CONFIG_MIXER_INPUTS];  /* +0x0ff0: Matrix mixer settings */
	u8 __pad5[0x01e0];                                                  /* +0x1590: ???????? */
	s8 mixer_pan[SCARLETT2_SW_CONFIG_MIXER_OUTPUTS][SCARLETT2_SW_CONFIG_MIXER_INPUTS];  /* +0x1684: Input pan settings for mixer */
	u8 __pad6[0x0078];                                                  /* +0x17ec: ???????? */
	__le32 mixer_mute[SCARLETT2_SW_CONFIG_MIXER_OUTPUTS];               /* +0x1950: Mute settings for mixer inputs */
	__le32 mixer_solo[SCARLETT2_SW_CONFIG_MIXER_OUTPUTS];               /* +0x1980: Solo settings for mixer inputs */
	u8 __pad7[0x004a];                                                  /* +0x19b0: ???????? */
	__le32 custom_mix;                                                  /* +0x19fa: output to 'custom mix' routing bitmap */
	u8 __pad8[0x006e];                                                  /* +0x19fe: ???????? */
	__le32 checksum;                                                    /* +0x1a6c: checksum of the area */
} __packed;

struct scarlett2_data {
	struct usb_mixer_interface *mixer;
	struct mutex usb_mutex; /* prevent sending concurrent USB requests */
	struct mutex data_mutex; /* lock access to this data */
	struct delayed_work work;
	const struct scarlett2_device_info *info;
	__u8 bInterfaceNumber;
	__u8 bEndpointAddress;
	__u16 wMaxPacketSize;
	__u8 bInterval;
	int num_mux_srcs;
	int num_mux_dsts;
	u16 scarlett2_seq;
	u8 sync_updated;
	u8 vol_updated;
	u8 input_other_updated;
	u8 monitor_other_updated;
	u8 mux_updated;
	u8 speaker_switching_switched;
	u8 sync;
	u8 master_vol;
	u8 vol[SCARLETT2_ANALOGUE_OUT_MAX];
	u8 vol_sw_hw_switch[SCARLETT2_ANALOGUE_OUT_MAX];
    /* Mute switches for each output including the sofware switches for non-analogue outputs */
	u8 mute_switch[SCARLETT2_ALL_OUT_MAX];
	u8 stereo_switch[SCARLETT2_ALL_OUT_MAX >> 1];
	u8 level_switch[SCARLETT2_LEVEL_SWITCH_MAX];
	u8 pad_switch[SCARLETT2_PAD_SWITCH_MAX];
	u8 dim_mute[SCARLETT2_DIM_MUTE_COUNT];
	u8 air_switch[SCARLETT2_AIR_SWITCH_MAX];
	u8 phantom_switch[SCARLETT2_PHANTOM_SWITCH_MAX];
	u8 phantom_persistence;
	u8 direct_monitor_switch;
	u8 speaker_switching_switch;
	u8 talkback_switch;
	u8 talkback_map[SCARLETT2_OUTPUT_MIX_MAX];
	u8 msd_switch;
	struct snd_kcontrol *sync_ctl;
	struct snd_kcontrol *master_vol_ctl;
	struct snd_kcontrol *vol_ctls[SCARLETT2_ANALOGUE_OUT_MAX];
	struct snd_kcontrol *sw_hw_ctls[SCARLETT2_ANALOGUE_OUT_MAX];
	struct snd_kcontrol *mute_ctls[SCARLETT2_ALL_OUT_MAX];
	struct snd_kcontrol *stereo_ctls[SCARLETT2_ALL_OUT_MAX >> 1];
	struct snd_kcontrol *dim_mute_ctls[SCARLETT2_DIM_MUTE_COUNT];
	struct snd_kcontrol *level_ctls[SCARLETT2_LEVEL_SWITCH_MAX];
	struct snd_kcontrol *pad_ctls[SCARLETT2_PAD_SWITCH_MAX];
	struct snd_kcontrol *air_ctls[SCARLETT2_AIR_SWITCH_MAX];
	struct snd_kcontrol *phantom_ctls[SCARLETT2_PHANTOM_SWITCH_MAX];
	struct snd_kcontrol *mux_ctls[SCARLETT2_MUX_MAX];
	struct snd_kcontrol *direct_monitor_ctl;
	struct snd_kcontrol *speaker_switching_ctl;
	struct snd_kcontrol *talkback_ctl;
	u8 mux[SCARLETT2_MUX_MAX];
	u8 mix[SCARLETT2_INPUT_MIX_MAX * SCARLETT2_OUTPUT_MIX_MAX];
    
	/* Software configuration */
	struct scarlett2_sw_cfg *sw_cfg;                                  /* Software configuration data */
};

/*** Model-specific data ***/

static const struct scarlett2_device_info s6i6_gen2_info = {
	.usb_id = USB_ID(0x1235, 0x8203),

	.has_mixer = 1,
	.level_input_count = 2,
	.pad_input_count = 2,

	.line_out_descrs = {
		"Headphones 1 L",
		"Headphones 1 R",
		"Headphones 2 L",
		"Headphones 2 R",
	},

	.port_count = {
		[SCARLETT2_PORT_TYPE_NONE]     = {  1,  0 },
		[SCARLETT2_PORT_TYPE_ANALOGUE] = {  4,  4 },
		[SCARLETT2_PORT_TYPE_SPDIF]    = {  2,  2 },
		[SCARLETT2_PORT_TYPE_MIX]      = { 10, 18 },
		[SCARLETT2_PORT_TYPE_PCM]      = {  6,  6 },
	},

	.mux_assignment = { {
		{ SCARLETT2_PORT_TYPE_PCM,      0,  6 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0,  4 },
		{ SCARLETT2_PORT_TYPE_SPDIF,    0,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,      0, 18 },
		{ SCARLETT2_PORT_TYPE_NONE,     0,  8 },
		{ 0,                            0,  0 },
	}, {
		{ SCARLETT2_PORT_TYPE_PCM,      0,  6 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0,  4 },
		{ SCARLETT2_PORT_TYPE_SPDIF,    0,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,      0, 18 },
		{ SCARLETT2_PORT_TYPE_NONE,     0,  8 },
		{ 0,                            0,  0 },
	}, {
		{ SCARLETT2_PORT_TYPE_PCM,      0,  6 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0,  4 },
		{ SCARLETT2_PORT_TYPE_SPDIF,    0,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,      0, 18 },
		{ SCARLETT2_PORT_TYPE_NONE,     0,  8 },
		{ 0,                            0,  0 },
	} },
};

static const struct scarlett2_device_info s18i8_gen2_info = {
	.usb_id = USB_ID(0x1235, 0x8204),

	.has_mixer = 1,
	.level_input_count = 2,
	.pad_input_count = 4,

	.line_out_descrs = {
		"Monitor L",
		"Monitor R",
		"Headphones 1 L",
		"Headphones 1 R",
		"Headphones 2 L",
		"Headphones 2 R",
	},

	.port_count = {
		[SCARLETT2_PORT_TYPE_NONE]     = {  1,  0 },
		[SCARLETT2_PORT_TYPE_ANALOGUE] = {  8,  6 },
		[SCARLETT2_PORT_TYPE_SPDIF]    = {  2,  2 },
		[SCARLETT2_PORT_TYPE_ADAT]     = {  8,  0 },
		[SCARLETT2_PORT_TYPE_MIX]      = { 10, 18 },
		[SCARLETT2_PORT_TYPE_PCM]      = {  8, 18 },
	},

	.mux_assignment = { {
		{ SCARLETT2_PORT_TYPE_PCM,      0, 18 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0,  6 },
		{ SCARLETT2_PORT_TYPE_SPDIF,    0,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,      0, 18 },
		{ SCARLETT2_PORT_TYPE_NONE,     0,  8 },
		{ 0,                            0,  0 },
	}, {
		{ SCARLETT2_PORT_TYPE_PCM,      0, 14 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0,  6 },
		{ SCARLETT2_PORT_TYPE_SPDIF,    0,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,      0, 18 },
		{ SCARLETT2_PORT_TYPE_NONE,     0,  8 },
		{ 0,                            0,  0 },
	}, {
		{ SCARLETT2_PORT_TYPE_PCM,      0, 10 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0,  6 },
		{ SCARLETT2_PORT_TYPE_SPDIF,    0,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,      0, 18 },
		{ SCARLETT2_PORT_TYPE_NONE,     0,  4 },
		{ 0,                            0,  0 },
	} },
};

static const struct scarlett2_device_info s18i20_gen2_info = {
	.usb_id = USB_ID(0x1235, 0x8201),

	.has_mixer = 1,
	.line_out_hw_vol = 1,

	.line_out_descrs = {
		"Monitor L",
		"Monitor R",
		NULL,
		NULL,
		NULL,
		NULL,
		"Headphones 1 L",
		"Headphones 1 R",
		"Headphones 2 L",
		"Headphones 2 R",
	},

	.port_count = {
		[SCARLETT2_PORT_TYPE_NONE]     = {  1,  0 },
		[SCARLETT2_PORT_TYPE_ANALOGUE] = {  8, 10 },
		[SCARLETT2_PORT_TYPE_SPDIF]    = {  2,  2 },
		[SCARLETT2_PORT_TYPE_ADAT]     = {  8,  8 },
		[SCARLETT2_PORT_TYPE_MIX]      = { 10, 18 },
		[SCARLETT2_PORT_TYPE_PCM]      = { 20, 18 },
	},

	.mux_assignment = { {
		{ SCARLETT2_PORT_TYPE_PCM,      0, 18 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0, 10 },
		{ SCARLETT2_PORT_TYPE_SPDIF,    0,  2 },
		{ SCARLETT2_PORT_TYPE_ADAT,     0,  8 },
		{ SCARLETT2_PORT_TYPE_MIX,      0, 18 },
		{ SCARLETT2_PORT_TYPE_NONE,     0,  8 },
		{ 0,                            0,  0 },
	}, {
		{ SCARLETT2_PORT_TYPE_PCM,      0, 14 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0, 10 },
		{ SCARLETT2_PORT_TYPE_SPDIF,    0,  2 },
		{ SCARLETT2_PORT_TYPE_ADAT,     0,  4 },
		{ SCARLETT2_PORT_TYPE_MIX,      0, 18 },
		{ SCARLETT2_PORT_TYPE_NONE,     0,  8 },
		{ 0,                            0,  0 },
	}, {
		{ SCARLETT2_PORT_TYPE_PCM,      0, 10 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0, 10 },
		{ SCARLETT2_PORT_TYPE_SPDIF,    0,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,      0, 18 },
		{ SCARLETT2_PORT_TYPE_NONE,     0,  6 },
		{ 0,                            0,  0 },
	} },
};

static const struct scarlett2_device_info solo_gen3_info = {
	.usb_id = USB_ID(0x1235, 0x8211),

	.has_msd_mode = 1,
	.config_size = 29,
	.level_input_count = 1,
	.level_input_first = 1,
	.air_input_count = 1,
	.phantom_count = 1,
	.inputs_per_phantom = 1,
	.direct_monitor = 1,
};

static const struct scarlett2_device_info s2i2_gen3_info = {
	.usb_id = USB_ID(0x1235, 0x8210),

	.has_msd_mode = 1,
	.config_size = 29,
	.level_input_count = 2,
	.air_input_count = 2,
	.phantom_count = 1,
	.inputs_per_phantom = 2,
	.direct_monitor = 2,
};

static const struct scarlett2_device_info s4i4_gen3_info = {
	.usb_id = USB_ID(0x1235, 0x8212),

	.has_msd_mode = 1,
	.has_mixer = 1,
    .has_sw_config = 1,
	.level_input_count = 2,
	.pad_input_count = 2,
	.air_input_count = 2,
	.phantom_count = 1,
	.inputs_per_phantom = 2,

	.line_out_descrs = {
		"Monitor L",
		"Monitor R",
		"Headphones L",
		"Headphones R",
	},

	.port_count = {
		[SCARLETT2_PORT_TYPE_NONE]     = { 1, 0 },
		[SCARLETT2_PORT_TYPE_ANALOGUE] = { 4, 4 },
		[SCARLETT2_PORT_TYPE_MIX]      = { 6, 8 },
		[SCARLETT2_PORT_TYPE_PCM]      = { 4, 6 },
	},

	.mux_assignment = { {
		{ SCARLETT2_PORT_TYPE_PCM,      0,  6 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0,  4 },
		{ SCARLETT2_PORT_TYPE_MIX,      0,  8 },
		{ SCARLETT2_PORT_TYPE_NONE,     0, 16 },
		{ 0,                            0,  0 },
	}, {
		{ SCARLETT2_PORT_TYPE_PCM,      0,  6 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0,  4 },
		{ SCARLETT2_PORT_TYPE_MIX,      0,  8 },
		{ SCARLETT2_PORT_TYPE_NONE,     0, 16 },
		{ 0,                            0,  0 },
	}, {
		{ SCARLETT2_PORT_TYPE_PCM,      0,  6 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0,  4 },
		{ SCARLETT2_PORT_TYPE_MIX,      0,  8 },
		{ SCARLETT2_PORT_TYPE_NONE,     0, 16 },
		{ 0,                            0,  0 },
	} },
};

static const struct scarlett2_device_info s8i6_gen3_info = {
	.usb_id = USB_ID(0x1235, 0x8213),

	.has_msd_mode = 1,
	.has_mixer = 1,
    .has_sw_config = 1,
	.level_input_count = 2,
	.pad_input_count = 2,
	.air_input_count = 2,
	.phantom_count = 1,
	.inputs_per_phantom = 2,

	.line_out_descrs = {
		"Headphones 1 L",
		"Headphones 1 R",
		"Headphones 2 L",
		"Headphones 2 R",
	},

	.port_count = {
		[SCARLETT2_PORT_TYPE_NONE]     = { 1,  0 },
		[SCARLETT2_PORT_TYPE_ANALOGUE] = { 6,  4 },
		[SCARLETT2_PORT_TYPE_SPDIF]    = { 2,  2 },
		[SCARLETT2_PORT_TYPE_MIX]      = { 8,  8 },
		[SCARLETT2_PORT_TYPE_PCM]      = { 6, 10 },
	},

	.mux_assignment = { {
		{ SCARLETT2_PORT_TYPE_PCM,      0,  8 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0,  4 },
		{ SCARLETT2_PORT_TYPE_SPDIF,    0,  2 },
		{ SCARLETT2_PORT_TYPE_PCM,      8,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,      0,  8 },
		{ SCARLETT2_PORT_TYPE_NONE,     0, 18 },
		{ 0,                            0,  0 },
	}, {
		{ SCARLETT2_PORT_TYPE_PCM,      0,  8 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0,  4 },
		{ SCARLETT2_PORT_TYPE_SPDIF,    0,  2 },
		{ SCARLETT2_PORT_TYPE_PCM,      8,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,      0,  8 },
		{ SCARLETT2_PORT_TYPE_NONE,     0, 18 },
		{ 0,                            0,  0 },
	}, {
		{ SCARLETT2_PORT_TYPE_PCM,      0,  8 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE, 0,  4 },
		{ SCARLETT2_PORT_TYPE_SPDIF,    0,  2 },
		{ SCARLETT2_PORT_TYPE_PCM,      8,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,      0,  8 },
		{ SCARLETT2_PORT_TYPE_NONE,     0, 18 },
		{ 0,                            0,  0 },
	} },
};

static const struct scarlett2_sw_port_mapping s18i8_gen3_sw_port_mapping[] =
{
	/* Input port mapping */
	{ SCARLETT2_PORT_TYPE_NONE,        SCARLETT2_PORT_IN,    0,  1,  0 }, /* bit  0 for disabled routing */
	{ SCARLETT2_PORT_TYPE_ANALOGUE,    SCARLETT2_PORT_IN,    0,  8,  1 }, /* bits 1..8 for Analogue inputs */
	{ SCARLETT2_PORT_TYPE_SPDIF,       SCARLETT2_PORT_IN,    0,  2,  9 }, /* bits 9..10 for S/PDIF 1-2 */
	{ SCARLETT2_PORT_TYPE_ADAT,        SCARLETT2_PORT_IN,    0,  8, 11 }, /* bits 11..18 for ADAT 1-8 */
	{ SCARLETT2_PORT_TYPE_PCM,         SCARLETT2_PORT_IN,    0,  8, 19 }, /* bits 19..27 for Playback  */
	
	/* Output port mapping */
	{ SCARLETT2_PORT_TYPE_ANALOGUE,    SCARLETT2_PORT_OUT,   0,  8,  0 }, /* bits 0..7 for Analogue outputs */
	{ SCARLETT2_PORT_TYPE_SPDIF,       SCARLETT2_PORT_OUT,   0,  2,  8 }, /* bits 8..9 for S/PDIF 1-2 */
	{ SCARLETT2_PORT_TYPE_PCM,         SCARLETT2_PORT_OUT,  10,  2, 10 }, /* bits 10..11 for Loopback (PCM11-PCM12) */
	
	/* End of record */
	{ SCARLETT2_PORT_TYPE_COUNT, 0, 0, 0, 0 }
};

static const struct scarlett2_device_info s18i8_gen3_info = {
	.usb_id = USB_ID(0x1235, 0x8214),

	.has_msd_mode = 1,
	.has_mixer = 1,
    .has_sw_config = 1,
	.line_out_hw_vol = 1,
	.has_speaker_switching = 1,
	.level_input_count = 2,
	.pad_input_count = 4,
	.air_input_count = 4,
	.phantom_count = 2,
	.inputs_per_phantom = 2,

	.line_out_remap_enable = 1,
	.line_out_remap = { 0, 1, 6, 7, 2, 3, 4, 5 },

	.line_out_descrs = {
		"Monitor L",
		"Monitor R",
		"Alt Monitor L",
		"Alt Monitor R",
		"Headphones 1 L",
		"Headphones 1 R",
		"Headphones 2 L",
		"Headphones 2 R",
	},

	.port_count = {
		[SCARLETT2_PORT_TYPE_NONE]     = {  1,  0 },
		[SCARLETT2_PORT_TYPE_ANALOGUE] = {  8,  8 },
		[SCARLETT2_PORT_TYPE_SPDIF]    = {  2,  2 },
		[SCARLETT2_PORT_TYPE_ADAT]     = {  8,  0 },
		[SCARLETT2_PORT_TYPE_MIX]      = { 10, 20 },
		[SCARLETT2_PORT_TYPE_PCM]      = {  8, 20 },
	},

	.mux_assignment = { {
		{ SCARLETT2_PORT_TYPE_PCM,       0, 10 },
		{ SCARLETT2_PORT_TYPE_PCM,      12,  8 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE,  0,  2 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE,  6,  2 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE,  2,  4 },
		{ SCARLETT2_PORT_TYPE_SPDIF,     0,  2 },
		{ SCARLETT2_PORT_TYPE_PCM,      10,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,       0, 20 },
		{ SCARLETT2_PORT_TYPE_NONE,      0, 10 },
		{ 0,                             0,  0 },
	}, {
		{ SCARLETT2_PORT_TYPE_PCM,       0, 10 },
		{ SCARLETT2_PORT_TYPE_PCM,      12,  4 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE,  0,  2 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE,  6,  2 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE,  2,  4 },
		{ SCARLETT2_PORT_TYPE_SPDIF,     0,  2 },
		{ SCARLETT2_PORT_TYPE_PCM,      10,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,       0, 20 },
		{ SCARLETT2_PORT_TYPE_NONE,      0, 10 },
		{ 0,                             0,  0 },
	}, {
		{ SCARLETT2_PORT_TYPE_PCM,       0, 10 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE,  0,  2 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE,  6,  2 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE,  2,  4 },
		{ SCARLETT2_PORT_TYPE_SPDIF,     0,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,       0, 20 },
		{ SCARLETT2_PORT_TYPE_NONE,      0, 10 },
		{ 0,                             0,  0 },
	} },
    
	.sw_port_mapping = s18i8_gen3_sw_port_mapping,
};

static const struct scarlett2_sw_port_mapping s18i20_gen3_sw_port_mapping[] =
{
	/* Input port mapping */
	{ SCARLETT2_PORT_TYPE_NONE,        SCARLETT2_PORT_IN,    0,  1,  0 }, /* bit  0 for disabled routing */
	{ SCARLETT2_PORT_TYPE_ANALOGUE,    SCARLETT2_PORT_IN,    0,  8,  1 }, /* bits 1..8 for Analogue inputs */
	{ SCARLETT2_PORT_TYPE_SPDIF,       SCARLETT2_PORT_IN,    0,  2,  9 }, /* bits 9..10 for S/PDIF 1-2 */
	{ SCARLETT2_PORT_TYPE_ADAT,        SCARLETT2_PORT_IN,    0,  8, 11 }, /* bits 11..18 for ADAT 1-8 */
	{ SCARLETT2_PORT_TYPE_PCM,         SCARLETT2_PORT_IN,    0, 20, 23 }, /* Playback  */
	
	/* Output port mapping */
	{ SCARLETT2_PORT_TYPE_ANALOGUE,    SCARLETT2_PORT_OUT,   0, 10,  0 }, /* bits 0..9 for Analogue outputs */
	{ SCARLETT2_PORT_TYPE_SPDIF,       SCARLETT2_PORT_OUT,   0,  2, 10 }, /* bits 10..11 for S/PDIF 1-2 */
	{ SCARLETT2_PORT_TYPE_ADAT,        SCARLETT2_PORT_OUT,   0,  8, 12 }, /* bits 12..19 for ADAT 1-8 */
	{ SCARLETT2_PORT_TYPE_PCM,         SCARLETT2_PORT_OUT,  10,  2, 24 }, /* bits 24..25 for Loopback (PCM11-PCM12) */
	
	/* End of record */
	{ SCARLETT2_PORT_TYPE_COUNT, 0, 0, 0, 0 }
};

static const struct scarlett2_device_info s18i20_gen3_info = {
	.usb_id = USB_ID(0x1235, 0x8215),

	.has_msd_mode = 1,
	.has_mixer = 1,
    .has_sw_config = 1,
	.line_out_hw_vol = 1,
	.has_speaker_switching = 1,
	.has_talkback = 1,
	.level_input_count = 2,
	.pad_input_count = 8,
	.air_input_count = 8,
	.phantom_count = 2,
	.inputs_per_phantom = 4,

	.line_out_descrs = {
		"Monitor 1 L",
		"Monitor 1 R",
		"Monitor 2 L",
		"Monitor 2 R",
		NULL,
		NULL,
		"Headphones 1 L",
		"Headphones 1 R",
		"Headphones 2 L",
		"Headphones 2 R",
	},

	.port_count = {
		[SCARLETT2_PORT_TYPE_NONE]     = {  1,  0 },
		[SCARLETT2_PORT_TYPE_ANALOGUE] = {  9, 10 },
		[SCARLETT2_PORT_TYPE_SPDIF]    = {  2,  2 },
		[SCARLETT2_PORT_TYPE_ADAT]     = {  8,  8 },
		[SCARLETT2_PORT_TYPE_MIX]      = { 12, 25 },
		[SCARLETT2_PORT_TYPE_PCM]      = { 20, 20 },
	},

	.mux_assignment = { {
		{ SCARLETT2_PORT_TYPE_PCM,       0,  8 },
		{ SCARLETT2_PORT_TYPE_PCM,      10, 10 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE,  0, 10 },
		{ SCARLETT2_PORT_TYPE_SPDIF,     0,  2 },
		{ SCARLETT2_PORT_TYPE_ADAT,      0,  8 },
		{ SCARLETT2_PORT_TYPE_PCM,       8,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,       0, 25 },
		{ SCARLETT2_PORT_TYPE_NONE,      0, 12 },
		{ 0,                             0,  0 },
	}, {
		{ SCARLETT2_PORT_TYPE_PCM,       0,  8 },
		{ SCARLETT2_PORT_TYPE_PCM,      10,  8 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE,  0, 10 },
		{ SCARLETT2_PORT_TYPE_SPDIF,     0,  2 },
		{ SCARLETT2_PORT_TYPE_ADAT,      0,  8 },
		{ SCARLETT2_PORT_TYPE_PCM,       8,  2 },
		{ SCARLETT2_PORT_TYPE_MIX,       0, 25 },
		{ SCARLETT2_PORT_TYPE_NONE,      0, 10 },
		{ 0,                             0,  0 },
	}, {
		{ SCARLETT2_PORT_TYPE_PCM,       0, 10 },
		{ SCARLETT2_PORT_TYPE_ANALOGUE,  0, 10 },
		{ SCARLETT2_PORT_TYPE_SPDIF,     0,  2 },
		{ SCARLETT2_PORT_TYPE_NONE,      0, 24 },
		{ 0,                             0,  0 },
	} },

	.sw_port_mapping = s18i20_gen3_sw_port_mapping,
};

static const struct scarlett2_device_info *scarlett2_devices[] = {
	/* Supported Gen 2 devices */
	&s6i6_gen2_info,
	&s18i8_gen2_info,
	&s18i20_gen2_info,

	/* Supported Gen 3 devices */
	&solo_gen3_info,
	&s2i2_gen3_info,
	&s4i4_gen3_info,
	&s8i6_gen3_info,
	&s18i8_gen3_info,
	&s18i20_gen3_info,

	/* End of list */
	NULL
};

/* get the starting port index number for a given port type/direction */
static int scarlett2_get_port_start_num(
	const int port_count[][SCARLETT2_PORT_DIRNS],
	int direction, int port_type)
{
	int i, num = 0;

	for (i = 0; i < port_type; i++)
		num += port_count[i][direction];

	return num;
}

/*** USB Interactions ***/

/* Notifications from the interface */
#define SCARLETT2_USB_NOTIFY_SYNC          0x00000008
#define SCARLETT2_USB_NOTIFY_DIM_MUTE      0x00200000
#define SCARLETT2_USB_NOTIFY_MONITOR       0x00400000
#define SCARLETT2_USB_NOTIFY_INPUT_OTHER   0x00800000
#define SCARLETT2_USB_NOTIFY_MONITOR_OTHER 0x01000000

/* Commands for sending/receiving requests/responses */
#define SCARLETT2_USB_CMD_INIT 0
#define SCARLETT2_USB_CMD_REQ  2
#define SCARLETT2_USB_CMD_RESP 3

#define SCARLETT2_USB_INIT_1    0x00000000
#define SCARLETT2_USB_INIT_2    0x00000002
#define SCARLETT2_USB_GET_METER 0x00001001
#define SCARLETT2_USB_GET_MIX   0x00002001
#define SCARLETT2_USB_SET_MIX   0x00002002
#define SCARLETT2_USB_GET_MUX   0x00003001
#define SCARLETT2_USB_SET_MUX   0x00003002
#define SCARLETT2_USB_GET_SYNC  0x00006004
#define SCARLETT2_USB_GET_DATA  0x00800000
#define SCARLETT2_USB_SET_DATA  0x00800001
#define SCARLETT2_USB_DATA_CMD  0x00800002

#define SCARLETT2_USB_CONFIG_SAVE 6

#define SCARLETT2_USB_VOLUME_STATUS_OFFSET 0x31
#define SCARLETT2_USB_METER_LEVELS_GET_MAGIC 1

/* volume status is read together (matches scarlett2_config_items[1]) */
struct scarlett2_usb_volume_status {
	/* dim/mute buttons */
	u8 dim_mute[SCARLETT2_DIM_MUTE_COUNT];

	u8 pad1;

	/* software volume setting */
	s16 sw_vol[SCARLETT2_ANALOGUE_OUT_MAX];

	/* actual volume of output inc. dim (-18dB) */
	s16 hw_vol[SCARLETT2_ANALOGUE_OUT_MAX];

	/* internal mute buttons */
	u8 mute_switch[SCARLETT2_ANALOGUE_OUT_MAX];

	/* sw (0) or hw (1) controlled */
	u8 sw_hw_switch[SCARLETT2_ANALOGUE_OUT_MAX];

	u8 pad3[6];

	/* front panel volume knob */
	s16 master_vol;
} __packed;

/* Configuration parameters that can be read and written */
enum {
	SCARLETT2_CONFIG_DIM_MUTE = 0,
	SCARLETT2_CONFIG_LINE_OUT_VOLUME = 1,
	SCARLETT2_CONFIG_MUTE_SWITCH = 2,
	SCARLETT2_CONFIG_SW_HW_SWITCH = 3,
	SCARLETT2_CONFIG_LEVEL_SWITCH = 4,
	SCARLETT2_CONFIG_PAD_SWITCH = 5,
	SCARLETT2_CONFIG_MSD_SWITCH = 6,
	SCARLETT2_CONFIG_AIR_SWITCH = 7,
	SCARLETT2_CONFIG_PHANTOM_SWITCH = 8,
	SCARLETT2_CONFIG_PHANTOM_PERSISTENCE = 9,
	SCARLETT2_CONFIG_DIRECT_MONITOR = 10,
	SCARLETT2_CONFIG_MONITOR_OTHER_SWITCH = 11,
	SCARLETT2_CONFIG_MONITOR_OTHER_ENABLE = 12,
	SCARLETT2_CONFIG_TALKBACK_MAP = 13,
	SCARLETT2_CONFIG_COUNT = 14
};

/* Location, size, and activation command number for the configuration
 * parameters. Size is in bits and may be 1, 8, or 16.
 */
struct scarlett2_config {
	u8 offset;
	u8 size;
	u8 activate;
};

/* scarlett2_config_items[0] is for devices without a mixer
 * scarlett2_config_items[1] is for devices with a mixer
 */
static const struct scarlett2_config
	scarlett2_config_items[2][SCARLETT2_CONFIG_COUNT] =

/* Devices without a mixer (Solo and 2i2 Gen 3) */
{ {
	[SCARLETT2_CONFIG_MSD_SWITCH] = {
		.offset = 0x04, .size = 8, .activate = 6 },

	[SCARLETT2_CONFIG_PHANTOM_PERSISTENCE] = {
		.offset = 0x05, .size = 8, .activate = 6 },

	[SCARLETT2_CONFIG_PHANTOM_SWITCH] = {
		.offset = 0x06, .size = 8, .activate = 3 },

	[SCARLETT2_CONFIG_DIRECT_MONITOR] = {
		.offset = 0x07, .size = 8, .activate = 4 },

	[SCARLETT2_CONFIG_LEVEL_SWITCH] = {
		.offset = 0x08, .size = 1, .activate = 7 },

	[SCARLETT2_CONFIG_AIR_SWITCH] = {
		.offset = 0x09, .size = 1, .activate = 8 },

/* Devices with a mixer (Gen 2 and all other Gen 3) */
}, {
	[SCARLETT2_CONFIG_DIM_MUTE] = {
		.offset = 0x31, .size = 8, .activate = 2 },

	[SCARLETT2_CONFIG_LINE_OUT_VOLUME] = {
		.offset = 0x34, .size = 16, .activate = 1 },

	[SCARLETT2_CONFIG_MUTE_SWITCH] = {
		.offset = 0x5c, .size = 8, .activate = 1 },

	[SCARLETT2_CONFIG_SW_HW_SWITCH] = {
		.offset = 0x66, .size = 8, .activate = 3 },

	[SCARLETT2_CONFIG_LEVEL_SWITCH] = {
		.offset = 0x7c, .size = 8, .activate = 7 },

	[SCARLETT2_CONFIG_PAD_SWITCH] = {
		.offset = 0x84, .size = 8, .activate = 8 },

	[SCARLETT2_CONFIG_AIR_SWITCH] = {
		.offset = 0x8c, .size = 8, .activate = 8 },

	[SCARLETT2_CONFIG_PHANTOM_SWITCH] = {
		.offset = 0x9c, .size = 1, .activate = 8 },

	[SCARLETT2_CONFIG_MSD_SWITCH] = {
		.offset = 0x9d, .size = 8, .activate = 6 },

	[SCARLETT2_CONFIG_PHANTOM_PERSISTENCE] = {
		.offset = 0x9e, .size = 8, .activate = 6 },

	[SCARLETT2_CONFIG_MONITOR_OTHER_SWITCH] = {
		.offset = 0x9f, .size = 1, .activate = 10 },

	[SCARLETT2_CONFIG_MONITOR_OTHER_ENABLE] = {
		.offset = 0xa0, .size = 1, .activate = 10 },

	[SCARLETT2_CONFIG_TALKBACK_MAP] = {
		.offset = 0xb0, .size = 16, .activate = 10 },
} };

/* proprietary request/response format */
struct scarlett2_usb_packet {
	__le32 cmd;
	__le16 size;
	__le16 seq;
	__le32 error;
	__le32 pad;
	u8 data[];
};

static char *scarlett2_fmt_port_name(char *out, int length, 
									 int direction, int port_type, int port_num,
									 const char * const postfix)
{
	int postfix_length;
	int printed;
	const char *const descr = (direction == SCARLETT2_PORT_IN)
								? scarlett2_ports[port_type].src_descr
								: scarlett2_ports[port_type].dst_descr;
	if (postfix != NULL)
	{
		postfix_length = strlen(postfix) + 1;
		printed = snprintf(out, length - postfix_length - 1, descr, port_num + 1);
		out[printed++] = ' ';
		memcpy(&out[printed], postfix, postfix_length);
	}
	else
		snprintf(out, length - 1, descr, port_num + 1);

	out[length-1] = '\0';
	return out;
}

static char *scarlett2_fmt_stereo_output_port_name(char *out, int length, 
									 int port_type, int port_num,
									 const char * const postfix)
{
	int postfix_length;
	int printed;
	const char *const descr = scarlett2_ports[port_type].dst_stereo_descr;
	if (postfix != NULL)
	{
		postfix_length = strlen(postfix) + 1;
		printed = snprintf(out, length - postfix_length - 1, descr, port_num + 1, port_num + 2);
		out[printed++] = ' ';
		memcpy(&out[printed], postfix, postfix_length);
	}
	else
		snprintf(out, length - 1, descr, port_num + 1);

	out[length-1] = '\0';
	return out;
}

/**
 * Convert the hardware port nuber to the FC software port number
 */
static int scarlett2_get_drv_to_sw_port_mapping(int direction, int port_type, int port_num, 
										        const struct scarlett2_device_info *info)
{
	const struct scarlett2_sw_port_mapping *port_mapping = info->sw_port_mapping;
	int offset;
	if (!port_mapping)
		return -1;

	/* Check that mapping is present and return the updated port number */
	for (; port_mapping->type != SCARLETT2_PORT_TYPE_COUNT; ++port_mapping)
	{
		if ((direction != port_mapping->direction) ||
			(port_type != port_mapping->type))
			continue;
		offset = port_num - port_mapping->hw_offset;
		if ((offset >= 0) &&
			(offset < port_mapping->count))
			return port_mapping->sw_offset + offset;
	}
	
	return -1;
}

/* Convert the FC software port number to the pair of numbers:
 *   - driver port type (port_type)
 *   - driver port number (port_num)
 */
static int scarlett2_get_sw_to_drv_port_mapping(int *port_type, int *port_num,
										  const struct scarlett2_device_info *info,
										  int direction, int sw_port_number)
{
	const struct scarlett2_sw_port_mapping *port_mapping = info->sw_port_mapping;
	
	*port_type = SCARLETT2_PORT_TYPE_NONE;
	*port_num = 0;
	
	/* Check that mapping is present and return the updated port number */
	for (; port_mapping->type != SCARLETT2_PORT_TYPE_COUNT; ++port_mapping)
	{
		if (direction != port_mapping->direction)
			continue;
		if ((sw_port_number >= port_mapping->sw_offset) &&
			(sw_port_number < (port_mapping->sw_offset + port_mapping->count)))
		{
			*port_type = port_mapping->type;
			*port_num  = sw_port_number - port_mapping->sw_offset + port_mapping->hw_offset;
			return 1;
		}
	}
	
	return 0;
}

/**
 * Convert the driver port defined by (direction, port_type, port_num) to the atual
 * MUX enum value index of the driver
 */
static int scarlett2_drv_port_number_to_mux_enum(int direction, int port_type, int port_num, 
										 const struct scarlett2_device_info *info)
{
	int mux_port_number = 0;
	int current_port_type;
	
	/* Iterate over all port types and add the offset if the provided port type
	   doesn't match the current port type */
	for (current_port_type = SCARLETT2_PORT_TYPE_NONE;
		 current_port_type < SCARLETT2_PORT_TYPE_COUNT;
	     ++current_port_type) {
		if (port_type == current_port_type)
		{
			return (port_num < info->port_count[current_port_type][direction])
				? mux_port_number + port_num
				: 0;
		}
		else
			mux_port_number += info->port_count[current_port_type][direction];
	}
	
	return 0;
}

/**
 * Convert the mux enum value to the driver port number by (port_type, port_num)
 */
static int scarlett2_mux_enum_to_drv_port_number(int *port_type, int *port_num,
										const struct scarlett2_device_info *info,
										int direction, int mux_port_number)
{
	int current_port_type;
	
	*port_type = 0;
	*port_num = 0;
	
	/* Iterate over all port types and add the offset if the provided port type
	   doesn't match the current port type */
	for (current_port_type = SCARLETT2_PORT_TYPE_NONE;
		 current_port_type < SCARLETT2_PORT_TYPE_COUNT;
	     ++current_port_type) {
		if (mux_port_number < info->port_count[current_port_type][direction])
		{
			*port_type = current_port_type;
			*port_num = mux_port_number;
			return 1;
		}
		else
			mux_port_number -= info->port_count[current_port_type][direction];
	}
	
	return 0;
}

/** Remap the number of analogue input if the device needs remapping */
static int line_out_remap(struct scarlett2_data *private, int index)
{
	const struct scarlett2_device_info *info = private->info;

	if (!info->line_out_remap_enable)
		return index;
	return info->line_out_remap[index];
}

static void scarlett2_fill_request_header(struct scarlett2_data *private,
					  struct scarlett2_usb_packet *req,
					  u32 cmd, u16 req_size)
{
	/* sequence must go up by 1 for each request */
	u16 seq = private->scarlett2_seq++;

	req->cmd = cpu_to_le32(cmd);
	req->size = cpu_to_le16(req_size);
	req->seq = cpu_to_le16(seq);
	req->error = 0;
	req->pad = 0;
}

static int scarlett2_usb_tx(struct usb_device *dev, int interface,
			    void *buf, u16 size)
{
	return snd_usb_ctl_msg(dev, usb_sndctrlpipe(dev, 0),
			SCARLETT2_USB_CMD_REQ,
			USB_RECIP_INTERFACE | USB_TYPE_CLASS | USB_DIR_OUT,
			0, interface, buf, size);
}

static int scarlett2_usb_rx(struct usb_device *dev, int interface,
			    u32 usb_req, void *buf, u16 size)
{
	return snd_usb_ctl_msg(dev, usb_rcvctrlpipe(dev, 0),
			usb_req,
			USB_RECIP_INTERFACE | USB_TYPE_CLASS | USB_DIR_IN,
			0, interface, buf, size);
}

/* Send a proprietary format request to the Scarlett interface */
static int scarlett2_usb(
	struct usb_mixer_interface *mixer, u32 cmd,
	void *req_data, u16 req_size, void *resp_data, u16 resp_size)
{
	struct scarlett2_data *private = mixer->private_data;
	struct usb_device *dev = mixer->chip->dev;
	u16 req_buf_size = sizeof(struct scarlett2_usb_packet) + req_size;
	u16 resp_buf_size = sizeof(struct scarlett2_usb_packet) + resp_size;
	struct scarlett2_usb_packet *req, *resp = NULL;
	int err;

	req = kmalloc(req_buf_size, GFP_KERNEL);
	if (!req) {
		err = -ENOMEM;
		goto error;
	}

	resp = kmalloc(resp_buf_size, GFP_KERNEL);
	if (!resp) {
		err = -ENOMEM;
		goto error;
	}

	mutex_lock(&private->usb_mutex);

	/* build request message and send it */

	scarlett2_fill_request_header(private, req, cmd, req_size);

	if (req_size)
		memcpy(req->data, req_data, req_size);

	err = scarlett2_usb_tx(dev, private->bInterfaceNumber,
			       req, req_buf_size);

	if (err != req_buf_size) {
		usb_audio_err(
			mixer->chip,
			"Scarlett Gen 2/3 USB request result cmd %x was %d\n",
			cmd, err);
		err = -EINVAL;
		goto unlock;
	}

	/* send a second message to get the response */

	err = scarlett2_usb_rx(dev, private->bInterfaceNumber,
			       SCARLETT2_USB_CMD_RESP,
			       resp, resp_buf_size);

	/* validate the response */

	if (err != resp_buf_size) {
		usb_audio_err(
			mixer->chip,
			"Scarlett Gen 2/3 USB response result cmd %x was %d "
			"expected %d\n",
			cmd, err, resp_buf_size);
		err = -EINVAL;
		goto unlock;
	}

	/* cmd/seq/size should match except when initialising
	 * seq sent = 1, response = 0
	 */
	if (resp->cmd != req->cmd ||
	    (resp->seq != req->seq &&
		(le16_to_cpu(req->seq) != 1 || resp->seq != 0)) ||
	    resp_size != le16_to_cpu(resp->size) ||
	    resp->error ||
	    resp->pad) {
		usb_audio_err(
			mixer->chip,
			"Scarlett Gen 2/3 USB invalid response; "
			   "cmd tx/rx %d/%d seq %d/%d size %d/%d "
			   "error %d pad %d\n",
			le32_to_cpu(req->cmd), le32_to_cpu(resp->cmd),
			le16_to_cpu(req->seq), le16_to_cpu(resp->seq),
			resp_size, le16_to_cpu(resp->size),
			le32_to_cpu(resp->error),
			le32_to_cpu(resp->pad));
		err = -EINVAL;
		goto unlock;
	}

	if (resp_data && resp_size > 0)
		memcpy(resp_data, resp->data, resp_size);

unlock:
	mutex_unlock(&private->usb_mutex);
error:
	kfree(req);
	kfree(resp);
	return err;
}

/* Send a USB message to get data; result placed in *buf */
static int scarlett2_usb_get(
	struct usb_mixer_interface *mixer,
	int offset, void *data, int bytes)
{
	struct {
		__le32 offset;
		__le32 size;
	} __packed req;

	int i, chunk, err;
	u8 *buf = (u8 *)data;

	/* Request the config space with fixed-size data chunks */
	for (i=0; i<bytes; i += chunk) {
		/* Compute the chunk size */
		chunk = (bytes - i);
		if (chunk > SCARLETT2_SW_CONFIG_PACKET_SIZE)
			chunk = SCARLETT2_SW_CONFIG_PACKET_SIZE;

		/* Request yet another chunk */
		req.offset = cpu_to_le32(offset + i);
		req.size   = cpu_to_le32(chunk);

		usb_audio_info(mixer->chip, "requesting chunk: req.offset=0x%x, req.bytes=%d\n", le32_to_cpu(req.offset), le32_to_cpu(req.size));
		err = scarlett2_usb(mixer, SCARLETT2_USB_GET_DATA, &req, sizeof(req), &buf[i], chunk);
		if (err < 0)
			return err;
	}

	return 0;
}

/* Send a set of USB messages to set configuration data */
static int scarlett2_usb_set(
	struct usb_mixer_interface *mixer,
	int offset, const void *data, int bytes)
{
	struct {
		__le32 offset;
		__le32 size;
		u8 data[];
	} __packed *req;
	int i, chunk, err = 0;
	const u8 *buf = (const u8 *)data;

	usb_audio_info(mixer->chip, "scarlett2_usb_set offset=0x%x, data=0x%lx, bytes=0x%x", offset, (long)data, bytes);

	/* Allocate buffer */
	req = kmalloc(sizeof(__le32)*2 + SCARLETT2_SW_CONFIG_PACKET_SIZE, GFP_KERNEL);
	if (!req) {
		err = -ENOMEM;
		goto error;
	}

	/* Transfer the configuration with fixed-size data chunks */
	for (i=0; i<bytes; i += chunk) {
		/* Compute the chunk size */
		chunk = (bytes - i);
		if (chunk > SCARLETT2_SW_CONFIG_PACKET_SIZE)
			chunk = SCARLETT2_SW_CONFIG_PACKET_SIZE;

		/* Send yet another chunk of data */
		req->offset = cpu_to_le32(offset + i);
		req->size   = cpu_to_le32(chunk);
		memcpy(req->data, &buf[i], chunk);

		usb_audio_info(mixer->chip, "transferring chunk: req->offset=0x%x, req->bytes=%d, req->data=0x%lx, ptr=0x%lx\n", 
		    le32_to_cpu(req->offset), le32_to_cpu(req->size), (long)&req->data[0], (long)&buf[i]);
		err = scarlett2_usb(mixer, SCARLETT2_USB_SET_DATA, req, chunk + sizeof(__le32)*2, NULL, 0);
		if (err < 0)
			goto error;
	}

error:
	kfree(req);
	return err;
}

static void scarlett2_calc_software_cksum(struct scarlett2_sw_cfg *sw) {
	int i;
	s32 checksum;
	const __le32 *ckptr;

	/* The checksum is a sign-invered sum of 32-bit integers
	  assuming checksum field value being 0 */
	checksum = 0;
	sw->checksum = 0;
	ckptr = (__le32 *)sw;
	for (i=0; i<sizeof(struct scarlett2_sw_cfg); i += sizeof(__le32))
		checksum -= le32_to_cpu(*(ckptr++));
	sw->checksum = cpu_to_le32(checksum);
}

static int scarlett2_read_software_configs(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	struct scarlett2_sw_cfg *sw;
	int err;
	__le16 le_sw_cfg_size;
	int sw_cfg_size;

	/* Check that device has software configuration */
	if (!private->info->has_sw_config) {
		usb_audio_info(mixer->chip, "Device has no software configuration");
		return 0;
	}

	/* Obtain actual size of sofware config */
	err = scarlett2_usb_get(mixer,
		SCARLETT2_SW_CONFIG_BASE + offsetof(struct scarlett2_sw_cfg, szof),
		&le_sw_cfg_size, sizeof(le_sw_cfg_size)
	);

	if (err < 0)
		return err;

	/* We need to create software configuration area if it does not exist
	   or it is present and has valid size */
	sw_cfg_size = le16_to_cpu(le_sw_cfg_size);

	/* Allocate space for software config */
	sw = kzalloc(sizeof(struct scarlett2_sw_cfg), GFP_KERNEL);
	if (sw == NULL)
		return -ENOMEM;

	if (sw_cfg_size == 0) {
		usb_audio_info(mixer->chip, "Creating software configuration area for device");

		sw->all_size    = cpu_to_le16(sizeof(struct scarlett2_sw_cfg) + 0x0c);
		sw->custom_mix  = (u32)-1;
		sw->magic1      = cpu_to_le16(0x3006);
		sw->version     = cpu_to_le32(0x5);
		sw->szof        = cpu_to_le16(sizeof(struct scarlett2_sw_cfg));
		scarlett2_calc_software_cksum(sw);

		err = scarlett2_usb_set(mixer, SCARLETT2_SW_CONFIG_BASE, sw, sizeof(struct scarlett2_sw_cfg));
	}
	else if (sw_cfg_size != sizeof(struct scarlett2_sw_cfg)) {
		/* Free allocated area, output warning and return */
		usb_audio_warn(mixer->chip, "Unsupported size of software configuration "
		    "area (0x%x), expected to be 0x%x, will proceed with significantly "
		    "lower functionality",
		    sw_cfg_size, (int)(sizeof(struct scarlett2_sw_cfg))
		);
		goto leave;
	} else
		err = scarlett2_usb_get(mixer, SCARLETT2_SW_CONFIG_BASE, sw, sizeof(struct scarlett2_sw_cfg));

	if (err < 0)
		goto leave;

	/* Validate the software configuration area header */
	usb_audio_info(mixer->chip, "sw.all_size = 0x%x\n", le16_to_cpu(sw->all_size));
	usb_audio_info(mixer->chip, "sw.magic1   = 0x%x\n", le16_to_cpu(sw->magic1));
	usb_audio_info(mixer->chip, "sw.version  = 0x%x\n", le32_to_cpu(sw->version));
	usb_audio_info(mixer->chip, "sw.szof     = 0x%x\n", le16_to_cpu(sw->szof));

	if ((le16_to_cpu(sw->all_size) != (sizeof(struct scarlett2_sw_cfg) + 0x0c)) ||
	    (le16_to_cpu(sw->magic1) != 0x3006) ||
	    (le32_to_cpu(sw->version) != 0x5) ||
	    (le16_to_cpu(sw->szof) != sizeof(struct scarlett2_sw_cfg))) {

		usb_audio_warn(mixer->chip, "The format of software configuration header "
		    "does not match expected, will proceed with significantly "
		    "lower functionality"
		);
		goto leave;
	}

	/* Commit the pointer */
	private->sw_cfg = sw;
	sw              = NULL;

	usb_audio_info(mixer->chip, "Successfully initialized software configuration area");

leave:
	if (sw != NULL)
		kfree(sw);
	return err;
}

static int scarlett2_commit_software_config(
	struct usb_mixer_interface *mixer,
	void *ptr, /* the pointer of the first changed byte in the configuration */
	int bytes /* the actual number of bytes changed in the configuration */
)
{
	struct scarlett2_data *private = mixer->private_data;
	int offset, err = 0;

	/* Check bounds, we should not exceed them */
	offset = ((u8 *)ptr) - ((u8 *)private->sw_cfg);
	usb_audio_info(mixer->chip, "scarlett2_commit_software_config offset=0x%04x(rel: 0x%04x), bytes=%d, ptr=%lx", offset + SCARLETT2_SW_CONFIG_BASE, offset, bytes, (long)ptr);

	if ((private->sw_cfg == NULL) ||
	    (offset < 0) ||
	    ((offset + bytes) > sizeof(struct scarlett2_sw_cfg))) {
		usb_audio_warn(mixer->chip, "tried to commit data with invalid offset %d", offset);
		return -EINVAL;
	}

	/* Re-compute the checksum of the software configuration area */
	scarlett2_calc_software_cksum(private->sw_cfg);

	/* Cancel any pending NVRAM save */
	usb_audio_info(mixer->chip, "cancel_delayed_work_sync");
	cancel_delayed_work_sync(&private->work);

	/* Transfer the configuration with fixed-size data chunks */
	err = scarlett2_usb_set(mixer, SCARLETT2_SW_CONFIG_BASE + offset, ptr, bytes);
	if (err < 0)
		goto leave;

	/* Transfer the actual checksum */
	err = scarlett2_usb_set(mixer, SCARLETT2_SW_CONFIG_BASE + offsetof(struct scarlett2_sw_cfg, checksum),
	                        &private->sw_cfg->checksum, sizeof(private->sw_cfg->checksum)
	         );
	if (err < 0)
		goto leave;

leave:
	/* Schedule the change to be written to NVRAM */
	usb_audio_info(mixer->chip, "calling schedule_delayed_work");
	schedule_delayed_work(&private->work, msecs_to_jiffies(2000));

	usb_audio_info(mixer->chip, "exit scarlett2_commit_software_config");
	return err;
}

/* DEBUG ONLY */
void scarlett2_dump_buffer(const void *data, int bytes)
{
	char s[0x80];
	int i, j;
	const u8 *buf = (const u8 *)data;
	int off = 0;

	s[off] = '\0';
	printk("      00 01 02 03 | 04 05 06 07 | 08 09 0a 0b | 0c 0d 0e 0f   0123456789abcdef");

	for (i=0; i<bytes; i += 16) {
		/* New line */
		if (!(i&0x0f)) {
			off += sprintf(&s[off], "\n");
			printk("%s", s);
			if (!(i&0xff))
				printk("      -----------------------------------------------------   ----------------\n");

			off  = 0;
			off += sprintf(&s[off], "%04x:", i);
		}

		/* Dump hex codes */
		for (j=i; j<i+16; ++j) {
			if (((j & 0x0c) > 0) && (!(j & 0x03)))
				off += sprintf(&s[off], " |");
			if (j < bytes)
				off += sprintf(&s[off], " %02x", buf[j]);
			else
				off += sprintf(&s[off], "   ");
		}

		off += sprintf(&s[off], "   ");

		for (j=i; j<i+16; ++j) {
			if (j < bytes)
				s[off++] = ((buf[j] >= 0x20) && (buf[j] < 0x7f)) ? buf[j] : '.';
			else
				s[off++] = ' ';
		}
	}

	strcpy(&s[off], "\n");
	printk("%s", s);
}

/* Debug function: dump configuration space of the device */
static int scarlett2_dump_config_space(struct usb_mixer_interface *mixer) {
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	__le32 le_buf_size;
	int buf_size;
	int err = 0;
	u8 *buf;

	/* Estimate the overall size of configuration area */
	if (!info->config_size) {
		err = scarlett2_usb_get(mixer, 0, &le_buf_size, sizeof(__le32));
		if (err < 0)
			return err;
		buf_size = le32_to_cpu(le_buf_size);
		if (buf_size > 0x4000)
			buf_size = 0x4000;
	}
	else
		buf_size = info->config_size;

	usb_audio_info(mixer->chip, "Configuration space size: 0x%x (%d)", buf_size, buf_size);

	/* Allocate memory for buffer */
	buf = kmalloc(buf_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	err = scarlett2_usb_get(mixer, 0, buf, buf_size);
	if (err < 0)
		goto error;

	usb_audio_info(mixer->chip, "Dump of the overall device configuration space:");
	scarlett2_dump_buffer(buf, buf_size);

error:
	kfree(buf);
	return err;
}

/* Send a USB message to get configuration parameters; result placed in *buf */
static int scarlett2_usb_get_config(
	struct usb_mixer_interface *mixer,
	int config_item_num, int count, void *buf)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const struct scarlett2_config *config_item =
		&scarlett2_config_items[info->has_mixer][config_item_num];
	int size, err, i;
	u8 *buf_8;
	u8 value;

	/* For byte-sized parameters, retrieve directly into buf */
	if (config_item->size >= 8) {
		size = config_item->size / 8 * count;
		err = scarlett2_usb_get(mixer, config_item->offset, buf, size);
		if (err < 0)
			return err;
		if (size == 2) {
			u16 *buf_16 = buf;

			for (i = 0; i < count; i++, buf_16++)
				*buf_16 = le16_to_cpu(*(__le16 *)buf_16);
		}
		return 0;
	}

	/* For bit-sized parameters, retrieve into value */
	err = scarlett2_usb_get(mixer, config_item->offset, &value, 1);
	if (err < 0)
		return err;

	/* then unpack from value into buf[] */
	buf_8 = buf;
	for (i = 0; i < 8 && i < count; i++, value >>= 1)
		*buf_8++ = value & 1;

	return 0;
}

/* Send SCARLETT2_USB_DATA_CMD SCARLETT2_USB_CONFIG_SAVE */
static void scarlett2_config_save(struct usb_mixer_interface *mixer)
{
	__le32 req = cpu_to_le32(SCARLETT2_USB_CONFIG_SAVE);

	scarlett2_usb(mixer, SCARLETT2_USB_DATA_CMD,
		      &req, sizeof(u32),
		      NULL, 0);
}

/* Delayed work to save config */
static void scarlett2_config_save_work(struct work_struct *work)
{
	struct scarlett2_data *private =
		container_of(work, struct scarlett2_data, work.work);

	scarlett2_config_save(private->mixer);
}

/* Send a USB message to set a SCARLETT2_CONFIG_* parameter */
static int scarlett2_usb_set_config(
	struct usb_mixer_interface *mixer,
	int config_item_num, int index, int value)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const struct scarlett2_config *config_item =
	       &scarlett2_config_items[info->has_mixer][config_item_num];
	struct {
		__le32 offset;
		__le32 bytes;
		__le32 value;
	} __packed req;
	__le32 req2;
	int offset, size;
	int err;

	/* Cancel any pending NVRAM save */
	cancel_delayed_work_sync(&private->work);

	/* Convert config_item->size in bits to size in bytes and
	 * calculate offset
	 */
	if (config_item->size >= 8) {
		size = config_item->size / 8;
		offset = config_item->offset + index * size;

	/* If updating a bit, retrieve the old value, set/clear the
	 * bit as needed, and update value
	 */
	} else {
		u8 tmp;

		size = 1;
		offset = config_item->offset;

		scarlett2_usb_get(mixer, offset, &tmp, 1);
		if (value)
			tmp |= (1 << index);
		else
			tmp &= ~(1 << index);

		value = tmp;
	}

	/* Send the configuration parameter data */
	req.offset = cpu_to_le32(offset);
	req.bytes = cpu_to_le32(size);
	req.value = cpu_to_le32(value);
	err = scarlett2_usb(mixer, SCARLETT2_USB_SET_DATA,
			    &req, sizeof(u32) * 2 + size,
			    NULL, 0);
	if (err < 0)
		return err;

	/* Activate the change */
	req2 = cpu_to_le32(config_item->activate);
	err = scarlett2_usb(mixer, SCARLETT2_USB_DATA_CMD,
			    &req2, sizeof(req2), NULL, 0);
	if (err < 0)
		return err;

	/* Schedule the change to be written to NVRAM */
	if (config_item->activate != SCARLETT2_USB_CONFIG_SAVE)
		schedule_delayed_work(&private->work, msecs_to_jiffies(2000));

	return 0;
}

/* Send a USB message to get sync status; result placed in *sync */
static int scarlett2_usb_get_sync_status(
	struct usb_mixer_interface *mixer,
	u8 *sync)
{
	__le32 data;
	int err;

	err = scarlett2_usb(mixer, SCARLETT2_USB_GET_SYNC,
			    NULL, 0, &data, sizeof(data));
	if (err < 0)
		return err;

	*sync = !!data;
	return 0;
}

/* Send a USB message to get volume status; result placed in *buf */
static int scarlett2_usb_get_volume_status(
	struct usb_mixer_interface *mixer,
	struct scarlett2_usb_volume_status *buf)
{
	return scarlett2_usb_get(mixer, SCARLETT2_USB_VOLUME_STATUS_OFFSET,
				 buf, sizeof(*buf));
}

/* Send a USB message to get the volumes for all inputs of one mix
 * and put the values into private->mix[]
 */
static int scarlett2_usb_get_mix(struct usb_mixer_interface *mixer,
				 int mix_num)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;

	int num_mixer_in =
		info->port_count[SCARLETT2_PORT_TYPE_MIX][SCARLETT2_PORT_OUT];
	int err, i, j, k;

	struct {
		__le16 mix_num;
		__le16 count;
	} __packed req;

	__le16 data[SCARLETT2_INPUT_MIX_MAX];

	req.mix_num = cpu_to_le16(mix_num);
	req.count = cpu_to_le16(num_mixer_in);

	err = scarlett2_usb(mixer, SCARLETT2_USB_GET_MIX,
			    &req, sizeof(req),
			    data, num_mixer_in * sizeof(u16));
	if (err < 0)
		return err;

	for (i = 0, j = mix_num * num_mixer_in; i < num_mixer_in; i++, j++) {
		u16 mixer_value = le16_to_cpu(data[i]);

		for (k = 0; k < SCARLETT2_MIXER_VALUE_COUNT; k++)
			if (scarlett2_mixer_values[k] >= mixer_value)
				break;
		if (k == SCARLETT2_MIXER_VALUE_COUNT)
			k = SCARLETT2_MIXER_MAX_VALUE;
		private->mix[j] = k;
	}

	return 0;
}

/* Send a USB message to set the volumes for all inputs of one mix
 * (values obtained from private->mix[])
 */
static int scarlett2_usb_set_mix(struct usb_mixer_interface *mixer,
				 int mix_num)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;

	struct {
		__le16 mix_num;
		__le16 data[SCARLETT2_INPUT_MIX_MAX];
	} __packed req;

	int i, j;
	int num_mixer_in =
		info->port_count[SCARLETT2_PORT_TYPE_MIX][SCARLETT2_PORT_OUT];

	req.mix_num = cpu_to_le16(mix_num);

	for (i = 0, j = mix_num * num_mixer_in; i < num_mixer_in; i++, j++)
		req.data[i] = cpu_to_le16(
			scarlett2_mixer_values[private->mix[j]]
		);

	return scarlett2_usb(mixer, SCARLETT2_USB_SET_MIX,
			     &req, (num_mixer_in + 1) * sizeof(u16),
			     NULL, 0);
}

/* Convert a port number index (per info->port_count) to a hardware ID */
static u32 scarlett2_mux_src_num_to_id(
	const int port_count[][SCARLETT2_PORT_DIRNS], int num)
{
	int port_type;

	for (port_type = 0;
	     port_type < SCARLETT2_PORT_TYPE_COUNT;
	     port_type++) {
		if (num < port_count[port_type][SCARLETT2_PORT_IN])
			return scarlett2_ports[port_type].id | num;
		num -= port_count[port_type][SCARLETT2_PORT_IN];
	}

	/* Oops */
	return 0;
}

/* Convert a hardware ID to a port number index */
static u32 scarlett2_mux_id_to_num(
	const int port_count[][SCARLETT2_PORT_DIRNS], int direction, u32 id)
{
	int port_type;
	int port_num = 0;

	for (port_type = 0;
	     port_type < SCARLETT2_PORT_TYPE_COUNT;
	     port_type++) {
		int base = scarlett2_ports[port_type].id;
		int count = port_count[port_type][direction];

		if (id >= base && id < base + count)
			return port_num + id - base;
		port_num += count;
	}

	/* Oops */
	return -1;
}

/* Convert one mux entry from the interface and load into private->mux[] */
static void scarlett2_usb_populate_mux(struct scarlett2_data *private,
				       u32 mux_entry)
{
	const struct scarlett2_device_info *info = private->info;
	const int (*port_count)[SCARLETT2_PORT_DIRNS] = info->port_count;

	int dst_idx, src_idx;

	dst_idx = scarlett2_mux_id_to_num(port_count, SCARLETT2_PORT_OUT,
					  mux_entry & 0xFFF);
	if (dst_idx < 0)
		return;

	if (dst_idx >= private->num_mux_dsts) {
		usb_audio_err(private->mixer->chip,
			"BUG: scarlett2_mux_id_to_num(%06x, OUT): %d >= %d",
			mux_entry, dst_idx, private->num_mux_dsts);
		return;
	}

	src_idx = scarlett2_mux_id_to_num(port_count, SCARLETT2_PORT_IN,
					  mux_entry >> 12);
	if (src_idx < 0)
		return;

	if (src_idx >= private->num_mux_srcs) {
		usb_audio_err(private->mixer->chip,
			"BUG: scarlett2_mux_id_to_num(%06x, IN): %d >= %d",
			mux_entry, src_idx, private->num_mux_srcs);
		return;
	}

	private->mux[dst_idx] = src_idx;
}

/* Send USB message to get mux inputs and then populate private->mux[] */
static int scarlett2_usb_get_mux(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	int count = private->num_mux_dsts;
	int err, i;

	struct {
		__le16 num;
		__le16 count;
	} __packed req;

	__le32 data[SCARLETT2_MUX_MAX];

	private->mux_updated = 0;

	req.num = 0;
	req.count = cpu_to_le16(count);

	err = scarlett2_usb(mixer, SCARLETT2_USB_GET_MUX,
			    &req, sizeof(req),
			    data, count * sizeof(u32));
	if (err < 0)
		return err;

	for (i = 0; i < count; i++)
		scarlett2_usb_populate_mux(private, le32_to_cpu(data[i]));

	return 0;
}

/* Send USB messages to set mux inputs */
static int scarlett2_usb_set_mux(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const int (*port_count)[SCARLETT2_PORT_DIRNS] = info->port_count;
	int i, j, err;
	int table, port_type, src_id, dst_id, mux_idx;

	struct {
		__le16 pad;
		__le16 num;
		__le32 data[SCARLETT2_MUX_MAX];
	} __packed req;

	req.pad = 0;

	/* set mux settings for each rate */
	for (table = 0; table < SCARLETT2_MUX_TABLES; table++) {
		const struct scarlett2_mux_entry *entry;

		/* i counts over the output array */
		i = 0;
		req.num = cpu_to_le16(table);
		
		usb_audio_info(mixer->chip, "UPDATING MUX # %d\n", table);

		/* loop through each entry */
		for (entry = info->mux_assignment[table];
		     entry->count;
		     entry++) {
			port_type = entry->port_type;
			mux_idx = scarlett2_get_port_start_num(port_count,
				SCARLETT2_PORT_OUT, port_type) + entry->start;
			dst_id = scarlett2_ports[port_type].id + entry->start;

			/* Empty slots */
			if (!dst_id) {
				for (j = 0; j < entry->count; j++)
				{
					req.data[i++] = 0;
					usb_audio_info(mixer->chip, "  routing[%d] = 0x%06x\n", i-1, req.data[i-1]);
				}
				continue;
			}

			/* Non-empty mux slots use the lower 12 bits
			 * for the destination and next 12 bits for
			 * the source
			 */
			for (j = 0; j < entry->count; j++) {
				usb_audio_info(mixer->chip, "  type = %d, mux[%d] == %d\n",
							port_type, mux_idx, private->mux[mux_idx]);
				src_id = scarlett2_mux_src_num_to_id(port_count, private->mux[mux_idx++]);

				/* Additional functions if software configuration area is present */
				if (private->sw_cfg != NULL) {
					int index = scarlett2_get_drv_to_sw_port_mapping(
						SCARLETT2_PORT_OUT, port_type, entry->start + j, info);
					
					/* Apply mutes to several device outputs */
					if (index >= 0)
					{
						if (private->mute_switch[index])
							src_id = 0;
						usb_audio_info(mixer->chip, "  type = %d, mute_switch[%d] = %d\n",
									   port_type, index, private->mute_switch[index]);
					}
				}

				req.data[i++] = cpu_to_le32(dst_id |
							    src_id << 12);
				usb_audio_info(mixer->chip, "  routing[%d] = 0x%06x\n", i-1, req.data[i-1]);
				dst_id++;
			}
		}

		/* Send the mux state to the device */
		err = scarlett2_usb(mixer, SCARLETT2_USB_SET_MUX,
				    &req, (i + 1) * sizeof(u32),
				    NULL, 0);
		if (err < 0)
			return err;
	}

	return 0;
}

/* Update routing, return: 
 *   0 if no changes set
 *   1 if changes have been applied
 *   negative value on error
 */
static int scarlett2_mux_commit_route(struct usb_mixer_interface *mixer,
									  int output_num, int input_number)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	struct snd_card *card = mixer->chip->card;
	
	int err = 0;
	int drv_port_type, drv_port_number;
	int sw_input_number, sw_output_number;
	int stereo_output;
	u32 custom_mix, custom_mix_mask, new_custom_mix, stereo_sw;
	
	/* Check that the routing settings have changed */
	if (private->mux[output_num] == input_number)
		return 0;
	
	private->mux[output_num] = input_number;
	usb_audio_info(mixer->chip, "  mux[%d] = %d", output_num, input_number);
	
	/* Check that software config is present */
	if (private->sw_cfg) {
		/* Read the custom mix settings and stereo switches */
		stereo_sw  = le32_to_cpu(private->sw_cfg->stereo_sw);
		custom_mix = le32_to_cpu(private->sw_cfg->custom_mix);
		new_custom_mix = custom_mix;
		
		/* Obtain the output number */
		scarlett2_mux_enum_to_drv_port_number(&drv_port_type, &drv_port_number,
								info, SCARLETT2_PORT_OUT, output_num);
		sw_output_number = scarlett2_get_drv_to_sw_port_mapping(
				SCARLETT2_PORT_OUT, drv_port_type, drv_port_number, info);
		
		stereo_output = sw_output_number & (~1);
		custom_mix_mask = (stereo_sw & (3 << stereo_output)) ? 3 << stereo_output : 1 << sw_output_number;
		
		usb_audio_info(mixer->chip, "  stereo_sw=0x%x, custom_mix=0x%x, sw_output_number=%d, stereo_output=%d, custom_mix_mask=0x%x", 
			(int)stereo_sw, (int)custom_mix, sw_output_number, stereo_output, (int)custom_mix_mask);
		
		/* Obtain the input number */
		scarlett2_mux_enum_to_drv_port_number(&drv_port_type, &drv_port_number,
								info, SCARLETT2_PORT_IN, input_number);
		
		if (drv_port_type == SCARLETT2_PORT_TYPE_MIX) {
			sw_input_number = drv_port_number + 1;
			/* Enable the custom mix in the custom mix mask */
			new_custom_mix &= ~custom_mix_mask;
		}
		else {
			/* The physical input is routed to the physical output */
			sw_input_number = scarlett2_get_drv_to_sw_port_mapping(
					SCARLETT2_PORT_IN, drv_port_type, drv_port_number, info);
			/* Disable the custom mix in the custom mix mask */
			new_custom_mix |= custom_mix_mask;
		}
		
		/* Obtain the output number */
		usb_audio_info(mixer->chip, "  sw_cfg->out_mux[%d] = %d", sw_output_number, sw_input_number);
		if ((sw_output_number >= 0) && (sw_input_number >= 0)) {
			private->sw_cfg->out_mux[sw_output_number] = sw_input_number;
			err = scarlett2_commit_software_config(mixer, &private->sw_cfg->out_mux[sw_output_number], sizeof(u8));
			if (err < 0)
				return err;
		}
		
		/* Commit the custom mix mask if it is necessary */
		if (new_custom_mix != custom_mix) {
			usb_audio_info(mixer->chip, "  custom_mix: 0x%x -> 0x%x",
				custom_mix, new_custom_mix
			);

			private->sw_cfg->custom_mix = cpu_to_le32(new_custom_mix);
			err = scarlett2_commit_software_config(mixer, &private->sw_cfg->custom_mix, sizeof(u32));
			if (err < 0)
				return err;
		}
	}

	/* Notify client software about changes */
	if (err >= 0)
		snd_ctl_notify(card,
			SNDRV_CTL_EVENT_MASK_VALUE | SNDRV_CTL_EVENT_MASK_INFO,
			&private->mux_ctls[output_num]->id);
	
	return (err < 0) ? err : 1;
}

/* Update state of mutes
 *   mute  - the start number of the mute
 *   val   - value to apply to mutes
 *   count - number of mutes to update
 * return: 
 *   number of updated mutes or negative value on error
 * The software configuration request should be issued by the caller
 */
static int scarlett2_mute_ctl_commit(struct usb_mixer_interface *mixer, int mute, int val, int count) {
	struct scarlett2_data *private = mixer->private_data;
	struct snd_card *card = mixer->chip->card;
	
	u32 sw_mutes = 0, sw_mask = 0;
	int i, index, err = 0, updated = 0;
	int num_line_out = private->info->port_count[SCARLETT2_PORT_TYPE_ANALOGUE][SCARLETT2_PORT_OUT];

	usb_audio_info(mixer->chip, ">>>scarlett2_mute_ctl_commit mute=%d, val=%d, count=%d", mute, val, count);
	
	/* Read software mute settings */
	if (private->sw_cfg != NULL)
		sw_mutes = le32_to_cpu(private->sw_cfg->mute_sw);
	
	for (i=0; i<count; ++i) {
		/* Ensure that output needs remapping */
		index = mute + i;
		if (index < num_line_out)
			index = line_out_remap(private, index);
	
		/* Ensure that mute settings have been updated and update it's state */
		if (val == private->mute_switch[index])
			continue;
		private->mute_switch[index] = val;

		/* Commit the mute state to device */
		if (index < num_line_out) {
			usb_audio_info(mixer->chip, "set HW mute[%d] to %d\n", index, val);
			/* Send mute change to the device */
			err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_MUTE_SWITCH, index, val);
			if (err < 0)
				return err;
		}
		else if (private->sw_cfg) {
			/* Set mute bit for the corresponding output in software configuration */
			sw_mask |= 1 << index;
			usb_audio_info(mixer->chip, "set SW mute[%d] to %d -> %x (mask:%x)\n", index, val, (int)sw_mutes, (int)sw_mask);
		}
		else {
			usb_audio_info(mixer->chip, "invalid mute index %d", index);
			return -EINVAL;
		}
		
		/* Notify clients about change and increment number of updated mutes */
		snd_ctl_notify(card,
			SNDRV_CTL_EVENT_MASK_VALUE | SNDRV_CTL_EVENT_MASK_INFO,
			&private->mute_ctls[mute + i]->id);
		++updated;
	}
	
	/* Commit software configuration updates if there are changes */
	if ((private->sw_cfg) && (sw_mask)) {
		sw_mutes = (val) ? (sw_mutes | sw_mask) : (sw_mutes & (~sw_mask));
		private->sw_cfg->mute_sw = cpu_to_le32(sw_mutes);
		err = scarlett2_commit_software_config(mixer, &private->sw_cfg->mute_sw, sizeof(__le32));
	}

	return (err < 0) ? err : updated;
}

/* Send USB message to get meter levels */
static int scarlett2_usb_get_meter_levels(struct usb_mixer_interface *mixer,
					  u16 num_meters, u16 *levels)
{
	struct {
		__le16 pad;
		__le16 num_meters;
		__le32 magic;
	} __packed req;
	u32 resp[SCARLETT2_MAX_METERS];
	int i, err;

	req.pad = 0;
	req.num_meters = cpu_to_le16(num_meters);
	req.magic = cpu_to_le32(SCARLETT2_USB_METER_LEVELS_GET_MAGIC);
	err = scarlett2_usb(mixer, SCARLETT2_USB_GET_METER,
			    &req, sizeof(req), resp, num_meters * sizeof(u32));
	if (err < 0)
		return err;

	/* copy, convert to u16 */
	for (i = 0; i < num_meters; i++)
		levels[i] = resp[i];

	return 0;
}

/*** Control Functions ***/

/* helper function to create a new control */
static int scarlett2_add_new_ctl(struct usb_mixer_interface *mixer,
				 const struct snd_kcontrol_new *ncontrol,
				 int index, int channels, const char *name,
				 struct snd_kcontrol **kctl_return)
{
	struct snd_kcontrol *kctl;
	struct usb_mixer_elem_info *elem;
	int err;

	elem = kzalloc(sizeof(*elem), GFP_KERNEL);
	if (!elem)
		return -ENOMEM;

	/* We set USB_MIXER_BESPOKEN type, so that the core USB mixer code
	 * ignores them for resume and other operations.
	 * Also, the head.id field is set to 0, as we don't use this field.
	 */
	elem->head.mixer = mixer;
	elem->control = index;
	elem->head.id = 0;
	elem->channels = channels;
	elem->val_type = USB_MIXER_BESPOKEN;

	kctl = snd_ctl_new1(ncontrol, elem);
	if (!kctl) {
		kfree(elem);
		return -ENOMEM;
	}
	kctl->private_free = snd_usb_mixer_elem_free;

	strscpy(kctl->id.name, name, sizeof(kctl->id.name));

	err = snd_usb_mixer_add_control(&elem->head, kctl);
	if (err < 0)
		return err;

	if (kctl_return)
		*kctl_return = kctl;

	return 0;
}

/*** Sync Control ***/

/* Update sync control after receiving notification that the status
 * has changed
 */
static int scarlett2_update_sync(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;

	private->sync_updated = 0;
	return scarlett2_usb_get_sync_status(mixer, &private->sync);
}

static int scarlett2_sync_ctl_info(struct snd_kcontrol *kctl,
				   struct snd_ctl_elem_info *uinfo)
{
	static const char *texts[2] = {
		"Unlocked", "Locked"
	};
	return snd_ctl_enum_info(uinfo, 1, 2, texts);
}

static int scarlett2_sync_ctl_get(struct snd_kcontrol *kctl,
				  struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	mutex_lock(&private->data_mutex);
	if (private->sync_updated)
		scarlett2_update_sync(mixer);
	ucontrol->value.enumerated.item[0] = private->sync;
	mutex_unlock(&private->data_mutex);

	return 0;
}

static const struct snd_kcontrol_new scarlett2_sync_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READ,
	.name = "",
	.info = scarlett2_sync_ctl_info,
	.get  = scarlett2_sync_ctl_get
};

static int scarlett2_add_sync_ctl(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;

	/* devices without a mixer also don't support reporting sync status */
	if (!private->info->has_mixer)
		return 0;

	return scarlett2_add_new_ctl(mixer, &scarlett2_sync_ctl,
				     0, 1, "Sync Status", &private->sync_ctl);
}

/*** Analogue Line Out Volume Controls ***/

/* Update hardware volume controls after receiving notification that
 * they have changed
 */
static int scarlett2_update_volumes(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const int (*port_count)[SCARLETT2_PORT_DIRNS] = info->port_count;
	struct scarlett2_usb_volume_status volume_status;
	int num_line_out =
		port_count[SCARLETT2_PORT_TYPE_ANALOGUE][SCARLETT2_PORT_OUT];
	int err, i;
	int mute;
	
	/* Check re-entrance */
	if (!private->vol_updated)
		return 0;
	
	usb_audio_info(mixer->chip, ">>>scarlett2_update_volumes\n");

	err = scarlett2_usb_get_volume_status(mixer, &volume_status);
	if (err < 0)
		return err;

	private->master_vol = clamp(
		volume_status.master_vol + SCARLETT2_VOLUME_BIAS,
		0, SCARLETT2_VOLUME_BIAS);

	if (info->line_out_hw_vol)
		for (i = 0; i < SCARLETT2_DIM_MUTE_COUNT; i++)
			private->dim_mute[i] = !!volume_status.dim_mute[i];

	mute = private->dim_mute[SCARLETT2_BUTTON_MUTE];

	for (i = 0; i < num_line_out; i++)
	{
		if (private->vol_sw_hw_switch[i]) {
			private->vol[i] = private->master_vol;
			private->mute_switch[i] = mute;
		}
		else
			private->mute_switch[i] = !! volume_status.mute_switch[i];
		
		usb_audio_info(mixer->chip, "  hw mute[%d] = %d\n", i, private->mute_switch[i]);
	}
	
	/* Reset flag AFTER the data has been received */
	private->vol_updated = 0;
	
	usb_audio_info(mixer->chip, "<<<scarlett2_update_volumes\n");

	return 0;
}

static int scarlett2_volume_ctl_info(struct snd_kcontrol *kctl,
				     struct snd_ctl_elem_info *uinfo)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = elem->channels;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = SCARLETT2_VOLUME_BIAS;
	uinfo->value.integer.step = 1;
	return 0;
}

static int scarlett2_master_volume_ctl_get(struct snd_kcontrol *kctl,
					   struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	mutex_lock(&private->data_mutex);
	scarlett2_update_volumes(mixer);
	mutex_unlock(&private->data_mutex);

	ucontrol->value.integer.value[0] = private->master_vol;
	return 0;
}

static int scarlett2_volume_ctl_get(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	int index = line_out_remap(private, elem->control);

	mutex_lock(&private->data_mutex);
	scarlett2_update_volumes(mixer);
	mutex_unlock(&private->data_mutex);

	ucontrol->value.integer.value[0] = private->vol[index];
	return 0;
}

static int scarlett2_volume_ctl_put(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	int index = line_out_remap(private, elem->control);
	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->vol[index];
	val = ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->vol[index] = val;
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_LINE_OUT_VOLUME,
				       index, val - SCARLETT2_VOLUME_BIAS);
	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const DECLARE_TLV_DB_MINMAX(
	db_scale_scarlett2_gain, -SCARLETT2_VOLUME_BIAS * 100, 0
);

static const struct snd_kcontrol_new scarlett2_master_volume_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READ |
		  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
	.name = "",
	.info = scarlett2_volume_ctl_info,
	.get  = scarlett2_master_volume_ctl_get,
	.private_value = 0, /* max value */
	.tlv = { .p = db_scale_scarlett2_gain }
};

static const struct snd_kcontrol_new scarlett2_line_out_volume_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
		  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
	.name = "",
	.info = scarlett2_volume_ctl_info,
	.get  = scarlett2_volume_ctl_get,
	.put  = scarlett2_volume_ctl_put,
	.private_value = 0, /* max value */
	.tlv = { .p = db_scale_scarlett2_gain }
};

/*** Mute Switch Controls ***/

static int scarlett2_mute_ctl_get(struct snd_kcontrol *kctl,
					struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	int index = elem->control;
	int num_line_out =
		private->info->port_count[SCARLETT2_PORT_TYPE_ANALOGUE][SCARLETT2_PORT_OUT];
	
	usb_audio_info(mixer->chip, ">>>scarlett2_mute_ctl_get idx=%d\n", index);

	mutex_lock(&private->data_mutex);
	
	scarlett2_update_volumes(mixer);
	if (index < num_line_out)
		index = line_out_remap(private, index);
	ucontrol->value.integer.value[0] = ! private->mute_switch[index];

	usb_audio_info(mixer->chip, 
				"<<<scarlett2_mute_ctl_get mute[%d]=%d return=%d\n", 
				index,
				private->mute_switch[index],
				! private->mute_switch[index]
  				);
	
	mutex_unlock(&private->data_mutex);

	return 0;
}

static int scarlett2_mute_ctl_put(struct snd_kcontrol *kctl,
					struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	int err, val, index, stereo_group;

	mutex_lock(&private->data_mutex);
	scarlett2_update_volumes(mixer);

	/* Update single mute or pair of mutes depending on stereo settings */
	index = elem->control;
	stereo_group = index >> 1;
	val = !ucontrol->value.integer.value[0];
	
	usb_audio_info(mixer->chip, ">>>scarlett2_mute_ctl_put index=%d, stereo_group=%d, val=%d\n", index, stereo_group, val);
	
	err = (private->stereo_switch[stereo_group]) ?
		scarlett2_mute_ctl_commit(mixer, index & (~1), val, 2) :
		scarlett2_mute_ctl_commit(mixer, index, val, 1);
	
	if (err > 0)
		err = 1;
	
	mutex_unlock(&private->data_mutex);
	usb_audio_info(mixer->chip, "<<<scarlett2_mute_ctl_put\n");
	return err;
}

static const struct snd_kcontrol_new scarlett2_mute_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_mute_ctl_get,
	.put  = scarlett2_mute_ctl_put,
};

/*** HW/SW Volume Switch Controls ***/

static void scarlett2_sw_hw_ctl_ro(struct scarlett2_data *private, int index)
{
	private->sw_hw_ctls[index]->vd[0].access &=
		~SNDRV_CTL_ELEM_ACCESS_WRITE;
}

static void scarlett2_sw_hw_ctl_rw(struct scarlett2_data *private, int index)
{
	private->sw_hw_ctls[index]->vd[0].access |=
		SNDRV_CTL_ELEM_ACCESS_WRITE;
}

static int scarlett2_sw_hw_enum_ctl_info(struct snd_kcontrol *kctl,
					 struct snd_ctl_elem_info *uinfo)
{
	static const char *const values[2] = {
		"SW", "HW"
	};

	return snd_ctl_enum_info(uinfo, 1, 2, values);
}

static int scarlett2_sw_hw_enum_ctl_get(struct snd_kcontrol *kctl,
					struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_data *private = elem->head.mixer->private_data;
	int index = line_out_remap(private, elem->control);

	ucontrol->value.enumerated.item[0] = private->vol_sw_hw_switch[index];
	return 0;
}

static void scarlett2_vol_ctl_set_writable(struct usb_mixer_interface *mixer,
					   int index, int value)
{
	struct scarlett2_data *private = mixer->private_data;
	struct snd_card *card = mixer->chip->card;

	/* Set/Clear write bits */
	if (value) {
		private->vol_ctls[index]->vd[0].access |=
			SNDRV_CTL_ELEM_ACCESS_WRITE;
		private->mute_ctls[index]->vd[0].access |=
			SNDRV_CTL_ELEM_ACCESS_WRITE;
	} else {
		private->vol_ctls[index]->vd[0].access &=
			~SNDRV_CTL_ELEM_ACCESS_WRITE;
		private->mute_ctls[index]->vd[0].access &=
			~SNDRV_CTL_ELEM_ACCESS_WRITE;
	}

	/* Notify of write bit and possible value change */
	snd_ctl_notify(card,
		       SNDRV_CTL_EVENT_MASK_VALUE | SNDRV_CTL_EVENT_MASK_INFO,
		       &private->vol_ctls[index]->id);
	snd_ctl_notify(card,
		       SNDRV_CTL_EVENT_MASK_VALUE | SNDRV_CTL_EVENT_MASK_INFO,
		       &private->mute_ctls[index]->id);
}

static int scarlett2_sw_hw_change(struct usb_mixer_interface *mixer,
				  int ctl_index, int val)
{
	struct scarlett2_data *private = mixer->private_data;
	int index = line_out_remap(private, ctl_index);
	int err;

	private->vol_sw_hw_switch[index] = val;

	/* Change access mode to RO (hardware controlled volume)
	 * or RW (software controlled volume)
	 */
	scarlett2_vol_ctl_set_writable(mixer, ctl_index, !val);

	/* Reset volume/mute to master volume/mute */
	private->vol[index] = private->master_vol;
	private->mute_switch[index] = private->dim_mute[SCARLETT2_BUTTON_MUTE];

	/* Set SW volume to current HW volume */
	err = scarlett2_usb_set_config(
		mixer, SCARLETT2_CONFIG_LINE_OUT_VOLUME,
		index, private->master_vol - SCARLETT2_VOLUME_BIAS);
	if (err < 0)
		return err;

	/* Set SW mute to current HW mute */
	err = scarlett2_usb_set_config(
		mixer, SCARLETT2_CONFIG_MUTE_SWITCH,
		index, private->dim_mute[SCARLETT2_BUTTON_MUTE]);
	if (err < 0)
		return err;

	/* Send SW/HW switch change to the device */
	return scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_SW_HW_SWITCH,
					index, val);
}

static int scarlett2_sw_hw_enum_ctl_put(struct snd_kcontrol *kctl,
					struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	int ctl_index = elem->control;
	int index = line_out_remap(private, ctl_index);
	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->vol_sw_hw_switch[index];
	val = !!ucontrol->value.enumerated.item[0];

	if (oval == val)
		goto unlock;

	err = scarlett2_sw_hw_change(mixer, ctl_index, val);
	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_sw_hw_enum_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = scarlett2_sw_hw_enum_ctl_info,
	.get  = scarlett2_sw_hw_enum_ctl_get,
	.put  = scarlett2_sw_hw_enum_ctl_put,
};

/*** Line Level/Instrument Level Switch Controls ***/

static int scarlett2_update_input_other(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;

	private->input_other_updated = 0;

	if (info->level_input_count) {
		int err = scarlett2_usb_get_config(
			mixer, SCARLETT2_CONFIG_LEVEL_SWITCH,
			info->level_input_count + info->level_input_first,
			private->level_switch);
		if (err < 0)
			return err;
	}

	if (info->pad_input_count) {
		int err = scarlett2_usb_get_config(
			mixer, SCARLETT2_CONFIG_PAD_SWITCH,
			info->pad_input_count, private->pad_switch);
		if (err < 0)
			return err;
	}

	if (info->air_input_count) {
		int err = scarlett2_usb_get_config(
			mixer, SCARLETT2_CONFIG_AIR_SWITCH,
			info->air_input_count, private->air_switch);
		if (err < 0)
			return err;
	}

	if (info->phantom_count) {
		int err = scarlett2_usb_get_config(
			mixer, SCARLETT2_CONFIG_PHANTOM_SWITCH,
			info->phantom_count, private->phantom_switch);
		if (err < 0)
			return err;

		err = scarlett2_usb_get_config(
			mixer, SCARLETT2_CONFIG_PHANTOM_PERSISTENCE,
			1, &private->phantom_persistence);
		if (err < 0)
			return err;
	}

	return 0;
}

static int scarlett2_level_enum_ctl_info(struct snd_kcontrol *kctl,
					 struct snd_ctl_elem_info *uinfo)
{
	static const char *const values[2] = {
		"Line", "Inst"
	};

	return snd_ctl_enum_info(uinfo, 1, 2, values);
}

static int scarlett2_level_enum_ctl_get(struct snd_kcontrol *kctl,
					struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;

	int index = elem->control + info->level_input_first;

	mutex_lock(&private->data_mutex);
	if (private->input_other_updated)
		scarlett2_update_input_other(mixer);
	ucontrol->value.enumerated.item[0] = private->level_switch[index];
	mutex_unlock(&private->data_mutex);

	return 0;
}

static int scarlett2_level_enum_ctl_put(struct snd_kcontrol *kctl,
					struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;

	int index = elem->control + info->level_input_first;
	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->level_switch[index];
	val = !!ucontrol->value.enumerated.item[0];

	if (oval == val)
		goto unlock;

	private->level_switch[index] = val;

	/* Send switch change to the device */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_LEVEL_SWITCH,
				       index, val);
	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_level_enum_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = scarlett2_level_enum_ctl_info,
	.get  = scarlett2_level_enum_ctl_get,
	.put  = scarlett2_level_enum_ctl_put,
};

/*** Pad Switch Controls ***/

static int scarlett2_pad_ctl_get(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	mutex_lock(&private->data_mutex);
	if (private->input_other_updated)
		scarlett2_update_input_other(mixer);
	ucontrol->value.integer.value[0] =
		private->pad_switch[elem->control];
	mutex_unlock(&private->data_mutex);

	return 0;
}

static int scarlett2_pad_ctl_put(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	int index = elem->control;
	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->pad_switch[index];
	val = !!ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->pad_switch[index] = val;

	/* Send switch change to the device */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_PAD_SWITCH,
				       index, val);
	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_pad_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_pad_ctl_get,
	.put  = scarlett2_pad_ctl_put,
};

/*** Air Switch Controls ***/

static int scarlett2_air_ctl_get(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	mutex_lock(&private->data_mutex);
	if (private->input_other_updated)
		scarlett2_update_input_other(mixer);
	ucontrol->value.integer.value[0] = private->air_switch[elem->control];
	mutex_unlock(&private->data_mutex);

	return 0;
}

static int scarlett2_air_ctl_put(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	int index = elem->control;
	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->air_switch[index];
	val = !!ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->air_switch[index] = val;

	/* Send switch change to the device */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_AIR_SWITCH,
				       index, val);
	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_air_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_air_ctl_get,
	.put  = scarlett2_air_ctl_put,
};

/*** Phantom Switch Controls ***/

static int scarlett2_phantom_ctl_get(struct snd_kcontrol *kctl,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	mutex_lock(&private->data_mutex);
	if (private->input_other_updated)
		scarlett2_update_input_other(mixer);
	ucontrol->value.integer.value[0] =
		private->phantom_switch[elem->control];
	mutex_unlock(&private->data_mutex);

	return 0;
}

static int scarlett2_phantom_ctl_put(struct snd_kcontrol *kctl,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	int index = elem->control;
	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->phantom_switch[index];
	val = !!ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->phantom_switch[index] = val;

	/* Send switch change to the device */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_PHANTOM_SWITCH,
				       index, val);
	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_phantom_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_phantom_ctl_get,
	.put  = scarlett2_phantom_ctl_put,
};

/*** Output Stereo Mode Control ***/

static int scarlett2_stereo_mode_ctl_get(struct snd_kcontrol *kctl,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	
	int index = elem->control;
	mutex_lock(&private->data_mutex);
	ucontrol->value.integer.value[0] = private->stereo_switch[index];
	mutex_unlock(&private->data_mutex);

	return 0;
}

static int scarlett2_stereo_mode_ctl_put(struct snd_kcontrol *kctl,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	int line_out_count = private->info->port_count[SCARLETT2_PORT_TYPE_ANALOGUE][SCARLETT2_PORT_OUT];
	int index = elem->control;
	int i, oval, val, err = 0;
	int output_number, input_number[2], new_input_number[2];
	int port_type, port_num, new_port_num[2];
	u32 stereo_mask;
	
	mutex_lock(&private->data_mutex);

	/* Check that value has changed and update it */
	oval = private->stereo_switch[index];
	val = !!ucontrol->value.integer.value[0];
	if (oval == val)
		goto unlock;
	private->stereo_switch[index] = val;
	
	/* Check that the software configuration area is present */
	if (private->sw_cfg == NULL)
	{
		err = -EINVAL;
		goto unlock;
	}
	
	/* For migrating from mono to stereo we need to to couple of changes:
	 * 1. Set the routing of two output ports that are grouped into the stereo pair.
	 * 2. The input port should match to output ports: Odd inputs should match odd outputs and vice verse
	 */
	output_number = index << 1;
	if (output_number < line_out_count)
		output_number = line_out_remap(private, output_number);
	stereo_mask = le32_to_cpu(private->sw_cfg->stereo_sw);

	usb_audio_info(mixer->chip, ">>>scarlett2_stereo_mode_ctl_put oval=%d, val=%d, output_number=%d, mask=0x%x",
				   oval, val, output_number, (int)stereo_mask);	

	if (val) {
		stereo_mask |= ((u32)3) << (index << 1);
		usb_audio_info(mixer->chip, "  updated stereo_mask=0x%x", (int)stereo_mask);
		
		/* Check the routing settings and update routing if they have been changed */
		input_number[0] = private->mux[output_number + 0];
		input_number[1] = private->mux[output_number + 1];
		
		usb_audio_info(mixer->chip, "  input_number={%d, %d}", input_number[0], input_number[1]);
		
		if (scarlett2_mux_enum_to_drv_port_number(&port_type, &port_num, private->info, SCARLETT2_PORT_IN, input_number[0])) {
			new_port_num[0] = port_num & (~1);
			new_port_num[1] = (port_type != SCARLETT2_PORT_TYPE_NONE) ? new_port_num[0] + 1 : new_port_num[0];
			
			new_input_number[0] = scarlett2_drv_port_number_to_mux_enum(SCARLETT2_PORT_IN, port_type, new_port_num[0], private->info);
			new_input_number[1] = scarlett2_drv_port_number_to_mux_enum(SCARLETT2_PORT_IN, port_type, new_port_num[1], private->info);
			
			usb_audio_info(mixer->chip, "  new_input_number={%d, %d}", new_input_number[0], new_input_number[1]);
			
			/* The routing settins have changed, we need to update the routing and notify the client software */
			if ((new_input_number[0] != input_number[0]) ||
				(new_input_number[1] != input_number[1])) {
				/* Update the device's internal mux state for 2 outputs */
				for (i=0; i<2; ++i) {
					err = scarlett2_mux_commit_route(mixer, output_number + i, new_input_number[i]);
					if (err < 0)
						goto unlock;
				}

				err = scarlett2_usb_set_mux(mixer);
				if (err < 0)
					goto unlock;
			}
		}
	}
	else {
		/* For stereo -> mono switch there are no exra things to do except resetting the stereo mask */
		stereo_mask &= ~(((u32)3) << output_number);
	}
	
	/* Update mute state for the second channel in the pair */
	val = private->mute_switch[output_number];
	err = scarlett2_mute_ctl_commit(mixer, ((index << 1) | 1), val, 1);
	if (err < 0)
		goto unlock;
	
	/* Update stereo mask */
	private->sw_cfg->stereo_sw = cpu_to_le32(stereo_mask);
	usb_audio_info(mixer->chip, "new stereo mask: 0x%x", (int)private->sw_cfg->stereo_sw);
	err = scarlett2_commit_software_config(mixer, &private->sw_cfg->stereo_sw, sizeof(__le32));
	if (err < 0)
		goto unlock;

	if (err >= 0)
		err = 1;

unlock:
	usb_audio_info(mixer->chip, "<<<scarlett2_stereo_mode_ctl_put");
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_stereo_mode_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_stereo_mode_ctl_get,
	.put  = scarlett2_stereo_mode_ctl_put,
};


/*** Phantom Persistence Control ***/

static int scarlett2_phantom_persistence_ctl_get(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_data *private = elem->head.mixer->private_data;

	ucontrol->value.integer.value[0] = private->phantom_persistence;
	return 0;
}

static int scarlett2_phantom_persistence_ctl_put(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	int index = elem->control;
	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->phantom_persistence;
	val = !!ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->phantom_persistence = val;

	/* Send switch change to the device */
	err = scarlett2_usb_set_config(
		mixer, SCARLETT2_CONFIG_PHANTOM_PERSISTENCE, index, val);
	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_phantom_persistence_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_phantom_persistence_ctl_get,
	.put  = scarlett2_phantom_persistence_ctl_put,
};

/*** Direct Monitor Control ***/

static int scarlett2_update_monitor_other(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	int err;

	/* monitor_other_enable[0] enables speaker switching
	 * monitor_other_enable[1] enables talkback
	 */
	u8 monitor_other_enable[2];

	/* monitor_other_switch[0] activates the alternate speakers
	 * monitor_other_switch[1] activates talkback
	 */
	u8 monitor_other_switch[2];

	private->monitor_other_updated = 0;

	if (info->direct_monitor)
		return scarlett2_usb_get_config(
			mixer, SCARLETT2_CONFIG_DIRECT_MONITOR,
			1, &private->direct_monitor_switch);

	/* if it doesn't do speaker switching then it also doesn't do
	 * talkback
	 */
	if (!info->has_speaker_switching)
		return 0;

	err = scarlett2_usb_get_config(
		mixer, SCARLETT2_CONFIG_MONITOR_OTHER_ENABLE,
		2, monitor_other_enable);
	if (err < 0)
		return err;

	err = scarlett2_usb_get_config(
		mixer, SCARLETT2_CONFIG_MONITOR_OTHER_SWITCH,
		2, monitor_other_switch);
	if (err < 0)
		return err;

	if (!monitor_other_enable[0])
		private->speaker_switching_switch = 0;
	else
		private->speaker_switching_switch = monitor_other_switch[0] + 1;

	if (info->has_talkback) {
		const int (*port_count)[SCARLETT2_PORT_DIRNS] =
			info->port_count;
		int num_mixes =
			port_count[SCARLETT2_PORT_TYPE_MIX][SCARLETT2_PORT_IN];
		u16 bitmap;
		int i;

		if (!monitor_other_enable[1])
			private->talkback_switch = 0;
		else
			private->talkback_switch = monitor_other_switch[1] + 1;

		err = scarlett2_usb_get_config(mixer,
					       SCARLETT2_CONFIG_TALKBACK_MAP,
					       1, &bitmap);
		if (err < 0)
			return err;
		for (i = 0; i < num_mixes; i++, bitmap >>= 1)
			private->talkback_map[i] = bitmap & 1;
	}

	return 0;
}

static int scarlett2_direct_monitor_ctl_get(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = elem->head.mixer->private_data;

	mutex_lock(&private->data_mutex);
	if (private->monitor_other_updated)
		scarlett2_update_monitor_other(mixer);
	ucontrol->value.enumerated.item[0] = private->direct_monitor_switch;
	mutex_unlock(&private->data_mutex);

	return 0;
}

static int scarlett2_direct_monitor_ctl_put(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	int index = elem->control;
	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->direct_monitor_switch;
	val = min(ucontrol->value.enumerated.item[0], 2U);

	if (oval == val)
		goto unlock;

	private->direct_monitor_switch = val;

	/* Send switch change to the device */
	err = scarlett2_usb_set_config(
		mixer, SCARLETT2_CONFIG_DIRECT_MONITOR, index, val);
	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static int scarlett2_direct_monitor_stereo_enum_ctl_info(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_info *uinfo)
{
	static const char *const values[3] = {
		"Off", "Mono", "Stereo"
	};

	return snd_ctl_enum_info(uinfo, 1, 3, values);
}

/* Direct Monitor for Solo is mono-only and only needs a boolean control
 * Direct Monitor for 2i2 is selectable between Off/Mono/Stereo
 */
static const struct snd_kcontrol_new scarlett2_direct_monitor_ctl[2] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "",
		.info = snd_ctl_boolean_mono_info,
		.get  = scarlett2_direct_monitor_ctl_get,
		.put  = scarlett2_direct_monitor_ctl_put,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "",
		.info = scarlett2_direct_monitor_stereo_enum_ctl_info,
		.get  = scarlett2_direct_monitor_ctl_get,
		.put  = scarlett2_direct_monitor_ctl_put,
	}
};

static int scarlett2_add_direct_monitor_ctl(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const char *s;

	if (!info->direct_monitor)
		return 0;

	s = info->direct_monitor == 1
	      ? "Direct Monitor Playback Switch"
	      : "Direct Monitor Playback Enum";

	return scarlett2_add_new_ctl(
		mixer, &scarlett2_direct_monitor_ctl[info->direct_monitor - 1],
		0, 1, s, &private->direct_monitor_ctl);
}

/*** Speaker Switching Control ***/

static int scarlett2_speaker_switch_enum_ctl_info(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_info *uinfo)
{
	static const char *const values[3] = {
		"Off", "Main", "Alt"
	};

	return snd_ctl_enum_info(uinfo, 1, 3, values);
}

static int scarlett2_speaker_switch_enum_ctl_get(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	mutex_lock(&private->data_mutex);
	if (private->monitor_other_updated)
		scarlett2_update_monitor_other(mixer);
	ucontrol->value.enumerated.item[0] = private->speaker_switching_switch;
	mutex_unlock(&private->data_mutex);

	return 0;
}

/* when speaker switching gets enabled, switch the main/alt speakers
 * to HW volume and disable those controls
 */
static int scarlett2_speaker_switch_enable(struct usb_mixer_interface *mixer)
{
	struct snd_card *card = mixer->chip->card;
	struct scarlett2_data *private = mixer->private_data;
	int i, err;

	for (i = 0; i < 4; i++) {
		int index = line_out_remap(private, i);

		/* switch the main/alt speakers to HW volume */
		if (!private->vol_sw_hw_switch[index]) {
			err = scarlett2_sw_hw_change(private->mixer, i, 1);
			if (err < 0)
				return err;
		}

		/* disable the line out SW/HW switch */
		scarlett2_sw_hw_ctl_ro(private, i);
		snd_ctl_notify(card,
			       SNDRV_CTL_EVENT_MASK_VALUE |
				 SNDRV_CTL_EVENT_MASK_INFO,
			       &private->sw_hw_ctls[i]->id);
	}

	/* when the next monitor-other notify comes in, update the mux
	 * configuration
	 */
	private->speaker_switching_switched = 1;

	return 0;
}

/* when speaker switching gets disabled, reenable the hw/sw controls
 * and invalidate the routing
 */
static void scarlett2_speaker_switch_disable(struct usb_mixer_interface *mixer)
{
	struct snd_card *card = mixer->chip->card;
	struct scarlett2_data *private = mixer->private_data;
	int i;

	/* enable the line out SW/HW switch */
	for (i = 0; i < 4; i++) {
		scarlett2_sw_hw_ctl_rw(private, i);
		snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_INFO,
			       &private->sw_hw_ctls[i]->id);
	}

	/* when the next monitor-other notify comes in, update the mux
	 * configuration
	 */
	private->speaker_switching_switched = 1;
}

static int scarlett2_speaker_switch_enum_ctl_put(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->speaker_switching_switch;
	val = min(ucontrol->value.enumerated.item[0], 2U);

	if (oval == val)
		goto unlock;

	private->speaker_switching_switch = val;

	/* enable/disable speaker switching */
	err = scarlett2_usb_set_config(
		mixer, SCARLETT2_CONFIG_MONITOR_OTHER_ENABLE,
		0, !!val);
	if (err < 0)
		goto unlock;

	/* if speaker switching is enabled, select main or alt */
	err = scarlett2_usb_set_config(
		mixer, SCARLETT2_CONFIG_MONITOR_OTHER_SWITCH,
		0, val == 2);
	if (err < 0)
		goto unlock;

	/* update controls if speaker switching gets enabled or disabled */
	if (!oval && val)
		err = scarlett2_speaker_switch_enable(mixer);
	else if (oval && !val)
		scarlett2_speaker_switch_disable(mixer);

	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_speaker_switch_enum_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = scarlett2_speaker_switch_enum_ctl_info,
	.get  = scarlett2_speaker_switch_enum_ctl_get,
	.put  = scarlett2_speaker_switch_enum_ctl_put,
};

static int scarlett2_add_speaker_switch_ctl(
	struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;

	if (!info->has_speaker_switching)
		return 0;

	return scarlett2_add_new_ctl(
		mixer, &scarlett2_speaker_switch_enum_ctl,
		0, 1, "Speaker Switching Playback Enum",
		&private->speaker_switching_ctl);
}

/*** Talkback and Talkback Map Controls ***/

static int scarlett2_talkback_enum_ctl_info(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_info *uinfo)
{
	static const char *const values[3] = {
		"Disabled", "Off", "On"
	};

	return snd_ctl_enum_info(uinfo, 1, 3, values);
}

static int scarlett2_talkback_enum_ctl_get(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	mutex_lock(&private->data_mutex);
	if (private->monitor_other_updated)
		scarlett2_update_monitor_other(mixer);
	ucontrol->value.enumerated.item[0] = private->talkback_switch;
	mutex_unlock(&private->data_mutex);

	return 0;
}

static int scarlett2_talkback_enum_ctl_put(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->talkback_switch;
	val = min(ucontrol->value.enumerated.item[0], 2U);

	if (oval == val)
		goto unlock;

	private->talkback_switch = val;

	/* enable/disable talkback */
	err = scarlett2_usb_set_config(
		mixer, SCARLETT2_CONFIG_MONITOR_OTHER_ENABLE,
		1, !!val);
	if (err < 0)
		goto unlock;

	/* if talkback is enabled, select main or alt */
	err = scarlett2_usb_set_config(
		mixer, SCARLETT2_CONFIG_MONITOR_OTHER_SWITCH,
		1, val == 2);
	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_talkback_enum_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = scarlett2_talkback_enum_ctl_info,
	.get  = scarlett2_talkback_enum_ctl_get,
	.put  = scarlett2_talkback_enum_ctl_put,
};

static int scarlett2_talkback_map_ctl_get(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	int index = elem->control;

	ucontrol->value.integer.value[0] = private->talkback_map[index];

	return 0;
}

static int scarlett2_talkback_map_ctl_put(
	struct snd_kcontrol *kctl, struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	const int (*port_count)[SCARLETT2_PORT_DIRNS] =
		private->info->port_count;
	int num_mixes = port_count[SCARLETT2_PORT_TYPE_MIX][SCARLETT2_PORT_IN];

	int index = elem->control;
	int oval, val, err = 0, i;
	u16 bitmap = 0;

	mutex_lock(&private->data_mutex);

	oval = private->talkback_map[index];
	val = !!ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->talkback_map[index] = val;

	for (i = 0; i < num_mixes; i++)
		bitmap |= private->talkback_map[i] << i;

	/* Send updated bitmap to the device */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_TALKBACK_MAP,
				       0, bitmap);
	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_talkback_map_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_talkback_map_ctl_get,
	.put  = scarlett2_talkback_map_ctl_put,
};

static int scarlett2_add_talkback_ctls(
	struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const int (*port_count)[SCARLETT2_PORT_DIRNS] = info->port_count;
	int num_mixes = port_count[SCARLETT2_PORT_TYPE_MIX][SCARLETT2_PORT_IN];
	int err, i;
	char s[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];

	if (!info->has_talkback)
		return 0;

	err = scarlett2_add_new_ctl(
		mixer, &scarlett2_talkback_enum_ctl,
		0, 1, "Talkback Playback Enum",
		&private->talkback_ctl);
	if (err < 0)
		return err;

	for (i = 0; i < num_mixes; i++) {
		snprintf(s, sizeof(s),
			 "Talkback Mix %c Playback Switch", i + 'A');
		err = scarlett2_add_new_ctl(mixer, &scarlett2_talkback_map_ctl,
					    i, 1, s, NULL);
		if (err < 0)
			return err;
	}

	return 0;
}

/*** Dim/Mute Controls ***/

static int scarlett2_dim_mute_ctl_get(struct snd_kcontrol *kctl,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	mutex_lock(&private->data_mutex);
	if (private->vol_updated)
		scarlett2_update_volumes(mixer);
	mutex_unlock(&private->data_mutex);

	ucontrol->value.integer.value[0] = private->dim_mute[elem->control];
	return 0;
}

static int scarlett2_dim_mute_ctl_put(struct snd_kcontrol *kctl,
				      struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const int (*port_count)[SCARLETT2_PORT_DIRNS] = info->port_count;
	int num_line_out =
		port_count[SCARLETT2_PORT_TYPE_ANALOGUE][SCARLETT2_PORT_OUT];

	int index = elem->control;
	int oval, val, err = 0, i;

	mutex_lock(&private->data_mutex);

	oval = private->dim_mute[index];
	val = !!ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->dim_mute[index] = val;

	/* Send switch change to the device */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_DIM_MUTE,
				       index, val);
	if (err == 0)
		err = 1;

	if (index == SCARLETT2_BUTTON_MUTE)
		for (i = 0; i < num_line_out; i++) {
			int line_index = line_out_remap(private, i);

			if (private->vol_sw_hw_switch[line_index]) {
				private->mute_switch[line_index] = val;
				snd_ctl_notify(mixer->chip->card,
					       SNDRV_CTL_EVENT_MASK_VALUE,
					       &private->mute_ctls[i]->id);
			}
		}

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_dim_mute_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_dim_mute_ctl_get,
	.put  = scarlett2_dim_mute_ctl_put
};

/*** Create the analogue output controls ***/

static int scarlett2_add_line_out_ctls(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	int num_line_out =
		info->port_count[SCARLETT2_PORT_TYPE_ANALOGUE][SCARLETT2_PORT_OUT];
	const struct scarlett2_sw_port_mapping *port_mapping = info->sw_port_mapping;
	int err, i, index, hw_index, stereo_index, output_index;
	int stereo_switch;
	u32 sw_mutes, stereo_mask;
	char s[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];

	/* Add R/O HW volume control */
	if (info->line_out_hw_vol) {
		snprintf(s, sizeof(s), "Master HW Playback Volume");
		err = scarlett2_add_new_ctl(mixer,
					    &scarlett2_master_volume_ctl,
					    0, 1, s, &private->master_vol_ctl);
		if (err < 0)
			return err;
	}

	/* Add volume controls */
	for (i = 0; i < num_line_out; i++) {
		index = line_out_remap(private, i);

		/* Fader */
		if (info->line_out_descrs[i])
			snprintf(s, sizeof(s),
				 "Line %02d (%s) Playback Volume",
				 i + 1, info->line_out_descrs[i]);
		else
			snprintf(s, sizeof(s),
				 "Line %02d Playback Volume",
				 i + 1);
		err = scarlett2_add_new_ctl(mixer,
					    &scarlett2_line_out_volume_ctl,
					    i, 1, s, &private->vol_ctls[i]);
		if (err < 0)
			return err;

		/* Make the fader and mute controls read-only if the
		 * SW/HW switch is set to HW
		 */
		if (private->vol_sw_hw_switch[index])
			scarlett2_vol_ctl_set_writable(mixer, i, 0);

		/* SW/HW Switch */
		if (info->line_out_hw_vol) {
			snprintf(s, sizeof(s),
				 "Line Out %02d Volume Control Playback Enum",
				 i + 1);
			err = scarlett2_add_new_ctl(mixer,
						    &scarlett2_sw_hw_enum_ctl,
						    i, 1, s,
						    &private->sw_hw_ctls[i]);
			if (err < 0)
				return err;

			/* Make the switch read-only if the line is
			 * involved in speaker switching
			 */
			if (private->speaker_switching_switch && i < 4)
				scarlett2_sw_hw_ctl_ro(private, i);
		}
	}

	/* Add dim/mute controls */
	if (info->line_out_hw_vol)
		for (i = 0; i < SCARLETT2_DIM_MUTE_COUNT; i++) {
			err = scarlett2_add_new_ctl(
				mixer, &scarlett2_dim_mute_ctl,
				i, 1, scarlett2_dim_mute_names[i],
				&private->dim_mute_ctls[i]);
			if (err < 0)
				return err;
		}

	/* Create sofware-controllable Mute Switches for output */
	if (port_mapping) {
		/* Read state of mutes from software config */
		sw_mutes = (private->sw_cfg) ? le32_to_cpu(private->sw_cfg->mute_sw) : 0;
		stereo_mask = (private->sw_cfg) ? le32_to_cpu(private->sw_cfg->stereo_sw) : 0;
		
		usb_audio_info(mixer->chip, "  sw_mutes=0x%x port_mapping=%p\n", (int)sw_mutes, port_mapping);
		usb_audio_info(mixer->chip, "  stereo_mask=0x%x\n", (int)stereo_mask);
		
		for (; port_mapping->type != SCARLETT2_PORT_TYPE_COUNT; ++port_mapping) {
			/* We process only output ports */
			if (port_mapping->direction != SCARLETT2_PORT_OUT)
				continue;
			
			for (i = 0; i<port_mapping->count; ++i) {
				/* Compute the index of the port and stereo index */
				index = port_mapping->sw_offset + i;
				stereo_index = index >> 1;
				hw_index = port_mapping->hw_offset + i;
				stereo_switch = !! (stereo_mask & (((u32)3) << (stereo_index << 1)));
				
				/* Update the state of software mutes */
				if (port_mapping->type != SCARLETT2_PORT_TYPE_ANALOGUE) {
					/* Skip mutes if we don't have software configuration */
					if (!private->sw_cfg)
						continue;
					/* For stereo configuration enabled, take switch from the first channel in the pair */
					private->mute_switch[index] = (stereo_switch)
						? !! (sw_mutes & (1 << (stereo_index << 1)))
						: !! (sw_mutes & (1 << index));
				}
				
				/* Create Mute Switch */
				scarlett2_fmt_port_name(s, sizeof(s), SCARLETT2_PORT_OUT,
										port_mapping->type, port_mapping->hw_offset + i,
										"Mute Switch");
				err = scarlett2_add_new_ctl(mixer,
							&scarlett2_mute_ctl,
							index, 1, s,
							&private->mute_ctls[index]);
				if (err < 0)
					return err;
				
				usb_audio_info(mixer->chip, "  mute switch %2d: %s = %d\n", index, s, private->mute_switch[index]);
				
				/* Create Stereo Mode Switch for each second output */
				if (index & 1) {
					output_index = scarlett2_drv_port_number_to_mux_enum(
						port_mapping->direction, port_mapping->type,
						hw_index, private->info) >> 1;
					
					private->stereo_switch[output_index] = stereo_switch;
					
					scarlett2_fmt_stereo_output_port_name(s, sizeof(s),
										port_mapping->type, port_mapping->hw_offset + i - 1,
										"Stereo Switch");
					err = scarlett2_add_new_ctl(mixer,
							&scarlett2_stereo_mode_ctl,
							output_index, 1, s,
							&private->stereo_ctls[output_index]);
					if (err < 0)
						return err;
					
					usb_audio_info(mixer->chip, "  stereo switch %2d: %s = %d, bits=0x%x, output=%d\n",
								   stereo_index, s, private->stereo_switch[stereo_index],
								  (int)(((u32)3) << (stereo_index << 1)),
								  output_index*2
  								);
				}
			}
		}
	}

	return 0;
}

/*** Create the analogue input controls ***/

static int scarlett2_add_line_in_ctls(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	int err, i;
	char s[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];
	const char *fmt = "Line In %d %s Capture %s";
	const char *fmt2 = "Line In %d-%d %s Capture %s";

	/* Add input level (line/inst) controls */
	for (i = 0; i < info->level_input_count; i++) {
		snprintf(s, sizeof(s), fmt, i + 1 + info->level_input_first,
			 "Level", "Enum");
		err = scarlett2_add_new_ctl(mixer, &scarlett2_level_enum_ctl,
					    i, 1, s, &private->level_ctls[i]);
		if (err < 0)
			return err;
	}

	/* Add input pad controls */
	for (i = 0; i < info->pad_input_count; i++) {
		snprintf(s, sizeof(s), fmt, i + 1, "Pad", "Switch");
		err = scarlett2_add_new_ctl(mixer, &scarlett2_pad_ctl,
					    i, 1, s, &private->pad_ctls[i]);
		if (err < 0)
			return err;
	}

	/* Add input air controls */
	for (i = 0; i < info->air_input_count; i++) {
		snprintf(s, sizeof(s), fmt, i + 1, "Air", "Switch");
		err = scarlett2_add_new_ctl(mixer, &scarlett2_air_ctl,
					    i, 1, s, &private->air_ctls[i]);
		if (err < 0)
			return err;
	}

	/* Add input phantom controls */
	if (info->inputs_per_phantom == 1) {
		for (i = 0; i < info->phantom_count; i++) {
			snprintf(s, sizeof(s), fmt, i + 1,
				 "Phantom Power", "Switch");
			err = scarlett2_add_new_ctl(
				mixer, &scarlett2_phantom_ctl,
				i, 1, s, &private->phantom_ctls[i]);
			if (err < 0)
				return err;
		}
	} else if (info->inputs_per_phantom > 1) {
		for (i = 0; i < info->phantom_count; i++) {
			int from = i * info->inputs_per_phantom + 1;
			int to = (i + 1) * info->inputs_per_phantom;

			snprintf(s, sizeof(s), fmt2, from, to,
				 "Phantom Power", "Switch");
			err = scarlett2_add_new_ctl(
				mixer, &scarlett2_phantom_ctl,
				i, 1, s, &private->phantom_ctls[i]);
			if (err < 0)
				return err;
		}
	}
	if (info->phantom_count) {
		err = scarlett2_add_new_ctl(
			mixer, &scarlett2_phantom_persistence_ctl, 0, 1,
			"Phantom Power Persistence Capture Switch", NULL);
		if (err < 0)
			return err;
	}

	return 0;
}

/*** Mixer Volume Controls ***/

static int scarlett2_mixer_ctl_info(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_info *uinfo)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = elem->channels;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = SCARLETT2_MIXER_MAX_VALUE;
	uinfo->value.integer.step = 1;
	return 0;
}

static int scarlett2_mixer_ctl_get(struct snd_kcontrol *kctl,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_data *private = elem->head.mixer->private_data;

	ucontrol->value.integer.value[0] = private->mix[elem->control];
	return 0;
}

static int scarlett2_mixer_ctl_put(struct snd_kcontrol *kctl,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const int (*port_count)[SCARLETT2_PORT_DIRNS] = info->port_count;
	int oval, val, num_mixer_in, mix_num, err = 0;
	int index = elem->control;

	mutex_lock(&private->data_mutex);

	oval = private->mix[index];
	val = ucontrol->value.integer.value[0];
	num_mixer_in = port_count[SCARLETT2_PORT_TYPE_MIX][SCARLETT2_PORT_OUT];
	mix_num = index / num_mixer_in;

	if (oval == val)
		goto unlock;

	private->mix[index] = val;
	err = scarlett2_usb_set_mix(mixer, mix_num);
	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const DECLARE_TLV_DB_MINMAX(
	db_scale_scarlett2_mixer,
	SCARLETT2_MIXER_MIN_DB * 100,
	SCARLETT2_MIXER_MAX_DB * 100
);

static const struct snd_kcontrol_new scarlett2_mixer_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.access = SNDRV_CTL_ELEM_ACCESS_READWRITE |
		  SNDRV_CTL_ELEM_ACCESS_TLV_READ,
	.name = "",
	.info = scarlett2_mixer_ctl_info,
	.get  = scarlett2_mixer_ctl_get,
	.put  = scarlett2_mixer_ctl_put,
	.private_value = SCARLETT2_MIXER_MAX_DB, /* max value */
	.tlv = { .p = db_scale_scarlett2_mixer }
};

static int scarlett2_add_mixer_ctls(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const int (*port_count)[SCARLETT2_PORT_DIRNS] = info->port_count;
	int err, i, j;
	int index;
	char s[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];

	int num_inputs =
		port_count[SCARLETT2_PORT_TYPE_MIX][SCARLETT2_PORT_OUT];
	int num_outputs =
		port_count[SCARLETT2_PORT_TYPE_MIX][SCARLETT2_PORT_IN];

	for (i = 0, index = 0; i < num_outputs; i++)
		for (j = 0; j < num_inputs; j++, index++) {
			snprintf(s, sizeof(s),
				 "Mix %c Input %02d Playback Volume",
				 'A' + i, j + 1);
			err = scarlett2_add_new_ctl(mixer, &scarlett2_mixer_ctl,
						    index, 1, s, NULL);
			if (err < 0)
				return err;
		}

	return 0;
}

/*** Mux Source Selection Controls ***/

static int scarlett2_mux_src_enum_ctl_info(struct snd_kcontrol *kctl,
					   struct snd_ctl_elem_info *uinfo)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_data *private = elem->head.mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const int (*port_count)[SCARLETT2_PORT_DIRNS] = info->port_count;
	unsigned int item = uinfo->value.enumerated.item;
	int items = private->num_mux_srcs;
	int port_type;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = elem->channels;
	uinfo->value.enumerated.items = items;

	if (item >= items)
		item = uinfo->value.enumerated.item = items - 1;

	for (port_type = 0;
	     port_type < SCARLETT2_PORT_TYPE_COUNT;
	     port_type++) {
		if (item < port_count[port_type][SCARLETT2_PORT_IN]) {
			const struct scarlett2_port *port =
				&scarlett2_ports[port_type];

			sprintf(uinfo->value.enumerated.name,
				port->src_descr, item + port->src_num_offset);
			return 0;
		}
		item -= port_count[port_type][SCARLETT2_PORT_IN];
	}

	return -EINVAL;
}

static int scarlett2_mux_src_enum_ctl_get(struct snd_kcontrol *kctl,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const int (*port_count)[SCARLETT2_PORT_DIRNS] = info->port_count;
	int line_out_count =
		port_count[SCARLETT2_PORT_TYPE_ANALOGUE][SCARLETT2_PORT_OUT];
	int index = elem->control;
	
	if (index < line_out_count)
		index = line_out_remap(private, index);
	
	usb_audio_info(mixer->chip, ">>>scarlett2_mux_src_enum_ctl_get index=%d\n", index);

	mutex_lock(&private->data_mutex);
	if (private->mux_updated)
	{
		usb_audio_info(mixer->chip, "issuing scarlett2_usb_get_mux");
		scarlett2_usb_get_mux(mixer);
	}
	ucontrol->value.enumerated.item[0] = private->mux[index];
	mutex_unlock(&private->data_mutex);
	
	usb_audio_info(mixer->chip, "<<scarlett2_mux_src_enum_ctl_get index=%d value=%d\n", 
				   index, ucontrol->value.enumerated.item[0]);

	return 0;
}

static int scarlett2_mux_src_enum_ctl_put(struct snd_kcontrol *kctl,
					  struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;
	int line_out_count = private->info->port_count[SCARLETT2_PORT_TYPE_ANALOGUE][SCARLETT2_PORT_OUT];
	int none_count = private->info->port_count[SCARLETT2_PORT_TYPE_NONE][SCARLETT2_PORT_IN];
	int i, oval, val[2], delta, err = 0;
	int index, stereo_group; 

	mutex_lock(&private->data_mutex);

	/* Stereo group should be computed before remapping */
	index = elem->control;
	stereo_group = index >> 1;
	if (index < line_out_count)
		index = line_out_remap(private, index);

	/* Ensure that mux settings have been updated */
	oval = private->mux[index];
	val[0] = min(ucontrol->value.enumerated.item[0], private->num_mux_srcs - 1U);
	delta = val[0] - oval;
	usb_audio_info(mixer->chip, ">>>scarlett2_mux_src_enum_ctl_put index=%d, val=%d oval=%d delta=%d\n", index, val[0], oval, delta);
	if (delta == 0)
		goto unlock;

	/* If we are working in stereo mode, we need to ensure that the new value matches
	   the port number: odd for odd outputs, even for even outputs */
	if (private->stereo_switch[stereo_group]) {
		/* We perform control over the first channel in a pair. If this is odd output (counting from zero),
		   then we need to switch to previous one (even) and ensure that it points to even input
		   except the NONE input type */
		if (val[0] >= none_count) {
			if (index & 1) {
				index -= 1;
				val[0] -= 1;
				oval -= 1;
			}
			val[0] = (val[0] - none_count) & (~1);
			oval   = (oval   - none_count) & (~1);
			if (val[0] == oval)
				val[0] = (delta > 0) ? val[0] + 2 : val[0] - 2;
			val[0] = clamp(val[0] + none_count, 0, (int)(private->num_mux_srcs - 2U));
		}

		/* Compute the routing of the second channel */
		val[1] = (val[0] >= none_count) ? val[0] + 1 : val[0];

		/* Update the device's internal mux state for 2 outputs */
		for (i=0; i<2; ++i) {
			usb_audio_info(mixer->chip, "scarlett2_mux_commit_route: index=%d, val=%d\n", index + i, val[i]);
			err = scarlett2_mux_commit_route(mixer, index + i, val[i]);
			if (err < 0)
				goto unlock;
		}
	}
	else {
		/* Perform routing of single channel */
		err = scarlett2_mux_commit_route(mixer, index, val[0]);
	}
	
	/* Update routing settings */
	if (err >= 0)
		err = scarlett2_usb_set_mux(mixer);
	
	if (err >= 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	usb_audio_info(mixer->chip, "<<scarlett2_mux_src_enum_ctl_put\n");
	return err;
}

static const struct snd_kcontrol_new scarlett2_mux_src_enum_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = scarlett2_mux_src_enum_ctl_info,
	.get  = scarlett2_mux_src_enum_ctl_get,
	.put  = scarlett2_mux_src_enum_ctl_put,
};

static int scarlett2_add_mux_enums(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	int port_type, channel, i;
	int drv_port_type, drv_port_number, mux_input_number;
	u8 sw_port_number;
	u32 custom_mix = 0, stereo_sw = 0, custom_mix_mask;
	int index, stereo_index;
	int none_count = private->info->port_count[SCARLETT2_PORT_TYPE_NONE][SCARLETT2_PORT_IN];
	int err = 0;
	int mux_updated = 0;
	
	/* Read the custom mix mask if it is present */
	if (private->sw_cfg != NULL) {
		stereo_sw = le32_to_cpu(private->sw_cfg->stereo_sw);
		custom_mix = le32_to_cpu(private->sw_cfg->custom_mix);
	}
	
	usb_audio_info(mixer->chip, "stereo_sw=0x%x, custom_mix=0x%x\n",
				(int)stereo_sw, (int)custom_mix);

	for (i = 0, port_type = 0;
	     port_type < SCARLETT2_PORT_TYPE_COUNT;
	     port_type++) {
		for (channel = 0;
		     channel < private->info->port_count[port_type][SCARLETT2_PORT_OUT];
		     channel++, i++) {
			char s[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];
			const char *const descr =
				scarlett2_ports[port_type].dst_descr;
			snprintf(s, sizeof(s) - 5, descr, channel + 1);
			strcat(s, " Enum");
			
			/* Check that software config is present */
			if (private->sw_cfg != NULL) {
				/* Convert the driver output number to the FC output number */
				index = scarlett2_get_drv_to_sw_port_mapping(SCARLETT2_PORT_OUT, port_type, channel, info);
				if (index >= 0) {
					/* Convert the driver input to the FC input number
					 * When the corresponding bit for the custom_mix is set to '0',
					 * then the mixer assignment should be done.
					 * Otherwise, the input is assigned to output.
					 */
					sw_port_number = private->sw_cfg->out_mux[index];
					stereo_index = index & (~1);
					
					usb_audio_info(mixer->chip, "index=%d, stereo_index=%d, sw_port_number=%d, mask=0x%x\n",
						index, stereo_index, sw_port_number, (3 << stereo_index));
					
					custom_mix_mask = 1 << index;
					if (stereo_sw & (3 << stereo_index)) {
						sw_port_number = private->sw_cfg->out_mux[stereo_index] + (index & 1);
						custom_mix_mask = 1 << stereo_index;
					}
					
					usb_audio_info(mixer->chip, "upd sw_port_number=%d\n", sw_port_number);
					
					if (custom_mix & custom_mix_mask) {
						/* The physical input is routed to the physical output */
						scarlett2_get_sw_to_drv_port_mapping(&drv_port_type, &drv_port_number,
															info, SCARLETT2_PORT_IN, sw_port_number);
						mux_input_number = scarlett2_drv_port_number_to_mux_enum(
							SCARLETT2_PORT_IN, drv_port_type, drv_port_number, info);
					
						usb_audio_info(mixer->chip, "SW ROUTING for %d: %d:%d,%d:%d -> %s (%d,%d)\n",
							i, (int)sw_port_number, drv_port_type, drv_port_number, mux_input_number,
							s, port_type, channel
						);
					}
					else {
						/* The mixer output is routed to the physical output */
						mux_input_number = (sw_port_number < none_count)
							? scarlett2_drv_port_number_to_mux_enum(
								SCARLETT2_PORT_IN, SCARLETT2_PORT_TYPE_NONE, 0, info)
							: scarlett2_drv_port_number_to_mux_enum(
								SCARLETT2_PORT_IN, SCARLETT2_PORT_TYPE_MIX, sw_port_number - 1, info);
							
						usb_audio_info(mixer->chip, "MIX ROUTING for %d: %d:%d -> %s (%d,%d)\n",
							i, (int)sw_port_number, mux_input_number,
							s, port_type, channel
						);
					}
					
					/* Change the mux settings if they differ from the hardware ones */
					usb_audio_info(mixer->chip, "mux[%d] = %d -> %d\n",
								   (int)i, private->mux[i], mux_input_number);
					if (private->mux[i] != mux_input_number) {
						private->mux[i] = mux_input_number;
						mux_updated = 1;
					}
				}
			}

			/* Add the control to the mixer */
			err = scarlett2_add_new_ctl(mixer,
						    &scarlett2_mux_src_enum_ctl,
						    i, 1, s,
						    &private->mux_ctls[i]);
			if (err < 0)
				return err;
		}
	}

	/* Update the mux before exit if necessary */
	return mux_updated ? scarlett2_usb_set_mux(mixer) : err;
}

/*** Meter Controls ***/

static int scarlett2_meter_ctl_info(struct snd_kcontrol *kctl,
				    struct snd_ctl_elem_info *uinfo)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = elem->channels;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 4095;
	uinfo->value.integer.step = 1;
	return 0;
}

static int scarlett2_meter_ctl_get(struct snd_kcontrol *kctl,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	u16 meter_levels[SCARLETT2_MAX_METERS];
	int i, err;

	err = scarlett2_usb_get_meter_levels(elem->head.mixer, elem->channels,
					     meter_levels);
	if (err < 0)
		return err;

	for (i = 0; i < elem->channels; i++)
		ucontrol->value.integer.value[i] = meter_levels[i];

	return 0;
}

static const struct snd_kcontrol_new scarlett2_meter_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_PCM,
	.access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	.name = "",
	.info = scarlett2_meter_ctl_info,
	.get  = scarlett2_meter_ctl_get
};

static int scarlett2_add_meter_ctl(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;

	/* devices without a mixer also don't support reporting levels */
	if (!private->info->has_mixer)
		return 0;

	return scarlett2_add_new_ctl(mixer, &scarlett2_meter_ctl,
				     0, private->num_mux_dsts,
				     "Level Meter", NULL);
}

/*** MSD Controls ***/

static int scarlett2_msd_ctl_get(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct scarlett2_data *private = elem->head.mixer->private_data;

	ucontrol->value.integer.value[0] = private->msd_switch;
	return 0;
}

static int scarlett2_msd_ctl_put(struct snd_kcontrol *kctl,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct usb_mixer_elem_info *elem = kctl->private_data;
	struct usb_mixer_interface *mixer = elem->head.mixer;
	struct scarlett2_data *private = mixer->private_data;

	int oval, val, err = 0;

	mutex_lock(&private->data_mutex);

	oval = private->msd_switch;
	val = !!ucontrol->value.integer.value[0];

	if (oval == val)
		goto unlock;

	private->msd_switch = val;

	/* Send switch change to the device */
	err = scarlett2_usb_set_config(mixer, SCARLETT2_CONFIG_MSD_SWITCH,
				       0, val);
	if (err == 0)
		err = 1;

unlock:
	mutex_unlock(&private->data_mutex);
	return err;
}

static const struct snd_kcontrol_new scarlett2_msd_ctl = {
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
	.name = "",
	.info = snd_ctl_boolean_mono_info,
	.get  = scarlett2_msd_ctl_get,
	.put  = scarlett2_msd_ctl_put,
};

static int scarlett2_add_msd_ctl(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;

	if (!info->has_msd_mode)
		return 0;

	/* If MSD mode is off, hide the switch by default */
	if (!private->msd_switch && !(mixer->chip->setup & SCARLETT2_MSD_ENABLE))
		return 0;

	/* Add MSD control */
	return scarlett2_add_new_ctl(mixer, &scarlett2_msd_ctl,
				     0, 1, "MSD Mode Switch", NULL);
}

/*** Cleanup/Suspend Callbacks ***/

static void scarlett2_private_free(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;

	cancel_delayed_work_sync(&private->work);
	if (private->sw_cfg != NULL) {
		kfree(private->sw_cfg);
		private->sw_cfg = NULL;
	}
	kfree(private);
	mixer->private_data = NULL;
}

static void scarlett2_private_suspend(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;

	if (cancel_delayed_work_sync(&private->work))
		scarlett2_config_save(private->mixer);
}

/*** Initialisation ***/

static void scarlett2_count_mux_io(struct scarlett2_data *private)
{
	const struct scarlett2_device_info *info = private->info;
	const int (*port_count)[SCARLETT2_PORT_DIRNS] = info->port_count;
	int port_type, srcs = 0, dsts = 0;

	for (port_type = 0;
	     port_type < SCARLETT2_PORT_TYPE_COUNT;
	     port_type++) {
		srcs += port_count[port_type][SCARLETT2_PORT_IN];
		dsts += port_count[port_type][SCARLETT2_PORT_OUT];
	}

	private->num_mux_srcs = srcs;
	private->num_mux_dsts = dsts;
}

/* Look through the interface descriptors for the Focusrite Control
 * interface (bInterfaceClass = 255 Vendor Specific Class) and set
 * bInterfaceNumber, bEndpointAddress, wMaxPacketSize, and bInterval
 * in private
 */
static int scarlett2_find_fc_interface(struct usb_device *dev,
				       struct scarlett2_data *private)
{
	struct usb_host_config *config = dev->actconfig;
	int i;

	for (i = 0; i < config->desc.bNumInterfaces; i++) {
		struct usb_interface *intf = config->interface[i];
		struct usb_interface_descriptor *desc =
			&intf->altsetting[0].desc;
		struct usb_endpoint_descriptor *epd;

		if (desc->bInterfaceClass != 255)
			continue;

		epd = get_endpoint(intf->altsetting, 0);
		private->bInterfaceNumber = desc->bInterfaceNumber;
		private->bEndpointAddress = epd->bEndpointAddress &
			USB_ENDPOINT_NUMBER_MASK;
		private->wMaxPacketSize = le16_to_cpu(epd->wMaxPacketSize);
		private->bInterval = epd->bInterval;
		return 0;
	}

	return -EINVAL;
}

/* Initialise private data */
static int scarlett2_init_private(struct usb_mixer_interface *mixer,
				  const struct scarlett2_device_info *info)
{
	struct scarlett2_data *private =
		kzalloc(sizeof(struct scarlett2_data), GFP_KERNEL);

	if (!private)
		return -ENOMEM;

	mutex_init(&private->usb_mutex);
	mutex_init(&private->data_mutex);
	INIT_DELAYED_WORK(&private->work, scarlett2_config_save_work);

	mixer->private_data = private;
	mixer->private_free = scarlett2_private_free;
	mixer->private_suspend = scarlett2_private_suspend;

	private->info = info;
	scarlett2_count_mux_io(private);
	private->scarlett2_seq = 0;
	private->mixer = mixer;
    private->sw_cfg = NULL;

	return scarlett2_find_fc_interface(mixer->chip->dev, private);
}

/* Cargo cult proprietary initialisation sequence */
static int scarlett2_usb_init(struct usb_mixer_interface *mixer)
{
	struct usb_device *dev = mixer->chip->dev;
	struct scarlett2_data *private = mixer->private_data;
	u8 buf[24];
	int err;

	if (usb_pipe_type_check(dev, usb_sndctrlpipe(dev, 0)))
		return -EINVAL;

	/* step 0 */
	err = scarlett2_usb_rx(dev, private->bInterfaceNumber,
			       SCARLETT2_USB_CMD_INIT, buf, sizeof(buf));
	if (err < 0)
		return err;

	/* step 1 */
	private->scarlett2_seq = 1;
	err = scarlett2_usb(mixer, SCARLETT2_USB_INIT_1, NULL, 0, NULL, 0);
	if (err < 0)
		return err;

	/* step 2 */
	private->scarlett2_seq = 1;
	return scarlett2_usb(mixer, SCARLETT2_USB_INIT_2, NULL, 0, NULL, 84);
}

/* Read configuration from the interface on start */
static int scarlett2_read_configs(struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const int (*port_count)[SCARLETT2_PORT_DIRNS] = info->port_count;
	int num_line_out =
		port_count[SCARLETT2_PORT_TYPE_ANALOGUE][SCARLETT2_PORT_OUT];
	int num_mixer_out =
		port_count[SCARLETT2_PORT_TYPE_MIX][SCARLETT2_PORT_IN];
	struct scarlett2_usb_volume_status volume_status;
	int err, i;

	if (info->has_msd_mode) {
		err = scarlett2_usb_get_config(
			mixer, SCARLETT2_CONFIG_MSD_SWITCH,
			1, &private->msd_switch);
		if (err < 0)
			return err;

		/* no other controls are created if MSD mode is on */
		if (private->msd_switch)
			return 0;
	}

	err = scarlett2_update_input_other(mixer);
	if (err < 0)
		return err;

	err = scarlett2_update_monitor_other(mixer);
	if (err < 0)
		return err;

	/* the rest of the configuration is for devices with a mixer */
	if (!info->has_mixer)
		return 0;

	err = scarlett2_update_sync(mixer);
	if (err < 0)
		return err;

	err = scarlett2_usb_get_volume_status(mixer, &volume_status);
	if (err < 0)
		return err;

	if (info->line_out_hw_vol)
		for (i = 0; i < SCARLETT2_DIM_MUTE_COUNT; i++)
			private->dim_mute[i] = !!volume_status.dim_mute[i];

	private->master_vol = clamp(
		volume_status.master_vol + SCARLETT2_VOLUME_BIAS,
		0, SCARLETT2_VOLUME_BIAS);

	for (i = 0; i < num_line_out; i++) {
		int volume, mute;

		private->vol_sw_hw_switch[i] =
			info->line_out_hw_vol
				&& volume_status.sw_hw_switch[i];

		volume = private->vol_sw_hw_switch[i]
			   ? volume_status.master_vol
			   : volume_status.sw_vol[i];
		volume = clamp(volume + SCARLETT2_VOLUME_BIAS,
			       0, SCARLETT2_VOLUME_BIAS);
		private->vol[i] = volume;

		mute = private->vol_sw_hw_switch[i]
			 ? private->dim_mute[SCARLETT2_BUTTON_MUTE]
			 : volume_status.mute_switch[i];
		private->mute_switch[i] = ! mute;
	}

	for (i = 0; i < num_mixer_out; i++) {
		err = scarlett2_usb_get_mix(mixer, i);
		if (err < 0)
			return err;
	}

	return scarlett2_usb_get_mux(mixer);
}

/* Notify on sync change */
static void scarlett2_notify_sync(
	struct usb_mixer_interface *mixer)
{
	struct scarlett2_data *private = mixer->private_data;

	private->sync_updated = 1;

	snd_ctl_notify(mixer->chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
		       &private->sync_ctl->id);
}

/* Notify on monitor change */
static void scarlett2_notify_monitor(
	struct usb_mixer_interface *mixer)
{
	struct snd_card *card = mixer->chip->card;
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const int (*port_count)[SCARLETT2_PORT_DIRNS] = info->port_count;
	int num_line_out =
		port_count[SCARLETT2_PORT_TYPE_ANALOGUE][SCARLETT2_PORT_OUT];
	int i;

	/* if line_out_hw_vol is 0, there are no controls to update */
	if (!info->line_out_hw_vol)
		return;

	private->vol_updated = 1;

	snd_ctl_notify(mixer->chip->card, SNDRV_CTL_EVENT_MASK_VALUE,
		       &private->master_vol_ctl->id);

	for (i = 0; i < num_line_out; i++)
		if (private->vol_sw_hw_switch[line_out_remap(private, i)])
			snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &private->vol_ctls[i]->id);
}

/* Notify on dim/mute change */
static void scarlett2_notify_dim_mute(
	struct usb_mixer_interface *mixer)
{
	struct snd_card *card = mixer->chip->card;
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	const int (*port_count)[SCARLETT2_PORT_DIRNS] = info->port_count;
	int num_line_out =
		port_count[SCARLETT2_PORT_TYPE_ANALOGUE][SCARLETT2_PORT_OUT];
	int i;

	private->vol_updated = 1;

	if (!info->line_out_hw_vol)
		return;

	for (i = 0; i < SCARLETT2_DIM_MUTE_COUNT; i++)
		snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &private->dim_mute_ctls[i]->id);

	for (i = 0; i < num_line_out; i++)
		if (private->vol_sw_hw_switch[line_out_remap(private, i)])
			snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &private->mute_ctls[i]->id);
}

/* Notify on "input other" change (level/pad/air) */
static void scarlett2_notify_input_other(
	struct usb_mixer_interface *mixer)
{
	struct snd_card *card = mixer->chip->card;
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;
	int i;

	private->input_other_updated = 1;

	for (i = 0; i < info->level_input_count; i++)
		snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &private->level_ctls[i]->id);
	for (i = 0; i < info->pad_input_count; i++)
		snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &private->pad_ctls[i]->id);
	for (i = 0; i < info->air_input_count; i++)
		snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &private->air_ctls[i]->id);
	for (i = 0; i < info->phantom_count; i++)
		snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &private->phantom_ctls[i]->id);
}

/* Notify on "monitor other" change (direct monitor, speaker
 * switching, talkback)
 */
static void scarlett2_notify_monitor_other(
	struct usb_mixer_interface *mixer)
{
	struct snd_card *card = mixer->chip->card;
	struct scarlett2_data *private = mixer->private_data;
	const struct scarlett2_device_info *info = private->info;

	private->monitor_other_updated = 1;

	if (info->direct_monitor) {
		snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &private->direct_monitor_ctl->id);
		return;
	}

	if (info->has_speaker_switching)
		snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &private->speaker_switching_ctl->id);

	if (info->has_talkback)
		snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
			       &private->talkback_ctl->id);

	/* if speaker switching was recently enabled or disabled,
	 * invalidate the dim/mute and mux enum controls
	 */
	if (private->speaker_switching_switched) {
		int i;

		scarlett2_notify_dim_mute(mixer);

		private->speaker_switching_switched = 0;
		private->mux_updated = 1;

		for (i = 0; i < private->num_mux_dsts; i++)
			snd_ctl_notify(card, SNDRV_CTL_EVENT_MASK_VALUE,
				       &private->mux_ctls[i]->id);
	}
}

/* Interrupt callback */
static void scarlett2_notify(struct urb *urb)
{
	struct usb_mixer_interface *mixer = urb->context;
	int len = urb->actual_length;
	int ustatus = urb->status;
	u32 data;

	if (ustatus != 0 || len != 8)
		goto requeue;

	data = le32_to_cpu(*(__le32 *)urb->transfer_buffer);
	if (data & SCARLETT2_USB_NOTIFY_SYNC)
		scarlett2_notify_sync(mixer);
	if (data & SCARLETT2_USB_NOTIFY_MONITOR)
		scarlett2_notify_monitor(mixer);
	if (data & SCARLETT2_USB_NOTIFY_DIM_MUTE)
		scarlett2_notify_dim_mute(mixer);
	if (data & SCARLETT2_USB_NOTIFY_INPUT_OTHER)
		scarlett2_notify_input_other(mixer);
	if (data & SCARLETT2_USB_NOTIFY_MONITOR_OTHER)
		scarlett2_notify_monitor_other(mixer);

requeue:
	if (ustatus != -ENOENT &&
	    ustatus != -ECONNRESET &&
	    ustatus != -ESHUTDOWN) {
		urb->dev = mixer->chip->dev;
		usb_submit_urb(urb, GFP_ATOMIC);
	}
}

static int scarlett2_init_notify(struct usb_mixer_interface *mixer)
{
	struct usb_device *dev = mixer->chip->dev;
	struct scarlett2_data *private = mixer->private_data;
	unsigned int pipe = usb_rcvintpipe(dev, private->bEndpointAddress);
	void *transfer_buffer;

	if (mixer->urb) {
		usb_audio_err(mixer->chip,
			      "%s: mixer urb already in use!\n", __func__);
		return 0;
	}

	if (usb_pipe_type_check(dev, pipe))
		return -EINVAL;

	mixer->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!mixer->urb)
		return -ENOMEM;

	transfer_buffer = kmalloc(private->wMaxPacketSize, GFP_KERNEL);
	if (!transfer_buffer)
		return -ENOMEM;

	usb_fill_int_urb(mixer->urb, dev, pipe,
			 transfer_buffer, private->wMaxPacketSize,
			 scarlett2_notify, mixer, private->bInterval);

	return usb_submit_urb(mixer->urb, GFP_KERNEL);
}

static int snd_scarlett_gen2_controls_create(struct usb_mixer_interface *mixer)
{
	const struct scarlett2_device_info **info = scarlett2_devices;
	int err;

	/* Find device in scarlett2_devices */
	while (*info && (*info)->usb_id != mixer->chip->usb_id)
		info++;
	if (!*info)
		return -EINVAL;

	/* Initialise private data */
	err = scarlett2_init_private(mixer, *info);
	if (err < 0)
		return err;

	/* Send proprietary USB initialisation sequence */
	err = scarlett2_usb_init(mixer);
	if (err < 0)
		return err;

	/* Read volume levels and controls from the interface */
	err = scarlett2_read_configs(mixer);
	if (err < 0)
		return err;
    
	/* Read software configuration containing mixer gains */
	err = scarlett2_read_software_configs(mixer);
	if (err < 0)
		return err;
    scarlett2_dump_config_space(mixer);

	/* Create the MSD control */
	err = scarlett2_add_msd_ctl(mixer);
	if (err < 0)
		return err;

	/* If MSD mode is enabled, don't create any other controls */
	if (((struct scarlett2_data *)mixer->private_data)->msd_switch)
		return 0;

	/* Create the analogue output controls */
	err = scarlett2_add_line_out_ctls(mixer);
	if (err < 0)
		return err;

	/* Create the analogue input controls */
	err = scarlett2_add_line_in_ctls(mixer);
	if (err < 0)
		return err;

	/* Create the input, output, and mixer mux input selections */
	err = scarlett2_add_mux_enums(mixer);
	if (err < 0)
		return err;

	/* Create the matrix mixer controls */
	err = scarlett2_add_mixer_ctls(mixer);
	if (err < 0)
		return err;

	/* Create the level meter controls */
	err = scarlett2_add_meter_ctl(mixer);
	if (err < 0)
		return err;

	/* Create the sync control */
	err = scarlett2_add_sync_ctl(mixer);
	if (err < 0)
		return err;

	/* Create the direct monitor control */
	err = scarlett2_add_direct_monitor_ctl(mixer);
	if (err < 0)
		return err;

	/* Create the speaker switching control */
	err = scarlett2_add_speaker_switch_ctl(mixer);
	if (err < 0)
		return err;

	/* Create the talkback controls */
	err = scarlett2_add_talkback_ctls(mixer);
	if (err < 0)
		return err;

	/* Set up the interrupt polling */
	err = scarlett2_init_notify(mixer);
	if (err < 0)
		return err;

	return 0;
}

int snd_scarlett_gen2_init(struct usb_mixer_interface *mixer)
{
	struct snd_usb_audio *chip = mixer->chip;
	int err;

	/* only use UAC_VERSION_2 */
	if (!mixer->protocol)
		return 0;

	if (!(chip->setup & SCARLETT2_ENABLE)) {
		usb_audio_info(chip,
			"Focusrite Scarlett Gen 2/3 Mixer Driver disabled; "
			"use options snd_usb_audio vid=0x%04x pid=0x%04x "
			"device_setup=1 to enable and report any issues "
			"to g@b4.vu",
			USB_ID_VENDOR(chip->usb_id),
			USB_ID_PRODUCT(chip->usb_id));
		return 0;
	}

	usb_audio_info(chip,
		"Focusrite Scarlett Gen 2/3 Mixer Driver enabled pid=0x%04x",
		USB_ID_PRODUCT(chip->usb_id));

	err = snd_scarlett_gen2_controls_create(mixer);
	if (err < 0)
		usb_audio_err(mixer->chip,
			      "Error initialising Scarlett Mixer Driver: %d",
			      err);

	return err;
}
