/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2014 Daniel Thompson <daniel@redfelineninja.org.uk>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/tools.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencmsis/core_cm3.h>

#include "usb-audio-structs.h"
#include "led.h"

#define MIDI_ENDP_SIZE 32
#define SAMPLE_RATE 42000
#define NUM_CHANNELS 2
#define SAMPLE_BITS 24
#define SAMPLE_BYTES ((SAMPLE_BITS + 7) / 8)
#define AS_PACKET_SAMPLES ((SAMPLE_RATE + 999) / 1000)
#define AS_ENDPOINT_TOLERANCE 1
#define AS_ENDPOINT_SAMPLES (AS_PACKET_SAMPLES + AS_ENDPOINT_TOLERANCE)
#define AS_ENDPOINT_BYTES (AS_ENDPOINT_SAMPLES * SAMPLE_BYTES * NUM_CHANNELS)

int usbaudio_control_callback(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len, usbd_control_complete_callback *complete);
void usbaudio_set_altsetting_cb(usbd_device *dev, uint16_t wIndex, uint16_t wValue);

/*
 * All references in this file come from Universal Serial Bus Device Class
 * Definition for MIDI Devices, release 1.0.
 */

/*
 * Table B-1: MIDI Adapter Device Descriptor
 */
static const struct usb_device_descriptor dev_desc = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,    /* was 0x0110 in Table B-1 example descriptor */
	.bDeviceClass = 0,   /* device defined at interface level */
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0499,  /* 0499=Yamaha */
	.idProduct = 0xffff, /* ffff=Inexistent */
	.bcdDevice = 0x0100,
	.iManufacturer = 1,  /* index to string desc */
	.iProduct = 2,       /* index to string desc */
	.iSerialNumber = 0,  /* index to string desc */
	.bNumConfigurations = 1,
};

/*
 * Midi specific endpoint descriptors.
 */
static const struct usb_midi_endpoint_descriptor midi_bulk_endp[] = {{
	/* Table B-12: MIDI Adapter Class-specific Bulk OUT Endpoint
	 * Descriptor
	 */
	.head = {
		.bLength = sizeof(struct usb_midi_endpoint_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
		.bDescriptorSubType = USB_MIDI_SUBTYPE_MS_GENERAL,
		.bNumEmbMIDIJack = 1,
	},
	.jack[0] = {
		.baAssocJackID = 0x01,
	},
}};

/*
 * Standard endpoint descriptors
 */
static const struct usb_endpoint_descriptor bulk_endp[] = {{
	/* Table B-11: MIDI Adapter Standard Bulk OUT Endpoint Descriptor */
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = MIDI_ENDP_SIZE,
	.bInterval = 0x00,

	.extra = &midi_bulk_endp[0],
	.extralen = sizeof(midi_bulk_endp[0])
}};

/*
 * Table B-4: MIDI Adapter Class-specific AC Interface Descriptor
 */
static const struct {
	struct usb_audio_header_descriptor_head header_head;
	struct usb_audio_header_descriptor_body header_body[2];
	struct usb_audio_control_input_terminal_descriptor recording_input_terminal;
	struct usb_audio_control_output_terminal_descriptor recording_output_terminal;
} __attribute__((packed)) audio_control_functional_descriptors = {
	.header_head = {
		.bLength = sizeof(struct usb_audio_header_descriptor_head) + 2 * sizeof(struct usb_audio_header_descriptor_body),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_AUDIO_TYPE_HEADER,
		.bcdADC = 0x0100,
		.wTotalLength = sizeof(audio_control_functional_descriptors),
		.binCollection = 2,
	},
	.header_body = {{
		.baInterfaceNr = 1,
	}, {
		.baInterfaceNr = 2,
	}},
	.recording_input_terminal = {
		.bLength =                12,
		.bDescriptorType =        36,
		.bDescriptorSubtype =      2, // (INPUT_TERMINAL)
		.bTerminalID =             1,
		.wTerminalType =      0x0201, // Microphone
		.bAssocTerminal =          0,
		.bNrChannels =  NUM_CHANNELS,
		.wChannelConfig =     0x0003,
		.iChannelNames =           2,
		.iTerminal =               0,
	},
	.recording_output_terminal = {
		.bLength =                 9,
		.bDescriptorType =        36,
		.bDescriptorSubtype =      3, // (OUTPUT_TERMINAL)
		.bTerminalID =             2,
		.wTerminalType =      0x0101, // USB Streaming
		.bAssocTerminal =          0,
		.bSourceID =               1,
		.iTerminal =               3,
	}
};

static const struct usb_interface_descriptor audio_control_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_CONTROL,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.extra = &audio_control_functional_descriptors,
	.extralen = sizeof(audio_control_functional_descriptors)
}};

/*
 * Table B-3: MIDI Adapter Standard AC Interface Descriptor
 */

/*
 * Class-specific MIDI streaming interface descriptor
 */
static const struct {
	struct usb_midi_header_descriptor header;
	struct usb_midi_out_jack_descriptor out_embedded;
	struct usb_midi_out_jack_descriptor out_external;
} __attribute__((packed)) midi_streaming_functional_descriptors = {
	/* Table B-6: Midi Adapter Class-specific MS Interface Descriptor */
	.header = {
		.bLength = sizeof(struct usb_midi_header_descriptor),
		.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
		.bDescriptorSubtype = USB_MIDI_SUBTYPE_MS_HEADER,
		.bcdMSC = 0x0100,
		.wTotalLength = sizeof(midi_streaming_functional_descriptors),
	},
	/* Table B-9: MIDI Adapter MIDI OUT Jack Descriptor (Embedded) */
	.out_embedded = {
		.head = {
			.bLength = sizeof(struct usb_midi_out_jack_descriptor),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
			.bJackType = USB_MIDI_JACK_TYPE_EMBEDDED,
			.bJackID = 0x03,
			.bNrInputPins = 1,
		},
		.source[0] = {
			.baSourceID = 0x02,
			.baSourcePin = 0x01,
		},
		.tail = {
			.iJack = 0x00,
		}
	},
	/* Table B-10: MIDI Adapter MIDI OUT Jack Descriptor (External) */
	.out_external = {
		.head = {
			.bLength = sizeof(struct usb_midi_out_jack_descriptor),
			.bDescriptorType = USB_AUDIO_DT_CS_INTERFACE,
			.bDescriptorSubtype = USB_MIDI_SUBTYPE_MIDI_OUT_JACK,
			.bJackType = USB_MIDI_JACK_TYPE_EXTERNAL,
			.bJackID = 0x04,
			.bNrInputPins = 1,
		},
		.source[0] = {
			.baSourceID = 0x01,
			.baSourcePin = 0x01,
		},
		.tail = {
			.iJack = 0x00,
		},
	},
};

/*
 * Table B-5: MIDI Adapter Standard MS Interface Descriptor
 */
static const struct usb_interface_descriptor midi_streaming_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 2,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_MIDISTREAMING,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = bulk_endp,

	.extra = &midi_streaming_functional_descriptors,
	.extralen = sizeof(midi_streaming_functional_descriptors)
}};

struct sample_freq {
	uint8_t b0, b1, b2;
} __attribute__((packed));
#define AUDIO_SAMPLE_FREQ(frq) \
	{ .b0 = (uint8_t)(frq), .b1 = (uint8_t)(((frq) >> 8)), .b2 = (uint8_t)(((frq) >> 16)) }

//#define ALL_FREQS FREQ_BLOCK(0, 96000) FREQ_BLOCK(1, 64000) FREQ_BLOCK(2, 48000) FREQ_BLOCK(3, 32000)
#define ALL_FREQS FREQ_BLOCK(0, 42000)


static const struct usb_audio_iso_endpoint_descriptor audio_iso_endp[] = {{
	.bLength = 7,
	.bDescriptorType = USB_AUDIO_DT_CS_ENDPOINT,
	.bDescriptorSubtype = 0x01,
	.bmAttributes = 0,
	.bLockDelayUnits = 0,
	.wLockDelay = 0
}};

static const struct usb_endpoint_descriptor iso_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_ISOCHRONOUS,
	.wMaxPacketSize = AS_ENDPOINT_BYTES,
	.bInterval = 1,

	.extra = &audio_iso_endp[0],
	.extralen = sizeof(audio_iso_endp[0])
}};

static const struct {
	struct usb_audio_streaming_header_descriptor header;
	struct usb_audio_streaming_format_type_descriptor format_type;
	struct sample_freq freq;
} __attribute__((packed)) audio_streaming_functional_descriptors = {
	.header = {
		.bLength = 7,
		.bDescriptorType = 0x24,
		.bDescriptorSubtype = 0x01,
		.bTerminalID = 2,
		.bInterfaceDelay = 0,
		.wFormat = 1
	},
	.format_type = {
		.bLength = 11,
		.bDescriptorType = 0x24,
		.bDescriptorSubtype = 0x02,
		.bFormatType = 1,
		.bNrChannels = NUM_CHANNELS,
		.bSubframeSize = SAMPLE_BYTES,
		.bBitResolution = SAMPLE_BITS,
		.bSamFreqType = 1
	},
	.freq = AUDIO_SAMPLE_FREQ(SAMPLE_RATE)
};

static const struct usb_interface_descriptor audio_streaming_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = 0,
	.iInterface = 0,
}, {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 1,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_AUDIO,
	.bInterfaceSubClass = USB_AUDIO_SUBCLASS_AUDIOSTREAMING,
	.bInterfaceProtocol = 0,
	.iInterface = 0,
	.endpoint = iso_endp,
	.extra = &audio_streaming_functional_descriptors,
	.extralen = sizeof(audio_streaming_functional_descriptors)
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = audio_control_iface,
}, {
	.num_altsetting = 2,
	.altsetting = audio_streaming_iface,
}, {
	.num_altsetting = 1,
	.altsetting = midi_streaming_iface,
}};

/*
 * Table B-2: MIDI Adapter Configuration Descriptor
 */
static const struct usb_config_descriptor config = {
	.bLength             = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType     = USB_DT_CONFIGURATION,
	.wTotalLength        = 0,       /* can be anything, it is updated automatically
	                                   when the usb code prepares the descriptor */
	.bNumInterfaces      = 3,       /* control, audio streaming, midi streaming */
	.bConfigurationValue = 1,
	.iConfiguration      = 0,
	.bmAttributes        = 0x80,    /* bus powered */
	.bMaxPower           = 200 / 2, /* mA / 2 */

	.interface = ifaces,
};

static const char * usb_strings[] = {
	"Yamaha",
	"TG100",
	"Zero-bandwidth iface",
	"Recording iface",
	"Hello"
};

usbd_device *usbd_dev;

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[300];

uint8_t midi_queue[256];
uint8_t midi_queue_top = 0, midi_queue_bottom = 0;
static void midi_queue_push(uint8_t b) {
	midi_queue[midi_queue_top] = b;
	midi_queue_top++; // wraps around to 0 at 256
	if(midi_queue_top == midi_queue_bottom)
		midi_queue_bottom++;
}

static void midi_queue_poll(void) {
	if((USART_SR(USART3) & USART_SR_TXE) != 0) {
		if(midi_queue_bottom != midi_queue_top) {
			led_on();
			usart_send(USART3, midi_queue[midi_queue_bottom]);
			midi_queue_bottom++;
		}
	}
}

static void usbmidi_data_rx_cb(usbd_device *dev, uint8_t ep) {
	(void)ep;

	char buf[MIDI_ENDP_SIZE];
	int len = usbd_ep_read_packet(dev, 0x01, buf, MIDI_ENDP_SIZE);

	int i;
	for(i = 0; i < len; i+=4) {
		int n = 3;
		switch(buf[i] & 0x0f) {
			case 0x2:
				n = 2; // Two-byte System Common messages like MTC, SongSelect, etc.
				break;
			case 0x3:
				n = 3; // Three-byte System Common messages like SPP, etc.
				break;
			case 0x4:
				n = 3; // SysEx starts or continues
				break;
			case 0x5:
				n = 1; // Single-byte System Common Message
				break;
			case 0x6:
				n = 2; // SysEx ends with following two bytes.
				break;
			case 0x7:
				n = 3; // SysEx ends with following three bytes.
				break;
			case 0x8:
				n = 3; // Note-off
				break;
			case 0x9:
				n = 3; // Note-on
				break;
			case 0xA:
				n = 3; // Poly-KeyPress
				break;
			case 0xB:
				n = 3; // Control Change
				break;
			case 0xC:
				n = 2; // Program Change
				break;
			case 0xD:
				n = 2; // Channel Pressure
				break;
			case 0xE:
				n = 3; // PitchBend Change
				break;
			case 0xF:
				n = 1; // Single Byte
				break;
		}
		if(n >= 1) midi_queue_push(buf[i + 1]);
		if(n >= 2) midi_queue_push(buf[i + 2]);
		if(n >= 3) midi_queue_push(buf[i + 3]);
	}
}

static int sending_iso = 0;

#define AUDIO_BUF_SIZE (48 * NUM_CHANNELS * SAMPLE_BYTES * 2)
static uint8_t audio_buf[AUDIO_BUF_SIZE];
static unsigned int audio_buf_bottom = 0;
static unsigned int audio_buf_top = 0;
static void audio_buf_push(uint32_t b) {
	audio_buf[audio_buf_top] = b;
	audio_buf_top++;
	if(audio_buf_top >= sizeof(audio_buf) / sizeof(audio_buf[0]))
		audio_buf_top = 0;
	if(audio_buf_top == audio_buf_bottom) {
		audio_buf_bottom++;
		if(audio_buf_bottom >= sizeof(audio_buf) / sizeof(audio_buf[0]))
			audio_buf_bottom = 0;
	}
}
static uint8_t audio_buf_pop(void) {
	uint8_t r = audio_buf[audio_buf_bottom];
	audio_buf_bottom++;
	if(audio_buf_bottom >= sizeof(audio_buf) / sizeof(audio_buf[0])) {
		audio_buf_bottom = 0;
	}
	return r;
}
static void audio_buf_reset(void) {
	audio_buf_top = audio_buf_bottom = 0;
}

static uint8_t iso_buf[AS_ENDPOINT_BYTES];
// Isochronous buffer send
// gets called about 1000 times per second
static void send_audiobuf(usbd_device *dev, int alt) {
	(void)alt;

	int i = 0;

	if(audio_buf_bottom != audio_buf_top) {
		uint8_t *p = iso_buf;
		for(i = 0; i < AS_ENDPOINT_BYTES && audio_buf_bottom != audio_buf_top; i++) {
			*(p++) = audio_buf_pop();
		}
	}

	USB_SET_EP_TX_STAT(2, USB_EP_TX_STAT_DISABLED);
	USB_CLR_EP_TX_DTOG(2);
	usbd_ep_write_packet(dev, 0x82, iso_buf, i);
}

#define DMABUFSIZE (AS_PACKET_SAMPLES * 28)
uint8_t dmabuf[DMABUFSIZE * 2];

static void push_dma_samples(uint8_t *buf) {
	uint8_t *p = buf;
	for(int j = 0; j < AS_PACKET_SAMPLES; j++) {
		uint32_t l = 0, r = 0;
		uint32_t m = 1 << 27;
		for(int i = 0; i < 28; i++) {
			if(*p & 0x40)
				l |= m;
			if(*p & 0x80)
				r |= m;
			m >>= 1;
			p++;
		}
		l >>= 2;
		r >>= 2;
		audio_buf_push(l);
		audio_buf_push(l >> 8);
		audio_buf_push(l >> 16);
		audio_buf_push(r);
		audio_buf_push(r >> 8);
		audio_buf_push(r >> 16);
	}
}

void dma1_channel2_isr(void) {
	gpio_set(GPIOB, GPIO10);
	if((DMA1_ISR & DMA_ISR_TCIF2) != 0) {
		// transfer complete
		DMA1_IFCR |= DMA_IFCR_CTCIF2;
		push_dma_samples(dmabuf + DMABUFSIZE);
	}
	if((DMA1_ISR & DMA_ISR_HTIF2) != 0) {
		// half transfer
		DMA1_IFCR |= DMA_IFCR_CHTIF2;
		push_dma_samples(dmabuf);
	}
	gpio_clear(GPIOB, GPIO10);
}

static void capture_setup(void) {
	// PA8 = TIM1_CH1 is connected to bit clock
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_AFIO);
	gpio_set_mode(
		GPIOA,
		GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_FLOAT,
		GPIO_TIM1_CH1
	);

	// PB0 and PB1 are L and R data
	// PB4 is word clock and used for sync only
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_set_mode(
		GPIOB, GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_FLOAT,
		GPIO6|GPIO7|GPIO4
	);

	gpio_set_mode(
		GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		GPIO10 | GPIO11
	);
	gpio_set(GPIOB, GPIO10 | GPIO11);

	rcc_periph_clock_enable(RCC_DMA1);
	rcc_periph_clock_enable(RCC_TIM1);
	// TIM1_CH1 is connected to DMA1 Channel 2
	nvic_enable_irq(NVIC_DMA1_CHANNEL2_IRQ);
}

static void capture_start(void) {
	// sync up
	while(!(GPIOB_IDR & GPIO4))
		__asm__("nop");
	while(GPIOB_IDR & GPIO4)
		__asm__("nop");
	for(int i = 0; i < 60; i++) // fine tuned delay for decently working sync
		__asm__("nop");

	audio_buf_reset();

	dma_channel_reset(DMA1, 2);
	DMA1_CNDTR2 = DMABUFSIZE * 2;
	DMA1_CPAR2 = (uint32_t)&GPIOB_IDR;
	DMA1_CMAR2 = (uint32_t)dmabuf;
	DMA1_CCR2 = 0
		| DMA_CCR_PL_MEDIUM   // Medium priority
		| DMA_CCR_MSIZE_8BIT  // 8 bit memory write
		| DMA_CCR_PSIZE_8BIT  // 8 bit peripheral read
		| DMA_CCR_MINC        // Increment memory address
		| DMA_CCR_CIRC        // Circular mode
		| DMA_CCR_TCIE        // Transfer Complete Interrupt Enable
		| DMA_CCR_HTIE        // Half Transfer Interrupt Enable (we use this as double buffering)
		| DMA_CCR_EN          // Enable this DMA channel
		;

	timer_reset(TIM1);
	TIM1_CCMR1 = TIM_CCMR1_CC1S_IN_TI1;
	TIM1_CCER = TIM_CCER_CC1E;
	TIM1_SMCR = TIM_SMCR_SMS_ECM1 | TIM_SMCR_TS_TI1FP1;
	TIM1_DIER = TIM_DIER_CC1DE;
	TIM1_CR1 = TIM_CR1_CEN;
}

static void capture_stop(void) {
	dma_channel_reset(DMA1, 2);
	timer_reset(TIM1);
}

static void usbaudio_data_tx_cb(usbd_device *dev, uint8_t ep) {
	if(ep == 2 && sending_iso) {
		send_audiobuf(dev, sending_iso);
	}
}

int usbaudio_control_callback(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len, usbd_control_complete_callback *complete) {
	(void)dev;
	(void)buf;
	(void)len;
	(void)complete;

	if((req->bmRequestType & 0x01) && req->bRequest == 0x0b && req->wIndex == 1) {
		if(req->wValue > 0) {
			// Begin sending isochronous data
			capture_start();
			led_on();
		} else {
			capture_stop();
			led_off();
		}
		sending_iso = req->wValue;
	}
	return 1;
}

static void usbaudio_set_config(usbd_device *dev, uint16_t wValue) {
	(void)wValue;

	usbd_ep_setup(dev, 0x01, USB_ENDPOINT_ATTR_BULK, MIDI_ENDP_SIZE, usbmidi_data_rx_cb);
	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_BULK, MIDI_ENDP_SIZE, NULL);

	usbd_ep_setup(dev, 0x82, USB_ENDPOINT_ATTR_ISOCHRONOUS, AS_ENDPOINT_BYTES, usbaudio_data_tx_cb);

	usbd_register_control_callback(
		dev,
		USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
		USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
		usbaudio_control_callback
	);
}

__attribute__((used)) void hard_fault_handler() {
	led_blink_forever();
}

__attribute__((used)) void usart3_isr(void) {
	midi_queue_poll();
}

static void midi_usart_setup(void) {
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_USART3);

	/* Setup GPIO pin GPIO_USART1_RE_TX on GPIO port A for transmit. */
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART3, 38400);
	usart_set_databits(USART3, 8);
	usart_set_stopbits(USART3, USART_STOPBITS_1);
	usart_set_parity(USART3, USART_PARITY_NONE);
	usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART3, USART_MODE_TX);
	usart_enable_tx_interrupt(USART3);
	nvic_enable_irq(NVIC_USART3_IRQ);

	/* Finally enable the USART. */
	usart_enable(USART3);
}

static void my_delay_1(void) {
	for(unsigned i = 0; i < 800000; i++) {
		__asm__("nop");
	}
}

__attribute__((used)) void usb_wakeup_isr(void) {
	usbd_poll(usbd_dev);
}

__attribute__((used)) void usb_lp_can_rx0_isr(void) {
	usbd_poll(usbd_dev);
}

static void setup_usb(void) {
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(
		GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO12
	);
	gpio_clear(GPIOA, GPIO12);
	my_delay_1();

	usbd_dev = usbd_init(
		&st_usbfs_v1_usb_driver, &dev_desc, &config,
		usb_strings, 2, usbd_control_buffer,
		sizeof(usbd_control_buffer)
	);
	usbd_register_set_config_callback(usbd_dev, usbaudio_set_config);

	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
	nvic_enable_irq(NVIC_USB_WAKEUP_IRQ);
}

int main(void) {
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	led_init();
	capture_setup();
	setup_usb();
	midi_usart_setup();

	/* Use interrupts for everything */
	while(1)
		__WFI();
}
