#ifndef USB_AUDIO_STRUCTS_H_
#define USB_AUDIO_STRUCTS_H_

struct usb_audio_iso_endpoint_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bmAttributes;
	uint8_t bLockDelayUnits;
	uint16_t wLockDelay;
} __attribute__((packed));

struct usb_audio_streaming_header_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bTerminalID;
	uint8_t bInterfaceDelay;
	uint16_t wFormat;
} __attribute__((packed));

struct usb_audio_streaming_format_type_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bFormatType;
	uint8_t bNrChannels;
	uint8_t bSubframeSize;
	uint8_t bBitResolution;
	uint8_t bSamFreqType;
} __attribute__((packed));

struct usb_audio_control_input_terminal_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bTerminalID;
	uint16_t wTerminalType;
	uint8_t bAssocTerminal;
	uint8_t bNrChannels;
	uint16_t wChannelConfig;
	uint8_t iChannelNames;
	uint8_t iTerminal;
} __attribute__((packed));

struct usb_audio_control_output_terminal_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t bTerminalID;
	uint16_t wTerminalType;
	uint8_t bAssocTerminal;
	uint8_t bSourceID;
	uint8_t iTerminal;
} __attribute__((packed));

#endif
