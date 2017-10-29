#include <string.h>
#include <unicore-mx/usbd/usbd.h>
#include <unicore-mx/usb/class/cdc.h>
#include <unicore-mx/stm32/gpio.h>
#include <unicore-mx/stm32/rcc.h>
#include "cdcacm.h"
#include "led.h"
#include "usart.h"

#define CDC_ENDP_SIZE 64

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDC_ENDP_SIZE,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = CDC_ENDP_SIZE,
	.bInterval = 1,
}};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1,
	 },
};

static const struct usb_interface_descriptor comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp,

	.extra = &cdcacm_functional_descriptors,
	.extra_len = sizeof(cdcacm_functional_descriptors),
}};

static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
	.altsetting = data_iface,
}};

const uint8_t *usb_strings_english[] = {
	(uint8_t *) "Yamaha",
	(uint8_t *) "TG100",
	(uint8_t *) "Suck my balls",
};

const struct usb_string_utf8_data usb_strings[] = {{
	.data = usb_strings_english,
	.count = 3,
	.lang_id = USB_LANGID_ENGLISH_UNITED_STATES
}, {
	.data = NULL
}};

static const struct usb_config_descriptor config[] = {{
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
	.string = usb_strings
}};

static const struct usb_device_descriptor desc = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0499,
	.idProduct = 0xffff,
	.bcdDevice = 0x0100,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,

	.config = config,
	.string = usb_strings
};

static usbd_device *usbd_dev;

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128] __attribute__((aligned(2)));

static void cdcacm_control_request(usbd_device *dev, uint8_t ep,
				const struct usb_setup_data *setup_data)
{
	(void) ep; /* assuming ep == 0 */

	const uint8_t bmReqMask = USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT;
	const uint8_t bmReqVal = USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE;

	if ((setup_data->bmRequestType & bmReqMask) != bmReqVal) {
		/* Pass on to usb stack internal */
		usbd_ep0_setup(dev, setup_data);
		return;
	}

	switch (setup_data->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		usbd_ep0_transfer(dev, setup_data, NULL, 0, NULL);
	return;
	case USB_CDC_REQ_SET_LINE_CODING:
		if (setup_data->wLength < sizeof(struct usb_cdc_line_coding)) {
			break;
		}

		/* Just read what ever host is sending and do the status stage */
		usbd_ep0_transfer(dev, setup_data, usbd_control_buffer,
			setup_data->wLength, NULL);
	return;
	}

	usbd_ep0_stall(dev);
}

static uint8_t writebuf[32768];
static uint16_t writebuf_top = 0, writebuf_bottom = 0;
static void writebuf_write(uint8_t *ptr, int len) {
	for(int i = 0; i < len; i++) {
		writebuf[writebuf_top] = ptr[i];
		writebuf_top++;
		if(writebuf_top >= sizeof(writebuf))
			writebuf_top = 0;
		if(writebuf_top == writebuf_bottom) {
			writebuf_top = writebuf_bottom = 0;
			led_on();
//			writebuf_bottom++;
//			if(writebuf_bottom >= sizeof(writebuf))
//				writebuf_bottom = 0;
		}
	}
}

static void tx_to_host(usbd_device *dev, void *data, size_t len);

static bool transmitting = false, initialized = false;
static uint8_t transmitbuf[CDC_ENDP_SIZE];
static void writebuf_transmit(usbd_device *dev) {
	int i;
	led_off();
	for(i = 0; i < 63; i++) {
		if(writebuf_bottom == writebuf_top)
			break;

		transmitbuf[i] = writebuf[writebuf_bottom];
		writebuf_bottom++;
		if(writebuf_bottom >= sizeof(writebuf))
			writebuf_bottom = 0;
	}
	if(i > 0) {
		tx_to_host(dev, transmitbuf, i);
	} else {
		transmitting = false;
	}
}

static int writebuf_len() {
	return writebuf_bottom <= writebuf_top ? writebuf_top - writebuf_bottom : (sizeof(writebuf) - writebuf_bottom) + writebuf_top;
}

static uint8_t bulk_buf[CDC_ENDP_SIZE];

static void cdcacm_data_rx_cb(usbd_device *dev,
	const usbd_transfer *_transfer, usbd_transfer_status status,
	usbd_urb_id urb_id);

static void cdcacm_data_tx_cb(usbd_device *dev,
	const usbd_transfer *_transfer, usbd_transfer_status status,
	usbd_urb_id urb_id);


static void rx_from_host(usbd_device *dev) {
	const usbd_transfer transfer = {
		.ep_type = USBD_EP_BULK,
		.ep_addr = 0x01,
		.ep_size = CDC_ENDP_SIZE,
		.ep_interval = USBD_INTERVAL_NA,
		.buffer = bulk_buf,
		.length = CDC_ENDP_SIZE,
		.flags = USBD_FLAG_SHORT_PACKET,
		.timeout = USBD_TIMEOUT_NEVER,
		.callback = cdcacm_data_rx_cb
	};

	usbd_transfer_submit(dev, &transfer);
}

static void tx_to_host(usbd_device *dev, void *data, size_t len) {
	const usbd_transfer transfer = {
		.ep_type = USBD_EP_BULK,
		.ep_addr = 0x82,
		.ep_size = CDC_ENDP_SIZE,
		.ep_interval = USBD_INTERVAL_NA,
		.buffer = data,
		.length = len,
		.flags = USBD_FLAG_SHORT_PACKET,
		.timeout = USBD_TIMEOUT_NEVER,
		.callback = cdcacm_data_tx_cb
	};

	debugf("tx %d %d  %d %d\n", len, writebuf_len(), writebuf_bottom, writebuf_top);
	usbd_transfer_submit(dev, &transfer);
	transmitting = true;
}

static void cdcacm_data_tx_cb(usbd_device *dev,
	const usbd_transfer *transfer, usbd_transfer_status status,
	usbd_urb_id urb_id) {
	(void)dev;
	(void)urb_id;
	(void)transfer;
	(void)status;

 	writebuf_transmit(dev);
}

static void cdcacm_data_rx_cb(usbd_device *dev,
	const usbd_transfer *transfer, usbd_transfer_status status,
	usbd_urb_id urb_id) {
	(void)status;
	(void)dev;
	(void)urb_id;
	(void)transfer;

	rx_from_host(dev);
}

static void cdcacm_set_config(usbd_device *dev, const struct usb_config_descriptor *cfg) {
	(void)cfg;
	usbd_ep_prepare(dev, 0x01, USBD_EP_BULK, CDC_ENDP_SIZE, USBD_INTERVAL_NA, USBD_EP_NONE);
	usbd_ep_prepare(dev, 0x82, USBD_EP_BULK, CDC_ENDP_SIZE, USBD_INTERVAL_NA, USBD_EP_NONE);
	usbd_ep_prepare(dev, 0x83, USBD_EP_INTERRUPT, 16, USBD_INTERVAL_NA, USBD_EP_NONE);

	initialized = true;
	writebuf_transmit(dev);
}

static const usbd_backend_config *cdcacm_target_usb_config(void) { return NULL; }

static void cdcacm_target_init(void) {
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_OTGFS);

	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO9 | GPIO11 | GPIO12);
}

void cdcacm_setup(void) {
	cdcacm_target_init();

	usbd_dev = usbd_init(USBD_STM32_OTG_FS, cdcacm_target_usb_config(), &desc, usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);
	usbd_register_setup_callback(usbd_dev, cdcacm_control_request);
}

void cdcacm_poll(void) {
	usbd_poll(usbd_dev, 0);
}

void cdcacm_write(uint8_t *buf, int length) {
	(void)buf;
	(void)length;

	if(!initialized) return;
	writebuf_write(buf, length);
	if(!transmitting) {
		if(writebuf_len() >= 63)
			writebuf_transmit(usbd_dev);
	}
}
