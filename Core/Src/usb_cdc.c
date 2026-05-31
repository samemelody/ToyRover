/**
 * @file    usb_cdc.c
 * @brief   USB CDC ACM device for STM32F103 — descriptors, EP0 enum, data xfer
 *
 * Works directly with HAL PCD. The F1 USB peripheral has 512 bytes of PMA
 * and supports 8 bidirectional endpoints.
 *
 * Endpoint assignment:
 *   EP0  — Control (handled by HAL PCD + our SetupStageCallback)
 *   EP1 IN  (0x81) — Bulk IN:  MCU → Host data
 *   EP1 OUT (0x01) — Bulk OUT: Host → MCU data
 *   EP2 IN  (0x82) — Interrupt IN: CDC notifications (SetLineCoding ack, etc.)
 *
 * PMA layout (512 bytes total):
 *   BTABLE:         0x000–0x03F  (64B, room for 8 EP descriptors)
 *   EP0 RX buffer:  0x040–0x07F  (64B)
 *   EP0 TX buffer:  0x080–0x0BF  (64B)
 *   EP1 OUT buffer: 0x0C0–0x0FF  (64B)
 *   EP1 IN buffer:  0x100–0x13F  (64B)
 *   EP2 IN buffer:  0x140–0x14F  (16B, notification only)
 */
#include "usb_cdc.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

extern PCD_HandleTypeDef hpcd_USB_FS;

/*===========================================================================
 * USB Standard Request codes
 *===========================================================================*/
#define REQ_GET_DESCRIPTOR        0x06
#define REQ_SET_ADDRESS           0x05
#define REQ_SET_CONFIGURATION     0x09
#define REQ_GET_CONFIGURATION     0x08
#define REQ_GET_STATUS            0x00
#define REQ_SET_FEATURE           0x03
#define REQ_CLEAR_FEATURE         0x01

/* Descriptor types */
#define DESC_DEVICE               0x01
#define DESC_CONFIGURATION        0x02
#define DESC_STRING               0x03
#define DESC_INTERFACE            0x04
#define DESC_ENDPOINT             0x05

/*===========================================================================
 * CDC ACM Class Request codes
 *===========================================================================*/
#define CDC_SET_LINE_CODING        0x20
#define CDC_GET_LINE_CODING        0x21
#define CDC_SET_CONTROL_LINE_STATE 0x22

/*===========================================================================
 * USB Descriptors
 *===========================================================================*/

/* ---- Device Descriptor ---- */
static const uint8_t CDC_DEVICE_DESC[] = {
    0x12,                      /* bLength            (18 bytes) */
    DESC_DEVICE,               /* bDescriptorType    (0x01)     */
    0x00, 0x02,                /* bcdUSB             2.00       */
    0x02,                      /* bDeviceClass       CDC        */
    0x00,                      /* bDeviceSubClass               */
    0x00,                      /* bDeviceProtocol              */
    64,                        /* bMaxPacketSize0    EP0=64B   */
    0x83, 0x04,                /* idVendor           0x0483    STMicro */
    0x40, 0x57,                /* idProduct          0x5740    ToyRover */
    0x00, 0x01,                /* bcdDevice          1.00       */
    1,                         /* iManufacturer                 */
    2,                         /* iProduct                      */
    3,                         /* iSerialNumber                 */
    0x01,                      /* bNumConfigurations            */
};

/* ---- Configuration Descriptor (CDC ACM + Data interface) ----
 * Total: 67 bytes
 *   Config   9B
 *   IAD      8B (Interface Association Descriptor, helps host bind CDC+Data)
 *   CDC I/F  9B
 *     Header     5B
 *     ACM        4B
 *     Union      5B
 *     Call Mgmt  5B
 *     EP2 IN     7B (Interrupt)
 *   Data I/F 9B
 *     EP1 OUT    7B (Bulk)
 *     EP1 IN     7B (Bulk)
 */
#define CDC_CFG_DESC_SIZE  67

static const uint8_t CDC_CONFIG_DESC[] = {
    /* ---- Configuration (9 bytes) ---- */
    0x09,                       /* bLength */
    DESC_CONFIGURATION,         /* bDescriptorType */
    CDC_CFG_DESC_SIZE, 0x00,    /* wTotalLength */
    0x02,                       /* bNumInterfaces    (CDC control + CDC data) */
    0x01,                       /* bConfigurationValue */
    0x00,                       /* iConfiguration */
    0xC0,                       /* bmAttributes       Bus powered, no wakeup */
    0x32,                       /* bMaxPower          100 mA */

    /* ---- IAD (8 bytes) — Interface Association Descriptor ---- */
    0x08,                       /* bLength */
    0x0B,                       /* bDescriptorType (IAD = 0x0B) */
    0x00,                       /* bFirstInterface   CDC control = 0 */
    0x02,                       /* bInterfaceCount    2 interfaces */
    0x02, 0x02, 0x01,          /* bFunctionClass/Sub/Pro   CDC ACM */
    0x00,                       /* iFunction */

    /* ---- Interface 0: CDC ACM Control (9 bytes) ---- */
    0x09,                       /* bLength */
    DESC_INTERFACE,             /* bDescriptorType */
    0x00,                       /* bInterfaceNumber   0 */
    0x00,                       /* bAlternateSetting */
    0x01,                       /* bNumEndpoints      1 (Interrupt IN) */
    0x02,                       /* bInterfaceClass    CDC Communication */
    0x02,                       /* bInterfaceSubClass Abstract Control Model */
    0x01,                       /* bInterfaceProtocol  AT commands V.250 */
    0x00,                       /* iInterface */

    /* ---- CDC Header (5 bytes) ---- */
    0x05,                       /* bLength */
    0x24,                       /* bDescriptorType    CS_INTERFACE */
    0x00,                       /* bDescriptorSubtype Header */
    0x10, 0x01,                 /* bcdCDC             1.10       */

    /* ---- CDC ACM (4 bytes) ---- */
    0x04,                       /* bLength */
    0x24,                       /* bDescriptorType    CS_INTERFACE */
    0x02,                       /* bDescriptorSubtype ACM */
    0x02,                       /* bmCapabilities     Line coding & serial state */

    /* ---- CDC Union (5 bytes) ---- */
    0x05,                       /* bLength */
    0x24,                       /* bDescriptorType    CS_INTERFACE */
    0x06,                       /* bDescriptorSubtype Union */
    0x00,                       /* bControlInterface   0 */
    0x01,                       /* bSubordinateInterface 1 (data) */

    /* ---- CDC Call Management (5 bytes) ---- */
    0x05,                       /* bLength */
    0x24,                       /* bDescriptorType    CS_INTERFACE */
    0x01,                       /* bDescriptorSubtype Call Management */
    0x00,                       /* bmCapabilities     No call mgmt */
    0x01,                       /* bDataInterface     1 */

    /* ---- EP2 IN: Interrupt (7 bytes) ---- */
    0x07,                       /* bLength */
    DESC_ENDPOINT,              /* bDescriptorType */
    0x82,                       /* bEndpointAddress   EP2 IN */
    0x03,                       /* bmAttributes       Interrupt */
    0x10, 0x00,                 /* wMaxPacketSize     16 bytes */
    0x10,                       /* bInterval          16 ms */

    /* ---- Interface 1: CDC ACM Data (9 bytes) ---- */
    0x09,                       /* bLength */
    DESC_INTERFACE,             /* bDescriptorType */
    0x01,                       /* bInterfaceNumber   1 */
    0x00,                       /* bAlternateSetting */
    0x02,                       /* bNumEndpoints      2 (Bulk IN + Bulk OUT) */
    0x0A,                       /* bInterfaceClass    CDC Data */
    0x00,                       /* bInterfaceSubClass */
    0x00,                       /* bInterfaceProtocol */
    0x00,                       /* iInterface */

    /* ---- EP1 OUT: Bulk (7 bytes) ---- */
    0x07,                       /* bLength */
    DESC_ENDPOINT,              /* bDescriptorType */
    0x01,                       /* bEndpointAddress   EP1 OUT */
    0x02,                       /* bmAttributes       Bulk */
    CDC_DATA_MAX_PACKET_SIZE, 0x00,  /* wMaxPacketSize 64B */
    0x00,                       /* bInterval          N/A for Bulk */

    /* ---- EP1 IN: Bulk (7 bytes) ---- */
    0x07,                       /* bLength */
    DESC_ENDPOINT,              /* bDescriptorType */
    0x81,                       /* bEndpointAddress   EP1 IN */
    0x02,                       /* bmAttributes       Bulk */
    CDC_DATA_MAX_PACKET_SIZE, 0x00,  /* wMaxPacketSize 64B */
    0x00,                       /* bInterval          N/A for Bulk */
};

/* ---- String Descriptors ---- */

/* Language ID */
static const uint8_t CDC_STRING_LANGID[] = {
    0x04, DESC_STRING, 0x09, 0x04,      /* English (US) */
};

/* Manufacturer */
static const uint8_t CDC_STRING_MFR[] = {
    0x0E, DESC_STRING,
    'T', 0, 'o', 0, 'y', 0, 'R', 0, 'o', 0, 'v', 0,
};

/* Product */
static const uint8_t CDC_STRING_PRODUCT[] = {
    0x14, DESC_STRING,
    'T', 0, 'o', 0, 'y', 0, 'R', 0, 'o', 0, 'v', 0,
    'e', 0, 'r', 0, ' ', 0,
};

/* Serial number */
static const uint8_t CDC_STRING_SERIAL[] = {
    0x0C, DESC_STRING,
    '0', 0, '0', 0, '0', 0, '1', 0, '0', 0,
};

/*===========================================================================
 * Internal state
 *===========================================================================*/

static usb_cdc_rx_cb_t  g_rx_cb;

static uint8_t  g_configured;           /* 1 = host has set configuration */
static uint8_t  g_tx_busy;              /* 1 = EP1 IN transfer in progress */
static uint8_t  g_tx_buf[CDC_DATA_MAX_PACKET_SIZE]; /* TX buffer */
static uint8_t  g_tx_len;
static uint8_t  g_pending_tx;           /* 1 = data queued while tx busy */
static uint8_t  g_pending_buf[CDC_DATA_MAX_PACKET_SIZE];
static uint8_t  g_pending_len;

/* Line coding state */
static uint8_t  g_line_coding[7];       /* dwDTERate(4) + bCharFormat(1)
                                           + bParityType(1) + bDataBits(1) */
/* EP1 OUT receive buffer */
static uint8_t  g_rx_buf[CDC_DATA_MAX_PACKET_SIZE];

/* Debug counters */
static volatile uint32_t g_dbg_resets;
static volatile uint32_t g_dbg_setups;
static volatile uint32_t g_dbg_dataout;

/*===========================================================================
 * Internal: send control data on EP0
 *===========================================================================*/
static void ep0_send_data(const uint8_t *data, uint16_t len)
{
    uint16_t send_len = (len > 64) ? 64 : len;

    /* Copy to EP0 TX PMA via HAL */
    HAL_PCD_EP_Transmit(&hpcd_USB_FS, 0x00, (uint8_t *)data, send_len);
}

static void ep0_send_stall(void)
{
    HAL_PCD_EP_SetStall(&hpcd_USB_FS, 0x00);
}

/*===========================================================================
 * Override weak HAL_PCDEx_SetConnectionState
 *
 * STM32F103 has NO software-controlled D+ pull-up. The pull-up is external
 * (1.5kΩ resistor from D+/PA12 to 3.3V — R10 on Blue Pill boards).
 * If your board uses a GPIO to control the pull-up, add that logic here.
 *===========================================================================*/
void HAL_PCDEx_SetConnectionState(PCD_HandleTypeDef *hpcd, uint8_t state)
{
    (void)hpcd;
    /* Many STM32F103 boards use a GPIO to control the D+ 1.5kΩ pull-up.
     * Common pins: PA8, PB14, PA15. Try each below.
     * If D+ is hardwired to 3.3V through R10, this is not needed —
     * but setting a GPIO high as a backup won't hurt. */
    if (state) {
        /* PA8 high → enable D+ pull-up (common on Blue Pill clones) */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    }
}

/*===========================================================================
 * USB Reset handler
 *===========================================================================*/
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    g_dbg_resets++;
    g_configured = 0;
    g_tx_busy    = 0;
    g_pending_tx = 0;

    /* Re-open EP0 after reset (HAL closes endpoints on reset) */
    HAL_PCD_EP_Open(hpcd, 0x00, 64, 0);  /* EP0 OUT, max 64 */
    HAL_PCD_EP_Open(hpcd, 0x80, 64, 0);  /* EP0 IN, max 64 */

    /* Open data endpoints */
    HAL_PCD_EP_Open(hpcd, 0x01, CDC_DATA_MAX_PACKET_SIZE, 2);   /* EP1 OUT Bulk */
    HAL_PCD_EP_Open(hpcd, 0x81, CDC_DATA_MAX_PACKET_SIZE, 2);   /* EP1 IN  Bulk */
    HAL_PCD_EP_Open(hpcd, 0x82, 16, 3);                          /* EP2 IN  Interrupt */

    /* Prepare to receive first packet on EP1 OUT */
    HAL_PCD_EP_Receive(hpcd, 0x01, g_rx_buf, CDC_DATA_MAX_PACKET_SIZE);
}

/*===========================================================================
 * EP0 Setup Packet handler
 *===========================================================================*/
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    g_dbg_setups++;
    uint8_t  bmReq  = hpcd->Setup[0];
    uint8_t  bReq   = hpcd->Setup[1];
    uint16_t wValue = (uint16_t)hpcd->Setup[2] | ((uint16_t)hpcd->Setup[3] << 8);
    uint16_t wIndex = (uint16_t)hpcd->Setup[4] | ((uint16_t)hpcd->Setup[5] << 8);
    uint16_t wLen   = (uint16_t)hpcd->Setup[6] | ((uint16_t)hpcd->Setup[7] << 8);

    (void)wLen;
    (void)wIndex;

    /*----------------------------------------------------------------
     * Standard Device Requests (bmReq type = 0x00, 0x80)
     *--------------------------------------------------------------*/

    if ((bmReq & 0x60) == 0x00) {
        /* ---- Standard request to Device ---- */
        switch (bReq) {

        case REQ_SET_ADDRESS:
            /* HAL handles actual address setting on status stage */
            HAL_PCD_EP_Transmit(hpcd, 0x00, NULL, 0);
            break;

        case REQ_SET_CONFIGURATION: {
            uint8_t cfg = (uint8_t)wValue;
            g_configured = (cfg != 0) ? 1 : 0;
            HAL_PCD_EP_Transmit(hpcd, 0x00, NULL, 0);
            break;
        }

        case REQ_GET_CONFIGURATION:
            ep0_send_data(&g_configured, 1);
            break;

        case REQ_GET_STATUS:
        case REQ_SET_FEATURE:
        case REQ_CLEAR_FEATURE:
            /* Accept silently */
            HAL_PCD_EP_Transmit(hpcd, 0x00, NULL, 0);
            break;

        case REQ_GET_DESCRIPTOR:
        default:
            ep0_send_stall();
            break;
        }
        return;
    }

    if ((bmReq & 0x60) == 0x20) {
        /* ---- Standard request to Interface ---- */
        if (bReq == REQ_GET_STATUS) {
            uint8_t zero[2] = {0, 0};
            ep0_send_data(zero, 2);
            return;
        }
        ep0_send_stall();
        return;
    }

    /*----------------------------------------------------------------
     * Class Requests (CDC ACM, bmReq type = 0x21, 0xA1)
     *--------------------------------------------------------------*/

    if (bmReq == 0x21) {
        /* ---- CDC class request to Interface ---- */
        switch (bReq) {

        case CDC_SET_LINE_CODING:
            /* wValue=0, wIndex=interface, wLen=7.
             * Host will send line coding in DATA OUT stage.
             * Store in g_line_coding when DataOutStageCallback fires for EP0. */
            break;  /* EP0 OUT data will arrive, handled in DataOutStageCallback */

        case CDC_SET_CONTROL_LINE_STATE:
            /* Host tells us DTR/RTS state — acknowledge */
            HAL_PCD_EP_Transmit(hpcd, 0x00, NULL, 0);
            break;

        default:
            ep0_send_stall();
            break;
        }
        return;
    }

    if (bmReq == 0xA1) {
        /* ---- CDC class request to Interface, device-to-host ---- */
        switch (bReq) {
        case CDC_GET_LINE_CODING:
            ep0_send_data(g_line_coding, 7);
            break;
        default:
            ep0_send_stall();
            break;
        }
        return;
    }

    /*----------------------------------------------------------------
     * Standard Device-to-Host Descriptor requests
     *--------------------------------------------------------------*/
    if (bmReq == 0x80) {
        /* Standard, device-to-host */
        uint8_t desc_type = (uint8_t)(wValue >> 8);
        uint8_t desc_idx  = (uint8_t)(wValue & 0xFF);

        switch (bReq) {
        case REQ_GET_DESCRIPTOR:
            switch (desc_type) {
            case DESC_DEVICE:
                ep0_send_data(CDC_DEVICE_DESC, sizeof(CDC_DEVICE_DESC));
                break;
            case DESC_CONFIGURATION:
                ep0_send_data(CDC_CONFIG_DESC, sizeof(CDC_CONFIG_DESC));
                break;
            case DESC_STRING:
                switch (desc_idx) {
                case 0: ep0_send_data(CDC_STRING_LANGID,  sizeof(CDC_STRING_LANGID));  break;
                case 1: ep0_send_data(CDC_STRING_MFR,     sizeof(CDC_STRING_MFR));     break;
                case 2: ep0_send_data(CDC_STRING_PRODUCT, sizeof(CDC_STRING_PRODUCT)); break;
                case 3: ep0_send_data(CDC_STRING_SERIAL,  sizeof(CDC_STRING_SERIAL));  break;
                default: ep0_send_stall(); break;
                }
                break;
            default:
                ep0_send_stall();
                break;
            }
            break;
        default:
            ep0_send_stall();
            break;
        }
        return;
    }

    /* Unhandled — stall */
    ep0_send_stall();
}

/*===========================================================================
 * Data Out Stage callback (EP0 OUT data + EP1 OUT data)
 *===========================================================================*/
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    g_dbg_dataout++;
    if (epnum == 0x00) {
        /* EP0 OUT — this is the data phase of a control write.
         * Currently only SET_LINE_CODING sends data to EP0 OUT.
         * Read the data from PMA into g_line_coding. */
        uint32_t len = HAL_PCD_EP_GetRxCount(hpcd, epnum);
        if (len > 0 && len <= 7) {
            /* Use HAL_PCD_EP_Receive with a temp buffer to read PMA */
            HAL_PCD_EP_Receive(hpcd, 0x00, g_line_coding, (uint32_t)len);
        }
        /* ACK the control transfer */
        HAL_PCD_EP_Transmit(hpcd, 0x00, NULL, 0);
        return;
    }

    if (epnum == 0x01) {
        /* EP1 OUT — data from host (phone) */
        uint32_t len = HAL_PCD_EP_GetRxCount(hpcd, epnum);
        if (len > 0 && len <= CDC_DATA_MAX_PACKET_SIZE) {
            /* Feed bytes to callback */
            if (g_rx_cb) {
                for (uint32_t i = 0; i < len; i++) {
                    g_rx_cb(g_rx_buf[i]);
                }
            }
        }
        /* Prepare to receive next packet */
        HAL_PCD_EP_Receive(hpcd, 0x01, g_rx_buf, CDC_DATA_MAX_PACKET_SIZE);
        return;
    }
}

/*===========================================================================
 * Data In Stage callback (EPx IN complete)
 *===========================================================================*/
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    (void)hpcd;

    if (epnum == 0x81) {
        /* EP1 IN transfer complete */
        g_tx_busy = 0;

        /* Flush any pending data */
        if (g_pending_tx) {
            g_pending_tx = 0;
            usb_cdc_send(g_pending_buf, g_pending_len);
        }
    }
}

/*===========================================================================
 * PMA buffer configuration
 *
 * STM32F103 has a 512-byte Packet Memory Area. We assign:
 *   EP0:   RX@0x040, TX@0x080
 *   EP1:   OUT@0x0C0, IN@0x100
 *   EP2:   IN@0x140
 *===========================================================================*/

#define PMA_EP0_RX_ADDR   0x040
#define PMA_EP0_TX_ADDR   0x080
#define PMA_EP1_OUT_ADDR  0x0C0
#define PMA_EP1_IN_ADDR   0x100
#define PMA_EP2_IN_ADDR   0x140

/*===========================================================================
 * Public API
 *===========================================================================*/

void usb_cdc_init(void)
{
    g_rx_cb      = NULL;
    g_configured  = 0;
    g_tx_busy    = 0;
    g_pending_tx = 0;

    /* Init line coding: 115200 8N1 */
    g_line_coding[0] = 0x00; g_line_coding[1] = 0xC2; g_line_coding[2] = 0x01; g_line_coding[3] = 0x00;  /* 115200 bps */
    g_line_coding[4] = 0x00;  /* 1 stop bit */
    g_line_coding[5] = 0x00;  /* No parity */
    g_line_coding[6] = 0x08;  /* 8 data bits */

    /* Init PA8 as GPIO output to control D+ pull-up (common on Blue Pill clones).
     * If your board has R10 hardwired to 3.3V instead, this is harmless. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_8;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  /* Pull D+ high */

    /* Configure PMA buffers */
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x00, PCD_SNG_BUF, PMA_EP0_RX_ADDR);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x80, PCD_SNG_BUF, PMA_EP0_TX_ADDR);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x01, PCD_SNG_BUF, PMA_EP1_OUT_ADDR);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x81, PCD_SNG_BUF, PMA_EP1_IN_ADDR);
    HAL_PCDEx_PMAConfig(&hpcd_USB_FS, 0x82, PCD_SNG_BUF, PMA_EP2_IN_ADDR);

    /* Open EP0 */
    HAL_PCD_EP_Open(&hpcd_USB_FS, 0x00, 64, 0);
    HAL_PCD_EP_Open(&hpcd_USB_FS, 0x80, 64, 0);

    /* Start USB device */
    HAL_PCD_Start(&hpcd_USB_FS);

    /* F103 errata: LP_MODE (bit 2) and FSUSP (bit 3) may be set by hardware
     * and not cleared by the HAL. FSUSP=Force Suspend kills all USB traffic.
     * Explicitly clear them to wake the peripheral. */
    USB->CNTR &= ~(USB_CNTR_FSUSP | USB_CNTR_LP_MODE);

    /* Verify CNTR after init */
    printf("[USB] init done, CNTR=0x%04X\r\n", USB->CNTR);
}

void usb_cdc_send(const uint8_t *data, uint8_t len)
{
    if (len == 0 || data == NULL) return;
    if (!g_configured) return;

    if (len > CDC_DATA_MAX_PACKET_SIZE) {
        len = CDC_DATA_MAX_PACKET_SIZE;
    }

    if (g_tx_busy) {
        /* Queue for when current transfer completes */
        memcpy(g_pending_buf, data, len);
        g_pending_len = len;
        g_pending_tx  = 1;
        return;
    }

    g_tx_busy = 1;
    memcpy(g_tx_buf, data, len);
    g_tx_len  = len;
    HAL_PCD_EP_Transmit(&hpcd_USB_FS, 0x81, g_tx_buf, g_tx_len);
}

void usb_cdc_set_rx_cb(usb_cdc_rx_cb_t cb)
{
    g_rx_cb = cb;
}

bool usb_cdc_is_ready(void)
{
    return (g_configured != 0);
}

void usb_cdc_print_debug(void)
{
    /* Keep clearing FSUSP+LP_MODE in case hardware re-sets them */
    if (USB->CNTR & (USB_CNTR_FSUSP | USB_CNTR_LP_MODE)) {
        USB->CNTR &= ~(USB_CNTR_FSUSP | USB_CNTR_LP_MODE);
    }

    uint16_t cntr = USB->CNTR;
    uint16_t istr = USB->ISTR;
    printf("[USB] cfg=%d rx=%lu stp=%lu dout=%lu | CNTR=0x%04X ISTR=0x%04X\r\n",
           g_configured,
           (unsigned long)g_dbg_resets,
           (unsigned long)g_dbg_setups,
           (unsigned long)g_dbg_dataout,
           cntr, istr);
}
