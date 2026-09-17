/*
 * PiStorm Classic / Raspberry Pi 3A+ bare-metal Paula -> USB Audio POC.
 *
 * Baseline transport: hardware-validated POC43 UVC DWC2 path; Paula source: CACHE-FIX2 milestone.
 * The Paula mirror, asynchronous Chip RAM cache, speculative render rollback,
 * recovery fades and mixer remain unchanged.  Only the final sink changes:
 * CPU1 renders 48 kHz stereo Paula PCM into a shared ring and CPU3 exposes it
 * as a driverless USB Audio Class 1 capture device.
 *
 * USB transport code is intentionally small and derives from the already
 * hardware-validated Emu68 UVC DWC2 POC.  No TinyUSB dependency is used.
 *
 * SPDX-License-Identifier: MPL-2.0
 */

#include <stdint.h>
#include "support.h"
#include "usb_audio_poc.h"
#include "paula/paula_live.h"
#include "paula/paula_mirror.h"
#include "paula_probe.h"
#include "paula/paula_replay.h"

#if defined(PISTORM_CLASSIC)

extern uint32_t set_power_state(uint32_t device_id, uint32_t state);

#define USB2_BASE               0xf2980000UL
#define USB_GAHBCFG             0x008
#define USB_GUSBCFG             0x00c
#define USB_GRSTCTL             0x010
#define USB_GINTSTS             0x014
#define USB_GINTMSK             0x018
#define USB_GRXSTSP             0x020
#define USB_GRXFSIZ             0x024
#define USB_GNPTXFSIZ           0x028
#define USB_GSNPSID             0x040
#define USB_DPTXFSIZ(n)         (0x100 + ((n) * 4))
#define USB_DCFG                0x800
#define USB_DCTL                0x804
#define USB_DSTS                0x808
#define USB_DIEPMSK             0x810
#define USB_DOEPMSK             0x814
#define USB_DAINT               0x818
#define USB_DAINTMSK            0x81c
#define USB_DIEPEMPMSK          0x834
#define USB_DIEPCTL(n)          (0x900 + ((n) * 0x20))
#define USB_DIEPINT(n)          (0x908 + ((n) * 0x20))
#define USB_DIEPTSIZ(n)         (0x910 + ((n) * 0x20))
#define USB_DTXFSTS(n)          (0x918 + ((n) * 0x20))
#define USB_DOEPCTL(n)          (0xb00 + ((n) * 0x20))
#define USB_DOEPINT(n)          (0xb08 + ((n) * 0x20))
#define USB_DOEPTSIZ(n)         (0xb10 + ((n) * 0x20))
#define USB_FIFO(n)             (0x1000 + ((n) * 0x1000))

#define USB_GAHBCFG_GLBL_INTR_EN        (1U << 0)
#define USB_GAHBCFG_DMA_EN              (1U << 5)
#define USB_GUSBCFG_FORCEDEVMODE        (1U << 30)
#define USB_GUSBCFG_FORCEHOSTMODE       (1U << 29)
#define USB_GUSBCFG_HNPCAP              (1U << 9)
#define USB_GUSBCFG_SRPCAP              (1U << 8)
#define USB_GUSBCFG_TOUTCAL_MASK        0x7U
#define USB_GRSTCTL_AHBIDLE             (1U << 31)
#define USB_GRSTCTL_TXFNUM_ALL          (0x10U << 6)
#define USB_GRSTCTL_TXFNUM(n)           (((uint32_t)(n) & 0x1fU) << 6)
#define USB_GRSTCTL_TXFFLSH             (1U << 5)
#define USB_GRSTCTL_RXFFLSH             (1U << 4)
#define USB_GRSTCTL_CSFTRST             (1U << 0)
#define USB_GINTSTS_OEPINT              (1U << 19)
#define USB_GINTSTS_IEPINT              (1U << 18)
#define USB_GINTSTS_ENUMDONE            (1U << 13)
#define USB_GINTSTS_USBRST              (1U << 12)
#define USB_GINTSTS_RXFLVL              (1U << 4)
#define USB_GINTSTS_CURMODE_HOST        (1U << 0)
#define USB_DCFG_DEVADDR_MASK           (0x7fU << 4)
#define USB_DCFG_DEVADDR(a)             (((uint32_t)(a) & 0x7fU) << 4)
#define USB_DCFG_DEVSPD_MASK            3U
#define USB_DCFG_DEVSPD_FS_HS_PHY       1U
#define USB_DCTL_SFTDISCON              (1U << 1)
#define USB_DSTS_ENUMSPD_MASK           (3U << 1)
#define USB_DSTS_ENUMSPD_HS             (0U << 1)
#define USB_DSTS_SOFFN_SHIFT            8U
#define USB_DAINT_INEP(n)               (1U << (n))
#define USB_DAINT_OUTEP(n)              (1U << ((n) + 16))
#define USB_DXEPCTL_EPENA               (1U << 31)
#define USB_DXEPCTL_EPDIS               (1U << 30)
#define USB_DXEPCTL_SODDFRM             (1U << 29)
#define USB_DXEPCTL_SEVNFRM             (1U << 28)
#define USB_DXEPCTL_CNAK                (1U << 26)
#define USB_DXEPCTL_SNAK                (1U << 27)
#define USB_DXEPCTL_TXFNUM(n)           (((uint32_t)(n) & 0xfU) << 22)
#define USB_DXEPCTL_STALL               (1U << 21)
#define USB_DXEPCTL_EPTYPE_ISOC         (1U << 18)
#define USB_DXEPCTL_USBACTEP            (1U << 15)
#define USB_DXEPCTL_MPS(n)              ((uint32_t)(n) & 0x7ffU)
#define USB_DXEPINT_SETUP               (1U << 3)
#define USB_DXEPINT_EPDISBLD            (1U << 1)
#define USB_DXEPINT_XFERCOMPL           (1U << 0)
#define USB_DXEPTSIZ_PKTCNT(n)          (((uint32_t)(n) & 0x3ffU) << 19)
#define USB_DXEPTSIZ_XFERSIZE(n)        ((uint32_t)(n) & 0x7ffffU)
#define USB_DXEPTSIZ_MC(n)              (((uint32_t)(n) & 3U) << 29)
#define USB_DIEPTSIZ0_PKTCNT(n)         (((uint32_t)(n) & 3U) << 19)
#define USB_DIEPTSIZ0_XFERSIZE(n)       ((uint32_t)(n) & 0x7fU)
#define USB_DOEPTSIZ0_SUPCNT(n)         (((uint32_t)(n) & 3U) << 29)
#define USB_DOEPTSIZ0_PKTCNT            (1U << 19)
#define USB_GRXSTS_PKTSTS(v)            (((v) >> 17) & 0xfU)
#define USB_GRXSTS_BYTECNT(v)           (((v) >> 4) & 0x7ffU)
#define USB_GRXSTS_EPNUM(v)             ((v) & 0xfU)
#define USB_PKTSTS_OUTRX                 2U
#define USB_PKTSTS_SETUPRX               6U

#define USB_REQ_GET_STATUS               0x00
#define USB_REQ_CLEAR_FEATURE            0x01
#define USB_REQ_SET_ADDRESS              0x05
#define USB_REQ_GET_DESCRIPTOR           0x06
#define USB_REQ_GET_CONFIGURATION        0x08
#define USB_REQ_SET_CONFIGURATION        0x09
#define USB_REQ_GET_INTERFACE            0x0a
#define USB_REQ_SET_INTERFACE            0x0b
#define USB_DT_DEVICE                    1
#define USB_DT_CONFIG                    2
#define USB_DT_STRING                    3
#define USB_DT_DEVICE_QUALIFIER          6

#define UAC_SET_CUR                      0x01
#define UAC_GET_CUR                      0x81
#define UAC_GET_MIN                      0x82
#define UAC_GET_MAX                      0x83
#define UAC_GET_RES                      0x84
#define UAC_EP_CS_SAMPLING_FREQ          0x01

#define USB_AUDIO_EP_IN                  1U
#define USB_AUDIO_RATE                   48000U
#define USB_AUDIO_CHANNELS               2U
#define USB_AUDIO_BYTES_PER_SAMPLE       2U
#define USB_AUDIO_NOMINAL_FRAMES         48U
#define USB_AUDIO_MAX_FRAMES             49U
#define USB_AUDIO_MAX_PACKET             (USB_AUDIO_MAX_FRAMES * USB_AUDIO_CHANNELS * USB_AUDIO_BYTES_PER_SAMPLE)
#define USB_AUDIO_RING_FRAMES            2048U
#define USB_AUDIO_RING_MASK              (USB_AUDIO_RING_FRAMES - 1U)
#define USB_AUDIO_TARGET_FILL            384U

#define WBVAL(x) ((uint8_t)((x) & 0xff)), ((uint8_t)(((x) >> 8) & 0xff))

struct usb_setup_packet {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
};

typedef struct {
    int16_t left;
    int16_t right;
} usb_audio_frame;

static usb_audio_frame audio_ring[USB_AUDIO_RING_FRAMES];
static volatile uint32_t audio_wr;
static volatile uint32_t audio_rd;
static volatile uint32_t audio_underruns;
static volatile uint32_t audio_overruns;
static volatile uint32_t audio_packets;

static volatile uint8_t usb_hw_up;
static volatile uint8_t usb_worker_enabled;
static volatile uint8_t usb_configured;
static volatile uint8_t usb_stream_alt;
static volatile uint8_t usb_ep1_busy;
static volatile uint8_t usb_ep0_out_kind;
static uint8_t audio_tx[USB_AUDIO_MAX_PACKET];

static uint8_t ep0_tx_buf[128];
static uint16_t ep0_tx_len;
static uint16_t ep0_tx_pos;

enum {
    EP0_IDLE = 0,
    EP0_IN_DATA,
    EP0_OUT_STATUS,
    EP0_OUT_DATA,
    EP0_IN_STATUS
};
static volatile uint8_t ep0_state;

enum {
    EP0_OUT_NONE = 0,
    EP0_OUT_SAMPLE_RATE = 1
};

static const uint8_t device_desc[] = {
    18, USB_DT_DEVICE,
    0x00, 0x02,
    0x00, 0x00, 0x00,
    64,
    0x25, 0x05,             /* experimental PiStorm VID used by the UVC POC */
    0xab, 0xa4,             /* separate experimental PID for Paula USB Audio */
    0x01, 0x00,
    1, 2, 3,
    1
};

static const uint8_t qualifier_desc[] = {
    10, USB_DT_DEVICE_QUALIFIER,
    0x00, 0x02,
    0x00, 0x00, 0x00,
    64, 1, 0
};

/* UAC1 capture-only topology:
 *   Line Input Terminal (ID 1) -> USB Streaming Output Terminal (ID 2)
 * Interface 1 alt 0 is idle; alt 1 enables one asynchronous ISO IN endpoint.
 */
static const uint8_t config_desc_template[] = {
    9, USB_DT_CONFIG, WBVAL(100), 2, 1, 0, 0x80, 50,

    /* Interface 0: AudioControl */
    9, 4, 0, 0, 0, 0x01, 0x01, 0x00, 0,

    /* Class-specific AC header, total class-specific AC length = 30 */
    9, 0x24, 0x01, 0x00, 0x01, WBVAL(30), 1, 1,

    /* Input Terminal ID 1: Line Connector, stereo L/R */
    12, 0x24, 0x02, 1, 0x03, 0x06, 0, 2, WBVAL(0x0003), 0, 0,

    /* Output Terminal ID 2: USB Streaming, source = terminal 1 */
    9, 0x24, 0x03, 2, 0x01, 0x01, 0, 1, 0,

    /* Interface 1 alt 0: zero bandwidth */
    9, 4, 1, 0, 0, 0x01, 0x02, 0x00, 0,

    /* Interface 1 alt 1: active AudioStreaming */
    9, 4, 1, 1, 1, 0x01, 0x02, 0x00, 0,

    /* AS general: terminal 2, PCM */
    7, 0x24, 0x01, 2, 1, 0x01, 0x00,

    /* Type I PCM: stereo, 16-bit, exactly 48 kHz */
    11, 0x24, 0x02, 1, 2, 2, 16, 1, 0x80, 0xbb, 0x00,

    /* EP1 IN: isochronous, asynchronous data endpoint.
     * wMaxPacketSize allows a 49-frame correction packet.  The POC forces
     * high-speed USB, so bInterval=4 is exactly one audio service interval per ms. */
    9, 5, 0x81, 0x05, WBVAL(USB_AUDIO_MAX_PACKET), 1, 0, 0,

    /* Class-specific isochronous endpoint, sampling-frequency control. */
    7, 0x25, 0x01, 0x01, 0, 0, 0
};

_Static_assert(sizeof(config_desc_template) == 100U, "UAC1 config descriptor size");

static const uint8_t str0[] = { 4, USB_DT_STRING, 0x09, 0x04 };
static const uint8_t str1[] = {
    16, USB_DT_STRING, 'P',0,'i',0,'S',0,'t',0,'o',0,'r',0,'m',0
};
static const uint8_t str2[] = {
    40, USB_DT_STRING,
    'P',0,'i',0,'S',0,'t',0,'o',0,'r',0,'m',0,' ',0,
    'P',0,'a',0,'u',0,'l',0,'a',0,' ',0,'A',0,'u',0,'d',0,'i',0,'o',0
};
static const uint8_t str3[] = {
    18, USB_DT_STRING, 'P',0,'A',0,'U',0,'L',0,'A',0,'0',0,'0',0,'1',0
};

static inline uint32_t rd(uint32_t off)
{
    volatile uint32_t *p = (volatile uint32_t *)(uintptr_t)(USB2_BASE + off);
    uint32_t v = *p;
    dsb();
    return LE32(v);
}

static inline void wr(uint32_t off, uint32_t v)
{
    volatile uint32_t *p = (volatile uint32_t *)(uintptr_t)(USB2_BASE + off);
    *p = LE32(v);
    dsb();
}

static inline uint64_t read_cntvct(void)
{
    uint64_t v;
    asm volatile("mrs %0, cntvct_el0" : "=r"(v));
    return v;
}

static inline uint64_t read_cntfrq(void)
{
    uint64_t v;
    asm volatile("mrs %0, cntfrq_el0" : "=r"(v));
    return v;
}

static void delay_ms(uint32_t ms)
{
    const uint64_t start = read_cntvct();
    const uint64_t ticks = (read_cntfrq() * ms) / 1000U;
    while ((read_cntvct() - start) < ticks)
        asm volatile("yield");
}

static int wait_mask(uint32_t off, uint32_t mask, uint32_t wanted, uint32_t loops)
{
    while (loops--) {
        if ((rd(off) & mask) == wanted)
            return 1;
    }
    return 0;
}

static void fifo_write(unsigned ep, const uint8_t *buf, uint32_t len)
{
    volatile uint32_t *fifo =
        (volatile uint32_t *)(uintptr_t)(USB2_BASE + USB_FIFO(ep));
    while (len) {
        uint32_t w = 0;
        uint32_t n = len > 4U ? 4U : len;
        for (uint32_t i = 0; i < n; ++i)
            w |= ((uint32_t)buf[i]) << (8U * i);
        *fifo = LE32(w);
        dsb();
        buf += n;
        len -= n;
    }
}

static void fifo_read(uint8_t *buf, uint32_t len)
{
    volatile uint32_t *fifo =
        (volatile uint32_t *)(uintptr_t)(USB2_BASE + USB_FIFO(0));
    while (len) {
        uint32_t w = LE32(*fifo);
        uint32_t n = len > 4U ? 4U : len;
        dsb();
        for (uint32_t i = 0; i < n; ++i)
            *buf++ = (uint8_t)(w >> (8U * i));
        len -= n;
    }
}

static void drain_fifo(uint32_t len)
{
    uint8_t tmp[16];
    while (len) {
        uint32_t n = len < sizeof(tmp) ? len : (uint32_t)sizeof(tmp);
        fifo_read(tmp, n);
        len -= n;
    }
}

static void flush_fifos(void)
{
    wr(USB_GRSTCTL, USB_GRSTCTL_TXFNUM_ALL |
                     USB_GRSTCTL_TXFFLSH |
                     USB_GRSTCTL_RXFFLSH);
    (void)wait_mask(USB_GRSTCTL,
                    USB_GRSTCTL_TXFFLSH | USB_GRSTCTL_RXFFLSH,
                    0, 1000000U);
}

static void ep0_arm_setup(void)
{
    wr(USB_DOEPTSIZ(0), USB_DOEPTSIZ0_SUPCNT(3) |
                         USB_DOEPTSIZ0_PKTCNT | 24U);
    wr(USB_DOEPCTL(0), USB_DXEPCTL_EPENA | USB_DXEPCTL_CNAK);
}

static void ep0_arm_out(uint16_t len)
{
    wr(USB_DOEPTSIZ(0), USB_DOEPTSIZ0_PKTCNT | (uint32_t)len);
    wr(USB_DOEPCTL(0), USB_DXEPCTL_EPENA | USB_DXEPCTL_CNAK);
}

static void ep0_send_next_chunk(void)
{
    uint16_t remain;
    uint16_t chunk;

    if (ep0_tx_pos >= ep0_tx_len)
        return;

    remain = (uint16_t)(ep0_tx_len - ep0_tx_pos);
    chunk = remain > 64U ? 64U : remain;

    wr(USB_DIEPTSIZ(0), USB_DIEPTSIZ0_PKTCNT(1) |
                         USB_DIEPTSIZ0_XFERSIZE(chunk));
    wr(USB_DIEPCTL(0), USB_DXEPCTL_EPENA | USB_DXEPCTL_CNAK);
    fifo_write(0, &ep0_tx_buf[ep0_tx_pos], chunk);
    ep0_tx_pos = (uint16_t)(ep0_tx_pos + chunk);
}

static void ep0_send(const uint8_t *buf, uint16_t len)
{
    if (len > sizeof(ep0_tx_buf))
        len = sizeof(ep0_tx_buf);
    for (uint16_t i = 0; i < len; ++i)
        ep0_tx_buf[i] = buf[i];
    ep0_tx_len = len;
    ep0_tx_pos = 0;
    if (len)
        ep0_send_next_chunk();
}

static void ep0_zlp(void)
{
    wr(USB_DIEPTSIZ(0), USB_DIEPTSIZ0_PKTCNT(1));
    wr(USB_DIEPCTL(0), USB_DXEPCTL_EPENA | USB_DXEPCTL_CNAK);
}

static void ep0_stall(void)
{
    wr(USB_DIEPCTL(0), rd(USB_DIEPCTL(0)) | USB_DXEPCTL_STALL);
    wr(USB_DOEPCTL(0), rd(USB_DOEPCTL(0)) | USB_DXEPCTL_STALL);
}

static void set_address_now(uint8_t addr)
{
    uint32_t v = rd(USB_DCFG);
    v &= ~USB_DCFG_DEVADDR_MASK;
    v |= USB_DCFG_DEVADDR(addr);
    wr(USB_DCFG, v);
}

static int16_t float_to_s16(float sample)
{
    if (sample > 1.0f) sample = 1.0f;
    if (sample < -1.0f) sample = -1.0f;
    if (sample >= 0.0f)
        return (int16_t)(sample * 32767.0f);
    return (int16_t)(sample * 32768.0f);
}

static uint32_t ring_count(void)
{
    uint32_t wrp = __atomic_load_n(&audio_wr, __ATOMIC_ACQUIRE);
    uint32_t rdp = __atomic_load_n(&audio_rd, __ATOMIC_ACQUIRE);
    uint32_t n = wrp - rdp;
    return n > USB_AUDIO_RING_FRAMES ? USB_AUDIO_RING_FRAMES : n;
}

static void ring_push(int16_t left, int16_t right)
{
    /* Always publish the newest Paula sample.  If USB is not streaming the
     * producer intentionally overwrites old history; when capture starts the
     * consumer jumps to a small recent pre-roll rather than stale audio. */
    uint32_t wrp = __atomic_load_n(&audio_wr, __ATOMIC_RELAXED);
    audio_ring[wrp & USB_AUDIO_RING_MASK].left = left;
    audio_ring[wrp & USB_AUDIO_RING_MASK].right = right;
    __atomic_store_n(&audio_wr, wrp + 1U, __ATOMIC_RELEASE);
}

static unsigned ring_pop_packet(uint8_t *dst)
{
    uint32_t fill = ring_count();
    unsigned wanted = USB_AUDIO_NOMINAL_FRAMES;

    /* Keep the independently-clocked 48 kHz Paula producer close to the
     * target ring occupancy.  Full-Speed UAC1 normally carries 48 stereo
     * frames per 1 ms USB frame; occasional 47/49-frame packets absorb the
     * small clock drift without resampling. */
    if (fill > USB_AUDIO_TARGET_FILL + 96U)
        wanted = 49U;
    else if (fill < USB_AUDIO_TARGET_FILL - 96U && fill >= 47U)
        wanted = 47U;

    uint32_t rdp = __atomic_load_n(&audio_rd, __ATOMIC_RELAXED);
    uint32_t wrp = __atomic_load_n(&audio_wr, __ATOMIC_ACQUIRE);
    uint32_t available = wrp - rdp;
    if (available > USB_AUDIO_RING_FRAMES) {
        /* Producer lapped the USB consumer: discard obsolete history and
         * resume from the newest ring window. */
        rdp = wrp - USB_AUDIO_RING_FRAMES;
        available = USB_AUDIO_RING_FRAMES;
        __atomic_add_fetch(&audio_overruns, 1U, __ATOMIC_RELAXED);
    }

    unsigned copied = available < wanted ? (unsigned)available : wanted;

    for (unsigned i = 0; i < wanted; ++i) {
        int16_t l = 0, r = 0;
        if (i < copied) {
            usb_audio_frame f = audio_ring[(rdp + i) & USB_AUDIO_RING_MASK];
            l = f.left;
            r = f.right;
        }
        dst[i * 4U + 0U] = (uint8_t)l;
        dst[i * 4U + 1U] = (uint8_t)((uint16_t)l >> 8);
        dst[i * 4U + 2U] = (uint8_t)r;
        dst[i * 4U + 3U] = (uint8_t)((uint16_t)r >> 8);
    }

    if (copied)
        __atomic_store_n(&audio_rd, rdp + copied, __ATOMIC_RELEASE);
    if (copied < wanted)
        __atomic_add_fetch(&audio_underruns, 1U, __ATOMIC_RELAXED);

    return wanted * 4U;
}

/*
 * UAC streaming endpoints are opened and closed through SET_INTERFACE.
 * Treat every alt=0 -> alt=1 transition as a real endpoint lifecycle, not
 * merely a software flag change.  This mirrors the behaviour of mature USB
 * device stacks: quiesce the endpoint, wait for disable acknowledgement,
 * flush its dedicated Tx FIFO, clear stale interrupt/transfer state, then
 * rebuild DIEPCTL from scratch on the next start.
 */
static void flush_audio_tx_fifo(void)
{
    wr(USB_GRSTCTL, USB_GRSTCTL_TXFNUM(USB_AUDIO_EP_IN) |
                     USB_GRSTCTL_TXFFLSH);
    (void)wait_mask(USB_GRSTCTL, USB_GRSTCTL_TXFFLSH, 0, 1000000U);
}

static void audio_stream_stop(void)
{
    uint32_t ctl = rd(USB_DIEPCTL(USB_AUDIO_EP_IN));

    /* Stop accepting new IN traffic first.  If a transfer is active, ask
     * DWC2 to disable it and wait (bounded) for EPDISD. */
    if (ctl & USB_DXEPCTL_EPENA) {
        wr(USB_DIEPINT(USB_AUDIO_EP_IN), USB_DXEPINT_EPDISBLD);
        wr(USB_DIEPCTL(USB_AUDIO_EP_IN),
           ctl | USB_DXEPCTL_SNAK | USB_DXEPCTL_EPDIS);
        (void)wait_mask(USB_DIEPINT(USB_AUDIO_EP_IN),
                        USB_DXEPINT_EPDISBLD,
                        USB_DXEPINT_EPDISBLD, 1000000U);
    } else {
        /* Even an idle endpoint may retain a stale NAK/transfer state after
         * a host closes a stream. */
        wr(USB_DIEPCTL(USB_AUDIO_EP_IN), ctl | USB_DXEPCTL_SNAK);
    }

    wr(USB_DIEPEMPMSK, rd(USB_DIEPEMPMSK) & ~(1U << USB_AUDIO_EP_IN));
    wr(USB_DIEPINT(USB_AUDIO_EP_IN), 0xffffffffU);
    wr(USB_DIEPTSIZ(USB_AUDIO_EP_IN), 0U);
    flush_audio_tx_fifo();

    usb_ep1_busy = 0;
}

static void audio_stream_start(void)
{
    /* Make reopen idempotent: alt=1 after an imperfect previous close still
     * starts from the same clean endpoint/FIFO state. */
    audio_stream_stop();

    uint32_t wrp = __atomic_load_n(&audio_wr, __ATOMIC_ACQUIRE);
    uint32_t preroll = wrp < USB_AUDIO_TARGET_FILL ? wrp : USB_AUDIO_TARGET_FILL;
    __atomic_store_n(&audio_rd, wrp - preroll, __ATOMIC_RELEASE);

    wr(USB_DIEPINT(USB_AUDIO_EP_IN), 0xffffffffU);
    wr(USB_DIEPTSIZ(USB_AUDIO_EP_IN), 0U);

    const uint32_t ctl = USB_DXEPCTL_MPS(USB_AUDIO_MAX_PACKET) |
                         USB_DXEPCTL_USBACTEP |
                         USB_DXEPCTL_EPTYPE_ISOC |
                         USB_DXEPCTL_TXFNUM(USB_AUDIO_EP_IN) |
                         USB_DXEPCTL_SEVNFRM;
    wr(USB_DIEPCTL(USB_AUDIO_EP_IN), ctl);
    usb_ep1_busy = 0;
}

static int start_audio_in(void)
{
    if (!usb_configured || usb_stream_alt != 1U || usb_ep1_busy)
        return 0;

    const unsigned len = ring_pop_packet(audio_tx);
    uint32_t ctl = rd(USB_DIEPCTL(USB_AUDIO_EP_IN));
    const uint32_t current_frame = (rd(USB_DSTS) >> USB_DSTS_SOFFN_SHIFT) & 1U;

    usb_ep1_busy = 1;
    wr(USB_DIEPINT(USB_AUDIO_EP_IN), 0xffffffffU);
    wr(USB_DIEPTSIZ(USB_AUDIO_EP_IN),
       USB_DXEPTSIZ_PKTCNT(1) | USB_DXEPTSIZ_XFERSIZE(len));

    ctl &= ~(USB_DXEPCTL_SEVNFRM | USB_DXEPCTL_SODDFRM);
    /* Arm the next (micro-)frame, not the one currently being serviced. */
    ctl |= current_frame ? USB_DXEPCTL_SEVNFRM : USB_DXEPCTL_SODDFRM;
    wr(USB_DIEPCTL(USB_AUDIO_EP_IN), ctl | USB_DXEPCTL_EPENA | USB_DXEPCTL_CNAK);
    fifo_write(USB_AUDIO_EP_IN, audio_tx, len);
    return 1;
}

static void reset_stream_state(void)
{
    usb_configured = 0;
    usb_stream_alt = 0;
    usb_ep1_busy = 0;
    usb_ep0_out_kind = EP0_OUT_NONE;
    ep0_state = EP0_IDLE;
    ep0_tx_len = ep0_tx_pos = 0;
    __atomic_store_n(&audio_rd, __atomic_load_n(&audio_wr, __ATOMIC_ACQUIRE), __ATOMIC_RELEASE);
}

static void on_reset(void)
{
    reset_stream_state();
    set_address_now(0);
    flush_fifos();
    wr(USB_DIEPINT(0), 0xffffffffU);
    wr(USB_DOEPINT(0), 0xffffffffU);
    wr(USB_DIEPINT(USB_AUDIO_EP_IN), 0xffffffffU);
    wr(USB_DAINTMSK,
       USB_DAINT_INEP(0) | USB_DAINT_INEP(USB_AUDIO_EP_IN) | USB_DAINT_OUTEP(0));
    ep0_arm_setup();
}

static void on_enum_done(void)
{
    uint32_t spd = rd(USB_DSTS) & USB_DSTS_ENUMSPD_MASK;
    kprintf("[USB-AUDIO] ENUMDONE speed=%08x\n", spd);
    ep0_arm_setup();
}

static void put_rate48(uint8_t out[3])
{
    out[0] = 0x80;
    out[1] = 0xbb;
    out[2] = 0x00;
}

static void handle_setup(const struct usb_setup_packet *r)
{
    const uint8_t *data = 0;
    uint16_t len = 0;
    uint8_t tmp[128];
    static const uint8_t zero2[2] = {0,0};

    if ((r->bmRequestType & 0x80U) && r->bRequest == USB_REQ_GET_DESCRIPTOR) {
        uint8_t type = (uint8_t)(r->wValue >> 8);
        uint8_t idx = (uint8_t)r->wValue;
        if (type == USB_DT_DEVICE) {
            data = device_desc; len = sizeof(device_desc);
        } else if (type == USB_DT_DEVICE_QUALIFIER) {
            data = qualifier_desc; len = sizeof(qualifier_desc);
        } else if (type == USB_DT_CONFIG) {
            for (uint32_t i = 0; i < sizeof(config_desc_template); ++i)
                tmp[i] = config_desc_template[i];
            data = tmp; len = sizeof(config_desc_template);
        } else if (type == USB_DT_STRING) {
            if (idx == 0) { data = str0; len = sizeof(str0); }
            else if (idx == 1) { data = str1; len = sizeof(str1); }
            else if (idx == 2) { data = str2; len = sizeof(str2); }
            else if (idx == 3) { data = str3; len = sizeof(str3); }
        }
        if (!data) { ep0_stall(); return; }
        if (len > r->wLength) len = r->wLength;
        ep0_state = EP0_IN_DATA;
        ep0_send(data, len);
        return;
    }

    if (r->bmRequestType == 0x00 && r->bRequest == USB_REQ_SET_ADDRESS) {
        set_address_now((uint8_t)(r->wValue & 0x7fU));
        ep0_state = EP0_IN_STATUS;
        ep0_zlp();
        return;
    }

    if (r->bmRequestType == 0x00 && r->bRequest == USB_REQ_SET_CONFIGURATION) {
        audio_stream_stop();
        usb_configured = r->wValue ? 1U : 0U;
        usb_stream_alt = 0;
        ep0_state = EP0_IN_STATUS;
        ep0_zlp();
        return;
    }

    if (r->bmRequestType == 0x80 && r->bRequest == USB_REQ_GET_CONFIGURATION) {
        tmp[0] = usb_configured ? 1U : 0U;
        ep0_state = EP0_IN_DATA;
        ep0_send(tmp, r->wLength < 1U ? r->wLength : 1U);
        return;
    }

    if ((r->bmRequestType & 0x7fU) == 0x01 && r->bRequest == USB_REQ_SET_INTERFACE) {
        const uint8_t intf = (uint8_t)r->wIndex;
        const uint8_t alt = (uint8_t)r->wValue;
        if (intf == 0U && alt == 0U) {
            ep0_state = EP0_IN_STATUS;
            ep0_zlp();
            return;
        }
        if (intf == 1U && (alt == 0U || alt == 1U) && usb_configured) {
            if (alt == 1U) {
                audio_stream_start();
                usb_stream_alt = 1U;
            } else {
                /* Clear the software alt state before touching hardware so
                 * usb_poll() cannot re-arm EP1 during teardown. */
                usb_stream_alt = 0U;
                audio_stream_stop();
            }
            ep0_state = EP0_IN_STATUS;
            ep0_zlp();
            return;
        }
    }

    if ((r->bmRequestType & 0x7fU) == 0x01 && r->bRequest == USB_REQ_GET_INTERFACE) {
        const uint8_t intf = (uint8_t)r->wIndex;
        if (intf <= 1U) {
            tmp[0] = (intf == 1U) ? usb_stream_alt : 0U;
            ep0_state = EP0_IN_DATA;
            ep0_send(tmp, r->wLength < 1U ? r->wLength : 1U);
            return;
        }
    }

    if ((r->bmRequestType & 0x7fU) == 0x00 && r->bRequest == USB_REQ_GET_STATUS) {
        ep0_state = EP0_IN_DATA;
        ep0_send(zero2, r->wLength < 2U ? r->wLength : 2U);
        return;
    }

    if ((r->bmRequestType & 0x7fU) == 0x02 &&
        r->bRequest == USB_REQ_CLEAR_FEATURE && r->wValue == 0U) {
        ep0_state = EP0_IN_STATUS;
        ep0_zlp();
        return;
    }

    /* UAC1 endpoint sampling-frequency control for EP1 IN. */
    if ((r->wIndex & 0xffU) == 0x81U &&
        ((r->wValue >> 8) & 0xffU) == UAC_EP_CS_SAMPLING_FREQ) {
        if (r->bmRequestType == 0x22 && r->bRequest == UAC_SET_CUR && r->wLength == 3U) {
            usb_ep0_out_kind = EP0_OUT_SAMPLE_RATE;
            ep0_state = EP0_OUT_DATA;
            ep0_arm_out(3);
            return;
        }
        if (r->bmRequestType == 0xa2 &&
            (r->bRequest == UAC_GET_CUR || r->bRequest == UAC_GET_MIN ||
             r->bRequest == UAC_GET_MAX || r->bRequest == UAC_GET_RES)) {
            put_rate48(tmp);
            if (r->bRequest == UAC_GET_RES) {
                tmp[0] = 0; tmp[1] = 0; tmp[2] = 0;
            }
            ep0_state = EP0_IN_DATA;
            ep0_send(tmp, r->wLength < 3U ? r->wLength : 3U);
            return;
        }
    }

    ep0_stall();
}

static void poll_rx(void)
{
    while (rd(USB_GINTSTS) & USB_GINTSTS_RXFLVL) {
        uint32_t st = rd(USB_GRXSTSP);
        uint32_t pkt = USB_GRXSTS_PKTSTS(st);
        uint32_t len = USB_GRXSTS_BYTECNT(st);
        uint32_t ep = USB_GRXSTS_EPNUM(st);

        if (pkt == USB_PKTSTS_SETUPRX && ep == 0U && len == 8U) {
            uint8_t raw[8];
            struct usb_setup_packet r;
            fifo_read(raw, 8);
            r.bmRequestType = raw[0];
            r.bRequest = raw[1];
            r.wValue = (uint16_t)(raw[2] | ((uint16_t)raw[3] << 8));
            r.wIndex = (uint16_t)(raw[4] | ((uint16_t)raw[5] << 8));
            r.wLength = (uint16_t)(raw[6] | ((uint16_t)raw[7] << 8));
            handle_setup(&r);
            continue;
        }

        if (pkt == USB_PKTSTS_OUTRX) {
            if (len == 0U) {
                if (ep == 0U && ep0_state == EP0_OUT_STATUS)
                    ep0_state = EP0_IDLE;
                continue;
            }
            if (ep == 0U && usb_ep0_out_kind == EP0_OUT_SAMPLE_RATE && len == 3U) {
                uint8_t rate[3];
                fifo_read(rate, 3);
                /* This POC is fixed at 48 kHz.  Accept Windows' SET_CUR only
                 * when it selects the one advertised rate. */
                usb_ep0_out_kind = EP0_OUT_NONE;
                if (rate[0] == 0x80 && rate[1] == 0xbb && rate[2] == 0x00) {
                    ep0_state = EP0_IN_STATUS;
                    ep0_zlp();
                } else {
                    ep0_stall();
                }
            } else {
                drain_fifo(len);
            }
        }
    }
}

static void poll_epints(void)
{
    uint32_t daint = rd(USB_DAINT);

    if (daint & USB_DAINT_INEP(0)) {
        uint32_t i = rd(USB_DIEPINT(0));
        if (i) wr(USB_DIEPINT(0), i);
        if (i & USB_DXEPINT_XFERCOMPL) {
            if (ep0_state == EP0_IN_STATUS) {
                ep0_state = EP0_IDLE;
                ep0_arm_setup();
            } else if (ep0_state == EP0_IN_DATA) {
                if (ep0_tx_pos < ep0_tx_len)
                    ep0_send_next_chunk();
                else {
                    ep0_state = EP0_OUT_STATUS;
                    ep0_arm_out(0);
                }
            }
        }
    }

    if (daint & USB_DAINT_OUTEP(0)) {
        uint32_t i = rd(USB_DOEPINT(0));
        if (i) wr(USB_DOEPINT(0), i);
        if ((i & USB_DXEPINT_XFERCOMPL) && ep0_state == EP0_OUT_STATUS) {
            ep0_state = EP0_IDLE;
            ep0_arm_setup();
        }
    }

    if (daint & USB_DAINT_INEP(USB_AUDIO_EP_IN)) {
        uint32_t i = rd(USB_DIEPINT(USB_AUDIO_EP_IN));
        if (i) wr(USB_DIEPINT(USB_AUDIO_EP_IN), i);
        if (i & USB_DXEPINT_XFERCOMPL) {
            usb_ep1_busy = 0;
            __atomic_add_fetch(&audio_packets, 1U, __ATOMIC_RELAXED);
            (void)start_audio_in();
        }
    }
}

static void usb_poll(void)
{
    if (!usb_hw_up)
        return;

    uint32_t g = rd(USB_GINTSTS);
    if (g & USB_GINTSTS_USBRST) {
        wr(USB_GINTSTS, USB_GINTSTS_USBRST);
        on_reset();
    }
    if (g & USB_GINTSTS_ENUMDONE) {
        wr(USB_GINTSTS, USB_GINTSTS_ENUMDONE);
        on_enum_done();
    }
    if (g & USB_GINTSTS_RXFLVL)
        poll_rx();
    if (g & (USB_GINTSTS_IEPINT | USB_GINTSTS_OEPINT))
        poll_epints();

    if (usb_configured && usb_stream_alt == 1U && !usb_ep1_busy)
        (void)start_audio_in();
}

static int hw_init(void)
{
    uint32_t v;
    usb_worker_enabled = 0;
    (void)set_power_state(3, 3);
    delay_ms(20);

    v = rd(USB_GSNPSID);
    kprintf("[USB-AUDIO] GSNPSID=%08x\n", v);
    if ((v & 0xffff0000U) != 0x4f540000U) {
        kprintf("[USB-AUDIO] no Synopsys OTG core\n");
        return 0;
    }

    wr(USB_DCTL, rd(USB_DCTL) | USB_DCTL_SFTDISCON);
    v = rd(USB_GUSBCFG);
    v &= ~(USB_GUSBCFG_FORCEHOSTMODE | USB_GUSBCFG_HNPCAP |
           USB_GUSBCFG_SRPCAP | USB_GUSBCFG_TOUTCAL_MASK);
    v |= USB_GUSBCFG_FORCEDEVMODE | 7U;
    wr(USB_GUSBCFG, v);
    delay_ms(25);

    if (!wait_mask(USB_GRSTCTL, USB_GRSTCTL_AHBIDLE,
                   USB_GRSTCTL_AHBIDLE, 5000000U))
        return 0;
    wr(USB_GRSTCTL, USB_GRSTCTL_CSFTRST);
    if (!wait_mask(USB_GRSTCTL, USB_GRSTCTL_CSFTRST, 0, 5000000U))
        return 0;
    delay_ms(10);

    v = rd(USB_GUSBCFG);
    v &= ~USB_GUSBCFG_FORCEHOSTMODE;
    v |= USB_GUSBCFG_FORCEDEVMODE;
    wr(USB_GUSBCFG, v);
    delay_ms(25);
    if (rd(USB_GINTSTS) & USB_GINTSTS_CURMODE_HOST)
        return 0;

    v = rd(USB_GAHBCFG);
    v &= ~(USB_GAHBCFG_DMA_EN | USB_GAHBCFG_GLBL_INTR_EN);
    wr(USB_GAHBCFG, v);
    wr(USB_GINTMSK, 0);

    /* EP1 only needs one ~200-byte audio packet per millisecond. */
    wr(USB_GRXFSIZ, 256U);
    wr(USB_GNPTXFSIZ, (128U << 16) | 256U);
    wr(USB_DPTXFSIZ(USB_AUDIO_EP_IN), (128U << 16) | 384U);
    flush_fifos();

    v = rd(USB_DCFG);
    v &= ~(USB_DCFG_DEVADDR_MASK | USB_DCFG_DEVSPD_MASK);
    /* Diagnostic: force Full Speed while preserving the known-good UVC DWC2 bring-up. */
    v |= 1U; /* Full Speed using the HS PHY: simplify UAC1 ISO scheduling */
    wr(USB_DCFG, v);
    wr(USB_DIEPMSK, USB_DXEPINT_XFERCOMPL);
    wr(USB_DOEPMSK, USB_DXEPINT_XFERCOMPL | USB_DXEPINT_SETUP);

    on_reset();
    usb_hw_up = 1;
    wr(USB_DCTL, rd(USB_DCTL) & ~USB_DCTL_SFTDISCON);
    return 1;
}

/* ------------------------------------------------------------------------- */
/* Validated Paula CACHE-FIX2 producer path.                                 */
/* ------------------------------------------------------------------------- */

typedef struct {
    uint32_t loc[4];
    uint16_t len[4];
    uint16_t dma;
} poc_live_state;

static void poc_prefetch_channel(const poc_live_state *state, unsigned ch)
{
    const uint32_t bytes = (state->len[ch] ? state->len[ch] : 65536u) * 2u;
    paula_mirror_prefetch(state->loc[ch], bytes < 1024u ? bytes : 1024u);
}

static void poc_apply_live_event(poc_live_state *state,
                                 const paula_mirror_event *event)
{
    if (event->width != 1u && event->width != 2u) {
        paula_probe_add(PP_IGNORED,1);
        return;
    }
    if (event->width == 2u && (event->address & 1u)) {
        paula_probe_add(PP_IGNORED,1);
        return;
    }

    const uint32_t r = event->address & 0xffeu;
    const uint16_t value = event->width == 1u
        ? (uint16_t)((event->value & 255u) * 0x0101u)
        : event->value;

    if (r >= 0xa0u && r <= 0xdau) {
        const unsigned ch = (r - 0xa0u) >> 4;
        const unsigned reg = (r - 0xa0u) & 0x0fu;
        if (ch < 4u) {
            if (reg == 0u)
                state->loc[ch] = (state->loc[ch] & 0xffffu) |
                                 ((uint32_t)value << 16);
            else if (reg == 2u) {
                state->loc[ch] = (state->loc[ch] & 0xffff0000u) | value;
                if (state->len[ch])
                    poc_prefetch_channel(state, ch);
            } else if (reg == 4u) {
                state->len[ch] = value;
                poc_prefetch_channel(state, ch);
            }
        }
    } else if (r == 0x096u) {
        const uint16_t before = state->dma;
        if (value & 0x8000u)
            state->dma |= value & 0x7fffu;
        else
            state->dma &= (uint16_t)~(value & 0x7fffu);
        if (state->dma & 0x0200u) {
            for (unsigned ch = 0; ch < 4u; ++ch) {
                const uint16_t mask = (uint16_t)(0x0200u | (1u << ch));
                if ((state->dma & mask) == mask && (before & mask) != mask)
                    poc_prefetch_channel(state, ch);
            }
        }
    }

    paula_live_write(0xdff000u | r, value);
    if (r == 0x096u || (r >= 0x0a0u && r <= 0x0dau &&
        ((r - 0x0a0u) & 15u) <= 4u))
        paula_live_prefetch();
}

static int poc_restore_live_state(poc_live_state *state,
                                  paula_mirror_snapshot *snapshot)
{
    if (!paula_mirror_snapshot_take(snapshot))
        return 0;

    paula_probe_flag(PP_FLAG_CAPTURE_FAILED,1);
    paula_live_reset();
    for (unsigned ch = 0; ch < 4u; ++ch) {
        state->loc[ch] = 0;
        state->len[ch] = 0;
    }
    state->dma = 0;

    for (unsigned ch = 0; ch < 4u; ++ch) {
        for (unsigned reg = 0; reg < 5u; ++reg) {
            if (!(snapshot->seen[ch] & (1u << reg)))
                continue;
            paula_mirror_event event;
            event.address = 0xdff0a0u + ch * 16u + reg * 2u;
            event.value = snapshot->reg[ch][reg];
            event.width = 2u;
            event.reserved = 0;
            event.ticks = snapshot->ticks;
            poc_apply_live_event(state, &event);
        }
    }
    if (snapshot->seen_global & 2u)
        paula_live_write(0xdff09au, (uint16_t)(0x8000u | snapshot->intena));
    if (snapshot->seen_global & 4u)
        paula_live_write(0xdff09cu, (uint16_t)(0x8000u | snapshot->intreq));
    if (snapshot->seen_global & 8u)
        paula_live_write(0xdff09eu, (uint16_t)(0x8000u | snapshot->adkcon));
    if (snapshot->seen_global & 1u) {
        paula_mirror_event event;
        event.address = 0xdff096u;
        event.value = (uint16_t)(0x8000u | snapshot->dma);
        event.width = 2u;
        event.reserved = 0;
        event.ticks = snapshot->ticks;
        poc_apply_live_event(state, &event);
    }

    paula_probe_add(PP_RESYNCS,1u);
    paula_probe_add(PP_RESYNC_SKIPPED,snapshot->skipped);
    paula_probe_flag(PP_FLAG_CAPTURE_FAILED,0);
    return 1;
}

void emu68_usb_audio_cpu1_worker(void)
{
    while (!__atomic_load_n(&usb_worker_enabled, __ATOMIC_ACQUIRE))
        asm volatile("wfe");

    paula_probe_init();
    paula_probe_set(PP_SAMPLE_RATE, USB_AUDIO_RATE);
    paula_probe_set(PP_COUNTER_FREQ, (uint32_t)read_cntfrq());
    paula_live_init();
    paula_mirror_enable();
    paula_probe_flag(PP_FLAG_RENDERING, 1);
    kprintf("[USB-AUDIO] CPU1 Paula CACHE-FIX2 producer active, 48 kHz stereo\n");

    poc_live_state state;
    for (unsigned ch = 0; ch < 4u; ++ch) {
        state.loc[ch] = 0;
        state.len[ch] = 0;
    }
    state.dma = 0;

    paula_mirror_event event;
    paula_mirror_snapshot snapshot;
    uint32_t frames_written = 0;
    uint32_t last_drop = paula_mirror_dropped();
    unsigned recovery_pending = 0;
    uint64_t last_report = read_cntvct();
    const uint64_t frequency = read_cntfrq();
    const uint64_t latency = frequency / 20u;
    const uint64_t max_lag = frequency / 4u;
    paula_replay_clock clock;
    paula_replay_start(&clock, last_report, (uint32_t)frequency, latency);

    for (;;) {
        paula_live_mixer_apply();
        uint64_t now = read_cntvct();
        const uint32_t drops = paula_mirror_dropped();

        if (drops != last_drop ||
            paula_mirror_pending() >= PAULA_MIRROR_EVENT_CAPACITY * 3u / 4u ||
            paula_replay_late(&clock, now, latency, max_lag))
            recovery_pending = 1;

        if (recovery_pending) {
            paula_probe_flag(PP_FLAG_CAPTURE_FAILED,1);
            if (poc_restore_live_state(&state, &snapshot)) {
                last_drop = snapshot.dropped;
                paula_replay_start(&clock, read_cntvct(),
                                   (uint32_t)frequency, latency);
                recovery_pending = 0;
            } else {
                paula_probe_add(PP_SNAPSHOT_RETRIES,1u);
            }
        }
        const unsigned recovering = recovery_pending;

        while (!recovering &&
               !paula_replay_due(read_cntvct(), clock.tick + latency))
            asm volatile("yield");

        unsigned processed = 0;
        while (!recovering && processed < 128u && paula_mirror_peek(&event)) {
            if (!paula_replay_due(clock.tick, event.ticks))
                break;
            poc_apply_live_event(&state, &event);
            paula_mirror_pop();
            ++processed;
        }

        unsigned backlog = 0;
        if (!recovering && paula_mirror_peek(&event) &&
            paula_replay_due(clock.tick, event.ticks))
            backlog = 1;

        float left = 0.0f, right = 0.0f;
        if (!recovering && !backlog)
            paula_live_render(&left, &right, 1u);
        else
            paula_probe_add(PP_REPLAY_SILENT_FRAMES,1u);

        const int16_t sl = float_to_s16(left);
        const int16_t sr = float_to_s16(right);
        ring_push(sl, sr);

        if (sl || sr)
            paula_probe_add(PP_NONZERO_FRAMES,1u);
        const uint32_t abs_l = (uint32_t)(sl < 0 ? -(int32_t)sl : sl);
        const uint32_t abs_r = (uint32_t)(sr < 0 ? -(int32_t)sr : sr);
        if (abs_l > paula_probe_get(PP_PEAK_LEFT))
            paula_probe_set(PP_PEAK_LEFT, abs_l);
        if (abs_r > paula_probe_get(PP_PEAK_RIGHT))
            paula_probe_set(PP_PEAK_RIGHT, abs_r);

        ++frames_written;
        paula_probe_set(PP_PCM_FRAMES, frames_written);
        paula_replay_advance(&clock);

        now = read_cntvct();
        if (now - last_report >= frequency) {
            const uint64_t target = clock.tick + latency;
            const uint64_t lag = paula_replay_due(now, target) ? now - target : 0;
            paula_probe_set(PP_REPLAY_LAG_US,
                (uint32_t)((lag * 1000000u) / frequency));
            kprintf("[USB-AUDIO] PCM=%u ring=%u pkt=%u under=%u over=%u miss=%u drop=%u\n",
                    frames_written, ring_count(),
                    __atomic_load_n(&audio_packets, __ATOMIC_RELAXED),
                    __atomic_load_n(&audio_underruns, __ATOMIC_RELAXED),
                    __atomic_load_n(&audio_overruns, __ATOMIC_RELAXED),
                    paula_mirror_misses(), paula_mirror_dropped());
            last_report = now;
        }
    }
}

void emu68_usb_audio_housekeeper_poll(void)
{
    /* CPU3 is the sole DWC2 owner, matching the known-good UVC transport. */
}

void emu68_usb_audio_cpu3_worker(void)
{
    for (;;) {
        if (!__atomic_load_n(&usb_worker_enabled, __ATOMIC_ACQUIRE)) {
            asm volatile("wfe");
            continue;
        }
        usb_poll();
    }
}

void emu68_usb_audio_init(void)
{
    audio_wr = audio_rd = 0;
    audio_underruns = audio_overruns = audio_packets = 0;

    kprintf("[USB-AUDIO] Paula -> UAC1 capture POC init\n");
    if (!hw_init()) {
        kprintf("[USB-AUDIO] DWC2 init failed\n");
        return;
    }

    asm volatile("dmb sy" ::: "memory");
    __atomic_store_n(&usb_worker_enabled, 1U, __ATOMIC_RELEASE);
    asm volatile("sev" ::: "memory");
    kprintf("[USB-AUDIO] connected; Windows should enumerate PiStorm Paula Audio\n");
}

#else

void emu68_usb_audio_init(void) {}
void emu68_usb_audio_cpu1_worker(void) { for (;;) asm volatile("wfe"); }
void emu68_usb_audio_cpu3_worker(void) { for (;;) asm volatile("wfe"); }

#endif
