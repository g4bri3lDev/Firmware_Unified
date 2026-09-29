/* Transfer-plane seams: od_xfer's full-image hooks over the imported Firmware_NRF52 panel drivers
 * (src/epd/), plus the NFC and inflate seams.
 *
 * The stream goes to the controller unchanged, one write_ram() per chunk, exactly as
 * Firmware_NRF52's EPD_service did with the same host data -- that firmware is what py-opendisplay
 * already drives these controllers through. No framebuffer: a chunk is on the panel's RAM before
 * od_xfer asks for the next one.
 *
 * Structure follows targets/efr32bg22-slc/opendisplay_display.cpp (the plane walk, the refresh
 * barrier, power down after refresh). One deliberate difference: a flush TIMEOUT before refresh
 * proceeds, as od_txq.h specifies, where BG22 aborts. */

#include "od_inflate_app.h"
#include "od_nfc_app.h"
#include "od_xfer_app.h"

#include "EPD_driver.h"
#include "od_color.h"
#include "od_hal_time.h"
#include "od_tlsr.h"
#include "od_txq.h"
#include "tlsr_port.h"

#include <string.h>

#define REFRESH_BARRIER_MS  2000u
#define WRITE_CHUNK_MAX     255u      /* write_ram() takes a uint8_t length */

static struct {
    bool                active;
    epd_model_t        *epd;
    od_color_geometry_t geometry;
    uint32_t            plane_size;   /* bytes in plane 0 when the layout has two planes */
    int8_t              plane;        /* plane currently open on the controller, -1 = none */
} s_xfer;

static uint32_t s_displayed_etag;
static uint8_t  s_inflate_scratch[256];

static const struct DisplayConfig *display_cfg(void)
{
    const struct od_config *cfg = od_tlsr_config();
    return cfg->display_count != 0u ? &cfg->displays[0] : NULL;
}

/* Canonical PanelIC 1000-1030 is the Firmware_NRF52 model line (model id + 999). The imported
 * map_panel_ic_to_model_id() also takes raw 1-31, a legacy nRF52 path -- but canonically 1-51 are
 * bb_epaper panels, so accepting them would drive a different controller. Anything else would hit
 * its silent fallback to a 4.2" UC8176. Refuse both here. */
static bool panel_type_known(uint16_t t)
{
    return t >= 1000u && t <= 1031u;   /* 1031: SSD16XX_HS_266_BWR, provisional */
}

/* The imported lookup only knows 1000-1030 (and falls back to a 4.2" UC8176 otherwise); past
 * that the model id is simply value - 999. Either way begin_full() re-checks size and colour. */
static epd_model_id_t model_for(uint16_t t, uint8_t scheme)
{
    return t <= 1030u ? map_panel_ic_to_model_id(t, scheme) : (epd_model_id_t)(t - 999u);
}

static bool scheme_matches(uint8_t scheme, epd_color_t color)
{
    switch (scheme) {
    case OD_COLOR_SCHEME_MONO: return color == COLOR_BW;
    case OD_COLOR_SCHEME_BWR:
    case OD_COLOR_SCHEME_BWY:  return color == COLOR_BWR;
    case OD_COLOR_SCHEME_BWRY: return color == COLOR_BWRY;
    default:                   return false;
    }
}

static void panel_down(void)
{
    if (s_xfer.epd != NULL) {
        s_xfer.epd->drv->sleep(s_xfer.epd);
    }
    EPD_GPIO_Uninit();
    memset(&s_xfer, 0, sizeof(s_xfer));
    s_xfer.plane = -1;
}

void od_xfer_app_prepare_start(void)
{
    if (s_xfer.active) {
        panel_down();
    }
}

bool od_xfer_app_panel_info(od_xfer_panel_info_t *out)
{
    const struct DisplayConfig *d = display_cfg();

    if (out == NULL || d == NULL || !panel_type_known(d->panel_ic_type)) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (od_color_direct_geometry(d->color_scheme, d->pixel_width, d->pixel_height,
                                 &out->geometry) != OD_COLOR_OK) {
        return false;
    }
    out->width = d->pixel_width;
    out->height = d->pixel_height;
    out->partial_enabled = false;
    return true;
}

bool od_xfer_app_begin_full(const od_color_geometry_t *geometry)
{
    const struct DisplayConfig *d = display_cfg();
    const struct od_config *cfg = od_tlsr_config();
    struct epd_io_pins pins;
    epd_model_t *epd;

    if (d == NULL || geometry == NULL || geometry->total_bytes == 0u ||
        geometry->layout == OD_COLOR_LAYOUT_SPLIT_HALVES || !panel_type_known(d->panel_ic_type)) {
        return false;
    }
    pins.mosi = d->data_pin;
    pins.sclk = d->clk_pin;
    pins.cs   = d->cs_pin;
    pins.dc   = d->dc_pin;
    pins.rst  = d->reset_pin;
    pins.busy = d->busy_pin;
    pins.pwr  = cfg->system_config.pwr_pin;
    epd_io_configure(&pins);
    EPD_GPIO_Init();

    epd = epd_init(model_for(d->panel_ic_type, d->color_scheme));
    /* The model table fixes the controller's resolution; a config that disagrees would be streamed
     * into the wrong RAM layout. */
    if (epd->width != d->pixel_width || epd->height != d->pixel_height ||
        !scheme_matches(d->color_scheme, epd->color)) {
        s_xfer.epd = epd;
        panel_down();
        return false;
    }

    memset(&s_xfer, 0, sizeof(s_xfer));
    s_xfer.active = true;
    s_xfer.epd = epd;
    s_xfer.geometry = *geometry;
    s_xfer.plane_size = geometry->layout == OD_COLOR_LAYOUT_CONTROLLER_PLANES
                        ? geometry->part_bytes[0] : geometry->total_bytes;
    s_xfer.plane = -1;
    return true;
}

/* cfg byte as Firmware_NRF52's write_ram() reads it: high nibble 0 opens the plane (sends the
 * DTM/WRITE_RAM command), low nibble 0xF selects the black plane, anything else the red one. */
static void write_plane(uint8_t plane, const uint8_t *p, uint32_t n)
{
    while (n != 0u) {
        uint8_t len = (uint8_t)(n > WRITE_CHUNK_MAX ? WRITE_CHUNK_MAX : n);
        uint8_t cfg = (uint8_t)((s_xfer.plane == (int8_t)plane ? 0x10u : 0x00u) |
                                (plane == 0u ? 0x0Fu : 0x00u));

        s_xfer.epd->drv->write_ram(s_xfer.epd, cfg, (uint8_t *)p, len);
        s_xfer.plane = (int8_t)plane;
        p += len;
        n -= len;
    }
}

uint32_t od_xfer_app_write(uint32_t stream_offset, od_span_t data)
{
    uint32_t consumed = 0u;

    if (!s_xfer.active || !od_span_valid(data) || data.n == 0u ||
        stream_offset > s_xfer.geometry.total_bytes ||
        data.n > s_xfer.geometry.total_bytes - stream_offset) {
        return 0u;
    }
    while (consumed < data.n) {
        uint32_t logical = stream_offset + consumed;
        uint8_t plane = logical < s_xfer.plane_size ? 0u : 1u;
        uint32_t plane_end = plane == 0u ? s_xfer.plane_size : s_xfer.geometry.total_bytes;
        uint32_t chunk = plane_end - logical;

        if (chunk > data.n - consumed) {
            chunk = (uint32_t)data.n - consumed;
        }
        write_plane(plane, data.p + consumed, chunk);
        consumed += chunk;
    }
    return consumed;
}

od_mut_span_t od_xfer_app_inflate_scratch(void)
{
    return od_mut_span_make(s_inflate_scratch, sizeof(s_inflate_scratch));
}

void od_xfer_app_abort(od_xfer_abort_reason_t reason)
{
    (void)reason;
    if (s_xfer.active) {
        panel_down();
    }
}

/* Get the END ack onto the air before a refresh that can block for a minute. The stack has to
 * run for a notification to leave, so each drain attempt is followed by a stack pass. */
od_xfer_barrier_t od_xfer_app_before_refresh(const od_reply_t *owner)
{
    uint32_t deadline = od_hal_uptime_ms() + REFRESH_BARRIER_MS;

    (void)owner;
    for (;;) {
        od_txq_status_t rc = od_txq_flush(od_hal_uptime_ms(), deadline);
        if (rc == OD_TXQ_OK || rc == OD_TXQ_TIMEOUT) {
            return OD_XFER_BARRIER_PROCEED;
        }
        tlsr_port_service_stack();
    }
}

void od_xfer_app_barrier_abort(const od_reply_t *owner)
{
    (void)owner;
    od_xfer_app_abort(OD_XFER_ABORT_REPLY_FAILED);
}

bool od_xfer_app_refresh(uint8_t mode, bool *completed)
{
    (void)mode;                        /* the imported drivers have one (full) refresh */
    if (completed == NULL || !s_xfer.active) {
        return false;
    }
    s_xfer.epd->drv->refresh(s_xfer.epd);
    *completed = !epd_io_busy_timed_out();
    panel_down();
    return true;
}

uint32_t od_xfer_app_displayed_etag(void)
{
    return s_displayed_etag;
}

void od_xfer_app_set_displayed_etag(uint32_t etag)
{
    s_displayed_etag = etag;
}

uint32_t od_xfer_app_now_ms(void)
{
    return od_hal_uptime_ms();
}

/* ------------------------------------------------------------------------------- nfc --- */

/* No NFC front end on these tags. */
bool od_nfc_app_read(uint8_t *type, uint8_t *data, uint16_t *len_io, uint16_t cap)
{
    (void)type; (void)data; (void)len_io; (void)cap;
    return false;
}

bool od_nfc_app_write(uint8_t type, const uint8_t *data, uint16_t len)
{
    (void)type; (void)data; (void)len;
    return false;
}

/* --------------------------------------------------------------------------- inflate --- */

void od_inflate_app_reset(uint32_t expected_output_size)
{
    od_zlib_stream_reset(expected_output_size);
}

od_zlib_status_t od_inflate_app_push(od_span_t input, bool final)
{
    return od_zlib_stream_push(input.p, input.n, final);
}

od_zlib_status_t od_inflate_app_poll(uint8_t *output, size_t capacity, size_t *produced)
{
    return od_zlib_stream_poll(output, capacity, produced);
}

const char *od_inflate_app_error(void)
{
    return od_zlib_stream_error();
}

uint32_t od_inflate_app_output_count(void)
{
    return od_zlib_stream_output_count();
}
