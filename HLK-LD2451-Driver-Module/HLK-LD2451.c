/*
 * HLK-LD2451.c
 *
 *  Created on: Sep 14, 2026
 *      Author: JK-01
 */


#include "HLK-LD2451.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* Konstanta protokol                                                  */
/* ------------------------------------------------------------------ */

static const uint8_t CFG_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CFG_TAIL[4]   = {0x04, 0x03, 0x02, 0x01};
static const uint8_t DATA_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t DATA_TAIL[4]   = {0xF8, 0xF7, 0xF6, 0xF5};

#define CMD_ENABLE_CONFIG     0x00FF
#define CMD_END_CONFIG        0x00FE
#define CMD_SET_DETECT_PARAMS 0x0002
#define CMD_READ_DETECT_PARAMS 0x0012
#define CMD_SET_SENSITIVITY   0x0003
#define CMD_READ_SENSITIVITY  0x0013
#define CMD_READ_FW_VERSION   0x00A0
#define CMD_SET_BAUDRATE      0x00A1
#define CMD_RESTORE_FACTORY   0x00A2
#define CMD_RESTART_MODULE    0x00A3

#define CFG_ACK_FLAG           0x0100u  /* ack_word = cmd_word | 0x0100 */

/* ------------------------------------------------------------------ */
/* State internal                                                      */
/* ------------------------------------------------------------------ */

static UART_HandleTypeDef *s_huart;

static uint8_t  s_dma_buf[LD2451_RX_DMA_BUF_SIZE];
static uint16_t s_dma_last_pos;

/* buffer akumulasi linear tempat kita cari frame */
static uint8_t  s_acc_buf[LD2451_FRAME_BUF_SIZE];
static uint16_t s_acc_len;

static volatile bool s_new_chunk_flag;

static ld2451_data_t s_data;

/* buffer sementara untuk terima ACK saat mode konfigurasi (blocking) */
static volatile bool     s_ack_pending;
static volatile bool     s_ack_ok;
static volatile uint16_t s_ack_cmd_word;

/* ------------------------------------------------------------------ */
/* Util internal                                                       */
/* ------------------------------------------------------------------ */

static uint16_t le16(uint8_t lo, uint8_t hi)
{
    return (uint16_t)((uint16_t)hi << 8 | lo);
}

static void put_le16(uint8_t *dst, uint16_t val)
{
    dst[0] = (uint8_t)(val & 0xFF);
    dst[1] = (uint8_t)((val >> 8) & 0xFF);
}

/**
 * @brief Bangun frame konfigurasi lengkap (header+len+cmd+value+tail).
 * @return panjang total frame dalam byte.
 */
static uint16_t build_cfg_frame(uint8_t *out, uint16_t cmd_word,
                                 const uint8_t *value, uint8_t value_len)
{
    uint16_t idx = 0;
    uint16_t inner_len = (uint16_t)(2 + value_len); /* cmd_word(2) + value */

    memcpy(&out[idx], CFG_HEADER, 4); idx += 4;
    put_le16(&out[idx], inner_len);   idx += 2;
    put_le16(&out[idx], cmd_word);    idx += 2;

    if (value_len > 0U) {
        memcpy(&out[idx], value, value_len);
        idx += value_len;
    }

    memcpy(&out[idx], CFG_TAIL, 4); idx += 4;
    return idx;
}

/**
 * @brief Cek apakah ada frame CONFIG (ACK) lengkap di s_acc_buf, dan jika ada,
 *        set s_ack_ok / s_ack_cmd_word. Dipanggil dari ld2451_process().
 */
static bool try_parse_config_ack(uint16_t start, uint16_t *consumed)
{
    if ((uint16_t)(s_acc_len - start) < 12U) {
        return false; /* frame config minimum = 4+2+2+2+4 = 14, tapi cek longgar dulu */
    }

    if (memcmp(&s_acc_buf[start], CFG_HEADER, 4) != 0) {
        return false;
    }

    uint16_t inner_len = le16(s_acc_buf[start + 4], s_acc_buf[start + 5]);
    uint16_t total_len = (uint16_t)(4 + 2 + inner_len + 4);

    if ((uint16_t)(s_acc_len - start) < total_len) {
        return false; /* belum lengkap, tunggu byte berikutnya */
    }

    const uint8_t *tail = &s_acc_buf[start + 4 + 2 + inner_len];
    if (memcmp(tail, CFG_TAIL, 4) != 0) {
        /* header cocok tapi tail tidak match -> data korup, buang 1 byte saja */
        *consumed = 1;
        return true;
    }

    uint16_t ack_word = le16(s_acc_buf[start + 6], s_acc_buf[start + 7]);
    uint16_t status    = le16(s_acc_buf[start + 8], s_acc_buf[start + 9]);

    s_ack_cmd_word = (uint16_t)(ack_word & ~CFG_ACK_FLAG);
    s_ack_ok       = (status == 0U);
    s_ack_pending  = false;

    *consumed = total_len;
    return true;
}

/**
 * @brief Cek apakah ada frame DATA (laporan target) lengkap di s_acc_buf.
 */
static bool try_parse_data_frame(uint16_t start, uint16_t *consumed)
{
    if ((uint16_t)(s_acc_len - start) < 10U) {
        return false;
    }

    if (memcmp(&s_acc_buf[start], DATA_HEADER, 4) != 0) {
        return false;
    }

    uint16_t inner_len = le16(s_acc_buf[start + 4], s_acc_buf[start + 5]);
    uint16_t total_len = (uint16_t)(4 + 2 + inner_len + 4);

    if (total_len > LD2451_FRAME_BUF_SIZE) {
        /* panjang tidak masuk akal, kemungkinan noise -> buang 1 byte */
        *consumed = 1;
        return true;
    }

    if ((uint16_t)(s_acc_len - start) < total_len) {
        return false; /* belum lengkap */
    }

    const uint8_t *payload = &s_acc_buf[start + 6];
    const uint8_t *tail     = &s_acc_buf[start + 6 + inner_len];

    if (memcmp(tail, DATA_TAIL, 4) != 0) {
        *consumed = 1;
        return true;
    }

    /* --- parsing isi frame --- */
    uint8_t target_count = payload[0];
    uint8_t alarm         = payload[1];

    if (target_count > LD2451_MAX_TARGETS) {
        target_count = LD2451_MAX_TARGETS; /* pengaman */
    }

    s_data.target_count = target_count;
    s_data.alarm        = (alarm != 0U);

    for (uint8_t i = 0; i < target_count; i++) {
        const uint8_t *t = &payload[2 + (uint16_t)i * 5U];

        s_data.targets[i].angle_deg  = (int8_t)((int16_t)t[0] - 0x80);
        s_data.targets[i].distance_m = t[1];
        s_data.targets[i].direction  = (ld2451_direction_t)t[2];
        s_data.targets[i].speed_kmh  = t[3];
        s_data.targets[i].snr        = t[4];
    }

    s_data.valid            = true;
    s_data.last_update_tick = HAL_GetTick();

    *consumed = total_len;
    return true;
}

/**
 * @brief Geser sisa byte yang belum terpakai ke depan buffer akumulasi.
 */
static void shift_acc_buf(uint16_t consumed)
{
    if (consumed >= s_acc_len) {
        s_acc_len = 0;
        return;
    }
    memmove(&s_acc_buf[0], &s_acc_buf[consumed], s_acc_len - consumed);
    s_acc_len = (uint16_t)(s_acc_len - consumed);
}

/**
 * @brief Generic blocking send command + tunggu ACK.
 */
static ld2451_status_t send_and_wait_ack(uint16_t cmd_word,
                                          const uint8_t *value, uint8_t value_len,
                                          uint32_t timeout_ms)
{
    uint8_t frame[32];
    uint16_t frame_len = build_cfg_frame(frame, cmd_word, value, value_len);

    s_ack_pending = true;
    s_ack_ok      = false;

    if (HAL_UART_Transmit(s_huart, frame, frame_len, 100U) != HAL_OK) {
        s_ack_pending = false;
        return LD2451_ERR_BUSY;
    }

    uint32_t t0 = HAL_GetTick();
    while (s_ack_pending) {
        ld2451_process(); /* proses buffer termasuk cari ACK frame */

        if ((HAL_GetTick() - t0) > timeout_ms) {
            return LD2451_ERR_TIMEOUT;
        }
    }

    if (!s_ack_ok || s_ack_cmd_word != cmd_word) {
        return LD2451_ERR_NACK;
    }

    return LD2451_OK;
}

/* ------------------------------------------------------------------ */
/* API publik                                                           */
/* ------------------------------------------------------------------ */

ld2451_status_t ld2451_init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return LD2451_ERR_PARAM;
    }

    s_huart = huart;
    memset(&s_data, 0, sizeof(s_data));
    s_acc_len = 0;
    s_dma_last_pos = 0;
    s_new_chunk_flag = false;
    s_ack_pending = false;

    if (HAL_UARTEx_ReceiveToIdle_DMA(s_huart, s_dma_buf,
                                      LD2451_RX_DMA_BUF_SIZE) != HAL_OK) {
        return LD2451_ERR_BUSY;
    }

    /* nonaktifkan HT interrupt supaya callback hanya jalan saat IDLE/full */
    __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);

    return LD2451_OK;
}

void ld2451_uart_rx_event(uint16_t size)
{
    /* size = posisi terakhir DMA menulis (indeks absolut di s_dma_buf) */
    uint16_t new_pos = size;

    while (s_dma_last_pos != new_pos) {
        if (s_acc_len < LD2451_FRAME_BUF_SIZE) {
            s_acc_buf[s_acc_len++] = s_dma_buf[s_dma_last_pos];
        }
        /* kalau acc buffer penuh dan belum ada frame valid, buang byte
           tertua supaya driver tidak macet karena noise */
        else {
            shift_acc_buf(1);
            s_acc_buf[s_acc_len++] = s_dma_buf[s_dma_last_pos];
        }

        s_dma_last_pos = (uint16_t)((s_dma_last_pos + 1U) % LD2451_RX_DMA_BUF_SIZE);
    }

    s_new_chunk_flag = true;
}

bool ld2451_process(void)
{
    bool got_data_frame = false;

    if (!s_new_chunk_flag && s_acc_len == 0U) {
        return false;
    }
    s_new_chunk_flag = false;

    uint16_t pos = 0;

    while (pos < s_acc_len) {
        /* cari kandidat header di posisi pos */
        if (memcmp(&s_acc_buf[pos], CFG_HEADER, 4) == 0) {
            uint16_t consumed = 0;
            if (try_parse_config_ack(pos, &consumed)) {
                shift_acc_buf((uint16_t)(pos + consumed));
                pos = 0;
                continue;
            }
            break; /* header cocok tapi frame belum lengkap -> tunggu data lagi */
        }
        else if (memcmp(&s_acc_buf[pos], DATA_HEADER, 4) == 0) {
            uint16_t consumed = 0;
            if (try_parse_data_frame(pos, &consumed)) {
                shift_acc_buf((uint16_t)(pos + consumed));
                pos = 0;
                got_data_frame = true;
                continue;
            }
            break;
        }
        else {
            pos++; /* bukan header, geser 1 byte (skip noise) */
        }
    }

    if (pos > 0U) {
        shift_acc_buf(pos);
    }

    return got_data_frame;
}

const ld2451_data_t *ld2451_get_data(void)
{
    return &s_data;
}

/* ---- Perintah konfigurasi ---- */

ld2451_status_t ld2451_enter_config_mode(uint32_t timeout_ms)
{
    uint8_t value[2];
    put_le16(value, 0x0001);
    return send_and_wait_ack(CMD_ENABLE_CONFIG, value, 2U, timeout_ms);
}

ld2451_status_t ld2451_exit_config_mode(uint32_t timeout_ms)
{
    return send_and_wait_ack(CMD_END_CONFIG, NULL, 0U, timeout_ms);
}

ld2451_status_t ld2451_set_detect_params(const ld2451_detect_params_t *params,
                                          uint32_t timeout_ms)
{
    if (params == NULL) {
        return LD2451_ERR_PARAM;
    }

    uint8_t value[4];
    value[0] = params->max_distance_m;
    value[1] = (uint8_t)params->direction;
    value[2] = params->min_speed_kmh;
    value[3] = params->no_target_delay_s;

    return send_and_wait_ack(CMD_SET_DETECT_PARAMS, value, 4U, timeout_ms);
}

ld2451_status_t ld2451_set_sensitivity(const ld2451_sensitivity_t *sens,
                                        uint32_t timeout_ms)
{
    if (sens == NULL) {
        return LD2451_ERR_PARAM;
    }

    uint8_t value[4];
    value[0] = sens->trigger_count;
    value[1] = sens->snr_threshold;
    value[2] = 0x00;
    value[3] = 0x00;

    return send_and_wait_ack(CMD_SET_SENSITIVITY, value, 4U, timeout_ms);
}

ld2451_status_t ld2451_restore_factory_settings(uint32_t timeout_ms)
{
    return send_and_wait_ack(CMD_RESTORE_FACTORY, NULL, 0U, timeout_ms);
}

ld2451_status_t ld2451_restart_module(uint32_t timeout_ms)
{
    return send_and_wait_ack(CMD_RESTART_MODULE, NULL, 0U, timeout_ms);
}

