/*
 * HLK-LD2451.h
 *
 *  Created on: Sep 14, 2026
 *      Author: JK-01
 */


#ifndef INC_HLK_LD2451_H_
#define INC_HLK_LD2451_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* ------------------------------------------------------------------ */
/* Konfigurasi                                                         */
/* ------------------------------------------------------------------ */

#define LD2451_RX_DMA_BUF_SIZE      256U   /* buffer mentah DMA circular   */
#define LD2451_FRAME_BUF_SIZE       128U   /* buffer kerja untuk 1 frame   */
#define LD2451_MAX_TARGETS          8U     /* batas aman jumlah target     */

/* ------------------------------------------------------------------ */
/* Tipe data                                                           */
/* ------------------------------------------------------------------ */

typedef enum {
    LD2451_DIR_APPROACHING = 0x00, /* mendekat / close  */
    LD2451_DIR_AWAY        = 0x01  /* menjauh  / away   */
} ld2451_direction_t;

typedef struct {
    int8_t              angle_deg;   /* derajat, sudah dikonversi (report - 0x80) */
    uint8_t              distance_m;  /* meter, 0..100 */
    ld2451_direction_t   direction;   /* mendekat / menjauh */
    uint8_t               speed_kmh;   /* km/h, 0..120 */
    uint8_t               snr;         /* signal-to-noise ratio, 0..255 */
} ld2451_target_t;

typedef struct {
    bool             valid;                 /* true kalau frame terakhir sukses diparsing */
    bool             alarm;                  /* true kalau ada target yang approaching */
    uint8_t           target_count;
    ld2451_target_t   targets[LD2451_MAX_TARGETS];
    uint32_t          last_update_tick;      /* HAL_GetTick() saat data ini didapat */
} ld2451_data_t;

/* Parameter deteksi (dipakai untuk konfigurasi radar) */
typedef enum {
    LD2451_SCAN_AWAY_ONLY       = 0x00,
    LD2451_SCAN_APPROACH_ONLY   = 0x01,
    LD2451_SCAN_BOTH            = 0x02
} ld2451_scan_dir_t;

typedef struct {
    uint8_t             max_distance_m;   /* 0x0A - 0xFF meter */
    ld2451_scan_dir_t   direction;
    uint8_t             min_speed_kmh;    /* 0x00 - 0x78 km/h */
    uint8_t             no_target_delay_s;/* 0x00 - 0xFF detik */
} ld2451_detect_params_t;

typedef struct {
    uint8_t trigger_count;      /* 1 - 0x0A, default 1 */
    uint8_t snr_threshold;      /* 0 (pakai default=4) - 8, default 4 */
} ld2451_sensitivity_t;

/* Status hasil operasi driver */
typedef enum {
    LD2451_OK = 0,
    LD2451_ERR_TIMEOUT,
    LD2451_ERR_NACK,
    LD2451_ERR_BUSY,
    LD2451_ERR_PARAM
} ld2451_status_t;

/* ------------------------------------------------------------------ */
/* API                                                                  */
/* ------------------------------------------------------------------ */

/**
 * @brief Inisialisasi driver. Memulai UART RX lewat DMA + IDLE line detection.
 * @param huart  handle UART yang SUDAH di-Init oleh CubeMX/HAL (115200 8N1)
 */
ld2451_status_t ld2451_init(UART_HandleTypeDef *huart);

/**
 * @brief Dipanggil dari HAL_UARTEx_RxEventCallback() di main.c / stm32f4xx_it.c
 *        Menyalin data mentah yang baru diterima ke ring buffer internal driver.
 */
void ld2451_uart_rx_event(uint16_t size);

/**
 * @brief Panggil secara berkala (misal tiap loop utama / tiap 10-20ms).
 *        Mencari & mem-parsing frame data yang lengkap di ring buffer.
 * @return true kalau ada frame data baru yang berhasil diparsing.
 */
bool ld2451_process(void);

/**
 * @brief Ambil pointer ke data hasil parsing terakhir (read-only).
 */
const ld2451_data_t *ld2451_get_data(void);

/* ---- Perintah konfigurasi (blocking, dipanggil saat setup saja) ---- */

/** Masuk mode konfigurasi. Wajib dipanggil sebelum command config lain. */
ld2451_status_t ld2451_enter_config_mode(uint32_t timeout_ms);

/** Keluar mode konfigurasi. Radar kembali ke mode kerja normal (streaming data). */
ld2451_status_t ld2451_exit_config_mode(uint32_t timeout_ms);

/** Set parameter deteksi (jarak maksimum, arah, kecepatan minimum, delay). */
ld2451_status_t ld2451_set_detect_params(const ld2451_detect_params_t *params,
                                          uint32_t timeout_ms);

/** Set parameter sensitivitas (trigger count & SNR threshold). */
ld2451_status_t ld2451_set_sensitivity(const ld2451_sensitivity_t *sens,
                                        uint32_t timeout_ms);

/** Kembalikan radar ke pengaturan pabrik. */
ld2451_status_t ld2451_restore_factory_settings(uint32_t timeout_ms);

/** Restart modul radar. */
ld2451_status_t ld2451_restart_module(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* INC_HLK_LD2451_H_ */
