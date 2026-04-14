#ifndef PLFM_DSP_H
#define PLFM_DSP_H

#include "plfm_types.h"

typedef struct
{
    uint16_t fft_size;
    plfm_window_t window;
    const float *matched_filter_i;
    const float *matched_filter_q;
} plfm_dsp_cfg_t;

typedef struct
{
    float noise_floor_dbfs;
    float peak_dbfs;
    uint16_t peak_bin;
} plfm_range_profile_meta_t;

plfm_status_t plfm_dsp_init(const plfm_dsp_cfg_t *cfg);
plfm_status_t plfm_dsp_apply_window(float *real, float *imag, uint16_t n, plfm_window_t w);
plfm_status_t plfm_dsp_matched_filter(plfm_cplx_f32_t *x,
                                      uint16_t n,
                                      const float *h_i,
                                      const float *h_q);
plfm_status_t plfm_dsp_range_profile(const int16_t *iq_interleaved,
                                     uint16_t sample_count,
                                     float *range_mag_out,
                                     uint16_t out_bins,
                                     plfm_range_profile_meta_t *meta);

#endif /* PLFM_DSP_H */
