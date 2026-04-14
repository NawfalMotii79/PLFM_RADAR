#include "plfm_dsp.h"

#include <math.h>
#include <string.h>

static plfm_dsp_cfg_t g_dsp;

plfm_status_t plfm_dsp_init(const plfm_dsp_cfg_t *cfg)
{
    if ((cfg == NULL) || (cfg->fft_size == 0U)) {
        return PLFM_ERR_INVALID_ARG;
    }
    g_dsp = *cfg;
    return PLFM_OK;
}

plfm_status_t plfm_dsp_apply_window(float *real, float *imag, uint16_t n, plfm_window_t w)
{
    uint16_t k;
    float c0;
    float c1;
    float c2;
    float phase;
    float win;

    if ((real == NULL) || (imag == NULL) || (n < 2U)) {
        return PLFM_ERR_INVALID_ARG;
    }

    for (k = 0U; k < n; k++) {
        phase = (2.0f * 3.14159265f * (float)k) / (float)(n - 1U);
        switch (w) {
        case PLFM_WINDOW_HANN:
            win = 0.5f * (1.0f - cosf(phase));
            break;
        case PLFM_WINDOW_HAMMING:
            win = 0.54f - 0.46f * cosf(phase);
            break;
        case PLFM_WINDOW_BLACKMAN:
            c0 = 0.42f;
            c1 = 0.5f;
            c2 = 0.08f;
            win = c0 - c1 * cosf(phase) + c2 * cosf(2.0f * phase);
            break;
        default:
            return PLFM_ERR_INVALID_ARG;
        }
        real[k] *= win;
        imag[k] *= win;
    }
    return PLFM_OK;
}

plfm_status_t plfm_dsp_matched_filter(plfm_cplx_f32_t *x, uint16_t n, const float *h_i, const float *h_q)
{
    uint16_t k;
    float xi;
    float xq;

    if ((x == NULL) || (h_i == NULL) || (h_q == NULL) || (n == 0U)) {
        return PLFM_ERR_INVALID_ARG;
    }

    for (k = 0U; k < n; k++) {
        xi = x[k].i;
        xq = x[k].q;
        x[k].i = (xi * h_i[k]) - (xq * h_q[k]);
        x[k].q = (xi * h_q[k]) + (xq * h_i[k]);
    }
    return PLFM_OK;
}

plfm_status_t plfm_dsp_range_profile(const int16_t *iq_interleaved,
                                     uint16_t sample_count,
                                     float *range_mag_out,
                                     uint16_t out_bins,
                                     plfm_range_profile_meta_t *meta)
{
    uint16_t k;
    uint16_t bins;
    float i_val;
    float q_val;
    float mag;
    float sum = 0.0f;
    float peak = -120.0f;
    uint16_t peak_bin = 0U;

    if ((iq_interleaved == NULL) || (range_mag_out == NULL) || (meta == NULL) || (sample_count < 2U)) {
        return PLFM_ERR_INVALID_ARG;
    }

    bins = (sample_count / 2U);
    if (bins > out_bins) {
        bins = out_bins;
    }

    for (k = 0U; k < bins; k++) {
        i_val = (float)iq_interleaved[2U * k];
        q_val = (float)iq_interleaved[(2U * k) + 1U];
        mag = 20.0f * log10f((sqrtf((i_val * i_val) + (q_val * q_val)) / 32768.0f) + 1.0e-6f);
        range_mag_out[k] = mag;
        sum += mag;
        if (mag > peak) {
            peak = mag;
            peak_bin = k;
        }
    }

    meta->noise_floor_dbfs = sum / (float)bins;
    meta->peak_dbfs = peak;
    meta->peak_bin = peak_bin;

    return PLFM_OK;
}
