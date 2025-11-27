typedef struct {
    int M;             // number of taps
    float *h;          // coefficients
    float *x;          // circular buffer
    int idx;
} FIRFilter;

void fir_init(FIRFilter *f, float *coeffs, float *buf, int M) {
    f->M = M;
    f->h = coeffs;
    f->x = buf;
    f->idx = 0;
    for (int i = 0; i < M; i++) f->x[i] = 0.0f;
}

float fir_process(FIRFilter *f, float sample) {
    f->x[f->idx] = sample;
    float y = 0.0f;

    int j = f->idx;
    for (int i = 0; i < f->M; i++) {
        y += f->h[i] * f->x[j];
        j--;
        if (j < 0) j = f->M - 1;
    }
    f->idx++;
    if (f->idx >= f->M) f->idx = 0;

    return y;
}

void fir_lpf_design(float *h, int M, float fc, float fs)
{
    int mid = (M - 1) / 2;
    float normCut = fc / fs;

    for (int n = 0; n < M; n++) {
        int k = n - mid;

        // ideal sinc LPF
        float sinc;
        if (k == 0)
            sinc = 2.0f * normCut;
        else
            sinc = sinf(2.0f * M_PI * normCut * k) / (M_PI * k);

        // Hamming window
        float w = 0.54f - 0.46f * cosf(2.0f * M_PI * n / (M - 1));

        h[n] = sinc * w;
    }

    // Normalize (sum to 1)
    float sum = 0.0f;
    for (int i = 0; i < M; i++) sum += h[i];
    for (int i = 0; i < M; i++) h[i] /= sum;
}

void fir_bpf_design(float *h, int M, float f_low, float f_high, float fs)
{
    float *hl = (float*)malloc(sizeof(float) * M);
    float *hh = (float*)malloc(sizeof(float) * M);

    fir_lpf_design(hl, M, f_high, fs);  // LPF at high cutoff
    fir_lpf_design(hh, M, f_low,  fs);  // LPF at low cutoff

    // BPF = LPF_high - LPF_low
    for (int i = 0; i < M; i++)
        h[i] = hl[i] - hh[i];

    free(hl);
    free(hh);
}
