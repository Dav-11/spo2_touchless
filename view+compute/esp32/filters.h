
/**************************************
 * Types Definitions
 **************************************/

typedef struct {
  float x1, x2; // previous input samples
  float y1, y2; // previous output samples
} BPFState;

/**************************************
 * Function Prototypes
 **************************************/

/**
 * Apply a low-pass filter to the input signal.
 *
 * @param input       The current input sample.
 * @param fps         The sampling rate (Hz).
 * @param cut_freq    The cutoff frequency (Hz).
 * @param prev_value  Pointer to the previous output sample (must persist between calls).
 * @return            The filtered output sample.
 */
float LPF(float input, float fps, float cut_freq, float *prev_value) {
  // fps: samples per second (sampling frequency)
  // cut_freq: desired cutoff frequency in Hz
  // prev_value: pointer to the previous filtered output

  // Protect against invalid parameters
  if (fps <= 0 || cut_freq <= 0) return input;

  // Calculate RC (time constant)
  float RC = 1.0f / (2.0f * M_PI * cut_freq);

  // Calculate sampling period
  float dt = 1.0f / fps;

  // Compute alpha (filter coefficient)
  float alpha = dt / (RC + dt);

  // Apply low-pass filter formula
  float output = *prev_value + alpha * (input - *prev_value);

  // Update previous value
  *prev_value = output;

  return output;
}

/**
 * Apply a band-pass filter to the input signal.
 *
 * @param input        The current input sample.
 * @param fps          The sampling rate (Hz).
 * @param center_freq  The center frequency of the band-pass filter (Hz).
 * @param bandwidth    The bandwidth of the filter (Hz).
 * @param s            Pointer to the filter state (must persist between calls).
 * @return             The filtered output sample.
 */
float BPF(float input, float fps, float center_freq, float bandwidth, BPFState *s) {
  // fps         = sampling rate (Hz)
  // center_freq = center frequency (Hz)
  // bandwidth   = bandwidth (Hz)
  // s           = filter state (must persist between calls)

  if (fps <= 0 || center_freq <= 0 || bandwidth <= 0) return input;

  // Pre-warped values
  float w0 = 2.0f * M_PI * center_freq / fps;
  float alpha = sinf(w0) * sinhf(logf(2.0f) / 2.0f * bandwidth * w0 / sinf(w0));

  // Coefficients for a constant skirt gain, peak gain = Q
  float b0 = alpha;
  float b1 = 0.0f;
  float b2 = -alpha;
  float a0 = 1.0f + alpha;
  float a1 = -2.0f * cosf(w0);
  float a2 = 1.0f - alpha;

  // Normalize coefficients
  b0 /= a0;
  b1 /= a0;
  b2 /= a0;
  a1 /= a0;
  a2 /= a0;

  // Biquad difference equation
  float output = b0 * input + b1 * s->x1 + b2 * s->x2
                               - a1 * s->y1 - a2 * s->y2;

  // Shift history
  s->x2 = s->x1;
  s->x1 = input;
  s->y2 = s->y1;
  s->y1 = output;

  return output;
}

