
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
float LPF(float input, float fps, float cut_freq, float *prev_value);

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
float BPF(float input, float fps, float center_freq, float bandwidth, BPFState *s);

