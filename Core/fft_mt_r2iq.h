#pragma once

#include "r2iq.h"
#include "fftw3.h"
#include "config.h"
#include <algorithm>
#include <string.h>

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <arm_neon.h>
#endif

#if defined(__AVX2__) || defined(_M_AVX2)
#include <immintrin.h>
#elif defined(__SSE3__) || defined(__AVX__) || defined(_M_X64)
#include <pmmintrin.h>
#include <xmmintrin.h>
#endif

// use up to this many threads
#define N_MAX_R2IQ_THREADS 1
#define PRINT_INPUT_RANGE  0

static const int halfFft = FFTN_R_ADC / 2;    // half the size of the first fft at ADC 64Msps real rate (2048)
static const int fftPerBuf = transferSize / sizeof(short) / (3 * halfFft / 2) + 1; // number of ffts per buffer with 256|768 overlap

class fft_mt_r2iq : public r2iqControlClass
{
public:
    fft_mt_r2iq();
    virtual ~fft_mt_r2iq();

    float setFreqOffset(float offset);

    void Init(float gain, ringbuffer<int16_t>* buffers, ringbuffer<float>* obuffers);
    void TurnOn();
    void TurnOff(void);
    bool IsOn(void);

protected:

    template<bool rand> void convert_float(const int16_t *input, float* output, int size)
    {
#if defined(__AVX2__) || defined(_M_AVX2)
        const __m128i ones = _mm_set1_epi16(1);
        const __m128i negTwo = _mm_set1_epi16(-2);
        int m = 0;
        for (; m + 16 <= size; m += 16)
        {
            __m128i lo = _mm_loadu_si128(reinterpret_cast<const __m128i*>(input + m));
            __m128i hi = _mm_loadu_si128(reinterpret_cast<const __m128i*>(input + m + 8));
            if constexpr (rand)
            {
                __m128i loMask = _mm_and_si128(lo, ones);
                __m128i hiMask = _mm_and_si128(hi, ones);
                lo = _mm_xor_si128(lo, _mm_mullo_epi16(loMask, negTwo));
                hi = _mm_xor_si128(hi, _mm_mullo_epi16(hiMask, negTwo));
            }
            __m256i lo32 = _mm256_cvtepi16_epi32(lo);
            __m256i hi32 = _mm256_cvtepi16_epi32(hi);
            __m256 loFloat = _mm256_cvtepi32_ps(lo32);
            __m256 hiFloat = _mm256_cvtepi32_ps(hi32);
            _mm256_storeu_ps(output + m, loFloat);
            _mm256_storeu_ps(output + m + 8, hiFloat);
        }
        for (; m < size; ++m)
        {
            int16_t val = input[m];
            if constexpr (rand)
            {
                if (val & 1)
                    val = static_cast<int16_t>(val ^ (-2));
            }
            output[m] = static_cast<float>(val);
        }
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
        int m = 0;
        for (; m + 8 <= size; m += 8)
        {
            int16x8_t vec = vld1q_s16(input + m);
            if constexpr (rand)
            {
                uint16x8_t mask = vandq_u16(vreinterpretq_u16_s16(vec), vdupq_n_u16(1));
                uint16x8_t adjust = vmulq_n_u16(mask, 0xFFFEu);
                vec = vreinterpretq_s16_u16(veorq_u16(vreinterpretq_u16_s16(vec), adjust));
            }
            int32x4_t lo = vmovl_s16(vget_low_s16(vec));
            int32x4_t hi = vmovl_s16(vget_high_s16(vec));
            vst1q_f32(output + m, vcvtq_f32_s32(lo));
            vst1q_f32(output + m + 4, vcvtq_f32_s32(hi));
        }
        for (; m < size; ++m)
        {
            int16_t val = input[m];
            if constexpr (rand)
            {
                if (val & 1)
                    val = static_cast<int16_t>(val ^ (-2));
            }
            output[m] = static_cast<float>(val);
        }
#else
        for (int m = 0; m < size; m++)
        {
            int16_t val = input[m];
            if constexpr (rand)
            {
                if (val & 1)
                    val = static_cast<int16_t>(val ^ (-2));
            }
            output[m] = static_cast<float>(val);
        }
#endif
    }

    void shift_freq(fftwf_complex* dest, const fftwf_complex* source1, const fftwf_complex* source2, int start, int end)
    {
        fftwf_complex* dst = dest + start;
        const fftwf_complex* src1 = source1 + start;
        const fftwf_complex* src2 = source2 + start;
        int count = end - start;

#if defined(__ARM_NEON) || defined(__ARM_NEON__)
        int i = 0;
        for (; i + 2 <= count; i += 2)
        {
            float32x4_t a = vld1q_f32(reinterpret_cast<const float*>(src1 + i));
            float32x4_t b = vld1q_f32(reinterpret_cast<const float*>(src2 + i));

            float32x4x2_t a_parts = vuzpq_f32(a, a);
            float32x4x2_t b_parts = vuzpq_f32(b, b);

            float32x4_t real = vsubq_f32(vmulq_f32(a_parts.val[0], b_parts.val[0]),
                                         vmulq_f32(a_parts.val[1], b_parts.val[1]));
            float32x4_t imag = vaddq_f32(vmulq_f32(a_parts.val[0], b_parts.val[1]),
                                         vmulq_f32(a_parts.val[1], b_parts.val[0]));

            float32x4x2_t interleave = vzipq_f32(real, imag);
            float32x4_t out = vextq_f32(interleave.val[0], interleave.val[1], 2);

            vst1q_f32(reinterpret_cast<float*>(dst + i), out);
        }
        for (; i < count; ++i)
        {
            dst[i][0] = src1[i][0] * src2[i][0] - src1[i][1] * src2[i][1];
            dst[i][1] = src1[i][1] * src2[i][0] + src1[i][0] * src2[i][1];
        }
#elif defined(__SSE3__) || defined(__AVX__) || defined(_M_X64)
        int i = 0;
        for (; i + 2 <= count; i += 2)
        {
            __m128 a = _mm_loadu_ps(reinterpret_cast<const float*>(src1 + i));
            __m128 b = _mm_loadu_ps(reinterpret_cast<const float*>(src2 + i));

            __m128 a_real = _mm_moveldup_ps(a);
            __m128 a_imag = _mm_movehdup_ps(a);
            __m128 b_real = _mm_moveldup_ps(b);
            __m128 b_imag = _mm_movehdup_ps(b);

            __m128 real = _mm_sub_ps(_mm_mul_ps(a_real, b_real),
                                     _mm_mul_ps(a_imag, b_imag));
            __m128 imag = _mm_add_ps(_mm_mul_ps(a_real, b_imag),
                                     _mm_mul_ps(a_imag, b_real));

            __m128 lo = _mm_unpacklo_ps(real, imag);
            __m128 hi = _mm_unpackhi_ps(real, imag);
            __m128 out = _mm_movelh_ps(lo, hi);

            _mm_storeu_ps(reinterpret_cast<float*>(dst + i), out);
        }
        for (; i < count; ++i)
        {
            dst[i][0] = src1[i][0] * src2[i][0] - src1[i][1] * src2[i][1];
            dst[i][1] = src1[i][1] * src2[i][0] + src1[i][0] * src2[i][1];
        }
#else
        for (int i = 0; i < count; ++i)
        {
            dst[i][0] = src1[i][0] * src2[i][0] - src1[i][1] * src2[i][1];
            dst[i][1] = src1[i][1] * src2[i][0] + src1[i][0] * src2[i][1];
        }
#endif
    }

    template<bool flip> void copy(fftwf_complex* dest, const fftwf_complex* source, int count)
    {
        if constexpr (!flip)
        {
            memcpy(dest, source, sizeof(fftwf_complex) * static_cast<size_t>(count));
            return;
        }

#if defined(__AVX2__) || defined(_M_AVX2)
        const __m256 flipMask = _mm256_set_ps(-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f);
        int i = 0;
        for (; i + 4 <= count; i += 4)
        {
            __m256 vals = _mm256_loadu_ps(reinterpret_cast<const float*>(source + i));
            __m256 flipped = _mm256_mul_ps(vals, flipMask);
            _mm256_storeu_ps(reinterpret_cast<float*>(dest + i), flipped);
        }
        for (; i < count; ++i)
        {
            dest[i][0] = source[i][0];
            dest[i][1] = -source[i][1];
        }
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
        const float32x4_t sign = {1.0f, -1.0f, 1.0f, -1.0f};
        int i = 0;
        for (; i + 2 <= count; i += 2)
        {
            float32x4_t vals = vld1q_f32(reinterpret_cast<const float*>(source + i));
            float32x4_t flipped = vmulq_f32(vals, sign);
            vst1q_f32(reinterpret_cast<float*>(dest + i), flipped);
        }
        for (; i < count; ++i)
        {
            dest[i][0] = source[i][0];
            dest[i][1] = -source[i][1];
        }
#else
        for (int i = 0; i < count; i++)
        {
            dest[i][0] = source[i][0];
            dest[i][1] = -source[i][1];
        }
#endif
    }

private:
    ringbuffer<int16_t>* inputbuffer;    // pointer to input buffers
    ringbuffer<float>* outputbuffer;    // pointer to ouput buffers
    int bufIdx;         // index to next buffer to be processed
    r2iqThreadArg* lastThread;

    float GainScale;
    int mfftdim [NDECIDX]; // FFT N dimensions: mfftdim[k] = halfFft / 2^k
    int mtunebin;

    void *r2iqThreadf(r2iqThreadArg *th);   // thread function

    void * r2iqThreadf_def(r2iqThreadArg *th);
    void * r2iqThreadf_avx(r2iqThreadArg *th);
    void * r2iqThreadf_avx2(r2iqThreadArg *th);
    void * r2iqThreadf_avx512(r2iqThreadArg *th);
    void * r2iqThreadf_neon(r2iqThreadArg *th);

    fftwf_complex **filterHw;       // Hw complex to each decimation ratio

	fftwf_plan plan_t2f_r2c;          // fftw plan buffers Freq to Time complex to complex per decimation ratio
	fftwf_plan *plan_f2t_c2c;          // fftw plan buffers Time to Freq real to complex per buffer
	fftwf_plan plans_f2t_c2c[NDECIDX];

    uint32_t processor_count;
    r2iqThreadArg* threadArgs[N_MAX_R2IQ_THREADS];
    std::mutex mutexR2iqControl;                   // r2iq control lock
    std::thread r2iq_thread[N_MAX_R2IQ_THREADS]; // thread pointers
};

// assure, that ADC is not oversteered?
struct r2iqThreadArg {

	r2iqThreadArg() :
        ADCinTime(nullptr),
        ADCinFreq(nullptr),
        inFreqTmp(nullptr),
        prevTail(nullptr),
        hasPrevTail(false),
        prevRandFlag(false)
	{
#if PRINT_INPUT_RANGE
		MinMaxBlockCount = 0;
		MinValue = 0;
		MaxValue = 0;
#endif
	}

	float *ADCinTime;                // point to each threads input buffers [nftt][n]
	fftwf_complex *ADCinFreq;         // buffers in frequency
	fftwf_complex *inFreqTmp;         // tmp decimation output buffers (after tune shift)
    float *prevTail;                 // cached tail from previous block
    bool hasPrevTail;
    bool prevRandFlag;
#if PRINT_INPUT_RANGE
	int MinMaxBlockCount;
	int16_t MinValue;
	int16_t MaxValue;
#endif
};
