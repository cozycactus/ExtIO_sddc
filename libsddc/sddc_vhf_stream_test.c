/*
 * sddc_vhf_stream_test - simple VHF/UHF stream test program for libsddc
 *
 * Copyright (C) 2020 by Franco Venturi
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "libsddc.h"
#include "wavewrite.h"

#if _WIN32
#include <Windows.h>
#define CLOCK_REALTIME 0
LARGE_INTEGER
getFILETIMEoffset()
{
  SYSTEMTIME s;
  FILETIME f;
  LARGE_INTEGER t;

  s.wYear = 1970;
  s.wMonth = 1;
  s.wDay = 1;
  s.wHour = 0;
  s.wMinute = 0;
  s.wSecond = 0;
  s.wMilliseconds = 0;
  SystemTimeToFileTime(&s, &f);
  t.QuadPart = f.dwHighDateTime;
  t.QuadPart <<= 32;
  t.QuadPart |= f.dwLowDateTime;
  return (t);
}

int
clock_gettime(int X, struct timeval* tv)
{
  LARGE_INTEGER           t;
  FILETIME            f;
  double                  microseconds;
  static LARGE_INTEGER    offset;
  static double           frequencyToMicroseconds;
  static int              initialized = 0;
  static BOOL             usePerformanceCounter = 0;

  if (!initialized) {
    LARGE_INTEGER performanceFrequency;
    initialized = 1;
    usePerformanceCounter = QueryPerformanceFrequency(&performanceFrequency);
    if (usePerformanceCounter) {
      QueryPerformanceCounter(&offset);
      frequencyToMicroseconds = (double)performanceFrequency.QuadPart / 1000000.;
    }
    else {
      offset = getFILETIMEoffset();
      frequencyToMicroseconds = 10.;
    }
  }
  if (usePerformanceCounter) QueryPerformanceCounter(&t);
  else {
    GetSystemTimeAsFileTime(&f);
    t.QuadPart = f.dwHighDateTime;
    t.QuadPart <<= 32;
    t.QuadPart |= f.dwLowDateTime;
  }

  t.QuadPart -= offset.QuadPart;
  microseconds = (double)t.QuadPart / frequencyToMicroseconds;
  t.QuadPart = microseconds;
  tv->tv_sec = t.QuadPart / 1000000;
  tv->tv_usec = t.QuadPart % 1000000;
  return (0);
}
#endif


static void count_bytes_callback(uint32_t data_size, uint8_t *data,
                                 void *context);

static unsigned long long received_samples = 0;
static unsigned long long total_samples = 0;
static int num_callbacks;
static uint8_t *sampleData = 0;            /* accumulated raw stream bytes */
static int output_int16 = 0;               /* 0 = CF32 I/Q (default), 1 = raw int16 */
static unsigned bytes_per_sample = 2 * sizeof(float);  /* set per format in main() */
static int runtime = 3000;
static struct timespec clk_start, clk_end;
static int stop_reception = 0;

static double clk_diff() {
  return ((double)clk_end.tv_sec + 1.0e-9*clk_end.tv_nsec) - 
           ((double)clk_start.tv_sec + 1.0e-9*clk_start.tv_nsec);
}


int main(int argc, char **argv)
{
  if (argc < 3) {
    fprintf(stderr, "usage: %s <image file> <sample rate> [<runtime_in_ms> [<output_filename> [iq|raw]]]\n", argv[0]);
    return -1;
  }
  char *imagefile = argv[1];
  const char *outfilename = 0;
  double sample_rate = 0.0;

  double vhf_frequency = 100e6;
  double vhf_attenuation = 20;  /* 20dB attenuation */

  sscanf(argv[2], "%lf", &sample_rate);
  if (3 < argc)
    runtime = atoi(argv[3]);
  if (4 < argc)
    outfilename = argv[4];
  if (5 < argc && strcmp(argv[5], "raw") == 0)
    output_int16 = 1;
  bytes_per_sample = output_int16 ? sizeof(int16_t) : 2 * sizeof(float);

  if (sample_rate <= 0) {
    fprintf(stderr, "ERROR - given samplerate '%f' should be > 0\n", sample_rate);
    return -1;
  }

  int ret_val = -1;

  sddc_t *sddc = sddc_open(0, imagefile);
  if (sddc == 0) {
    fprintf(stderr, "ERROR - sddc_open() failed\n");
    return -1;
  }

  /* select output format BEFORE the sample rate (rate semantics differ by format) */
  if (sddc_set_stream_format(sddc, output_int16 ? SDDC_STREAM_INT16 : SDDC_STREAM_CF32) < 0) {
    fprintf(stderr, "ERROR - sddc_set_stream_format() failed\n");
    goto DONE;
  }

  if (sddc_set_sample_rate(sddc, sample_rate) < 0) {
    fprintf(stderr, "ERROR - sddc_set_sample_rate() failed\n");
    goto DONE;
  }

  if (sddc_set_async_params(sddc, 0, 0, count_bytes_callback, sddc) < 0) {
    fprintf(stderr, "ERROR - sddc_set_async_params() failed\n");
    goto DONE;
  }

  if (sddc_set_rf_mode(sddc, VHF_MODE) < 0) {
    fprintf(stderr, "ERROR - sddc_set_rf_mode() failed\n");
    goto DONE;
  }

  if (sddc_set_tuner_frequency(sddc, vhf_frequency) < 0) {
    fprintf(stderr, "ERROR - sddc_set_vhf_frequency() failed\n");
    goto DONE;
  }

  if (sddc_set_tuner_rf_attenuation(sddc, vhf_attenuation) < 0) {
    fprintf(stderr, "ERROR - sddc_set_tuner_rf_attenuation() failed\n");
    goto DONE;
  }

  received_samples = 0;
  num_callbacks = 0;
  if (sddc_start_streaming(sddc) < 0) {
    fprintf(stderr, "ERROR - sddc_start_streaming() failed\n");
    return -1;
  }

  fprintf(stderr, "started streaming .. for %d ms ..\n", runtime);
  total_samples = (unsigned long long)(runtime * sample_rate / 1000.0);

  if (outfilename)
    sampleData = (uint8_t*)malloc(total_samples * bytes_per_sample);

  /* todo: move this into a thread */
  stop_reception = 0;
  clock_gettime(CLOCK_REALTIME, &clk_start);
  /* wall-clock watchdog: stop even if data stalls (e.g. hardware hiccup) so the
   * loop can never hang forever. ~2x the requested runtime, plus 1s of slack. */
  double timeout_sec = runtime / 1000.0 * 2.0 + 1.0;
  while (!stop_reception) {
    sddc_handle_events(sddc);
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    double elapsed = ((double)now.tv_sec + 1.0e-9 * now.tv_nsec) -
                     ((double)clk_start.tv_sec + 1.0e-9 * clk_start.tv_nsec);
    if (elapsed > timeout_sec) {
      fprintf(stderr, "WARNING - wall-clock timeout (%.1f s) reached after %llu samples; stopping.\n",
              timeout_sec, received_samples);
      clk_end = now;
      stop_reception = 1;
    }
  }

  fprintf(stderr, "finished. now stop streaming ..\n");
  if (sddc_stop_streaming(sddc) < 0) {
    fprintf(stderr, "ERROR - sddc_stop_streaming() failed\n");
    return -1;
  }

  double dur = clk_diff();
  fprintf(stderr, "received=%llu %s samples in %d callbacks\n", received_samples,
          output_int16 ? "16-bit real ADC" : "complex CF32 IQ", num_callbacks);
  fprintf(stderr, "run for %f sec\n", dur);
  fprintf(stderr, "approx. samplerate is %f kSamples/sec\n", received_samples / (1000.0*dur) );

  if (outfilename && sampleData && received_samples) {
    FILE * f = fopen(outfilename, "wb");
    if (f) {
      if (output_int16) {
        /* raw real 16-bit ADC samples as 16-bit PCM mono WAV */
        int16_t *s = (int16_t*)sampleData;
        fprintf(stderr, "saving %llu real ADC samples as 16-bit WAV ..\n", received_samples);
        waveWriteHeader( (unsigned)(0.5 + sample_rate), 0U /*frequency*/, 16 /*bitsPerSample*/, 1 /*numChannels*/, f);
        for ( unsigned long long off = 0; off + 65536 < received_samples; off += 65536 )
          waveWriteSamples(f, s + off, 65536, 0 /*needCleanData*/);
        waveFinalizeHeader(f);
      } else {
        /* interleaved complex float32 (CF32): numpy.fromfile(name, dtype=complex64) */
        fprintf(stderr, "saving %llu complex CF32 IQ samples to raw file ..\n", received_samples);
        fwrite(sampleData, bytes_per_sample, received_samples, f);
      }
      fclose(f);
    }
  }

  /* done - all good */
  ret_val = 0;

DONE:
  sddc_close(sddc);

  return ret_val;
}

static void count_bytes_callback(uint32_t data_size,
                                 uint8_t *data,
                                 void *context)
{
  if (stop_reception)
    return;
  ++num_callbacks;
  unsigned N = data_size / bytes_per_sample;
  if ( received_samples + N < total_samples ) {
    if (sampleData)
      memcpy( sampleData + received_samples * bytes_per_sample, data, data_size);
    received_samples += N;
  }
  else {
    clock_gettime(CLOCK_REALTIME, &clk_end);
    stop_reception = 1;
  }
}
