/*!
 * \brief Short Time Fourier Transform.
 * \author Thomas Hamboeck, Austrian Kangaroos 2014
 */
#include "STFT.h"

#include <limits>
#include <complex>
#include <iostream>

#ifndef WINDOWS
#define WARN(cond, str)     do { if(!(cond)) { std::cerr << "Warning: " << str << std::endl; } } while(0);

STFT::STFT(const int channelOffset, const int windowTime, const int windowTimeStep, const int windowFrequency,
           std::function<void (const double *spectrum, int length)> handleSpectrum)
    : offset(channelOffset),
      windowTime(windowTime), windowTimeStep(windowTimeStep), windowFrequency(windowFrequency), windowFrequencyHalf(windowFrequency / 2 + 1),
      handleSpectrum(handleSpectrum),
      nOverflow(0), overflownData(NULL), input(NULL), output(NULL), outputMag(NULL)
{
    overflownData   = new double[windowTime]; /* actually a max of (windowTime - 1) */
    input           = static_cast<double*>(fftw_malloc(sizeof(double) * windowFrequency));
    output          = static_cast<fftw_complex*>(fftw_malloc(sizeof(fftw_complex) * windowFrequencyHalf));
    outputMag       = new double[windowFrequencyHalf];

    WARN(windowFrequency >= windowTime, "Frequency window must be greater than Time Window.");

    for(int i = 0; i < windowFrequency; ++i) {
        input[i] = 0.0f;
    }

    plan = fftw_plan_dft_r2c_1d(windowFrequency, input, output, FFTW_MEASURE);

}

STFT::~STFT()
{
    if(overflownData) {
        delete[] overflownData;
    }
    if(input) {
        fftw_free(input);
    }
    if(output) {
        fftw_free(output);
    }
    if(outputMag) {
        delete[] outputMag;
    }
    if(plan) {
        fftw_destroy_plan(plan);
    }
}

void STFT::newData(const double *data, int length, short channels)
{
    int iBegin, iEnd, iDataChannel, iBuffer;

    iBuffer = 0;

    /* for each overflown data */
    while(iBuffer < nOverflow) {
        input[iBuffer] = overflownData[iBuffer];
        ++iBuffer;
    }

    iBegin = 0;
    while(true) {
        iEnd = iBegin + windowTimeStep;
        if(iEnd > length) {
            break;
        }

        iDataChannel = iBegin * channels + offset;
        while(iBuffer < windowTime) {
            input[iBuffer] = data[iDataChannel];
            ++iBuffer;
            iDataChannel += channels;
        }
        /* and the rest is zero */

        fftw_execute(plan);

        /* calc magnitude */
        for(int i = 0; i < windowFrequencyHalf; ++i) {
            outputMag[i] = std::abs(*reinterpret_cast<std::complex<double>* >(&output[i]));
        }
        handleSpectrum(outputMag, windowFrequencyHalf);

        /* next cycle */
        iBuffer = 0;
        iBegin  += windowTimeStep;
    }

    nOverflow = 0;
    iDataChannel = iBegin * channels + offset;
    while(iBegin < length) {
        /* copy to overflow buffer */
        overflownData[nOverflow] = data[iDataChannel];
        ++nOverflow;
        ++iBegin;
        iDataChannel += channels;
    }
}

#endif
