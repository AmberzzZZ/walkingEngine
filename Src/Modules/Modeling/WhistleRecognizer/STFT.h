/*!
 * \brief Short Time Fourier Transform.
 * \author Thomas Hamboeck, Austrian Kangaroos 2014
 */

#pragma once

#ifndef WINDOWS
#include <fftw3.h>
#include <complex>
#include <functional>

class STFT
{
public:
    STFT(const int channelOffset, const int windowTime, const int windowTimeStep, const int windowFrequency,
         std::function<void (const double *spectrum, int length)> handleSpectrum);
    virtual ~STFT();

    void newData(const double*data, int length, short channels);

protected:
    const int offset;
    const int windowTime, windowTimeStep, windowFrequency, windowFrequencyHalf;
    std::function<void (const double *spectrum, int length)> handleSpectrum;

    int nOverflow;
    double *overflownData;
    double *input;
    fftw_complex *output;
    double *outputMag;

    fftw_plan plan;
};

#endif
