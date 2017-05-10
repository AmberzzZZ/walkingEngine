/*
 * @file WhistleRecognizer.cpp
 *
 * Implementation of module that identifies the sound of a whistle
 *
 * @author Dennis Schuethe
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#include "WhistleRecognizer2.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDrawings.h"

#include <iostream>

MAKE_MODULE(WhistleRecognizer2, modeling)

#ifndef WINDOWS
// We currently do not have a working FFTW3 implementation
// for Windows in our repository.

#include <limits>
#include "Platform/Linux/Thread.h"

static SyncObject _syncObject;

WhistleRecognizer2::WhistleRecognizer2()
{
  rawData = new double[WHISTLE_BUFF_LEN];
  for(int i = 0; i < WHISTLE_BUFF_LEN; ++i)
  {
    inputChannel0.push_front(0);
    inputChannel1.push_front(0);
  }
  cmpCnt = 0;
  lastGameState = STATE_INITIAL;
  lastTimeWhistleDetectedInBothChannels = 0;

  ASSERT(theAudioData.sampleRate == 8000); // sampleRate should be 8000;
  beg = whistleBegin * windowSizePadded / theAudioData.sampleRate;
  end = whistleEnd * windowSizePadded / theAudioData.sampleRate;

  // Create plans
  //  - after plan has been created, in and out buffer are set to zero
  //  - plan has to be created only once
  // Creation of FFTW plans is not thread-safe, thus we need to synchronize with the other threads
  //   - This is only relevant for simulations that contain multiple robots
  SYNC;
}

WhistleRecognizer2::~WhistleRecognizer2()
{
  delete[] rawData;
  // Destruction of FFTW plans is not thread-safe
  SYNC;
}

#endif

void WhistleRecognizer2::update(Whistle& whistle)
{
  DEBUG_RESPONSE_ONCE("module:WhistleRecognizer:whistle")
  {
    whistle.lastTimeWhistleDetected = theFrameInfo.time;
    whistle.lastTimeOfIncomingSound = theFrameInfo.time;
    whistle.confidenceOfLastWhistleDetection = 100;
    return;
  }
  DEBUG_RESPONSE_ONCE("module:WhistleRecognizer:sound")
  {
    whistle.lastTimeOfIncomingSound = theFrameInfo.time;
    return;
  }

#ifdef WINDOWS // Windows implementation ends here, remaning stuff uses FFTW
}
#else

  // Only listen to the whistle in set state and clear
  // the buffers when entering a set state:
  if(lastGameState != STATE_SET && theGameInfo.state == STATE_SET)
  {
    for(int i = 0; i < WHISTLE_BUFF_LEN; ++i)
    {
      inputChannel0.push_front(0);
      inputChannel1.push_front(0);
    }
    cmpCnt = 0;
  }
  lastGameState = theGameInfo.state;
  if(theGameInfo.state != STATE_SET)
    return;

  // Check input data:
  if(theAudioData.channels != 2)
  {
    OUTPUT_TEXT("Wrong number of channels! WhistleRecognizer expects 2 channels, but AudioData has " << theAudioData.channels << "!");
  }
  if(theAudioData.samples.size() == 0)
    return;
  else
    whistle.lastTimeOfIncomingSound = theFrameInfo.time;

  // Add incoming audio data to the two buffers
  unsigned int i = 0;
  while(i < theAudioData.samples.size())
  {
    cmpCnt++;
    const float sample0 = static_cast<float>(theAudioData.samples[i]) / static_cast<float>(std::numeric_limits<short>::max());
    ++i;
    const float sample1 = static_cast<float>(theAudioData.samples[i]) / static_cast<float>(std::numeric_limits<short>::max());
    ++i;
    inputChannel0.push_front(sample0);
    inputChannel1.push_front(sample1);
  }

  // Recognize the whistle
  float  currentVolume = 0.0;
  if(inputChannel0.full() && cmpCnt >= WHISTLE_OVERLAP)
  {
    cmpCnt = 0;
    const bool w0 = detectWhistle(inputChannel0);
    const bool w1 = detectWhistle(inputChannel1);
    currentVolume = computeCurrentVolume();
//    std::cout<< "vol:" <<currentVolume<<std::endl;
    const bool vol = currentVolume > volumeThreshold;
    // Best case: Everything is fine!
//    std::cout<<w0<<"   "<<w1<<"   "<<vol<<std::endl;
    if(w0 && w1 && vol && !theDamageConfigurationHead.audioChannel0Defect && !theDamageConfigurationHead.audioChannel1Defect)
    {
      lastTimeWhistleDetectedInBothChannels = theFrameInfo.time;
      whistle.lastTimeWhistleDetected = theFrameInfo.time;
      whistle.confidenceOfLastWhistleDetection = 100;
    }
    // One ear ist damaged but I can hear the sound on the other ear:
    else if((w0 && vol && !theDamageConfigurationHead.audioChannel0Defect && theDamageConfigurationHead.audioChannel1Defect) ||
            (w1 && vol && !theDamageConfigurationHead.audioChannel1Defect && theDamageConfigurationHead.audioChannel0Defect))
    {
      whistle.lastTimeWhistleDetected = theFrameInfo.time;
      whistle.confidenceOfLastWhistleDetection = 66;
    }
    // Last (positive) case: Both ears are OK, but I can hear the sound on one ear, only:
    else if(!theDamageConfigurationHead.audioChannel0Defect && !theDamageConfigurationHead.audioChannel1Defect &&
            ((w0 && !w1) || (!w0 && w1)) && vol)
    {
      whistle.lastTimeWhistleDetected = theFrameInfo.time;
      if(theFrameInfo.getTimeSince(lastTimeWhistleDetectedInBothChannels) > timeForOneChannelAcceptance)
        whistle.confidenceOfLastWhistleDetection = 66;
      else
        whistle.confidenceOfLastWhistleDetection = 100;
    }
    // Finally, a completely deaf robot has a negative confidence:
    else
    {
      whistle.confidenceOfLastWhistleDetection = -1; // Not a detection but other robots get the information to ignore me
    }
//    std::cout<<"whistle confidence is: "<<(int)whistle.confidenceOfLastWhistleDetection<<std::endl;
  };
}

bool WhistleRecognizer2::detectWhistle(
    const RingBuffer<float, WHISTLE_BUFF_LEN>& inputChannel)
{
  bool isDetect = false;
  auto calcMeanDeviation = [&] (const double *data, int length, float &mean, float &dev) {
    mean = dev = 0;
    for(int i = 0; i < length; ++i) {
      mean += data[i];
      dev += data[i] * data[i];
    }
    dev = std::sqrt(length * dev - mean * mean) / length;
    mean /= length;
  };

  /* start fft stuff */
  auto handleSpectrum = [&] (const double *spectrum, int length) {
    static unsigned whistleCounter(0), whistleMissCounter(0), whistleDone(false);
    float mean, dev;
    calcMeanDeviation(spectrum, length, mean, dev);
    bool found;
    const float whistleThresh = mean + whistleThreshold * dev;
    found = false;
    std::cout<<"whistleThresh: "<<whistleThresh<<" mean:"<<mean<<" dev:"<<dev<<std::endl;
    for (int i = beg; i < end; ++i) {
      if (spectrum[i] > whistleThresh) {
        found = true;
        break;
      }
    }
    if (whistleDone) {
      if (!found) {
        ++whistleMissCounter;
        if (whistleMissCounter > frameMisses) {
          whistleCounter = 0;
          whistleMissCounter = 0;
          whistleDone = false;
        }
      }
    }
    else {
      if (found) {
        ++whistleCounter;
        whistleMissCounter = 0;
      }
      else if (whistleCounter > 0) {
        ++whistleMissCounter;
        if(whistleMissCounter > frameMisses) {
          whistleCounter = 0;
          whistleMissCounter = 0;
          whistleDone = false;
        }
      }
      if (whistleCounter >= frameOkays) {
        std::cout<< "detect..."<<std::endl;
        isDetect = true;
        whistleCounter = 0;
        whistleMissCounter = 0;
        whistleDone = true;
      }
    }
  };
//  std::cout<<"spectrum=------->"<<handleSpectrum<<std::endl;
  STFT stft(0, windowSize, windowSkipping, windowSizePadded, handleSpectrum);
  for (int j=0; j<WHISTLE_BUFF_LEN; j++)
  {
    rawData[j] = inputChannel[j]; // write just the half of dataIn and the rest is zero padded
  }
  stft.newData(rawData, WHISTLE_BUFF_LEN, 1);
  return isDetect;
}

float WhistleRecognizer2::computeCurrentVolume()
{
  float volume0 = 0.0;
  float volume1 = 0.0;
  if(!theDamageConfigurationHead.audioChannel0Defect)
  {
    for(float sample : inputChannel0)
      volume0 += std::abs(sample);
  }
  volume0 /= static_cast<float>(inputChannel0.size());
  if(!theDamageConfigurationHead.audioChannel1Defect)
  {
    for(float sample : inputChannel1)
      volume1 += std::abs(sample);
  }
  volume1 /= static_cast<float>(inputChannel1.size());
  if(!theDamageConfigurationHead.audioChannel0Defect && !theDamageConfigurationHead.audioChannel1Defect)
    return (volume0 + volume1) / 2.f;
  else if(theDamageConfigurationHead.audioChannel0Defect)
    return volume1;
  else // this means if(theDamageConfiguration.audioChannel1Defect)
    return volume0;
}

#endif
