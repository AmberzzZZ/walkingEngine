/*
 * @file WhistleRecognizer.h
 *
 * Declaration of module that identifies the sound of a whistle
 *
 * @author Dennis Schuethe
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Modeling/Whistle.h"
#include "STFT.h"

#ifndef WINDOWS
// We currently do not have a working FFTW3 implementation
// for Windows in our repository.
#include <fftw3.h>

const int WHISTLE_BUFF_LEN = 1024;                  // dataIn
const int WHISTLE_OVERLAP = (WHISTLE_BUFF_LEN-256); // make 1/4 of the buffer old data and 3/4 new data (256 of old data)
#endif

MODULE(WhistleRecognizer2,
{,
  REQUIRES(AudioData),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(DamageConfigurationHead),
  PROVIDES(Whistle),
  DEFINES_PARAMETERS(
  {,
    (float)(5.5f) whistleThreshold,            /**< Minimum correlation for accepting sound as whistle */
    (float)(0.25) volumeThreshold,             /**< Minimum sound intensity for accepting sound as whistle */
//    (int)(2000) whistleBegin,
//    (int)(3000) whistleEnd,
    (int)(5000) whistleBegin,
	(int)(6000) whistleEnd,
    (int)(160) windowSize,
    (int)(200) windowSizePadded,
    (int)(80) windowSkipping,
    (unsigned)(30) frameOkays,//default 30
    (unsigned)(7) frameMisses,
    (int)(600) timeForOneChannelAcceptance, /**< After having heard the whistle on both channels, this is the amount of time in which one channel is still sufficient */
  }),
});

/*
 * @class WhistleRecognizer
 *
 * Module that identifies the sound of a whistle in audio data
 */
class WhistleRecognizer2 : public WhistleRecognizer2Base
{
#ifndef WINDOWS
public:
  /** Constructor */
  WhistleRecognizer2();

  /** Destructor */
  ~WhistleRecognizer2();

private:
  double *rawData;
  RingBuffer<float, WHISTLE_BUFF_LEN> inputChannel0; /** Audio data from the first channel */
  RingBuffer<float, WHISTLE_BUFF_LEN> inputChannel1; /** Audio data from the second channel */

  int cmpCnt;                    /**< Number of new audio samples */
  uint8_t lastGameState;         /**< Keep last game state for checking state transition to SET */
  unsigned int lastTimeWhistleDetectedInBothChannels; /**< As the name says ... */

  int beg;
  int end;
  /**
   * Method for recognizing a whistle in one channel
   * @param inputChannel The incoming audio data
   * @return true, if a whistle has been recognized, false otherwise
   */
  bool detectWhistle(const RingBuffer<float, WHISTLE_BUFF_LEN>& inputChannel);

  /**
   * Determines the current average sound level
   */
  float computeCurrentVolume();

#endif

  /**
   * The method that detects the whistle
   * @param Whistle The identified whistle
   */
  void update(Whistle& whistle);
};
