#ifndef __JERSEYCOLORDETECTOR2_H__
#define __JERSEYCOLORDETECTOR2_H__

#include <stdint.h>

#include <vector>

#include "RobotRect.h"
//#include "TeamMembership2.h"
//#include "FieldColorDetector2.h"
#include "Representations/Infrastructure/Image.h"

class JerseyColorDetector
{
 private:
  struct ycbcr32_t
  {
    int32_t y;
    int32_t cb;
    int32_t cr;
  };

  float offsetColorAngle;
  float offsetColors;
  float sensitivityColors;
  float sensitivityColorAngle;
  float colorAngleBlue;
  float colorAngleRed;
  float pixelCalibrationRatio;
  float pixelSelectionRatio;
  float topRegionHeight;
  float bottomRegionHeight;

  int gridSize;

  int histCbTop[256];
  int histCrTop[256];
  int histYBottom[256];

 public:
  JerseyColorDetector();
  ~JerseyColorDetector(){}
    float isBlueJersey(const Image& img, RobotRect r);
	void setHistogramY(const Image& img, RobotRect rect, int* hist) __attribute__((nonnull));
	void setHistogramCb(const Image& img, RobotRect rect, int* hist) __attribute__((nonnull));
	void setHistogramCr(const Image& img, RobotRect rect, int* hist) __attribute__((nonnull));
	int getThreshold(const int* const hist, float ratio) __attribute__((nonnull));
	int getAvgMaxValue(const Image& img, RobotRect rect, int thres, int ay, int ab, int ar, int by, int bb, int br);
	
	void reset();
	
	inline int getY(const Image & img, int x, int y)
	{
		return static_cast<int>(img.getFullSizePixel(y,x).y);
	}
	
	inline int getCb(const Image & img, int x, int y)
	{
		return static_cast<int>(img.getFullSizePixel(y,x).cb);
	}
	
	inline int getCr(const Image & img, int x, int y)
	{
		return static_cast<int>(img.getFullSizePixel(y,x).cr);
	}
};

#endif
