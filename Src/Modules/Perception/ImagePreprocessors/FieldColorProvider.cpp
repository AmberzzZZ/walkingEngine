/*
 * @file FieldColorProvider.cpp
 * @This module provider an auto color detect of field color. Currently support Green White Black
 * @author Li Shu
 */
#include "FieldColorProvider.h"

#include "Representations/Perception/ImagePreprocessing/ColorConvert.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/ImageProcessing/YHSColorConversion.h"

MAKE_MODULE(FieldColorProvider, perception);
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <deque>
#include <chrono>
#include "ext_math.h"

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Transformation.h"

//#define SHOW_TIME //Processing time will be shown if enabled
//#define NO_CHECK

using namespace ext_math;
using namespace std;
using namespace std::chrono;

FieldColorProvider::FieldColorProvider()
{
}

FieldColorProvider::~FieldColorProvider()
{
}

void FieldColorProvider::update(FieldColor& fieldColor)
{
#ifdef SHOW_TIME
    high_resolution_clock::time_point startT, endT;
    startT = high_resolution_clock::now();
#endif
    /*excute proceed function to get field color features*/
    proceed(theImage, theColorConvert);
    if (checkColorValidity())
//    if(1)
    {
    /*Update representation variables*/
    fieldColor.seedY = this->seedY;
    fieldColor.seedCb = this->seedCb;
    fieldColor.seedCr = this->seedCr;
    fieldColor.minCy = this->minCy;
    fieldColor.minCb = this->minCb;
    fieldColor.minCr = this->minCr;
    fieldColor.minH = this->minH;
    fieldColor.minS = this->minS;
    fieldColor.maxCy = this->maxCy;
    fieldColor.maxCb = this->maxCb;
    fieldColor.maxCr = this->maxCr;
    fieldColor.maxH = this->maxH;
    fieldColor.maxS = this->maxS;
    fieldColor.W_Ymin = this->seedY + 10;
    fieldColor.W_Cbmin = this->seedCb - 20;
    fieldColor.W_Crmin = this->seedCr - 5;
    fieldColor.W_Smax = static_cast<int>(this->G_Smax * 0.8f);
    if (theCameraInfo.camera == CameraInfo::lower)
    {
//    	fieldColor.minS -= 10;
    }
//    fieldColor.W_Vmin = this->G_Vmax + 10;
//    fieldColor.W_Rmin = this->G_Rmax * 2;
//    fieldColor.W_Gmin = static_cast<int>(this->G_Gmax * 1.1f);
//    fieldColor.W_Bmin = this->G_Bmax;
    }
#ifdef SHOW_TIME
    endT = high_resolution_clock::now();
    duration<double> dTotal = duration_cast<duration<double> >(endT - startT);
    std::cout << "[Processing Time] FieldColorProvider----->" << dTotal.count() * 1000 << "ms" << std::endl;
#endif
}

bool FieldColorProvider::checkColorValidity() const
{
#ifdef NO_CHECK
	return true;
#else
	unsigned char h,s,i,sr,sg,sb;
    ColorModelConversions::fromYUVToHSI(static_cast<unsigned char>(this->seedY),
										static_cast<unsigned char>(this->seedCb),
										static_cast<unsigned char>(this->seedCr),h,s,i);
    ColorModelConversions::fromYUVToRGB(static_cast<unsigned char>(this->seedY),
										static_cast<unsigned char>(this->seedCb),
										static_cast<unsigned char>(this->seedCr),sr,sg,sb);
	if (int(s) - int(i) < 0 || int(sg) - int(sr) < 40 || int(sb) - int(sr) < -5
			|| this->seedCb < this->seedCr)
		return false;
	return true;
#endif
}

/**
 * @brief main function, should be processed every frame to get color thresholds
 * @param img image to be scaned
 * @param ct the color convert table
 */
void FieldColorProvider::proceed(const Image& img, const ColorConvert& ct)
{
    width = img.width;
    height = img.height;
    seedY = 0;
    seedCr = 0;
    seedCb = 0;
    resetArrays();
    searchInitialSeed(img);
    /*if what want to do something here*/
    //  time++;
    //  if (time % 30 == 0)
    //  {
    //    time = 0;
    //  }
    extractFeatures(img, features);
    setYCbCrCube(features);
    setHSVCube(img, ct);
}

/**
 * @brief reset each histograms
 */
void FieldColorProvider::resetArrays()
{
    memset(histY, 0, sizeof(histY));
    memset(histCb, 0, sizeof(histCb));
    memset(histCr, 0, sizeof(histCr));
    memset(histH, 0, sizeof(histH));
    memset(histS, 0, sizeof(histS));
    memset(histV, 0, sizeof(histV));
//    memset(histR, 0, sizeof(histR));
//    memset(histG, 0, sizeof(histG));
//    memset(histB, 0, sizeof(histB));
}

/**
 * @brief find potential seeds of green
 * @param img the image to be scaned
 */
void FieldColorProvider::searchInitialSeed(const Image& img)
{
    // Cr通道柱状图
    int seedSearchBorder = width / 16;
    for(int y = seedSearchBorder; y < height - 1 - seedSearchBorder; y += pixelSpacing)
	{
	    for(int x = seedSearchBorder; x < width - seedSearchBorder; x += pixelSpacing)
		{
		    //            if (bc.isValidPoint(Vector2i(x,y)))
		    //            {
		    int cr = img[y][x].cr;
		    histCr[cr]++;

		    //            }
		}
	}

    //绿色Cr
    seedCr = clamp(colorBorder, getStableMin(histCr, 150), 255 - colorBorder);

    // Cb通道柱状图(只考虑与绿色Cr相近的像素点)
    for(int y = 0; y < height - 1; y += pixelSpacing)
	{
	    for(int x = 0; x < width; x += pixelSpacing)
		{
		    //            if (bc.isValidPoint(Vector2i(x,y)))
		    //            {
		    int cr = img[y][x].cr;
		    if(abs(cr - seedCr) < 4)
			{
			    int cb = img[y][x].cb;
			    histCb[cb]++;
			    //            }
			}
		}
	}
    //绿色Cb
    seedCb = clamp(colorBorder, getPeak(histCb), 255 - colorBorder);

    //绿色Cy
    for(int y = 0; y < height - 1; y += pixelSpacing)
	{
	    for(int x = 0; x < width; x += pixelSpacing)
		{
		    //            if (bc.isValidPoint(Vector2i(x,y)))
		    //            {
		    int cr = img[y][x].cr;
		    if(abs(cr - seedCr) < 8)
			{
			    int cb = img[y][x].cb;
			    if(abs(cb - seedCb) < 8)
				{
				    int cy = img[y][x].y;
				    histY[cy]++;
				}
			    //            }
			}
		}
	}

    seedY = clamp(colorBorder, getPeak(histY), 255 - colorBorder);
    greenCy = seedY;
    greenCb = seedCb;
    greenCr = seedCr;
}

/**
 * @brief get stable min value of a histogram
 * @param hist histogram to be scaned
 * @param thres min threshold
 * @return the index of stabel min
 */
int FieldColorProvider::getStableMin(const int* const hist, int thres)
{
    int sum = 0;
    for(int i = 0; i < 256; i++)
	{
	    sum += hist[i];
	    if(sum > thres)
		{
		    return i;
		}
	}
    return 0;
}

/**
 * @brief find max value of a histogram
 * @param hist histogram to be scaned
 * @return the index fo max value
 */
int FieldColorProvider::getPeak(const int* const hist)
{
    int max = 0;
    int maxIdx = 0;
    for(int i = 0; i < 256; i++)
	{
	    if(hist[i] > max)
		{
		    max = hist[i];
		    maxIdx = i;
		}
	}
    return maxIdx;
}

/**
 * @brief find field features
 * @param img the image to be scaned
 * @param features features of the field
 */
void FieldColorProvider::extractFeatures(const Image& img, float* features)
{
    int cnt = 0;
    float meanY = 0;
    float varY = 0;
    float varCb = 0;
    float varCr = 0;
    float sumGreen1 = 0;
    float sumGreen2 = 0;
    for(int y = pixelSpacing / 2; y < height; y += pixelSpacing)
	{
	    for(int x = pixelSpacing / 2; x < width; x += pixelSpacing)
		{
		    //            if (bc.isValidPoint(Vector2i(x,y)))
		    //            {
		    int cy = img[y][x].y;
		    int cb = img[y][x].cb;
		    int cr = img[y][x].cr;
		    meanY += cy;
		    varCb += (cb - 128) * (cb - 128);
		    varCr += (cr - 128) * (cr - 128);
		    if(abs(cr - seedCr) <= 2 && abs(cb - seedCb) <= 2 && abs(cy - seedY) <= 2)
			{
			    sumGreen2++;
			    if(abs(cr - seedCr) <= 1 && abs(cb - seedCb) <= 1 && abs(cy - seedY) <= 1)
				{
				    sumGreen1++;
				}
			}
		    cnt++;
		    //            }
		}
	}
    sumGreen1 /= cnt;
    sumGreen2 /= cnt;
    varCb = sqrtf(varCb / cnt);
    varCr = sqrtf(varCr / cnt);
    meanY /= cnt;
    for(int y = pixelSpacing / 2; y < height; y += pixelSpacing)
	{
	    for(int x = pixelSpacing; x < width; x += pixelSpacing)
		{
		    //            if (bc.isValidPoint(Vector2i(x,y)))
		    //            {
		    int cy = img[y][x].y;
		    varY += (cy - meanY) * (cy - meanY);
		    //            }
		}
	}
    varY = sqrtf(varY / cnt);
    features[0] = greenCy / 256;
    features[1] = varY / 32;
    features[2] = varCb / 16;
    features[3] = varCr / 16;
    features[4] = sumGreen1 * 50;
    features[5] = sumGreen2 * 25;
    features[6] = (sumGreen2 - sumGreen1) * 50;
}

/**
 * @brief get range of YCbCr space
 * @param features detected field features
 */
void FieldColorProvider::setYCbCrCube(float* features)
{
    int idx = 0;
    float minCy_ = 25 + 50 * thetas[idx++];
    float minCb_ = 8 + 15 * thetas[idx++];
    float minCr_ = 8 + 15 * thetas[idx++];
    for(int j = 1; j <= NUM_FEATURES; j++)
	{
	    float feature = static_cast<float>(pow(features[j - 1], 1.3 + thetas[idx++]));
	    minCy_ += 100 * thetas[idx++] * feature;
	    minCb_ += 30 * thetas[idx++] * feature;
	    minCr_ += 30 * thetas[idx++] * feature;
	}
#ifdef GRASSGROUND
    if(minCy_ < 32)
	minCy_ = 32;
    if(minCy_ > 80)
	minCy_ = 80;
    if(minCb_ < 16)
	minCb_ = 16;
    if(minCb_ > 30)
	minCb_ = 30;
    if(minCr_ < 16)
	minCr_ = 16;
    if(minCr_ > 30)
	minCr = 30;
#else
    if(minCy_ < 1)
	minCy_ = 1;
    if(minCy_ > 80)
	minCy_ = 80;
    if(minCb_ < 1)
	minCb_ = 1;
    if(minCb_ > 30)
	minCb_ = 30;
    if(minCr_ < 1)
	minCr_ = 1;
    if(minCr_ > 30)
	minCr_ = 30;
#endif
    //    std::cout<< "CalculatedMinCy=" << minCy << std::endl;
    this->minCy = (int)(greenCy - minCy_);
    this->minCb = (int)(greenCb - minCb_);
    this->minCr = (int)(greenCr - minCr_);
    this->minCy2 = (int)(greenCy - minCy_ * greenGain);
    this->minCb2 = (int)(greenCb - minCb_ * greenGain);
    this->minCr2 = (int)(greenCr - minCr_ * greenGain);

    float maxCy_ = 25 + 50 * thetas[idx++];
    float maxCb_ = 8 + 15 * thetas[idx++];
    float maxCr_ = 8 + 15 * thetas[idx++];
    for(int j = 1; j <= NUM_FEATURES; j++)
	{
	    float feature = static_cast<float>(pow(features[j - 1], 1.3 + thetas[idx++]));
	    maxCy_ += 100 * thetas[idx++] * feature;
	    maxCb_ += 30 * thetas[idx++] * feature;
	    maxCr_ += 30 * thetas[idx++] * feature;
	}
#ifdef GRASSGROUND
    if(maxCy_ < 32)
	maxCy_ = 32;
    if(maxCy_ > 80)
	maxCy_ = 80;
    if(maxCb_ < 16)
	maxCb_ = 16;
    if(maxCb_ > 30)
	maxCb_ = 30;
    if(maxCr_ < 16)
	maxCr_ = 16;
    if(maxCr_ > 30)
	maxCr_ = 30;
#else
    if(maxCy_ < 1)
	maxCy_ = 10;
    if(maxCy_ > 80)
	maxCy_ = 80;
    if(maxCb_ < 1)
	maxCb_ = 1;
    if(maxCb_ > 30)
	maxCb_ = 30;
    if(maxCr_ < 1)
	maxCr_ = 1;
    if(maxCr_ > 30)
	maxCr_ = 30;
#endif
    this->maxCy = (int)(greenCy + maxCy_);
    this->maxCb = (int)(greenCb + maxCb_);
    this->maxCr = (int)(greenCr + maxCr_);
    this->maxCy2 = (int)(greenCy + maxCy_ * greenGain);
    this->maxCb2 = (int)(greenCb + maxCb_ * greenGain);
    this->maxCr2 = (int)(greenCr + maxCr_ * greenGain);

    this->minCy = clamp(0, this->minCy, 255);
    this->maxCy = clamp(0, this->maxCy, 255);

    DRAWTEXT("module:FieldColorProvider:ColorRange",
             20,
             20,
             15,
             ColorRGBA::magenta,
             "maxCy = " << maxCy_ << " minCy = " << minCy_);
}

/**
 * @brief get range of HSV space
 * @param img the image to be scaned
 * @param ct the color convert table
 */
void FieldColorProvider::setHSVCube(const Image& img, const ColorConvert& ct)
{
    DECLARE_DEBUG_DRAWING("module:FieldColorProvider:ColorRange", "drawingOnImage");
    for(int j = 16; j < height - 16; j += 8)
	{
	    for(int x = 16; x < width - 16; x += 8)
		{
		    Image::Pixel ss, sss;
		    ct.getHSV(img[j][x], ss);
//		    std::cout<<"s---->"<<static_cast<int>(ss.s)<<std::endl;
		    ct.getRGB(img[j][x], sss);
		    if(isGreen(img[j][x].y, img[j][x].cb, img[j][x].cr))
			{
			    histH[ss.h]++;
			    histS[ss.s]++;
//			    histV[ss.i]++;
//			    histR[sss.r]++;
//			    histG[sss.g]++;
//			    histB[sss.b]++;
			}
		}
	}

    this->G_Smax = getPeak(histS);
//    this->G_Vmax = getPeak(histV);
//    this->G_Rmax = getPeak(histR);
//    this->G_Gmax = getPeak(histG);
//    this->G_Bmax = getPeak(histB);
    int peakH = getPeak(histH);
    int peakS = this->G_Smax;
    int maxH_ = 0;
    for(int j = 0; j < 256; j++)
	{
	    if(histH[j] > maxH_)
		maxH_ = histH[j];
	}
    int boundH = static_cast<int>(0.001 * maxH_);
    boundH = boundH < 1 ? 1 : boundH;
    int maxS_ = 0;
    for(int j = 0; j < 256; j++)
	{
	    if(histS[j] > maxS_)
		maxS_ = histS[j];
	}
    int boundS = static_cast<int>(0.001 * maxS_);
    boundS = boundS < 1 ? 1 : boundS;

    this->maxH = findContinuousUpperBound(histH, boundH);
    this->minH = findContinuousLowerBound(histH, boundH);
    this->maxS = findContinuousUpperBound(histS, boundS);
    this->minS = findContinuousLowerBound(histS, boundS);

    // Determine white threshold

    DRAWTEXT("module:FieldColorProvider:ColorRange",
             20,
             0,
             15,
             ColorRGBA(0, 0, 255, 255),
             "boundH = " << boundH << " boundS = " << peakS << "  maxH = " << maxH_ << " maxS = " << maxS_);
    //     "GreenH="<<this->maxH+8 << "~" << this->minH-8
    //     <<"  GreenS="<<this->maxS+8 << "~" << this->minS-8);
}

int FieldColorProvider::findContinuousLowerBound(const int* hist, int bound)
{
    const int skipMax = 8;
    int skip = 0;
    int peak = getPeak(hist);
    int i = peak;
    for(i = peak; i >= 0; --i)
	{
	    if(hist[i] < bound)
		skip++;
	    else
		skip--;
	    if(skip < 0)
		skip = 0;
	    if(skip > skipMax)
		break;
	}
    i += skip;
    i = i < 0 ? 0 : i;
    return i;
}

int FieldColorProvider::findContinuousUpperBound(const int* hist, int bound)
{
    const int skipMax = 8;
    int skip = 0;
    int peak = getPeak(hist);
    int i = peak;
    for(i = peak; i <= 255; ++i)
	{
	    if(hist[i] < bound)
		skip++;
	    else
		skip--;
	    if(skip < 0)
		skip = 0;
	    if(skip > skipMax)
		break;
	}
    i -= skip;
    i = i > 255 ? 255 : i;
    return i;
}

