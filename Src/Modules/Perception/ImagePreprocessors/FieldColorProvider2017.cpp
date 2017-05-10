/*
 * @file FieldColorProvider2017.cpp
 * @This module provider an auto color detect of field color. Currently support Green White Black
 * @author Zeng Zhiying
 */
#include "FieldColorProvider2017.h"

#include "Representations/Perception/ImagePreprocessing/ColorConvert.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugImages.h"
#include "Tools/ImageProcessing/YHSColorConversion.h"

MAKE_MODULE(FieldColorProvider2017, perception);
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

FieldColorProvider2017::FieldColorProvider2017()
{
}

FieldColorProvider2017::~FieldColorProvider2017()
{
}

void FieldColorProvider2017::update(FieldColor& fieldColor)
{
	DECLARE_DEBUG_DRAWING("module:FieldColorProvider2017:ColorRange", "drawingOnImage");
#ifdef SHOW_TIME
    high_resolution_clock::time_point startT, endT;
    startT = high_resolution_clock::now();
#endif
    initial();
	proceed(theImage, theColorConvert);
	fieldColor.seedY = this->seedY;
	fieldColor.minCy = this->minY;
	fieldColor.maxCy = this->maxY;
	fieldColor.minH = this->minH;
	fieldColor.maxH = this->maxH;
	fieldColor.minS = this->minS;
	fieldColor.maxS = this->maxS;
}

void FieldColorProvider2017::proceed(const Image& img, const ColorConvert& ct)
{
    width = img.width;
    height = img.height;
    int seedSearchBorder = width / 16;
    const int minH_ = 170;
    const int maxH_ = 210;
    for(int y = seedSearchBorder; y < height -1 - seedSearchBorder; y += pixelSpacing)
    {
    	for(int x = seedSearchBorder; x < width - 1 - seedSearchBorder; x += pixelSpacing)
    	{
    		Image::Pixel hsv;
    		ct.getHSV(img[y][x],hsv);
    		int h = hsv.h;
    		if(h > minH_ && h < maxH_)
    		{
				int s = static_cast<int>(hsv.s);
				histS[s] ++;
    		}
    	}
    }
    int min_cnt_s = 100000;
    int minS_ = 100;
    int maxS_ = 255;
    int skip_s = 0;
    for(int i = 100; i < 140; i ++)
    {
//    	int cnt = histS[i - 2] * 0.1f + histS[i - 1] * 0.15f + histS[i] * 0.5f + histS[i + 1] * 0.15f + histS[i + 2] * 0.1f;
//    	if(min_cnt_s > cnt)
//    	{
//    		min_cnt_s = cnt;
//    		minS_ = i;
//    	}
    	int cnt = histS[i];
    	if(cnt > 20)
    	{
    		skip_s ++;
    		if(skip_s > 2)
    		{
    			minS_ = i;
    			break;
    		}
    	}
    }
//    minS_ -= 5;
//    if(minS_ < 100)
//    {
//    	minS_ = 100;
//    }
//    minS_ = 100;

    int sum_y = 0;
    int cnt_y = 0;
    for(int y = seedSearchBorder; y < height -1 - seedSearchBorder; y += pixelSpacing)
	{
		for(int x = seedSearchBorder; x < width - 1 - seedSearchBorder; x += pixelSpacing)
		{
			Image::Pixel hsv;
			ct.getHSV(img[y][x],hsv);
			int h = hsv.h;
			int s = hsv.s;
			if(h > minH_ && h < maxH_ && s > minS_ + 15)
			{
				int cy = img[y][x].y;
				sum_y += cy;
				cnt_y ++;
				histY[cy] ++;
			}
		}
	}
//    ASSERT(cnt_y != 0);
    sum_y = cnt_y == 0? 0: sum_y / cnt_y;
    int minY_ = 0;
    int maxY_ = 0;
    int skip = 0;
    for(int i = sum_y; i < 255; i ++)
    {
		if(histY[i] < 3)
		{
			maxY_ = i;
			skip ++;
			if(skip > 2)
			{
				break;
			}
		}
    }
    int seedY_ = 0;
    int NonColoredY = 0;
    for(int i = 0; i < 255; i ++)
    {
    	seedY_ += histY[i] * i;
    	NonColoredY += histY[i];
    }
//    ASSERT(NonColoredY != 0);
    seedY_ = NonColoredY == 0? 0: seedY_ / NonColoredY;

    seedY = seedY_;
    minY = minY_;
    maxY = maxY_;
    minH = minH_;
    maxH = maxH_;
    minS = minS_;
    maxS = maxS_;
    DRAWTEXT("module:FieldColorProvider2017:ColorRange",
                 20,
                 0,
                 15,
                 ColorRGBA(0, 0, 255, 255),
                 "seedY_ = " << seedY_ << " sum_y = " << sum_y << " maxY = " << maxY << " minH = " << minH << " maxH = "<<maxH<<" minS = "<<minS<<" maxS = "<< maxS);
}

void FieldColorProvider2017::initial()
{
	for(int i = 0; i < 256; i++)
	{
		histS[i] = 0;
		histH[i] = 0;
		histY[i] = 0;
	}
	seedY = 0;
	minH = 0;
	maxH = 0;
	minS = 0;
	maxS = 0;
	minY = 0;
	maxY = 0;

}


