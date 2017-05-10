/**
 * @author Li Shu
 * @function Field color auto-detector without using color table
 */

#pragma once

#include <vector>
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Configuration/FieldColors.h"
#include "Representations/Perception/ImagePreprocessing/ColorConvert.h"

STREAMABLE(FieldColor,
{
    public:
	bool isGreen(const Image::Pixel & p, const ColorConvert & ct) const;
	bool isGreen(const Image::Pixel * p, const ColorConvert & ct) const;
	bool isGreen(const int cy, const int cb, const int cr, const ColorConvert & ct) const;
	bool isWhite(const Image::Pixel & p, const ColorConvert & ct) const;
	bool isWhite(const Image::Pixel * p, const ColorConvert & ct) const;
	bool isWhite(const int cy, const int cb, const int cr, const ColorConvert & ct) const;
	bool isBlack(const Image::Pixel & p, const ColorConvert & ct) const;
	bool isBlack(const Image::Pixel * p, const ColorConvert & ct) const;
	bool isBlack(const int cy, const int cb, const int cr, const ColorConvert & ct) const;
	FieldColors::Color determineColor(const Image::Pixel & p, const ColorConvert & ct) const;
	FieldColors::Color determineColor(const Image::Pixel * p, const ColorConvert & ct) const;
	FieldColors::Color determineColor(const int cy, const int cb, const int cr, const ColorConvert & ct) const;
    void draw() const;,

    //Color Detect
    (int)(0) seedY,
    (int)(0) seedCb,
    (int)(0) seedCr,
    (int)(0) maxCy,
    (int)(0) minCy,
    (int)(0) maxCb,
    (int)(0) minCb,
    (int)(0) maxCr,
    (int)(0) minCr,
    (int)(0) minH,
    (int)(0) maxH,
    (int)(0) minS,
    (int)(0) maxS,
	//White Detect
    (int)(0) W_Ymin,
	(int)(0) W_Cbmin,
	(int)(0) W_Crmin,
	(int)(0) W_Smax,
	(int)(0) W_Vmin,
	(int)(0) W_Rmin,
	(int)(0) W_Gmin,
	(int)(0) W_Bmin,
});

