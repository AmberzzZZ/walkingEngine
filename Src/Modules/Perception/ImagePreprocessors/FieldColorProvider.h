/**
 * @file FieldColorProvider.h
 * 定义了一个类，用来自动检测场地颜色（绿色），自动确定绿色的YCbCr的值
 * @author 李树
 */
#pragma once

#include <stdint.h>

#include "Representations/Perception/ImagePreprocessing/ColorConvert.h"
#include "color.h"
#include "Tools/Module/Module.h"
#include "Representations/Infrastructure/Image.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Representations/Perception/ImagePreprocessing/FieldColor.h"
#include "Representations/Infrastructure/CameraInfo.h"

MODULE(FieldColorProvider,
{,
  REQUIRES(Image),
  REQUIRES(CameraInfo),
  REQUIRES(ColorConvert),
  PROVIDES(FieldColor),
});

#define NUM_FEATURES 7
class FieldColorProvider : public FieldColorProviderBase
{
 private:
  //建立YCbCr色彩空间的各个通道的柱状图
  int histY[256];
  int histCb[256];
  int histCr[256];

  int histH[265];
  int histS[265];
  int histV[256];
  int histR[265];
  int histG[265];
  int histB[265];
  //根据柱状图得到的色彩特征
  float features[NUM_FEATURES];

  //与绿色较为相近的YCbCr值
  int minCy2;
  int maxCy2;

  int minCb2;
  int maxCb2;

  int minCr2;
  int maxCr2;

  //需要预先设定的一些参数
  const int pixelSpacing = 4;
  const int minFieldArea = 150;
  const int colorBorder = 8;
  const float greenGain = 2;
  const float thetas[62] = { -0.003928473492851304f, 0.3267591297421786f, 0.15024038619767252f,
	    -0.0026580701516830778f, -0.06626938819648565f, -0.0918234800891045f,
	    -0.007024391380169659f, -0.19444191475988953f, 0.10805971362794498f,
	    -0.013870596388515415f, 0.020367431009978596f, 0.5006415354683456f,
	    0.0523626396220556f, 0.06796659398791673f, 0.13091006086089688f,
	    -0.04015537555250251f, 0.2534669682609816f, 0.22259232118528924f,
	    0.08825125863663277f, -0.06163688650966539f, 0.18292785975365364f,
	    0.18490873559957913f, 0.1308039353774413f, 0.15043700747876884f,
	    -0.030408070393040373f, -0.18234162918227365f, 0.30516577669883815f,
	    -0.060749446765493896f, 0.473586856960429f, 0.31872308251277015f,
	    -0.04073046667475529f, -0.157079077089501f, -0.4325757850385499f,
	    0.04335112440158662f, 0.05425490604442203f, -0.21977115232887673f,
	    0.06711078780969373f, -0.08743680491075988f, -0.07236939332440999f,
	    0.03686565322899083f, -0.0034490299130395608f, -0.1224550945463936f,
	    0.14004650713616937f, 0.053306649062165465f, 0.20536885019856824f,
	    0.010868449852810068f, 0.14466029588975235f, 0.08223414822611032f,
	    0.09088654555978222f, 0.12192354321809107f, -0.15900537301205442f,
	    -0.07653902693746543f, -0.13043295430645066f, 0.10422567492674986f,
	    0.12357034980655468f, 0.06425325379992515f, 0.1547906519403598f,
	    -0.11945473558477798f, -0.15195968390312445f, -0.060779372474056875f,
	    -0.06570198431532723f, 0.07563040487570107f };;

  int width;
  int height;

 public:
  void update(FieldColor& fieldColor);
  int seedCr;
  int seedCb;
  int seedY;
  int greenCr;
  int greenCb;
  int greenCy;
  //绿色的范围
  int minCy;
  int maxCy;
  int minCb;
  int maxCb;
  int minCr;
  int maxCr;
  int minH;
  int maxH;
  int minS;
  int maxS;

  int G_Smax;
  int G_Vmax;
  int G_Rmax;
  int G_Gmax;
  int G_Bmax;

  void proceed(const Image& img, const ColorConvert& ct);
  void searchInitialSeed(const Image& img);
  void setYCbCrCube(float* features);
  void setHSVCube(const Image& img, const ColorConvert& ct);
  void extractFeatures(const Image& img, float* hist);
  static int getStableMin(const int* const hist, int thres);
  static int getPeak(const int* const hist);
  void resetArrays();
  int findContinuousLowerBound(const int * hist, int bound);
  int findContinuousUpperBound(const int * hist, int bound);
  bool checkColorValidity() const;

  inline bool maybeGreen(int cyReal, int cbReal, int crReal) const
  {
    if (crReal > maxCr2)
      return false;
    if (cyReal > maxCy2)
      return false;
    if (cbReal > maxCb2)
      return false;
    if (cbReal < minCb2)
      return false;
    if (crReal < minCr2)
      return false;
    if (cyReal < minCy2)
      return false;
    return true;
  }

  inline bool isGreen(int cy, int cb, int cr) const
  {
    if (cy > minCy && cy < maxCy
          && cb > minCb && cb < maxCb
          && cr > minCr && cr < maxCr)
        return true;
//      unsigned char h, s, v;
//      ColorModelConversions::fromYUVToHSI(static_cast<char>(cy), static_cast<char>(cb), static_cast<char>(cr), h, s, v);
//      if (h > minH && h < maxH
//          && s > minS && s < maxS)
//        return true;
      return false;
  }

  inline bool isWhite(int cy, int cb, int cr) const
    {
     unsigned char h, s, v;
      ColorModelConversions::fromYUVToHSI(static_cast<char>(cy), static_cast<char>(cb), static_cast<char>(cr), h, s, v);
      if ((h > 0.8 * minH && h < 1.2 * maxH)
          && s < 0.8 * minS && cy > (seedY + 20))
        return true;
      else
        return false;
    }

  inline bool isWhiteOrBlack(int cy, int cb, int cr) const
      {
       unsigned char h, s, v;
        ColorModelConversions::fromYUVToHSI(static_cast<char>(cy), static_cast<char>(cb), static_cast<char>(cr), h, s, v);
        if ((h > 0.8 * minH && h < 1.2 * maxH)
            && s < 0.8 * minS)
          return true;
        else
          return false;
      }

  color getColor() const
  {
    color c;
    c.cy = greenCy;
    c.cb = greenCb;
    c.cr = greenCr;
    return c;
  }



  FieldColorProvider();
  ~FieldColorProvider();
  int time ;

};

