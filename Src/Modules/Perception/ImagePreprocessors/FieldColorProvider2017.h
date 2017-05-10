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

MODULE(FieldColorProvider2017,
{,
  REQUIRES(Image),
  REQUIRES(CameraInfo),
  REQUIRES(ColorConvert),
  PROVIDES(FieldColor),
});

class FieldColorProvider2017 : public FieldColorProvider2017Base
{
 private:
  //建立YCbCr色彩空间的各个通道的柱状图
  int histY[256];

  int histH[256];
  int histS[256];

  //需要预先设定的一些参数
  const int pixelSpacing = 4;
  const int colorBorder = 8;
  const float greenGain = 2;

  int width;
  int height;

 public:
  void update(FieldColor& fieldColor);
  void proceed(const Image& img, const ColorConvert& ct);
  void initial();

  int seedY;

  //绿色的范围
  int minH;
  int maxH;
  int minS;
  int maxS;
  int minY;
  int maxY;

  FieldColorProvider2017();
  ~FieldColorProvider2017();
  int time ;

};

