/**
 * @file ColorConvert.h
 * @author Li Shu
 * @function Converting to different color space
 */

#pragma once

#include "Representations/Infrastructure/Image.h"
#include "Platform/BHAssert.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/ImageProcessing/YHSColorConversion.h"


struct ColorConvert : public Streamable
{

private:

  const struct ColorSpaceMapper
  {
    struct RB
    {
      unsigned char r;
      unsigned char b;
      unsigned char g;
      unsigned short rb;
    };

    Image::Pixel hsi[32][256][256];
    RB rgb[32][256][256];
    float stdOfRGB[32][256][256];
    // add STD @lishu Oct.07 on CHR train


    ColorSpaceMapper()
    {
      Image::Pixel* p = &hsi[0][0][0];
      RB* q = &rgb[0][0][0];
      float* z = &stdOfRGB[0][0][0];

      for(int y = 7; y < 256; y += 8)
        for(int cb = 0; cb < 256; cb++)
          for(int cr = 0; cr < 256; cr++, ++p, ++q)
          {
            p->h = YHSColorConversion::computeHue(cb,cr);
//			p->s = YHSColorConversion::computeLightIndependentSaturation(cb,cr,y);
            p->s = YHSColorConversion::computeSaturation(cb,cr);
			p->i = 0;
            ColorModelConversions::fromYUVToRGB(static_cast<unsigned char>(y),
                                                  static_cast<unsigned char>(cb),
                                                  static_cast<unsigned char>(cr),
                                                  q->r, q->g, q->b);
            int maxrgb = std::max(std::max(q->r,q->g),q->b);
            int minrgb = std::min(std::min(q->r,q->g),q->b);
//            p->s = static_cast<unsigned char>(static_cast<float>(maxrgb - minrgb) / static_cast<float>(maxrgb) * 256);
//            int sum = q->r + q->g + q->b;
//            float ave = sum / 3.0f;
//            float Cor = int(q->r)*int(q->r) + int(q->g)*int(q->g) + int(q->b)*int(q->b)
//                        + 3 * ave*ave - 2 * (int(q->r) * ave + int(q->g) * ave + int(q->b) * ave);
//            float Var=sqrtf(Cor);
//            *z = Var;


          }
    }
  } colorSpaceMapper;

public:

  void getHSV(const Image::Pixel* p,Image::Pixel& s) const
  {
    s.h = colorSpaceMapper.hsi[p->y>>3][p->cb][p->cr].h;
    s.s = colorSpaceMapper.hsi[p->y>>3][p->cb][p->cr].s;
    s.i = colorSpaceMapper.hsi[p->y>>3][p->cb][p->cr].i;
  }
  void getHSV(const Image::Pixel& p,Image::Pixel& s) const
  {
    s.h = colorSpaceMapper.hsi[p.y>>3][p.cb][p.cr].h;
    s.s = colorSpaceMapper.hsi[p.y>>3][p.cb][p.cr].s;
    s.i = colorSpaceMapper.hsi[p.y>>3][p.cb][p.cr].i;
  }
  void getHSV(const int cy, const int cb, const int cr,Image::Pixel& s) const
  {
    s.h = colorSpaceMapper.hsi[cy>>3][cb][cr].h;
    s.s = colorSpaceMapper.hsi[cy>>3][cb][cr].s;
    s.i = colorSpaceMapper.hsi[cy>>3][cb][cr].i;
  }
  void getRGB(const Image::Pixel& p,Image::Pixel& s) const
  {
	s.r = colorSpaceMapper.rgb[p.y>>3][p.cb][p.cr].r;
	s.g = colorSpaceMapper.rgb[p.y>>3][p.cb][p.cr].g;
	s.b = colorSpaceMapper.rgb[p.y>>3][p.cb][p.cr].b;
  }
  void getRGB(const Image::Pixel* p,Image::Pixel& s) const
  {
    s.r = colorSpaceMapper.rgb[p->y>>3][p->cb][p->cr].r;
    s.g = colorSpaceMapper.rgb[p->y>>3][p->cb][p->cr].g;
    s.b = colorSpaceMapper.rgb[p->y>>3][p->cb][p->cr].b;
  }
  void getRGB(const int cy, const int cb, const int cr,Image::Pixel& s) const
  {
    s.r = colorSpaceMapper.rgb[cy>>3][cb][cr].r;
    s.g = colorSpaceMapper.rgb[cy>>3][cb][cr].g;
    s.b = colorSpaceMapper.rgb[cy>>3][cb][cr].b;
  }
  float getSTD(const Image::Pixel& p) const
  {
	  return colorSpaceMapper.stdOfRGB[p.y>>3][p.cb][p.cr];
  }
  float getSTD(const Image::Pixel* p) const
	{
	  return colorSpaceMapper.stdOfRGB[p->y>>3][p->cb][p->cr];
	}
  int dummy;

private:

  void serialize(In* in, Out* out);
};

inline void ColorConvert::serialize(In * in, Out * out)
{
  STREAM_REGISTER_BEGIN;
  STREAM(ColorConvert::dummy);
  STREAM_REGISTER_FINISH;
}
