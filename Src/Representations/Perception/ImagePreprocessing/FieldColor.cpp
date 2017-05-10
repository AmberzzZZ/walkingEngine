/**
 * @author Li Shu
 */
#include "FieldColor.h"
#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/Image.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Modify.h"

void FieldColor::draw() const
{

  //show what is the best green detected
  DECLARE_DEBUG_DRAWING("representation:FieldColor",
                        "drawingOnImage");
  DRAWTEXT(
      "representation:FieldColor",
      0,
      0,
      20,
      ColorRGBA(0, 0, 255, 255),
      "Y:"<<minCy<<"~"<<maxCy <<"  Cb="<<minCb<<"~"<<maxCb <<"  Cr="<<minCr<<"~"<<maxCr
      <<"  seedY=" << seedY<<"  seedCb="<<seedCb<<"  seedCr="<<seedCr);
  DRAWTEXT(
      "representation:FieldColor",
      0,
      20,
      20,
      ColorRGBA(0, 0, 255, 255),
      "H:"<<minH<<"~"<<maxH <<"  s="<<minS<<"~"<<maxS<<"  W_Smax="<<W_Smax);

}

bool FieldColor::isGreen(const Image::Pixel & p, const ColorConvert & ct) const
{
	if (p.y > FieldColor::minCy && p.y < FieldColor::maxCy
      && p.cb > FieldColor::minCb && p.cb < FieldColor::maxCb
      && p.cr > FieldColor::minCr && p.cr < FieldColor::maxCr)
    return true;
  Image::Pixel ss;
  ct.getHSV(p,ss);
  if (ss.h > FieldColor::minH && ss.h < FieldColor::maxH
      && ss.s > FieldColor::minS)
    return true;
  return false;
}

bool FieldColor::isGreen(const Image::Pixel * p, const ColorConvert & ct) const
{
	if (p->y > FieldColor::minCy && p->y < FieldColor::maxCy
      && p->cb > FieldColor::minCb && p->cb < FieldColor::maxCb
      && p->cr > FieldColor::minCr && p->cr < FieldColor::maxCr)
    return true;
  Image::Pixel ss;
  ct.getHSV(p,ss);
  if (ss.h > FieldColor::minH && ss.h < FieldColor::maxH
      && ss.s > FieldColor::minS && ss.s < FieldColor::maxS)
    return true;
  return false;
}

bool FieldColor::isGreen(const int cy, const int cb, const int cr, const ColorConvert & ct) const
{
	if (cy > FieldColor::minCy && cy < FieldColor::maxCy
      && cb > FieldColor::minCb && cb < FieldColor::maxCb
      && cr > FieldColor::minCr && cr < FieldColor::maxCr)
    return true;
  Image::Pixel ss;
  ct.getHSV(cy,cb,cr,ss);
  if (ss.h > FieldColor::minH && ss.h < FieldColor::maxH
      && ss.s > FieldColor::minS && ss.s < FieldColor::maxS)
    return true;
  return false;
}

bool FieldColor::isWhite(const Image::Pixel & p, const ColorConvert & ct) const
{
  if (p.y<FieldColor::W_Ymin)
	 return false;
  if (p.cb<FieldColor::W_Cbmin)
	 return false;
  if (p.cr<FieldColor::W_Crmin)
	 return false;
  Image::Pixel ss;
  ct.getHSV(p,ss);
  if (ss.s>FieldColor::W_Smax)
	 return false;
  if (ss.i<FieldColor::W_Vmin)
	 return false;
  Image::Pixel sss;
  ct.getRGB(p,sss);
  if (sss.r<FieldColor::W_Rmin)
	 return false;
  if (sss.g<FieldColor::W_Gmin)
	 return false;
  if (sss.b<FieldColor::W_Bmin)
	 return false;
  return true;
}

bool FieldColor::isWhite(const Image::Pixel * p, const ColorConvert & ct) const
{
  if (p->y<FieldColor::W_Ymin)
	 return false;
  if (p->cb<FieldColor::W_Cbmin)
	 return false;
  if (p->cr<FieldColor::W_Crmin)
	 return false;
  Image::Pixel ss;
  ct.getHSV(p,ss);
  if (ss.s>FieldColor::W_Smax)
	 return false;
  if (ss.i<FieldColor::W_Vmin)
	 return false;
  Image::Pixel sss;
  ct.getRGB(p,sss);
  if (sss.r<FieldColor::W_Rmin)
	 return false;
  if (sss.g<FieldColor::W_Gmin)
	 return false;
  if (sss.b<FieldColor::W_Bmin)
	 return false;
  return true;
}

bool FieldColor::isWhite(const int cy, const int cb, const int cr, const ColorConvert & ct) const
{
  if (cy<FieldColor::W_Ymin)
	 return false;
  if (cb<FieldColor::W_Cbmin)
	 return false;
  if (cr<FieldColor::W_Crmin)
	 return false;
  Image::Pixel ss;
  ct.getHSV(cy,cb,cr,ss);
  if (ss.s>FieldColor::W_Smax)
	 return false;
  if (ss.i<FieldColor::W_Vmin)
	 return false;
  Image::Pixel sss;
  ct.getRGB(cy,cb,cr,sss);
  if (sss.r<FieldColor::W_Rmin)
	 return false;
  if (sss.g<FieldColor::W_Gmin)
	 return false;
  if (sss.b<FieldColor::W_Bmin)
	 return false;
  return true;
}

bool FieldColor::isBlack(const Image::Pixel * p, 
                    const ColorConvert & ct) const
{
  Image::Pixel sss;
  ct.getRGB(p,sss);
  if (sss.r>FieldColor::W_Rmin)
	 return false;
  if (sss.g>FieldColor::W_Gmin)
	 return false;
  if (sss.b>FieldColor::W_Bmin)
	 return false;	
  return true;		
}

bool FieldColor::isBlack(const Image::Pixel & p, 
                    const ColorConvert & ct) const
{
  Image::Pixel sss;
  ct.getRGB(p,sss);
  if (sss.r>FieldColor::W_Rmin)
	 return false;
  if (sss.g>FieldColor::W_Gmin)
	 return false;
  if (sss.b>FieldColor::W_Bmin)
	 return false;	
  return true;		
}

bool FieldColor::isBlack(const int cy, const int cb, const int cr, 
                    const ColorConvert & ct) const
{
  Image::Pixel sss;
  ct.getRGB(cy,cb,cr,sss);
  if (sss.r>FieldColor::W_Rmin)
	 return false;
  if (sss.g>FieldColor::W_Gmin)
	 return false;
  if (sss.b>FieldColor::W_Bmin)
	 return false;	
  return true;		
}

FieldColors::Color FieldColor::determineColor(const Image::Pixel & p, 
                                  const ColorConvert & ct) const
{
	Image::Pixel sss;
  ct.getRGB(p,sss);
	return FieldColors::none;
}

FieldColors::Color FieldColor::determineColor(const Image::Pixel * p, 
                                  const ColorConvert & ct) const
{
	Image::Pixel sss;
  ct.getRGB(p,sss);
	return FieldColors::none;
}

FieldColors::Color FieldColor::determineColor(const int cy, const int cb, const int cr,
                                  const ColorConvert & ct) const
{
	Image::Pixel sss;
  ct.getRGB(cy,cb,cr,sss);
	return FieldColors::none;
}
