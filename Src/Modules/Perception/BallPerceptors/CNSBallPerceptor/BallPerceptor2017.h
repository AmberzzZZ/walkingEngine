/**
* @file BallPerceptor.cpp
* This file declares a module that provides a ball percept of the new oficial Robocup SPL ball.
* @author Gabriel Azocar
* @author Pablo Cano
* UChile Robotics Team
*
* Partially based on B-Human's ball perceptor of the coderelease 2014
* whose authors are:
* @author Colin Graf
* @author marcel
* @author Florian Maaß
* @author Thomas Röfer
*/

#pragma once

#define IS_FULL_SIZE true

#include "Tools/Module/Module.h"
#include "Tools/Math/Transformation.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Representations/Perception/ImagePreprocessing/FieldColor.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/BallPercepts/BallPercept2017.h"
#include "Representations/Perception/BallPercepts/BallSpots.h"
//#include "Representations/Perception/PlayersPercept.h"
//#include "Representations/Perception/FieldBoundary.h"
//#include "Representations/Modeling/BallModel.h"
//#include "Representations/Modeling/Odometer.h"
//#include "Representations/Perception/BallSpots.h"
//#include "Representations/Perception/LinePercept.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/DebugImages.h"
#include <math.h>
#include <limits.h>
#include <iostream>
//#include "Dependence/ImageToolbox.h"
#include <chrono>

MODULE(BallPerceptor2017,
{ ,
	REQUIRES(FieldDimensions),
	REQUIRES(ECImage),
	REQUIRES(BodyContour),
	REQUIRES(CameraMatrix),
	REQUIRES(ImageCoordinateSystem),
	REQUIRES(CameraInfo),
	REQUIRES(FieldColor),
	REQUIRES(BallSpots),
	REQUIRES(BallPercept2017),
//	REQUIRES(RobotPose),
//	REQUIRES(Odometer),
//	REQUIRES(TJArkVision),
//	REQUIRES(LinePercept),
//    REQUIRES(BlackAndWhiteBallPercept),
//	REQUIRES(BallSpots),
//	REQUIRES(PlayersPercept),
	USES(RobotPose),
	PROVIDES(BallPercept),
  DEFINES_PARAMETERS(
  {,
    (float)(0.4f) pointsRotation,
    (float)(2) scanTolerance,
    (float)(1.8) upRadiusTolerance,
    (float)(0.6f) downRadiusTolerance,
    (int)(6) searchEdgePointsTolerance,
    (float)(0.2f) minNumOfTransitions,
    (float)(0.2f) greenThrld,
    (int)(20) gradient,
    (float)(1.8f) regionSizeTolerance,
    (int)(11) minValidEdges,
	(float)(1.f) radiusGap,
	(float)(2.f) areaFactor,
  }),
});


/**
 * The class scales the input and output data if full size images
 * are avalable.
 */
class BallPerceptorScaler2017 : public BallPerceptor2017Base
{
  
private:
  

protected:
  CameraInfo theCameraInfo;
  BallSpots theBallSpots;
  ImageCoordinateSystem theImageCoordinateSystem;
  
  /**
   * The only access to image pixels.
   * @param y The y coordinate of the pixel in the range defined in theCameraInfo.
   * @param y The y coordinate of the pixel in the range defined in theCameraInfo.
   * @param The pixel as a temporary object. Do not use yCbCrPadding of that pixel.
   */
  const Image::Pixel getPixel(int y, int x) const
  {
//    if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
//      return theImage.getFullSizePixel(y, x);
//    else
//      return theImage[y][x];
  }
  
  /**
   * Update the copies of input representations that contain data
   * that might have to be scaled up.
   * Must be called in each cycle before any computations are performed.
   */
  void scaleInput();
  
  /**
   * Scale down the output if required.
   * @param ballPercept The ball percept the fields of which might be scaled.
   */
  void scaleOutput(BallPercept& ball) const;
  
  /**
   * Scale down a single value if required.
   * This is only supposed to be used in debug drawings.
   * @param value The value that might be scaled.
   * @return The scaled value.
   */
  template<typename T> T scale(T value) const
  {
    if(theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
      return value / (T) 2;
    else
      return value;
  }
};

class BallPerceptor2017 : public BallPerceptorScaler2017
{
public:
  //Constructor
  BallPerceptor2017();
	static const int NUM_STAR_SCANLINES = 16;
	static const int NUM_STAR_SCANLINE_CANDIDATES = 2 * NUM_STAR_SCANLINES;
	const float maxBallRadius = 120;
  
private:
  class BallPoint
  {
  public:
    Vector2i step;
    Vector2i start;
    Vector2i point;
    Vector2f pointf;
    bool atBorder;
    bool isValid;
    
    BallPoint() : atBorder(false), isValid(false) {}
  };
  
  struct Region
  {
    bool horizontal;
    Vector2i init;
    Vector2i end;
    int index;
    
    bool areIntersected(Region& other)
    {
      if(horizontal == other.horizontal)
      {
        if(horizontal){
          return std::abs(init.y() - other.init.y()) < 3 && init.x() <= other.end.x() && end.x() >= other.init.x();
        }
        else{
          return std::abs(init.x() - other.init.x()) < 3 && init.y() <= other.end.y() && end.y() >= other.init.y();
        }
      }
      else{
        if(horizontal){
          return (other.init.x() >= init.x() && other.init.x() < end.x()) && (init.y() >= other.init.y() && init.y() <= other.end.y());
        }
        else{
          return (init.x() >= other.init.x() && init.x() < other.end.x()) && (other.init.y() >= init.y() && other.init.y() <= end.y());
        }
      }
    }
  };
  struct Ball
  	{
  		int x, y, radius, q;
  		bool found;
  	};
	struct candidateBall
	{
		Ball ball;
		float disError;
		float score;
	};

	struct circle
	{
		float x, y, r, q;
	};

  struct Pentagon
  {
    Pentagon() : width(INT_MAX,0), heigth(INT_MAX,0), points(0){}

    void addRegion(Region& region)
    {
      center += region.init + region.end;
      points += 2;

      width.x() = region.init.x() < width.x() ? region.init.x() : width.x();
      width.y() = region.end.x() > width.y() ? region.end.x() : width.y();

      heigth.x() = region.init.y() < heigth.x() ? region.init.y() : heigth.x();
      heigth.y() = region.end.y() > heigth.y() ? region.end.y() : heigth.y();
        
    }
    
    void isPentagon(float maxArea, float radius)
    {
      float totalHeigh = float(heigth.y() - heigth.x());
      float totalWidth = float(width.y() - width.x());
      if(totalHeigh > 2*radius/3 || totalWidth > 2*radius/3 || totalHeigh < 2 || totalWidth < 2)
      {
         valid = false;
         return;
      }
      float factor = totalHeigh/totalWidth;
      if(factor < 0.5f || factor > 2.f)
      {
        valid = false;
        return;
      }
      area = totalWidth*totalHeigh;
      valid = area < maxArea;
    }

    void setCenter()
    {
      center = center / points;
    }
    
    Vector2i center;
    Vector2i width;
    Vector2i heigth;
    int points;
    float area;
    bool valid;
  };

	//Update Functions
	void update(BallPercept& ballPercept);

  //Verification Functions
  void searchBallFromBallSpots(BallPercept2017& ballPercept);
  bool analyzeBallSpot(const BallSpot ballSpot, BallPercept2017& ballPercept);
  bool calculateBallOnField(BallPercept& ballPercept) const;
  bool checkBallOnField(BallPercept2017 ball) const;
   
  //Cascade Functions
	bool checkBallSpot(const BallSpot& ballSpot);
  bool checkRegionSizes(const BallSpot& ballSpot);
	bool searchEdgePoints(const BallSpot& ballSpot,const ColorRGBA& color);
  bool isBallFromPoints();
  bool checkNewRadius();
  bool checkBallNear(BallPercept2017& ballPercept);
  bool isRobotNear();
	bool searchValidEdges();
  bool checkGreenInside();
  bool checkPentagons();

  //Pentagon Functions
  void scanLine(Vector2i step, Vector2i origin, Vector2i end);
  void mergeRegions();
  void changeIndex(int from, int to);
  int createPentagons();
  int validatePentagons();
 
  //Debug Functions
	void drawBall(const Vector2f& pos) const;
	bool drawError(BallSpot ballSpot, std::string message);
	bool showRegionSizes(const BallSpot& ballSpot);

	Ball fitBall(const Vector2f& center, const float searchR, Vector2f& ballColor);
//	circle ransacCircle(point_2d points[NUM_STAR_SCANLINE_CANDIDATES],
//	                      float maxDistEdge);

	circle getCircle(float x1, float y1, float x2,float y2, float x3, float y3);

//	float searchEdges(const Image& img, point_2d pos,
//	                                     point_2d points[NUM_STAR_SCANLINES * 2],
//	                                     Vector2f& ballColor,float searchR);

	Ball newBall(float x, float y, float radius, bool found, float q);

//	float guessDiameter(point_2d points[NUM_STAR_SCANLINES]);

	circle newCircle(float x, float y, float r,float q);

	bool classifyBalls2(Ball & b);

//	bool possibleGreen(const Image::Pixel& p) const;
//
//	bool possibleGrayScale(const Image::Pixel& p) const;
//
//	bool isWhite(const Image::Pixel* p) const;
//
//	ColorTable::Colors determineColor(const Image::Pixel& p) const;

	float tansig(float per,float ref);

	int myThreshold1(int centerX, int centerY, int radius);

	void save_bw(Vector2f c, float radius);

	void countAround(const Vector2i& center, float radius, int& greenCount, int& nonWhiteCount, int& avgGreenY) const;

	void countAtRadius(const Vector2i& center, float radius, int& greenCount, int& nonWhiteCount, int& avgGreenY) const;

	bool checkInnerGreen(Vector2i center, float r) const;

	bool classifyBalls3(BallPerceptor2017::Ball & b);
  
	// Internal Variables
	double imageHeigth, imageWidth, approxRadius;
  float radius;
	int validBallPoints;
	bool ballValidity;
	Vector2f center, pentagonsCenter;
	BallPoint ballPoints[16];
  std::vector<Region> regions;
  std::vector<Pentagon> pentagons;
  std::vector<BallPercept2017>  balls;
  std::vector<candidateBall> possibleBalls;
  int greenAround;
};
