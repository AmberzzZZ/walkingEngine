/**
 * @file CNSBallPerceptor.h
 * This file declares a module that detects balls in CNS images.
 * @author Thomas Röfer
 * @author Udo Frese
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include <unordered_set>
#include "Tools/Math/Eigen.h"
#include "objectCNSStereoDetector.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CameraIntrinsics.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallPrediction.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/BallPercepts/BallPercept2017.h"
#include "Representations/Perception/BallPercepts/BallRegions.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CNSImage.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Perception/ImagePreprocessing/FieldColor.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/OutStreams.h"

MODULE(CNSBallPerceptor,
{,
  REQUIRES(BallPrediction),
  REQUIRES(BallRegions),
  REQUIRES(BodyContour),
  REQUIRES(CameraInfo),
  REQUIRES(CameraIntrinsics),
  REQUIRES(CameraMatrix),
  REQUIRES(CNSImage),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(FieldColor),
  REQUIRES(ECImage),
  REQUIRES(ImageCoordinateSystem),
  PROVIDES(BallPercept2017),
  LOADS_PARAMETERS(
  {,
    (int) contourSubDivisions, /**< The number of times triangles of a cube are split to form a sphere as ball model for contour computation. */
    (float) maxTableRadius, /**< The maximum distance from the robot covered by the precomputed ball contour table. */
    (float) minTableHeight, /**< The minimum height of the camera covered by the precomputed ball contour table. */
    (float) maxTableHeight, /**< The maximum height of the camera covered by the precomputed ball contour table. */
    (float) spacing, /**< The spatial descretization of the precomputed ball contour table. */
    (int) sampleSubDivisions, /**< The number of times triangles of a cube are split to form a sphere as ball model for determining sample points. */
    (float) sampleDepthRatio, /**< Depth clipping of sample points. Larger means less points. */
    (float) sampleHeightRatio, /**< Height clipping of sample points. Larger means more points. */
    (Angle) sampleRange, /**< Rotations are sampled in the range of [-sampleRange .. sampleRange[ when computing the ball patterns. */
    (Angle) sampleStep, /**< The sampling is done in these step width when computing the ball patterns. */
    (Angle) sampleNeighborhood, /**< Sampling is blurred in the range of [-sampleNeighborhood .. sampleNeighborhood[ when computing the ball patterns. */
    (bool) usePrediction, /**< Include the ball prediction as a starting point for the search? */
    (int) refineIterations, /**< The number of refinements performed after the global search. */
    (float) minResponse, /**< The minimum response returned by the contour detector required for a ball candidate. */
    (float) maxResponse, /**< The response required if other criteria are not satisfied by a ball candidate. */
    (float) minResponseWithinGreen, /**< Minimum response required to accept a ball just because it is surrounded by green. */
    (float) maxRadiusDeviation, /**< The maximum deviation between measured and expected ball radius allowed. */
    (std::vector<float>) greenCheckRadiusRatios, /**< The additional radius (to the ball radius) to define the search rectangle for the "all green around" - check */
    (int) numberOfGreenChecks, /**< The number of pixels checked per scan around the ball. */
    (float) minAroundGreenRatio, /**< The minimum ratio of pixels that must be green when scanning around the ball. */
    (float) minAroundNonWhiteRatio, /**< The minimum ratio of pixels that must be green when scanning around the ball. */
    (float) minAroundDistance, /**< The minimum distance from which balls are accepted just because they are surrounded by green. */
    (float) maxAroundDistance, /**< The maximum distance up to which balls are accepted just because they are surrounded by green. */
    (float) brightnessBonus, /**< Increase brightness based on the height of a point on the ball surface. */
    (float) minContrast, /**< Minimum contrast between black and white parts of the ball. */
  }),
});

class CNSBallPerceptor : public CNSBallPerceptorBase
{
private:

/*@zhiying*/
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


  class GLCM
  {
  public:
	  int glcm_0[2][2];
	  int glcm_45[2][2];
      int glcm_90[2][2];
	  int glcm_135[2][2];

	  inline int getMaxMargin()
	  {
		  int max0 = std::max(glcm_0[0][0],glcm_0[1][1]);
		  int max1 = std::max(glcm_45[0][0],glcm_45[1][1]);
		  int max2 = std::max(glcm_90[0][0],glcm_90[1][1]);
		  int max3 = std::max(glcm_135[0][0],glcm_135[1][1]);
		  return std::min(std::min(max0,max1),std::min(max2,max3));
	  };

	  inline int getMinMargin()
	  {
		  return std::min(std::min(glcm_0[0][1],glcm_45[0][1]),std::min(glcm_90[0][1],glcm_135[0][1]));
	  };

	  inline int getDeltaMargin()
	  {
		  int max0 = abs(glcm_0[0][0]-glcm_0[1][1]);
		  int max1 = abs(glcm_45[0][0]-glcm_45[1][1]);
		  int max2 = abs(glcm_90[0][0]-glcm_90[1][1]);
		  int max3 = abs(glcm_135[0][0]-glcm_135[1][1]);
		  return std::max(std::max(max0,max1),std::max(max2,max3));
	  };
	  inline float getFeature()
	  	  {
	  		float max_ = std::max(std::max(glcm_0[0][1],glcm_45[0][1]),std::max(glcm_90[0][1],glcm_135[0][1]));
	  		float min_ = std::min(std::min(glcm_0[0][1],glcm_45[0][1]),std::min(glcm_90[0][1],glcm_135[0][1]));
	  		if (min_<5)
	  			return 1000.f;
	  		return max_/min_;
	  	  };

  };
  
  CameraModelOpenCV cameras[CameraInfo::numOfCameras]; /**< The parameters of the cameras. */
  ObjectCNSStereoDetector detector; /**< The detector. */
  ObjectCNSStereoDetector::IsometryWithResponses objects; /**< The poses of the detected objects in the coordinate system of the camera and the responses. */
  SearchSpecification spec; /**< The current search specification. */
  mutable OutTextRawMemory* tip; /**< Used to collect text for the tool tips of debug drawings. */
  std::vector<Vector3d> samplePoints; /**< The 3-D points of a ball candidate sampled for the surface pattern. */

  static const int ballTextureWidth = 300; /**< Width of the texture the ball patterns are generated from (in pixels). */
  static const int ballTextureHeight = 150; /**< Height of the texture the ball patterns are generated from (in pixels). */

  using BallPattern = unsigned; /**< The type of a ball pattern. */
  std::unordered_set<BallPattern> ballPatterns; /**< Is the bit pattern a valid ball pattern? */
//
//  int sampleImage[900];
//  GLCM glcm;

  /**
   * The main method of this module.
   * @param ballPercept The percept that is filled by this module.
   */
  void update(BallPercept2017& ballPercept);

  /**
   * Setup the camera used by the detector.
   */
  void updateCamera();

  /**
   * The method updates the search space relative to the pose of the camera.
   */
  void updateSearchSpace();

  /**
   * Performs several tests to determine whether a ball candidate
   * can be an actual ball.
   * @param object The object found to check.
   * @return Could it be the ball?
   */
  BallPercept2017::Status checkBall(const IsometryWithResponse& object, BallPercept2017& ballPercept) const;

  /**
   * The method counts how much the ball is surrounded by enough green.
   * @param center The assumed center of the ball candidate.
   * @param radius The expected ball radius at that point in the image.
   * @param greenCount The number of green pixels found is returned here.
   * @param nonWhiteCount The number of non-white pixels found is returned here.
   */
  void countAround(const Vector2i& center, float radius,
                   int& greenCount, int& nonWhiteCount, int& avgGreenY0, int& avgGreenY1, int& avgGreenY2, int& avgGreenY3) const;

  /**
   * The method counts how much the ball is surrounded by green
   * for a single radius.
   * @param center The assumed center of the ball candidate.
   * @param radius The radius that is scanned around the center.
   * @param greenCount The number of green pixels found is added here.
   * @param nonWhiteCount The number of non-white pixels found is added here.
   */
  void countAtRadius(const Vector2i& center, float radius,
                     int& greenCount, int& nonWhiteCount, int& avgGreenY0, int& avgGreenY1, int& avgGreenY2, int& avgGreenY3) const;

  bool isInside(int x, int y) const;

  /**
   * Check whether an object has the correct black and white pattern to
   * be the official ball.
   * @param object The object to check.
   * @param accepted If drawn, draw as accepted pattern.
   * @return Could it be the ball?
   */
  bool checkSamplePoints(const IsometryWithResponse& object, int& avgGreenY0, int& avgGreenY1, int& avgGreenY2, int& avgGreenY3, const Vector2i center, bool accepted = false) const;

  /**
   * Determine the threshold separating the brightnesses into two classes
   * using Otsu's method. There are a few optimizations for the histogram
   * being very sparse.
   * @param brigtnesses The brightnesses measured.
   * @return The best threshold to split them into two classes.
   */
  static int calcThreshold(const std::vector<unsigned char>& brightnesses);

  /**
   * Checks whether there is enogh contrast between the dark samples and the
   * bright samples.
   * @param brightnesses The brighnesses sampled.
   * @param threshold the threshold between dark and bright.
   * @return Is there enough contrast?
   */
  bool checkContrast(const std::vector<unsigned char>& brightnesses, int threshold) const;

  /**
   * Determines whether the first parameter is less or equal to the
   * second parameter during the ball check.
   * The method can also output data for a debug drawing if requested.
   * @param op1 The first argument of the comparison.
   * @param op2 The second argument of the comparison.
   * @param label A label that is used for the debug output.
   * @return Is the first argument less or equal to the second argument?
   */
  bool leq(float op1, float op2, const char* label) const;

  /**
   * Adds this result to the debug output during the ball check if
   * request.
   * @param result The result of the check.
   * @param label A label that is used for the debug output.
   * @return The result.
   */
  bool print(bool result, const char* label) const;

  /**
   * Adds this value to the debug output during the ball check if
   * request.
   * @param value The result of the check.
   * @param label A label that is used for the debug output.
   */
  void print(float value, const char* label) const;

  /**
   * Fills in all the fields of the ball percept.
   * @param ballPercept The representation that is filled.
   */
  void fillBallPercept(BallPercept2017& ballPercept) const;

  /**
   * This helper converts an Eigen matrix to a B-Human 3-D pose.
   * @param m The Eigen matrix describing a pose in 3-D space.
   * @return The corresponding Pose3D.
   */
  static Pose3f toPose3f(const Matrix4d& m);

  /**
   * This helper converts a B-Human 3-D pose to an Eigen matrix.
   * @param p The Pose3f describing a pose in 3-D space.
   * @return The corresponding Eigen matrix.
   */
  static Matrix4d toMatrix4d(const Pose3f& p);

  /**
   * Create or load the ball pattern table from the ball texture.
   */
  void createOrLoadPatternTable();

  /**
   * Determine a ball pattern from the sample points and the ball texture for a
   * rotation.
   * @param ballTexture The texture to get the pattern from.
   * @param x The rotation around the x axis.
   * @param y The rotation around the y axis.
   * @param z The rotation around the z axis.
   * @return The pattern. 0 bits mean black and 1 bits mean white.
   */
  BallPattern getPattern(unsigned char ballTexture[ballTextureHeight][ballTextureWidth], float x, float y, float z) const;

  /**
   * Determine the brightness of the ball at a point on its surface.
   * @param ballTexture The texture to get the pattern from.
   * @param v The vector from the center of the ball to the surface point
   *          that is sampled.
   * @return The brightness at the surface point.
   */
  static unsigned char getBrightness(unsigned char ballTexture[ballTextureHeight][ballTextureWidth], const Vector3d& v);
  
  /**
   * Sample an image of possible ball to 30*30 size. (Stored in a 900-array)
   * @param sampleImage The 900-array to store possible image.
   */
  float normalizedSampleImage(int& avgGreenY0, int& avgGreenY1, int& avgGreenY2, int& avgGreenY3, Vector2i center, float r, int* sampleImage) const;
  
  /**
   * Calculate the gray-level co-occurance matrix of a given image.
   * @param sampleImage The image to be calculated.
   * @param glcm The result.
   */
  void calcGrayLevalCoMatrix(int* sampleImage, GLCM& glcm) const;
  
  /**
   * Display the calculated glcm result
   * @param glcm Gray-level Co-ocurrance Matrix
   */
  void disPlayGLCM(GLCM& glcm, const Vector2i center, const float radius) const;

  // Drawing methods for debugging
  void draw();
  void drawContourViaLutRasterizer(const LutRasterizer& lr, const Eigen::Isometry3d& object2World, const CameraModelOpenCV& camera, const ColorRGBA& color) const;
  void drawRasteredContour(const Contour& contour, const ColorRGBA& color) const;
  void drawCylinderRing(const CylinderRing& cylinderRing, const CameraModelOpenCV& camera) const;

  bool isLastDuplicative(BallPercept2017& ballPercept, const int minAllowedDistanz) const;

public:

  CNSBallPerceptor();
};
