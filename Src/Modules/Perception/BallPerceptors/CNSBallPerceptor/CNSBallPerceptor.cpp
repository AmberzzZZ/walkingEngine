/**
 * @file CNSBallPerceptor.cpp
 * This file implements a module that detects balls in CNS images.
 * @author Thomas Röfer
 * @author Udo Frese
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "CNSBallPerceptor.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/ImageProcessing/InImageSizeCalculations.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"
#include "lutRasterizer.h"
#include <chrono>

//#define SHOW_TIME
//#define BHUMAN

using namespace std::chrono;

MAKE_MODULE(CNSBallPerceptor, perception)

CNSBallPerceptor::CNSBallPerceptor()
{
  // Create or load contour table
  Eigen::AlignedBox3d viewpointRange(Vector3d(-maxTableRadius,
                                              -maxTableRadius,
                                              minTableHeight),
                                     Vector3d(maxTableRadius,
                                              maxTableRadius,
                                              maxTableHeight));
  const TriangleMesh& mesh = TriangleMesh::sphere(theFieldDimensions.ballRadius, contourSubDivisions);
  detector.create(mesh,
                  CameraModelOpenCV(),
                  viewpointRange,
                  spacing,
                  SearchSpecification(),
                  ParametricLaplacian(),
                  (std::string(File::getBHDir()) + "/Config/cnsBallPerceptor.dat").c_str());

  // Select sample points for checking the ball's pattern
  for(const Eigen::Vector3d& v : TriangleMesh::sphere(theFieldDimensions.ballRadius, sampleSubDivisions).vertex)
    if(v.y() <= theFieldDimensions.ballRadius * sampleHeightRatio && v.z() <= -theFieldDimensions.ballRadius * sampleDepthRatio)
      samplePoints.push_back(v);

  createOrLoadPatternTable();
}

void CNSBallPerceptor::update(BallPercept2017& ballPercept)
{
  DECLARE_DEBUG_DRAWING("module:CNSBallPerceptor:candidates", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CNSBallPerceptor:ballRegion", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CNSBallPerceptor:ring", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CNSBallPerceptor:searchSpace", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CNSBallPerceptor:around", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CNSBallPerceptor:samplePoints", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CNSBallPerceptor:acceptedPattern", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:CNSBallPerceptor:glcm", "drawingOnImage");
  
#ifdef SHOW_TIME
  high_resolution_clock::time_point startT,endT;
  startT = high_resolution_clock::now();
#endif

  updateCamera();

  objects.clear();
  ballPercept.candidateBalls.clear();
  ballPercept.status = BallPercept2017::notSeen;

  if(theCameraMatrix.isValid)
  {
    updateSearchSpace();

    if(usePrediction && theBallPrediction.isValid)
    {
      Matrix4d cameraInImage;
      cameraInImage << 0, -1,  0, 0,
                       0,  0, -1, 0,
                       1,  0,  0, 0,
                       0,  0,  0, 1;
      Matrix4d cameraInRobot = toMatrix4d(theCameraMatrix);
      Matrix4d ballInRobot;
      ballInRobot << 1, 0, 0, theBallPrediction.position.x(),
                     0, 1, 0, theBallPrediction.position.y(),
                     0, 0, 1, theFieldDimensions.ballRadius,
                     0, 0, 0, 1;
      Matrix4d ballInImage = cameraInImage * cameraInRobot.inverse() * ballInRobot;
      objects.push_back(IsometryWithResponse(Eigen::Isometry3d(ballInImage), 0));
      if(spec.nRefineIterations > 0)
        detector.refine(theCNSImage, objects.front(), spec.nRefineIterations);
      if(objects.front().response < minResponse)
        objects.clear();
    }

    for(const Boundaryi& region : theBallRegions.regions)
    {
      spec.blockX = region.x.getSize();
      spec.blockY = region.y.getSize();
      detector.setSearchSpecification(spec);
      ObjectCNSStereoDetector::IsometryWithResponses newObjects;
      detector.searchBlockAllPoses(newObjects, theCNSImage, region.x.min, region.y.min);

      for(const IsometryWithResponse& object : newObjects)
        if(object.response >= minResponse)
          objects.emplace_back(object);

      if(spec.nRefineIterations > 0)
      {
        for(IsometryWithResponse& object : newObjects)
          detector.refine(theCNSImage, object, spec.nRefineIterations);

        for(const IsometryWithResponse& object : newObjects)
          if(object.response >= minResponse)
            objects.emplace_back(object);
      }
    }

    sort(objects.begin(), objects.end(), MoreOnResponse());

    std::vector<IsometryWithResponse> guesses;
    while(!objects.empty())
    {
      ballPercept.status = checkBall(objects.front(),ballPercept);
//      if(ballPercept.status == BallPercept2017::seen)
//        break;
//      else if(ballPercept.status == BallPercept2017::guessed)
//        guesses.push_back(objects.front());

      objects.erase(objects.begin());
    }

//    if(ballPercept.status != BallPercept2017::seen && !guesses.empty())
//    {
//      objects.push_back(guesses.front());
//      ballPercept.status = BallPercept2017::guessed;
//    }

//    fillBallPercept(ballPercept);
  }

#ifdef SHOW_TIME
  endT = high_resolution_clock::now();
  duration<double> dTotal = duration_cast<duration<double>>(endT-startT);
  std::cout << "[Processing Time] BallPerceptor----->" << dTotal.count()*1000 <<"ms"<< std::endl;
#endif

  draw();
}

void CNSBallPerceptor::updateCamera()
{
  if(cameras[theCameraInfo.camera].width == 0)
  {
    if(theCameraInfo.camera == CameraInfo::lower)
      cameras[CameraInfo::lower] = CameraModelOpenCV(Eigen::Isometry3d::Identity(),
                                   theCameraInfo.width,
                                   theCameraInfo.height,
                                   theCameraInfo.width / 2.f / std::tan(theCameraIntrinsics.lowerOpeningAngleWidth / 2.f),
                                   theCameraInfo.height / 2.f / std::tan(theCameraIntrinsics.lowerOpeningAngleHeight / 2.f),
                                   theCameraInfo.width * theCameraIntrinsics.lowerOpticalCenter.x(),
                                   theCameraInfo.height * theCameraIntrinsics.lowerOpticalCenter.y());
    else
      cameras[CameraInfo::upper] = CameraModelOpenCV(Eigen::Isometry3d::Identity(),
                                   theCameraInfo.width,
                                   theCameraInfo.height,
                                   theCameraInfo.width / 2.f / std::tan(theCameraIntrinsics.upperOpeningAngleWidth / 2.f),
                                   theCameraInfo.height / 2.f / std::tan(theCameraIntrinsics.upperOpeningAngleHeight / 2.f),
                                   theCameraInfo.width * theCameraIntrinsics.upperOpticalCenter.x(),
                                   theCameraInfo.height * theCameraIntrinsics.upperOpticalCenter.y());
  }

  detector.setCamera(cameras[theCameraInfo.camera]);
}

void CNSBallPerceptor::updateSearchSpace()
{
  Matrix4d cameraInImage;
  cameraInImage << 0, -1,  0, 0,
                   0,  0, -1, 0,
                   1,  0,  0, 0,
                   0,  0,  0, 1;
  Matrix4d cameraInRingCenter = toMatrix4d(theCameraMatrix);
  Matrix4d ringCenterInImage = cameraInImage * cameraInRingCenter.inverse();
  Pose3f p = toPose3f(ringCenterInImage);
  spec.positionSpace = CylinderRing(Eigen::Isometry3d(ringCenterInImage),
                                    0,
                                    maxTableRadius,
                                    theFieldDimensions.ballRadius - 1.f,
                                    theFieldDimensions.ballRadius + 1.f);
  spec.nRefineIterations = refineIterations;
  spec.object2WorldOrientation.clear();
  Vector3f up = theCameraMatrix.rotation.inverse() * Vector3f::UnitZ();
  spec.object2WorldOrientation.push_back(fromTo(Vector3d::UnitZ(), Vector3d(-up.y(), -up.z(), up.x())));
}

void CNSBallPerceptor::fillBallPercept(BallPercept2017& ballPercept) const
{
  if(!objects.empty())
  {
    const IsometryWithResponse& object = objects.front();
    double x, y;
    detector.camera.camera2Image(Vector3d(object(0, 3), object(1, 3), object(2, 3)), x, y);
    ballPercept.positionInImage = Vector2f(static_cast<float>(x), static_cast<float>(y));

    CodedContour contour;
    detector.lr.rasterize(contour, object, detector.camera);
    float sum = 0;
    for(const CodedContourPoint& p : contour)
      sum += Vector2f(static_cast<float>(xOfCCP(p)), static_cast<float>(yOfCCP(p))).norm();
    ballPercept.radiusInImage = sum / static_cast<float>(contour.size());

    const Vector2f correctedCenter = theImageCoordinateSystem.toCorrected(ballPercept.positionInImage);
    Vector3f cameraToBall(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x() - correctedCenter.x(), theCameraInfo.opticalCenter.y() - correctedCenter.y());
    cameraToBall.normalize(theFieldDimensions.ballRadius * theCameraInfo.focalLength / ballPercept.radiusInImage);
    Vector3f rotatedCameraToBall = theCameraMatrix.rotation * cameraToBall;
    const Vector3f sizeBasedCenterOnField = theCameraMatrix.translation + rotatedCameraToBall;
    const Vector3f bearingBasedCenterOnField = theCameraMatrix.translation - rotatedCameraToBall * ((theCameraMatrix.translation.z() - theFieldDimensions.ballRadius) / rotatedCameraToBall.z());

    if(rotatedCameraToBall.z() < 0)
    {
      ballPercept.positionOnField.x() = bearingBasedCenterOnField.x();
      ballPercept.positionOnField.y() = bearingBasedCenterOnField.y();
    }
    else
    {
      ballPercept.positionOnField.x() = sizeBasedCenterOnField.x();
      ballPercept.positionOnField.y() = sizeBasedCenterOnField.y();
    }
  }
}

BallPercept2017::Status CNSBallPerceptor::checkBall(const IsometryWithResponse& object, BallPercept2017& ballPercept) const
{
  char buf[10000];
  COMPLEX_DRAWING("module:CNSBallPerceptor:candidates")
  {
    tip = new OutTextRawMemory(buf);
    leq(minResponse, static_cast<float>(object.response), "min response");
  }

  CodedContour contour;
  detector.lr.rasterize(contour, object, detector.camera);
  const Vector2i center(contour.referenceX, contour.referenceY);

  // Compute dimensions in image.
  Boundaryi range(Rangei(std::numeric_limits<int>::max(), std::numeric_limits<int>::min()),
                  Rangei(std::numeric_limits<int>::max(), std::numeric_limits<int>::min()));
  for(const CodedContourPoint& p : contour)
    range.add(Vector2i(xOfCCP(p), yOfCCP(p)));
  range.add(Vector2i(range.x.max + 1, range.y.max + 1));
  
  const float radius = static_cast<float>(std::max(-range.x.min, range.x.max) + std::max(-range.y.min, range.y.max)) / 2.f;

  float expectedRadius = IISC::getImageBallRadiusByCenter(Vector2f(static_cast<float>(center.x()),
                         static_cast<float>(center.y())),
                         theCameraInfo,
                         theCameraMatrix,
                         theFieldDimensions);
  BallPercept2017::Status status = BallPercept2017::notSeen;
  int sampleImage[900]={0};
  int greenCount, nonWhiteCount;
  int avgGreenY0;
  int avgGreenY1;
  int avgGreenY2;
  int avgGreenY3;

  countAtRadius(center, expectedRadius, greenCount, nonWhiteCount,avgGreenY0,avgGreenY1,avgGreenY2,avgGreenY3);
  float f5 = static_cast<float>(greenCount) / static_cast<float>(numberOfGreenChecks * greenCheckRadiusRatios.size());

  GLCM glcm;
  float greenInCircle = normalizedSampleImage(avgGreenY0,avgGreenY1,avgGreenY2,avgGreenY3,center,expectedRadius,sampleImage);
  calcGrayLevalCoMatrix(sampleImage,glcm);
//  disPlayGLCM(glcm,center,expectedRadius);

  float dR = std::abs(radius - expectedRadius) / expectedRadius;

  int glcmFeature1 = glcm.getMaxMargin();
  int glcmFeature2 = glcm.getMinMargin();
  float glcmFeature3 = glcm.getFeature();
  int glcmFeature4 = glcm.getDeltaMargin();

  if ((f5<0.75f && (glcmFeature1>1200 || glcmFeature2<30 || glcmFeature3>2.f || glcmFeature4>300)) || f5 < 0.4f)
  {
	  RECTANGLE("module:CNSBallPerceptor:ballRegion",center.x()-radius,center.y()-radius,center.x()+radius,center.y()+radius,2,Drawings::solidPen,ColorRGBA::black);
//	  DRAWTEXT("module:CNSBallPerceptor:ballRegion",center.x(),center.y(),10, ColorRGBA::cyan,f5);
  }
  else
  {
	  bool pattern = checkSamplePoints(object,avgGreenY0,avgGreenY1,avgGreenY2,avgGreenY3, center);
//	  disPlayGLCM(glcm,center,expectedRadius);
	    BallPercept2017::BallRegions ballRegions;
	    ballRegions.center = center;
	    ballRegions.radius = radius;
	    ballRegions.expectedRadius = expectedRadius;
	    ballRegions.avgGreen = (avgGreenY0 + avgGreenY1 + avgGreenY2 + avgGreenY3) / 4;
	    ballRegions.greenRatio = f5;
	    ballRegions.greenInCircle = greenInCircle;
	    ballRegions.patternCheck = pattern;
	    ballPercept.candidateBalls.push_back(ballRegions);
		if(f5 > 0.95)
			RECTANGLE("module:CNSBallPerceptor:ballRegion",center.x()-radius,center.y()-radius,center.x()+radius,center.y()+radius,2,Drawings::solidPen,ColorRGBA::red);
	    else
			RECTANGLE("module:CNSBallPerceptor:ballRegion",center.x()-radius,center.y()-radius,center.x()+radius,center.y()+radius,2,Drawings::solidPen,ColorRGBA::green);
		DRAWTEXT("module:CNSBallPerceptor:ballRegion",center.x()-radius,center.y()-radius-7,7, ColorRGBA::red,pattern);
//	    DRAWTEXT("module:CNSBallPerceptor:ballRegion",center.x(),center.y()+7,7, ColorRGBA::violet,object.response);
  }
  if(isLastDuplicative(ballPercept,  static_cast<int>(radius * 1.3)) && !ballPercept.candidateBalls.empty())
  {
	  ballPercept.candidateBalls.erase(ballPercept.candidateBalls.end() - 1);
  }

//  BallPercept2017::Status status = BallPercept2017::notSeen;
//  BallPercept2017::BallRegions ballRegions;
//  ballRegions.center = center;
//  ballRegions.radius = radius;
//  ballRegions.expectedRadius = expectedRadius;
//  ballPercept.candidateBalls.push_back(ballRegions);
//  RECTANGLE("module:CNSBallPerceptor:ballRegion",center.x()-radius,center.y()-radius,center.x()+radius,center.y()+radius,2,Drawings::solidPen,ColorRGBA::black);
//  bool pass = print(checkSamplePoints(object,100), "pattern");
//  float distance = static_cast<float>(object.translation().squaredNorm()) - sqr(theCameraMatrix.translation.z());
//  if(distance > 0)
//    distance = std::sqrt(distance);
//  else
//    distance = 0;
//  DRAWTEXT("module:CNSBallPerceptor:ballRegion",center.x()-radius,center.y()-radius,15,ColorRGBA::magenta,object.response);
//  DRAWTEXT("module:CNSBallPerceptor:ballRegion",center.x()-radius,center.y()-radius+15,15,ColorRGBA::red,pass);
//  int greenCount;
//  int nonWhiteCount;
//  int avgGreenY;
//  
//  int sampleImage[900]={0};
//  GLCM glcm;
//  const float response = static_cast<float>(object.response);
//
//#if defined TARGET_ROBOT || defined NDEBUG
//#define AND &&
//#define OR ||
//#else
//#define AND &
//#define OR |
//#endif
//
//  BallPercept::Status status = BallPercept::notSeen;
//  if(leq(std::abs(radius - expectedRadius) / expectedRadius, maxRadiusDeviation, "delta radius")
//     AND print(theBodyContour.isValidPoint(center), "outside body")
//     AND (((countAround(center, expectedRadius, greenCount, nonWhiteCount,avgGreenY),
//            (leq(minAroundNonWhiteRatio, static_cast<float>(nonWhiteCount) / static_cast<float>(numberOfGreenChecks * greenCheckRadiusRatios.size()), "min around nonWhite")
//             AND leq(minAroundGreenRatio, static_cast<float>(greenCount) / static_cast<float>(numberOfGreenChecks * greenCheckRadiusRatios.size()), "min around green")
//             ? status = BallPercept::guessed, true : false))
//            AND leq(minResponseWithinGreen, response, "min response around")
//            AND leq(minAroundDistance, distance, "min around distance")
//            AND leq(distance, maxAroundDistance, "max around distance"))
//          OR (leq(minResponse + (maxResponse - minResponse) * (1.f - static_cast<float>(greenCount) / static_cast<float>(numberOfGreenChecks * greenCheckRadiusRatios.size())), response, "min response green")
//              AND print(checkSamplePoints(object,avgGreenY), "pattern"))))
//  {
//	  normalizedSampleImage(avgGreenY,center,expectedRadius,sampleImage);
//	  calcGrayLevalCoMatrix(sampleImage,glcm);
//	  disPlayGLCM(glcm,center,expectedRadius);
//    status = BallPercept::seen;
//    COMPLEX_DRAWING("module:CNSBallPerceptor:acceptedPattern")
//      checkSamplePoints(object, avgGreenY, true);
//  }
//
//  COMPLEX_DRAWING("module:CNSBallPerceptor:candidates")
//  {
//    static const ColorRGBA colors[] =
//    {
//      ColorRGBA::red,
//      ColorRGBA::green,
//      ColorRGBA::yellow
//    };
//
//    drawContourViaLutRasterizer(detector.lr, object, detector.camera, colors[status]);
//    if(tip->getLength() > 1)
//    {
//      ASSERT(tip->getLength() < 9999);
//      reinterpret_cast<char*>(tip->getMemory())[tip->getLength()] = 0;
//      TIP("module:CNSBallPerceptor:candidates", center.x(), center.y(), radius, (reinterpret_cast<const char*>(tip->getMemory()) + 1));
//    }
//    delete tip;
//  }

  return status;
}

void CNSBallPerceptor::countAround(const Vector2i& center, float radius, int& greenCount, int& nonWhiteCount, int& avgGreenY0, int& avgGreenY1, int& avgGreenY2, int& avgGreenY3) const
{
  greenCount = nonWhiteCount = 0;
  for(float radiusRatio : greenCheckRadiusRatios)
    countAtRadius(center, radius * radiusRatio, greenCount, nonWhiteCount, avgGreenY0, avgGreenY1, avgGreenY2, avgGreenY3);
//  if (greenCount != 0)
//	avgGreenY /= greenCount;
//  else
//    avgGreenY = theFieldColor.seedY;
  DRAWTEXT("module:CNSBallPerceptor:around",center.x(),center.y(),10,ColorRGBA::yellow,"recommended threshold = " << avgGreenY0<<","
		  <<avgGreenY1<<","<<avgGreenY2<<","<<avgGreenY3);
}

void CNSBallPerceptor::countAtRadius(const Vector2i& center, float radius, int& greenCount, int& nonWhiteCount, int& avgGreenY0, int& avgGreenY1, int& avgGreenY2, int& avgGreenY3) const
{
//  std::function<bool(int, int)> isInside;
//  if(center.x() - radius > 0 && center.x() + radius + 1 < theECImage.colored.width &&
//    center.y() - radius > 0 && center.y() + radius + 1 < theECImage.colored.height)
//    isInside = [&](int, int) -> bool {return true;};
//  else
//    isInside = [&](int x, int y) -> bool
//    {
//      return x >= 0 && x < theECImage.colored.width &&
//             y >= 0 && y < theECImage.colored.height;
//    };
  greenCount = 0;
  avgGreenY0 = 0;
  avgGreenY1 = 0;
  avgGreenY2 = 0;
  avgGreenY3 = 0;
  int greenCnt[4] = {0};
  int avgGreen = 0;
  for(float radiusRatio : greenCheckRadiusRatios)
  {
	  const float epsilon = pi2 / static_cast<float>(numberOfGreenChecks);
	  float dx = -radius * radiusRatio;
	  float dy = 0.f;
	  for(int i = 0; i < numberOfGreenChecks; ++i)
	  {
		int x = center.x() + static_cast<int>(std::round(dx));
		int y = center.y() + static_cast<int>(std::round(dy));
		if(isInside(x, y))
		{
		  const FieldColors::Color color = theECImage.colored[y][x];
		  if(color == FieldColors::green)
		  {
			++greenCount;
			++nonWhiteCount;
			avgGreen += theECImage.grayscaled[y][x];
	//        std::cout<<x<<","<<theECImage.grayscaled.width<<","<<y<<","<<theECImage.grayscaled.height<<std::endl;
			ASSERT(x >= 0 && x < theECImage.grayscaled.width &&
					 y >= 0 && y < theECImage.grayscaled.height);
	//		avgGreenY[0] += theECImage.grayscaled[y][x];
			if(x <= center.x() && y <= center.y())
			{
				greenCnt[0] ++;
//				greenCnt1++;
//				avgGreen += theECImage.grayscaled[y][x];
				avgGreenY0 += theECImage.grayscaled[y][x];
			}
			else if(x >= center.x() && y <= center.y())
			{
				greenCnt[1] ++;
//				greenCnt1++;
//				avgGreen += theECImage.grayscaled[y][x];
				avgGreenY1 += theECImage.grayscaled[y][x];
			}
			else if(x <= center.x() && y >= center.y())
			{
				greenCnt[2] ++;
//				greenCnt1++;
//				avgGreen += theECImage.grayscaled[y][x];
				avgGreenY2 += theECImage.grayscaled[y][x];
			}
			else if(x >= center.x() && y >= center.y())
			{
				greenCnt[3] ++;
//				greenCnt1++;
//				avgGreen += theECImage.grayscaled[y][x];
				avgGreenY3 += theECImage.grayscaled[y][x];
			}
			DOT("module:CNSBallPerceptor:around", x, y, ColorRGBA::green, ColorRGBA::green);
		  }
		  else if(color != FieldColors::white)
		  {
			++nonWhiteCount;
			DOT("module:CNSBallPerceptor:around", x, y, ColorRGBA::red, ColorRGBA::red);
		  }
		  else
			DOT("module:CNSBallPerceptor:around", x, y, ColorRGBA::yellow, ColorRGBA::yellow);
		}
		dx -= dy * epsilon;
		dy += dx * epsilon;
	  }
  }
  if(greenCount == 0)
  {
	  ASSERT(greenCount == 0);
	  avgGreenY0 = theFieldColor.seedY + 20;
	  avgGreenY1 = theFieldColor.seedY + 20;
	  avgGreenY2 = theFieldColor.seedY + 20;
	  avgGreenY3 = theFieldColor.seedY + 20;
	  return;
  }
  int greenAvg =  avgGreen / greenCount;
  if(greenCnt[0] == 0)
	{
	  avgGreenY0 = greenAvg;
	}
	else
	{
		ASSERT(greenCnt[0] != 0);
	  avgGreenY0 /= greenCnt[0];
	}
    if(greenCnt[1] == 0)
	{
	  avgGreenY1 = greenAvg;
	}
	else
	{
		ASSERT(greenCnt[1] != 0);
	  avgGreenY1 /= greenCnt[1];
	}
	if(greenCnt[2] == 0)
	{
	  avgGreenY2 = greenAvg;
	}
	else
	{
		ASSERT(greenCnt[2] != 0);
	  avgGreenY2 /= greenCnt[2];
	}
    if(greenCnt[3] == 0)
    {
  	  avgGreenY3 = greenAvg;
    }
    else
    {
    	ASSERT(greenCnt[3] != 0);
  	  avgGreenY3 /= greenCnt[3];
    }
    avgGreenY0 += 20;
    avgGreenY1 += 20;
    avgGreenY2 += 5;
    avgGreenY3 += 5;
    DRAWTEXT("module:CNSBallPerceptor:around",center.x(),center.y()+10,10,ColorRGBA::red,"recommended threshold = " << avgGreenY0<<","
    		  <<avgGreenY1<<","<<avgGreenY2<<","<<avgGreenY3);
    DRAWTEXT("module:CNSBallPerceptor:around",center.x(),center.y()+20,10,ColorRGBA::red,"recommended threshold = " <<greenAvg);
}

bool CNSBallPerceptor::isInside(int x, int y) const
{
	return x >= 0 && x < theECImage.colored.width &&
	             y >= 0 && y < theECImage.colored.height;
}

bool CNSBallPerceptor::leq(float op1, float op2, const char* label) const
{
  COMPLEX_DRAWING("module:CNSBallPerceptor:candidates")
    *tip << endl << (op1 <= op2 ? "x" : "  ") << "  " << label << "\t"
         << std::round(op1 * 100.f) / 100.f << "\t" << std::round(op2 * 100.f) / 100.f;

  return op1 <= op2;
}

bool CNSBallPerceptor::print(bool result, const char* label) const
{
  COMPLEX_DRAWING("module:CNSBallPerceptor:candidates")
    *tip << endl << (result ? "x" : "  ") << "  " << label;

  return result;
}

void CNSBallPerceptor::print(float value, const char* label) const
{
  COMPLEX_DRAWING("module:CNSBallPerceptor:candidates")
  *tip << endl << "  " << value << "  " << label;
}

Pose3f CNSBallPerceptor::toPose3f(const Matrix4d& m)
{
  RotationMatrix r;
  r << static_cast<float>(m(0, 0)), static_cast<float>(m(0, 1)), static_cast<float>(m(0, 2)),
       static_cast<float>(m(1, 0)), static_cast<float>(m(1, 1)), static_cast<float>(m(1, 2)),
       static_cast<float>(m(2, 0)), static_cast<float>(m(2, 1)), static_cast<float>(m(2, 2));
  return Pose3f(r, Vector3f(static_cast<float>(m(0, 3)), static_cast<float>(m(1, 3)), static_cast<float>(m(2, 3))));
}

Matrix4d CNSBallPerceptor::toMatrix4d(const Pose3f& p)
{
  const Vector3f& t = p.translation;
  const Matrix3f& r = p.rotation;
  Matrix4d result;
  result << r(0, 0), r(0, 1), r(0, 2), t.x(),
            r(1, 0), r(1, 1), r(1, 2), t.y(),
            r(2, 0), r(2, 1), r(2, 2), t.z(),
            0,       0,       0,       1;
  return result;
}

bool CNSBallPerceptor::checkSamplePoints(const IsometryWithResponse& object, int& avgGreenY0, int& avgGreenY1, int& avgGreenY2, int& avgGreenY3, const Vector2i center, bool accepted) const
{
#ifndef BHUMAN
  const double aroundX = std::atan2(object.translation().y(), object.translation().z());
  const double aroundY = std::atan2(object.translation().x(), object.translation().z());
  const Quaterniond rotation = Quaterniond(Eigen::AngleAxisd(-aroundX, Vector3d::UnitX())) *
                               Quaterniond(Eigen::AngleAxisd(aroundY, Vector3d::UnitY()));
  float avgFactor = 0;
  std::vector<Vector2i> points;
  points.reserve(samplePoints.size());
  std::vector<unsigned char> brightnesses;
  brightnesses.reserve(samplePoints.size());
  Eigen::Isometry3d ballInWorld = spec.positionSpace.cylinder2World.inverse();
  ballInWorld(0, 3) = ballInWorld(1, 3) = ballInWorld(2, 3) = 0;

  for(const Eigen::Vector3d& samplePoint : samplePoints)
  {
    double x, y;
    const Vector3d visiblePoint = rotation * samplePoint;
    if(!detector.camera.world2ImageClipped(object.translation() + visiblePoint, x, y))
      return false;
    const Vector2i point(static_cast<int>(std::round(x)), static_cast<int>(std::round(y)));
    if(point.x() >= theCameraInfo.width || point.y() >= theCameraInfo.height || point.x() < 0 || point.y() < 0)
      return false;
    points.push_back(point);
    const float z = static_cast<float>((ballInWorld * visiblePoint).z());
    const float factor = 1.f ;//+ (1.f - z / theFieldDimensions.ballRadius) * brightnessBonus;
	avgFactor += factor;
    const int brightness = std::min(255, static_cast<int>(theECImage.grayscaled[point.y()][point.x()] * factor));
    brightnesses.push_back(static_cast<unsigned char>(brightness));
  }
  
  avgFactor /= float(samplePoints.size());

//  const int threshold = calcThreshold(brightnesses);
  BallPattern bits = 0;
  int whiteCnt = 0;
  int whiteSum=0, blackSum=0;
  for(size_t i = 0; i < points.size(); ++i)
  {
    const Vector2i& point = points[i];
    bool dark;// = brightnesses[i] <= threshold;
    if(point.x() <= center.x() && point.y() <= center.y())
	{
    	dark = brightnesses[i] <= avgGreenY0;
	}
	else if(point.x() >= center.x() && point.y() <= center.y())
	{
		dark = brightnesses[i] <= avgGreenY1;
	}
	else if(point.x() <= center.x() && point.y() >= center.y())
	{
		dark = brightnesses[i] <= avgGreenY2;
	}
	else if(point.x() >= center.x() && point.y() >= center.y())
	{
		dark = brightnesses[i] <= avgGreenY3;
	}
//    if(accepted)
//      DOT("module:CNSBallPerceptor:acceptedPattern", point.x(), point.y(), dark ? ColorRGBA::red : ColorRGBA::blue, dark ? ColorRGBA::red : ColorRGBA::blue);
//    else
      DOT("module:CNSBallPerceptor:samplePoints", point.x(), point.y(), dark ? ColorRGBA::red : ColorRGBA::blue, dark ? ColorRGBA::red : ColorRGBA::blue);
//    if(theECImage.colored[point.y()][point.x()] != FieldColors::Color::green)
    	bits = bits << 1 | (dark ? 0 : 1);
    if (!dark)
    {
    	whiteSum += brightnesses[i];
    	whiteCnt++;
    }
    else
    {
    	blackSum += brightnesses[i];
    }

  }
  float contrast = 0;
  if (blackSum==0 || whiteSum==0)
	  contrast = 0;
  else
  {
	  float meanWhite = whiteSum/float(whiteCnt);
	  float meanBlack = blackSum/float( points.size()-whiteCnt);
	  contrast = (meanWhite-meanBlack)/meanWhite;
  }
//  return checkContrast(brightnesses, threshold) && ballPatterns.find(bits) != ballPatterns.end();
  DRAWTEXT("module:CNSBallPerceptor:ballRegion",center.x()+10,center.y()+10,10, ColorRGBA::cyan,contrast);
  float distance = static_cast<float>(object.translation().squaredNorm()) - sqr(theCameraMatrix.translation.z());
    if(distance > 0)
      distance = std::sqrt(distance);
    else
      distance = 0;
  float thre = distance > 2500.f? 0.4f:0.5f;
  if(contrast < 0.25)
	  return false;
  return ballPatterns.find(bits) != ballPatterns.end();
#else
  const double aroundX = std::atan2(object.translation().y(), object.translation().z());
    const double aroundY = std::atan2(object.translation().x(), object.translation().z());
    const Quaterniond rotation = Quaterniond(Eigen::AngleAxisd(-aroundX, Vector3d::UnitX())) *
                                 Quaterniond(Eigen::AngleAxisd(aroundY, Vector3d::UnitY()));
    float avgFactor = 0;
    std::vector<Vector2i> points;
    points.reserve(samplePoints.size());
    std::vector<unsigned char> brightnesses;
    brightnesses.reserve(samplePoints.size());
    Eigen::Isometry3d ballInWorld = spec.positionSpace.cylinder2World.inverse();
    ballInWorld(0, 3) = ballInWorld(1, 3) = ballInWorld(2, 3) = 0;

    for(const Eigen::Vector3d& samplePoint : samplePoints)
    {
      double x, y;
      const Vector3d visiblePoint = rotation * samplePoint;
      if(!detector.camera.world2ImageClipped(object.translation() + visiblePoint, x, y))
        return false;
      const Vector2i point(static_cast<int>(std::round(x)), static_cast<int>(std::round(y)));
      if(point.x() >= theCameraInfo.width || point.y() >= theCameraInfo.height || point.x() < 0 || point.y() < 0)
        return false;
      points.push_back(point);
      const float z = static_cast<float>((ballInWorld * visiblePoint).z());
      const float factor = 1.f + (1.f - z / theFieldDimensions.ballRadius) * brightnessBonus;
  	avgFactor += factor;
      const int brightness = std::min(255, static_cast<int>(theECImage.grayscaled[point.y()][point.x()] * factor));
      brightnesses.push_back(static_cast<unsigned char>(brightness));
    }

    avgFactor /= float(samplePoints.size());

    const int threshold = calcThreshold(brightnesses);
    BallPattern bits = 0;
    int whiteCnt = 0;
    int whiteSum=0, blackSum=0;
    for(size_t i = 0; i < points.size(); ++i)
    {
      const Vector2i& point = points[i];
      bool dark = brightnesses[i] <= threshold;
//      if(point.x() <= center.x() && point.y() <= center.y())
//  	{
//      	dark = brightnesses[i] <= avgGreenY0;
//  	}
//  	else if(point.x() >= center.x() && point.y() <= center.y())
//  	{
//  		dark = brightnesses[i] <= avgGreenY1;
//  	}
//  	else if(point.x() <= center.x() && point.y() >= center.y())
//  	{
//  		dark = brightnesses[i] <= avgGreenY2;
//  	}
//  	else if(point.x() >= center.x() && point.y() >= center.y())
//  	{
//  		dark = brightnesses[i] <= avgGreenY3;
//  	}
  //    if(accepted)
  //      DOT("module:CNSBallPerceptor:acceptedPattern", point.x(), point.y(), dark ? ColorRGBA::red : ColorRGBA::blue, dark ? ColorRGBA::red : ColorRGBA::blue);
  //    else
        DOT("module:CNSBallPerceptor:samplePoints", point.x(), point.y(), dark ? ColorRGBA::red : ColorRGBA::blue, dark ? ColorRGBA::red : ColorRGBA::blue);
  //    if(theECImage.colored[point.y()][point.x()] != FieldColors::Color::green)
      	bits = bits << 1 | (dark ? 0 : 1);
      if (!dark)
      {
      	whiteSum += brightnesses[i];
      	whiteCnt++;
      }
      else
      {
      	blackSum += brightnesses[i];
      }

    }
    float contrast = 0;
    if (blackSum==0 || whiteSum==0)
  	  contrast = 0;
    else
    {
  	  float meanWhite = whiteSum/float(whiteCnt);
  	  float meanBlack = blackSum/float( points.size()-whiteCnt);
  	  contrast = (meanWhite-meanBlack)/meanWhite;
    }
  //  return checkContrast(brightnesses, threshold) && ballPatterns.find(bits) != ballPatterns.end();
    DRAWTEXT("module:CNSBallPerceptor:ballRegion",center.x()+10,center.y()+10,10, ColorRGBA::cyan,contrast);
    float distance = static_cast<float>(object.translation().squaredNorm()) - sqr(theCameraMatrix.translation.z());
      if(distance > 0)
        distance = std::sqrt(distance);
      else
        distance = 0;
    float thre = distance > 2500.f? 0.4f:0.5f;
    if(contrast < 0.25)
  	  return false;
    return ballPatterns.find(bits) != ballPatterns.end();
#endif
}

int CNSBallPerceptor::calcThreshold(const std::vector<unsigned char>& brightnesses)
{
  // Build histogram
  std::array<unsigned char, 256> histogram;
  histogram.fill(0);
  int minIndex = 255;
  int maxIndex = 0;
  for(unsigned char brightness : brightnesses)
  {
    ++histogram[brightness];
    minIndex = std::min(minIndex, static_cast<int>(brightness));
    maxIndex = std::max(maxIndex, static_cast<int>(brightness));
  }

  int sumCov2 = 0;
  int sumLow = 0; // Sum of coverage values <= the threshold
  int sumHigh = 0; // Sum of coverage values > the threshold
  int sumCovLow = 0; // Sum of coverage values <= the threshold weighted with its occurrence
  int sumCovHigh = 0; // Sum of coverage values > the threshold weighted with its occurrence

  // Initial threshold is 0.
  for(int i = minIndex; i <= maxIndex; ++i)
    if(histogram[i])
    {
      sumCov2 += histogram[i] * i * i;
      sumCovHigh += histogram[i] * i;
      sumHigh += histogram[i];
    }

  int bestThreshold = -1; // Best coverage threshold
  float bestBalance = static_cast<float>(sumCovHigh * sumCovHigh) / static_cast<float>(sumHigh);
  for(int i = minIndex; i <= maxIndex; ++i)
    if(histogram[i])
    {
      sumHigh -= histogram[i];
      sumLow += histogram[i];
      sumCovLow += histogram[i] * i;
      sumCovHigh -= histogram[i] * i;
      if(sumLow > 0 && sumHigh > 0)
      {
        float balance = static_cast<float>(sumCovLow * sumCovLow) / static_cast<float>(sumLow)
                        + static_cast<float>(sumCovHigh * sumCovHigh) / static_cast<float>(sumHigh);
        if(balance > bestBalance)
        {
          bestBalance = balance;
          bestThreshold = static_cast<int>(i);
        }
      }
    }

  return bestThreshold;
}

bool CNSBallPerceptor::checkContrast(const std::vector<unsigned char>& brightnesses, int threshold) const
{
  unsigned sumBlack = 0;
  unsigned sumWhite = 0;
  unsigned countBlack = 0;
  for(unsigned char brightness : brightnesses)
    if(brightness <= threshold)
    {
      sumBlack += brightness;
      ++countBlack;
    }
    else
      sumWhite += brightness;

  if(countBlack == 0 || countBlack == brightnesses.size())
    return print(false, "contrast");
  else
  {
    unsigned meanBlack = sumBlack / countBlack;
    unsigned meanWhite = sumWhite / (static_cast<unsigned>(brightnesses.size()) - countBlack);
    return leq(minContrast, (static_cast<float>(meanWhite) - static_cast<float>(meanBlack)) / static_cast<float>(meanWhite), "minContrast");
  }
}

void CNSBallPerceptor::createOrLoadPatternTable()
{
  const size_t bytesPerPattern = (samplePoints.size() + 7) / 8;
  ASSERT(sizeof(BallPattern) >= bytesPerPattern);

  InBinaryFile in("ballPatterns.dat");
  if(in.exists())
  {
    unsigned size;
    BallPattern ballPattern = 0;
    in >> size;
    ballPatterns.clear();
    ballPatterns.reserve(size);
    while(size-- > 0)
    {
      in.read(&ballPattern, bytesPerPattern);
      ballPatterns.insert(ballPattern);
    }
    return;
  }

  unsigned char ballTexture[ballTextureHeight][ballTextureWidth];
  InBinaryFile stream("ballTexture.dat");
  if(stream.exists())
    stream.read(ballTexture, sizeof(ballTexture));

  ballPatterns.clear();
  for(float x = -sampleRange; x < sampleRange; x += sampleStep)
  {
    printf("%f\n", x);
    for(float y = -sampleRange; y < sampleRange; y += sampleStep)
      for(float z = -sampleRange; z < sampleRange; z += sampleStep)
      {
        const BallPattern pattern = getPattern(ballTexture, x, y, z);
        for(float dx = -sampleNeighborhood; dx < sampleNeighborhood; dx += sampleStep)
          for(float dy = -sampleNeighborhood; dy < sampleNeighborhood; dy += sampleStep)
            {
              const BallPattern neighborPattern = getPattern(ballTexture, x + dx, y + dy, z);
              ballPatterns.insert(pattern & neighborPattern);
              ballPatterns.insert(pattern | neighborPattern);
            }
      }
  }

  OutBinaryFile out("ballPatterns.dat");
  out << static_cast<unsigned>(ballPatterns.size());
  for(BallPattern ballPattern : ballPatterns)
    out.write(&ballPattern, bytesPerPattern);
}

CNSBallPerceptor::BallPattern CNSBallPerceptor::getPattern(unsigned char ballTexture[ballTextureHeight][ballTextureWidth], float x, float y, float z) const
{
  Quaterniond rotation = Quaterniond(Eigen::AngleAxisd(x, Vector3d::UnitX())) *
                         Quaterniond(Eigen::AngleAxisd(y, Vector3d::UnitY())) *
                         Quaterniond(Eigen::AngleAxisd(z, Vector3d::UnitZ()));
  BallPattern bits = 0;
  for(const Eigen::Vector3d& v : samplePoints)
    bits = bits << 1 | (getBrightness(ballTexture, rotation * v) < 128 ? 0 : 1);
  return bits;
}

unsigned char CNSBallPerceptor::getBrightness(unsigned char ballTexture[ballTextureHeight][ballTextureWidth], const Vector3d& v)
{
  Angle aroundY = static_cast<float>(std::atan2(v.x(), v.z()));
  const Vector3d v2 = Quaterniond(Eigen::AngleAxisd(-aroundY, Vector3d::UnitY())) * v;
  const Angle aroundX = static_cast<float>(std::atan2(v2.y(), v2.z())) + pi_2;
  aroundY += pi;
  const int x = static_cast<int>(aroundY * ballTextureWidth / pi2) % ballTextureWidth;
  const int y = static_cast<int>(aroundX * ballTextureHeight / pi) % ballTextureHeight;
  ASSERT(x >= 0 && x < ballTextureWidth);
  ASSERT(y >= 0 && y < ballTextureHeight);
  return ballTexture[y][x];
}

void CNSBallPerceptor::draw()
{
  spec.blockX = 64;
  spec.blockY = 48;
  detector.setSearchSpecification(spec);
  COMPLEX_DRAWING("module:CNSBallPerceptor:ring")
  {
    drawCylinderRing(detector.spec.positionSpace, detector.camera);
  }
  COMPLEX_DRAWING("module:CNSBallPerceptor:searchSpace")
  {
    Contour ctAll;
    detector.renderSearchSpace(ctAll, detector.camera, 1);
    drawRasteredContour(ctAll, ColorRGBA::green);
  }
}

void CNSBallPerceptor::drawContourViaLutRasterizer(const LutRasterizer& lr, const Eigen::Isometry3d& object2World, const CameraModelOpenCV& camera, const ColorRGBA& color) const
{
  CodedContour contour;
  lr.rasterize(contour, object2World, camera);
  for(unsigned i = 0; i < contour.size(); ++i)
  {
    int x = xOfCCP(contour[i]) + contour.referenceX,
    y = yOfCCP(contour[i]) + contour.referenceY;
    double alpha = angleOfCCP(contour[i]);
    double c = cos(alpha),
    s = sin(alpha);
    LINE("module:CNSBallPerceptor:candidates", (int) round(x - 2 * s), (int) round(y + 2 * c),
         (int) round(x + 2 * s), (int) round(y - 2 * c), 1, Drawings::solidPen, color);
  }
}

void CNSBallPerceptor::drawRasteredContour(const Contour& contour, const ColorRGBA& color) const
{
  for(unsigned i = 0; i < contour.size(); ++i)
  {
    int x = contour[i].x + contour.referenceX,
    y = contour[i].y + contour.referenceY;
    double alpha = contour[i].angle;
    double c = cos(alpha),
    s = sin(alpha);
    LINE("module:CNSBallPerceptor:searchSpace", round(x - 2 * s), round(y + 2 * c),
         round(x + 2 * s), round(y - 2 * c), 1, Drawings::solidPen, color);
  }
}

void CNSBallPerceptor::drawCylinderRing(const CylinderRing& cylinderRing, const CameraModelOpenCV& camera) const
{
  ColorRGBA colors[4] =
  {
    ColorRGBA(255, 255, 255),
    ColorRGBA(255, 0, 0),
    ColorRGBA(0, 255, 0),
    ColorRGBA(0, 0, 255)
  };
  for(int i = 0; i < 4; ++i)
  {
    ColorRGBA& color = colors[i];
    for(double alpha = 0; alpha < 2 * M_PI; alpha += 0.02)
    {
      double x, y;
      if(camera.world2ImageClipped(cylinderRing.sampleEdgePoint(alpha, i), x, y))
        DOT("module:CNSBallPerceptor:ring", x, y, color, color);
    }
  }
}

void CNSBallPerceptor::calcGrayLevalCoMatrix(int* sampleImage, GLCM& glcm) const
{
	int P[2][2][4]={};
	for (int i=0; i<2; ++i)
		for (int j=0; j<2; ++j)
		{
			for (int m=0; m<30; ++m)
				for (int n=0; n<30; ++n)
				{
					if (n<29 && sampleImage[30*m+n]==i && sampleImage[m*30+n+1]==j)
					{
						P[i][j][0] += 1;
						P[j][i][0] = P[i][j][0];
					}
					if (m>0 && n<29 && sampleImage[30*m+n]==i && sampleImage[(m-1)*30+n+1]==j)
					{
						P[i][j][1] += 1;
						P[j][i][1] = P[i][j][1];
					}
					if (m<29 && sampleImage[30*m+n]==i && sampleImage[(m+1)*30+n]==j)
					{
						P[i][j][2] += 1;
						P[j][i][2] = P[i][j][2];
					}
					if (m<29 && n<29 && sampleImage[30*m+n]==i && sampleImage[(m+1)*30+n+1]==j)
					{
						P[i][j][3] += 1;
						P[j][i][3] = P[i][j][3];
					}
				}
			if (i==j)
				for(int l=0; l<4; ++l)
					P[i][j][l] *= 2;
		}
	// Decompose P to GLCM structure
	for (int l=0; l<2; ++l)
		for (int k=0; k<2; ++k)
	{
		glcm.glcm_0[l][k] = P[l][k][0];
		glcm.glcm_45[l][k] = P[l][k][1];
		glcm.glcm_90[l][k] = P[l][k][2];
		glcm.glcm_135[l][k] = P[l][k][3];
	}
}

float CNSBallPerceptor::normalizedSampleImage(int& avgGreenY0, int& avgGreenY1, int& avgGreenY2, int& avgGreenY3, Vector2i center, float r, int* sampleImage) const
{

  float imageScale = r / 15;  // (2 * r) / 30, 30 is dest size
  int x = static_cast<int>(center.x() - r);
  int y = static_cast<int>(center.y() - r);
  int imageWidth = theECImage.grayscaled.width;
  int imageHeight = theECImage.grayscaled.height;
//  int thre1 = avgGreenY; //theFieldColor.seedY+5;
//  int thre2 = avgGreenY + 30; //theFieldColor.seedY+30;
  float meanBlack = 0;
  float meanWhite = 0;
  int whiteCnt = 0;
  int tempI, tempJ;
  float greenCnt = 0;
  int cnt = 0;
  for (int i = 0; i < 30; ++i)
  {
    for (int j = 0; j < 30; ++j)
    {
      tempI = y + static_cast<int>(i * imageScale);
      tempJ = x + static_cast<int>(j * imageScale);
      if (tempI >= imageHeight)
        tempI = imageHeight - 1;
      if (tempJ >= imageWidth)
        tempJ = imageWidth - 1;
      if (tempI < 0)
        tempI = 0;
      if (tempJ < 0)
        tempJ = 0;
      ASSERT(tempI < imageHeight && tempI >= 0 && tempJ < imageWidth && tempJ >= 0);
      ASSERT(i * 30 + j < 900);
	  Vector2f point(i-15,j-15);
	  if (point.norm()<15)
	  {
		  cnt++;
		  if(theECImage.colored[tempI][tempJ] == FieldColors::Color::green)
			  greenCnt ++;
		if(tempJ <= center.x() && tempI <= center.y())
		{
			sampleImage[i * 30 + j] = theECImage.grayscaled[tempI][tempJ] >= avgGreenY0;
		}
		else if(tempJ >= center.x() && tempI <= center.y())
		{
			sampleImage[i * 30 + j] = theECImage.grayscaled[tempI][tempJ] >= avgGreenY1;
		}
		else if(tempJ <= center.x() && tempI >= center.y())
		{
			sampleImage[i * 30 + j] = theECImage.grayscaled[tempI][tempJ] >= avgGreenY2;
		}
		else if(tempJ >= center.x() && tempI >= center.y())
		{
			sampleImage[i * 30 + j] = theECImage.grayscaled[tempI][tempJ] >= avgGreenY3;
		}
		if(sampleImage[i * 30 + j] == 1)
		{
			meanWhite += theECImage.grayscaled[tempI][tempJ];
			whiteCnt ++;
		}
		else
		{
			meanBlack += theECImage.grayscaled[tempI][tempJ];
		}
//		  if(theECImage.colored[tempI][tempJ] == FieldColors::Color::black)
//			  sampleImage[i * 30 + j] = 0;
//		  else if(theECImage.colored[tempI][tempJ] == FieldColors::Color::white)
//			  sampleImage[i * 30 + j] = 1;
//		  if (theECImage.grayscaled[tempI][tempJ] < thre2 && tempI < std::max(0, center.y())) {
//			  sampleImage[i * 30 + j] = 0;
//		  			} else if (theECImage.grayscaled[tempI][tempJ] < thre1
//		  					&& tempI>= std::max(0, center.y())) {
//		  				sampleImage[i * 30 + j] = 0;
//		  			} else if (theECImage.grayscaled[tempI][tempJ] > thre2
//		  					&& tempI < std::max(0, center.y())) {
//		  				sampleImage[i * 30 + j] = 1;
//		  				greenCnt ++;
//		  			} else if (theECImage.grayscaled[tempI][tempJ] > thre1
//		  					&& tempI >= std::max(0, center.y())) {
//		  				sampleImage[i * 30 + j] = 1;
//		  				greenCnt ++;
//		  			}
//		  if(point.norm()<14)
//		  if (theECImage.colored[tempI][tempJ] == FieldColors::Color::white)
//		  {
////			  DOT("module:CNSBallPerceptor:ballRegion", tempI, tempJ,
////			  						ColorRGBA::green, ColorRGBA::green);
////			  greenCnt ++;
//		  }
	  }
    }
  }
  float contrast = 0;
  if(whiteCnt == cnt || whiteCnt == 0)
  {

  }
  else
  {
	  meanWhite /= static_cast<float>(whiteCnt);
	  meanBlack /= static_cast<float>(cnt - whiteCnt);
	  contrast = (meanWhite - meanBlack) / meanWhite;
  }

//  DRAWTEXT("module:CNSBallPerceptor:ballRegion",center.x()+14,center.y()+14,7, ColorRGBA::magenta,contrast);
  greenCnt = (greenCnt / static_cast<float>(cnt));
  return greenCnt;
}

void CNSBallPerceptor::disPlayGLCM(GLCM& glcm, const Vector2i center, const float radius) const
{
		int f1 = glcm.getMaxMargin();
		int f2 = glcm.getMinMargin();
		int f3 = glcm.getDeltaMargin();
	TIP("module:CNSBallPerceptor:glcm", center.x(), center.y(), radius,
	  "+00\t" << glcm.glcm_0[0][0] << "\t" << glcm.glcm_0[0][1] << "\t" << glcm.glcm_0[1][0] << "\t" << glcm.glcm_0[1][1] << "\n"
	  << "+45\t" << glcm.glcm_45[0][0] << "\t" << glcm.glcm_45[0][1] << "\t" << glcm.glcm_45[1][0] << "\t" << glcm.glcm_45[1][1] << "\n"
	  << "+90\t" << glcm.glcm_90[0][0] << "\t" << glcm.glcm_90[0][1] << "\t" << glcm.glcm_90[1][0] << "\t" << glcm.glcm_90[1][1] << "\n"
	  << "-45\t" << glcm.glcm_135[0][0] << "\t" << glcm.glcm_135[0][1] << "\t" << glcm.glcm_135[1][0] << "\t" << glcm.glcm_135[1][1] << "\n"
	  << f1 << "\t" << f2 << "\t" << f3 << "\t" << "\t" << glcm.getFeature());

//
//	TIP("module:CNSBallPerceptor:glcm", center.x(), center.y(), radius,
//			"max = "<< f1 << "\n" << "min = " << f2 << "\n" << f3);
}

bool CNSBallPerceptor::isLastDuplicative(BallPercept2017& ballPercept, const int minAllowedDistanz) const
{
  if(ballPercept.candidateBalls.size() < 2)
    return false;

  const int sqaredAllowdDistanz = sqr(minAllowedDistanz);
  const Vector2i& ballPerceptToCheck = ballPercept.candidateBalls.back().center;
  for(auto ptr = ballPercept.candidateBalls.begin(); ptr < ballPercept.candidateBalls.end() - 1; ptr++)
    if(((*ptr).center - ballPerceptToCheck).squaredNorm() < sqaredAllowdDistanz)
      return true;

  return false;
}







