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

#include "BallPerceptor2017.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Math/Geometry.h"
#include <algorithm>

using namespace std::chrono;

MAKE_MODULE(BallPerceptor2017, perception)

// Alternate drawing macros that scale down if required
#define LINE2(n, x1, y1, x2, y2, w, s, c) LINE(n, scale(x1), scale(y1), scale(x2), scale(y2), w, s, c)
#define CROSS2(n, x, y, r, w, s, c) CROSS(n, scale(x), scale(y), scale(r), w, s, c)
#define CIRCLE2(n, x, y, r, w, s1, c1, s2, c2) CIRCLE(n, scale(x), scale(y), scale(r), w, s1, c1, s2, c2)
#define RECTANGLE3(n, x1, y1, x2, y2, w, s, c) RECTANGLE(n, scale(x1), scale(y1), scale(x2), scale(y2), w, s, c)
#define DRAWTEXT2(n, x1, y1, s, c, m) DRAWTEXT(n, scale(x1), scale(y1), s, c, m)
#define DOT2(n, x, y, c1, c2) DOT(n, scale(x), scale(y), c1, c2)

//Constructor
BallPerceptor2017::BallPerceptor2017() {

}
//bool BallPerceptor2017::isWhite(const Image::Pixel* p) const
//{
//	//original
//	     if (p->y<theTJArkVision.W_Ymin)
//	         	 return false;
//	if (p->cb<theTJArkVision.W_Cbmin)
//	         	 return false;
//	     if (p->cr<theTJArkVision.W_Crmin)
//	         	 return false;
//  Image::Pixel ss;
//  theColorTable.getHSV(p,ss);
//
//
////     if (p.y<theTJArkVision.W_Ymin)
////         	 return false;
//
//     if (ss.s>theTJArkVision.W_Smax)
//         	 return false;
//     if (ss.i<theTJArkVision.W_Vmin)
//         	 return false;
//     Image::Pixel sss;
//     theColorTable.getRGB(p,sss);
//     if (sss.r<theTJArkVision.W_Rmin)
//         	 return false;
//     if (sss.g<theTJArkVision.W_Gmin)
//         	 return false;
//     if (sss.b<theTJArkVision.W_Bmin)
//         	 return false;
////      if ((ss.h > 0.8 * theTJArkVision.minH && ss.h < 1.2 * theTJArkVision.maxH)
////          && ss.s < 0.8 * theTJArkVision.minS )
////      return true;
////    theColorTable.getRGB(p,ss);
////      int sum = ss.r + ss.g + ss.b;
////      float ave = sum / 3.0f;
////      float Cor = int(ss.r)*int(ss.r) + int(ss.g)*int(ss.g) + int(ss.b)*int(ss.b)
////              + 3 * ave*ave - 2 * (int(ss.r) * ave + int(ss.g) * ave + int(ss.b) * ave);
////      float Var=sqrtf(Cor);
////      if (Var<35)
////      {
////        if (p.y>theTJArkVision.seedY + 30)
////          return true;
////      }
//    return true;
//}

//ColorTable::Colors BallPerceptor::determineColor(const Image::Pixel& p) const
//{
//  if (p.y > theTJArkVision.minCy && p.y < theTJArkVision.maxCy
//      && p.cb > theTJArkVision.minCb && p.cb < theTJArkVision.maxCb
//      && p.cr > theTJArkVision.minCr && p.cr < theTJArkVision.maxCr)
//    return ColorClasses::green;
//  Image::Pixel ss;
//  theColorTable.getHSV(p, ss);
//  if (ss.h > theTJArkVision.minH && ss.h < theTJArkVision.maxH
//      && ss.s > theTJArkVision.minS && ss.s < theTJArkVision.maxS)
//    return ColorClasses::green;
//  if ((ss.h > 0.8 * theTJArkVision.minH && ss.h < 1.2 * theTJArkVision.maxH)
//      && ss.s < 0.8 * theTJArkVision.minS)
//  {
//    if (p.y < (theTJArkVision.seedY + 5))
//      return ColorClasses::black;
//    if (p.y > (theTJArkVision.seedY + 30))
//      return ColorClasses::white;
////    else
////      return ColorClasses::white;
//  }
//  theColorTable.getRGB(p,ss);
//  int sum = ss.r + ss.g + ss.b;
//  float ave = sum / 3.0f;
//  float Cor = int(ss.r)*int(ss.r) + int(ss.g)*int(ss.g) + int(ss.b)*int(ss.b)
//          + 3 * ave*ave - 2 * (int(ss.r) * ave + int(ss.g) * ave + int(ss.b) * ave);
//  float Var=sqrtf(Cor);
//  if (Var<35)
//  {
//    if (p.y>theTJArkVision.seedY + 20)
//      return ColorClasses::white;
//    else
//      return ColorClasses::black;
//  }
//  return ColorClasses::none;
//}
//ColorTable::Colors BallPerceptor2017::determineColor(
//    const Image::Pixel & p) const
//{
//  if (p.y > theTJArkVision.minCy && p.y < theTJArkVision.maxCy
//      && p.cb > theTJArkVision.minCb && p.cb < theTJArkVision.maxCb
//      && p.cr > theTJArkVision.minCr && p.cr < theTJArkVision.maxCr)
//    return ColorClasses::green;
//  Image::Pixel ss;
//  theColorTable.getHSV(p, ss);
//  if (ss.h > theTJArkVision.minH && ss.h < theTJArkVision.maxH
//      && ss.s > theTJArkVision.minS && ss.s < theTJArkVision.maxS)
//    return ColorClasses::green;
////  if ((ss.h > 0.8 * theTJArkVision.minH && ss.h < 1.2 * theTJArkVision.maxH)
////      && ss.s < 0.8 * theTJArkVision.minS)
////  {
////    if (p.y < (theTJArkVision.seedY + 10))
////      return ColorClasses::black;
//////    if (p.y > (theTJArkVision.seedY + 30))
//////      return ColorClasses::white;
////    else
////      return ColorClasses::white;
////  }
////  theColorTable.getRGB(p,ss);
////    int sum = ss.r + ss.g + ss.b;
////    float ave = sum / 3.0f;
////    float Cor = int(ss.r)*int(ss.r) + int(ss.g)*int(ss.g) + int(ss.b)*int(ss.b)
////            + 3 * ave*ave - 2 * (int(ss.r) * ave + int(ss.g) * ave + int(ss.b) * ave);
////    float Var=sqrtf(Cor);
////    if (Var<35)
////    {
////      if (p.y>theTJArkVision.seedY + 20)
////        return ColorClasses::white;
////      else
////        return ColorClasses::black;
////    }
//  if (isWhite(&p))
//    {
//      if (p.y < (theTJArkVision.seedY - 10))
//        return ColorClasses::black;
//      if (p.y > (theTJArkVision.seedY + 15))
//        return ColorClasses::white;
//    }
//  return ColorClasses::none;
//}

//bool BallPerceptor2017::possibleGreen(const Image::Pixel& p) const
//{
//  if (p.y > theTJArkVision.minCy && p.y < theTJArkVision.maxCy
//      && p.cb > theTJArkVision.minCb && p.cb < theTJArkVision.maxCb
//      && p.cr > theTJArkVision.minCr && p.cr < theTJArkVision.maxCr)
//    return true;
//  Image::Pixel ss;
//  theColorTable.getHSV(p,ss);
//  if (ss.h > theTJArkVision.minH && ss.h < theTJArkVision.maxH
//      && ss.s > theTJArkVision.minS && ss.s < theTJArkVision.maxS)
//    return true;
//  return false;
//}
//
//bool BallPerceptor2017::possibleGrayScale(const Image::Pixel& p) const
//{
//   Image::Pixel ss;
//   theColorTable.getHSV(p,ss);
//    if ((ss.h > 0.8 * theTJArkVision.minH && ss.h < 1.2 * theTJArkVision.maxH)
//        && ss.s < 0.8 * theTJArkVision.minS && abs(p.cb-p.cr)<60)
//    return true;
//    theColorTable.getRGB(p,ss);
//      int sum = ss.r + ss.g + ss.b;
//      float ave = sum / 3.0f;
//      float Cor = int(ss.r)*int(ss.r) + int(ss.g)*int(ss.g) + int(ss.b)*int(ss.b)
//              + 3 * ave*ave - 2 * (int(ss.r) * ave + int(ss.g) * ave + int(ss.b) * ave);
//      float Var=sqrtf(Cor);
////      if (Var<35 && (ss.h > 0.8 * theTJArkVision.minH && ss.h < 1.2 * theTJArkVision.maxH) && abs(p.cb-p.cr)<45)
//      if (Var < 35)
//      {
//          return true;
//      }
//    return false;
//}
//Update Function
void BallPerceptor2017::update(BallPercept& ballPercept) {
	DECLARE_DEBUG_DRAWING("module:BallPerceptor2017:image", "drawingOnImage");
	DECLARE_DEBUG_DRAWING("module:BallPerceptor2017:zdots", "drawingOnImage");
	DECLARE_DEBUG_DRAWING("module:BallPerceptor2017:edgePoints",
			"drawingOnImage");
	DECLARE_DEBUG_DRAWING("module:BallPerceptor2017:blackwhiteball",
			"drawingOnImage");
	DECLARE_DEBUG_DRAWING("module:BallPerceptor2017:colorCounter",
			"drawingOnImage");
	DECLARE_DEBUG_DRAWING("module:BallPerceptor2017:possibleBalls",
			"drawingOnImage");
	DECLARE_DEBUG_DRAWING("module:BallPerceptor2017:passball",
			"drawingOnImage");
//  DECLARE_DEBUG_DRAWING("module:BallPerceptor2017:around", "drawingOnImage");
	std::chrono::high_resolution_clock::time_point startT, endT;
	startT = high_resolution_clock::now();
    scaleInput();
	possibleBalls.clear();

	ballPercept.status = BallPercept::notSeen;
	if (!theCameraMatrix.isValid) {
		return;
	}
	if (theBallPercept2017.candidateBalls.empty())
		return;
	imageHeigth = theECImage.colored.height;
	imageWidth = theECImage.colored.width;

	float bestError = 10.f;
	int bestIdx = 0;
	float maxScore = 0;
//	for(int i=0;i<static_cast<int>(ballPercept.candidateBalls.size());++i)
	for (auto & b : theBallPercept2017.candidateBalls) {
		if (true) {
			Vector2f ballPosition = Vector2f(b.center.x(), b.center.y());

			float disError = 0.f; //(ballPosition-spotPosition).norm()/theBallSpots.ballSpots[i].shouldR;
			float radError = 0.f;//fabs(b.radius - b.expectedRadius)/ b.expectedRadius;
			if ((disError < 0.8f && theCameraInfo.camera == CameraInfo::upper)
					|| (disError < 1.f
							&& theCameraInfo.camera == CameraInfo::lower))
				if ((radError > -1.5f && radError < 1.5f
						&& theCameraInfo.camera == CameraInfo::upper)
						|| (radError > -1.5f && radError < 1.5f
								&& theCameraInfo.camera == CameraInfo::lower)) {
//		  DRAWTEXT("module:BallPerceptor:errors", b.x, b.y-10,10,ColorRGBA::orange,"disError="<<disError);
//		  DRAWTEXT("module:BallPerceptor:errors", b.x, b.y,10,ColorRGBA::magenta,"radError="<<radError);
//		  DRAWTEXT("module:BallPerceptor:errors", b.x, b.y+10,10,ColorRGBA::cyan,"Quality="<<b.q);
//		  DRAWTEXT("module:BallPerceptor:errors", b.x, b.y+20,10,ColorRGBA::magenta,"Distance="<<theBallSpots.ballSpots[i].distance);

					if (b.radius < 6 || b.radius > 100)
						continue;

					CIRCLE("module:BallPerceptor2017:possibleBalls",
							b.center.x(), b.center.y(), b.radius, 2,
							Drawings::solidPen, ColorRGBA::blue,
							Drawings::solidBrush, ColorRGBA(255, 0, 0, 150));
					DRAWTEXT("module:BallPerceptor2017:possibleBalls",
							b.center.x(), b.center.y() + 10, 10,
							ColorRGBA::orange, b.radius);

					candidateBall cb;
					cb.ball.x = b.center.x();
					cb.ball.y = b.center.y();
					cb.ball.radius = b.radius;
					cb.disError = disError;
					greenAround = b.avgGreen;
//					classifyBalls3(cb.ball);
					if(b.patternCheck)
					{
						possibleBalls.push_back(cb);
						continue;
					}
					else if(b.greenRatio>0.95f)
					{
						Vector2f ballPos;
						Vector2f ballOnField;
						Transformation::imageToRobot(b.center, theCameraMatrix, theCameraInfo, ballPos);
						ballOnField = Transformation::robotToField(theRobotPose, ballPos);
						if((ballOnField - Vector2f(3200,0)).norm() > 300.f && (ballOnField - Vector2f(-3200,0)).norm() > 300.f)
						{
							possibleBalls.push_back(cb);
							continue;
						}
						else
						{
							continue;
						}
					}
//					if ((b.greenRatio>0.95f || (b.patternCheck)) && theCameraInfo.camera == CameraInfo::upper)// && b.greenInCircle < 10)
//					{
////						if (checkInnerGreen(b.center, b.radius))
//							possibleBalls.push_back(cb);
//						continue;
//					}
//					else if(theCameraInfo.camera == CameraInfo::lower)
//					{
//						center = Vector2f(b.center.x(), b.center.y());
//						radius = b.radius;
//						DRAWTEXT("module:BallPerceptor2017:passball", center.x()-radius, center.y()-radius ,10,ColorRGBA::magenta,b.patternCheck);
//						if((b.patternCheck))
//						{
//							possibleBalls.push_back(cb);
//							DRAWTEXT("module:BallPerceptor2017:passball", center.x(), center.y() ,10,ColorRGBA::magenta,"pass-PatterbCheck");
//							continue;
//						}
//						else if(b.greenRatio>0.95f && classifyBalls3(cb.ball))
//						{
//							possibleBalls.push_back(cb);
//							DRAWTEXT("module:BallPerceptor2017:passball", center.x(), center.y() ,10,ColorRGBA::magenta,"pass-classifier");
//							continue;
//						}
//						DRAWTEXT("module:BallPerceptor2017:passball", center.x(), center.y() ,10,ColorRGBA::magenta,"FAIL");
//					}
//
//					if (b.patternCheck && classifyBalls2(cb.ball)) {
//						center = Vector2f(b.center.x(), b.center.y());
//						radius = b.radius;
//						DRAWTEXT("module:BallPerceptor2017:possibleBalls",
//								b.center.x(), b.center.y() + 10, 10,
//								ColorRGBA::red, "pass1");
//						if (checkPentagons()) {
//							possibleBalls.push_back(cb);
//							CIRCLE("module:BallPerceptor2017:passball",
//									cb.ball.x, cb.ball.y, cb.ball.radius, 2,
//									Drawings::solidPen, ColorRGBA::blue,
//									Drawings::solidBrush,
//									ColorRGBA(255, 0, 0, 150));
//						}
//					}
				}
		}
	}

//	for (int i = 0; i < static_cast<int>(possibleBalls.size()); ++i) {
//		if (possibleBalls[i].disError < bestError) {
//
//			bestError = possibleBalls[i].disError;
//			bestIdx = i;
//		}
//	}
	if (!possibleBalls.empty()) {
		ballPercept.status = BallPercept::seen;
		ballPercept.positionInImage.x() = possibleBalls[bestIdx].ball.x;
		ballPercept.positionInImage.y() = possibleBalls[bestIdx].ball.y;
		ballPercept.radiusInImage = possibleBalls[bestIdx].ball.radius;
		calculateBallOnField(ballPercept);
		ballPercept.radiusOnField = 50;
		ballPercept.validity = 1.f;
	}
	endT = high_resolution_clock::now();
	duration<double> dTotal = duration_cast<duration<double>>(endT - startT);
//	std::cout << "BallPercept=" << dTotal.count() * 1000 << "ms" << std::endl;
	scaleOutput(ballPercept);
}

//Verification Functions
void BallPerceptor2017::searchBallFromBallSpots(BallPercept2017& ballPercept) {
//  std::vector<BallSpot> ballSpots = theBallSpots.ballSpots;
//
//  balls.clear();
//  ballPercept.status = BallPercept::notSeen;
//  for (const BallSpot& ballSpot : ballSpots)
//  {
//    bool analyse = true;
//    for (const BallPercept& ball : balls)
//    {
//      if ((Vector2f((ballSpot.position.x()), (ballSpot.position.y())) - ball.positionInImage).norm() < (theCameraInfo.camera == CameraInfo::upper ? 4.f * ball.radiusInImage : ball.radiusInImage)) {
//        analyse = false;
//        break;
//      }
//    }
//    
//    if (analyse)
//    {
//    	int meanH = (theTJArkVision.minH+theTJArkVision.maxH)>>1;
//		int meanS = (theTJArkVision.minS+theTJArkVision.maxS)>>1;
//		int halfMeanS = ((meanS/10)>>1)*10;
//		int diffH = abs(meanH-ballSpot.colorH);
//		int diffS = abs(meanS-ballSpot.colorS);
//		BallPerceptor2017::Ball simpleBall;
//		simpleBall.radius = ballSpot.shouldR;
//		simpleBall.x = ballSpot.position.x();
//		simpleBall.y = ballSpot.position.y();
//		candidateBall simpleCb;
//		simpleCb.ball = simpleBall;
//		simpleCb.disError = 0;
//		Vector2f spotPosition = Vector2f(ballSpot.position.x(),ballSpot.position.y());
//  //      DRAWTEXT("module:BallPerceptor:possibleBalls", simpleBall.x, simpleBall.y-10, 10,
//  //                           ColorRGBA::black, "("<<(meanH-theBallSpots.ballSpots[i].colorH)<<","<<(meanS-theBallSpots.ballSpots[i].colorS)-halfMeanS<<")"<<theBallSpots.ballSpots[i].distance);
//  //      DRAWTEXT("module:BallPerceptor:possibleBalls", simpleBall.x, simpleBall.y-10, 10,
//  //                           ColorRGBA::black, theBlackAndWhiteBallPercept.balls[i].distance);
//  //      if (diffH > 30)
//  //          continue;
//  //      if ((theBallSpots.ballSpots[i].colorH>theTJArkVision.minH && theCameraInfo.camera == CameraInfo::upper)
//  //          || (theBallSpots.ballSpots[i].colorH>theTJArkVision.minH && theCameraInfo.camera == CameraInfo::lower))
//  //          continue;
//		if (ballSpot.distance > 7000.f)
//			continue;
//  //      if ((diffS-halfMeanS)<-20)
//  //          continue;
//
//		Vector2f ballColor = Vector2f(ballSpot.colorH,ballSpot.colorS);
//		BallPerceptor2017::Ball b = fitBall(spotPosition,ballSpot.shouldR, ballColor);
//		if (b.found == false)
//			continue;
//		center = Vector2f(b.x, b.y);
//		radius = b.radius;
//      if (analyzeBallSpot(ballSpot, ballPercept))
//      {
//        BallPercept ball;
//        ball.positionInImage = center;
//        ball.radiusInImage = radius;
//        calculateBallOnField(ball);
//        if(checkBallOnField(ball))
//        {
//          drawBall(center);
//          scaleOutput(ball);
//          balls.push_back(ball);
//        }
//        else
//        {
//    ball.positionInImage.x() = 0;
//    ball.positionInImage.y() = 0;
//    ball.radiusInImage = 0;
//    calculateBallOnField(ball);
//    balls.push_back(ball);
//  }
//          drawError(ballSpot, "checkBallOnField");
//        }
//      }
//    }
//  }
//  if (balls.size() != 0) {
//    ballPercept.status = BallPercept::seen;
//    ballPercept.positionInImage = balls[0].positionInImage;
//	ballPercept.radiusInImage = balls[0].radiusInImage;
//	calculateBallOnField(ballPercept);
//	ballPercept.radiusOnField = 50;
//  }
//  else
//  {
//    BallPercept ball;
//    ball.positionInImage.x() = 0;
//    ball.positionInImage.y() = 0;
//    ball.radiusInImage = 0;
//    calculateBallOnField(ball);
//    balls.push_back(ball);
//  }
}

bool BallPerceptor2017::analyzeBallSpot(const BallSpot ballSpot,
		BallPercept2017& ballPercept) {
//  ballValidity = false;
//  bool ans = false;
//  
//  ans =
////  !searchValidEdges() ? drawError(ballSpot, "validEdges") :
//  !checkGreenInside() ? drawError(ballSpot, "checkGreenInside") :
//  !ballValidity ? drawError(ballSpot, "BallValidity") :
//  !checkPentagons() ? drawError(ballSpot, "checkPentagon()") :
//  true;
//  
//  return ans;
}

bool BallPerceptor2017::calculateBallOnField(
		BallPercept& ballPercept) const {
	const Vector2f correctedCenter = theImageCoordinateSystem.toCorrected(
			ballPercept.positionInImage);
	Vector3f cameraToBall(theCameraInfo.focalLength,
			theCameraInfo.opticalCenter.x() - correctedCenter.x(),
			theCameraInfo.opticalCenter.y() - correctedCenter.y());
	cameraToBall.normalize(
			theFieldDimensions.ballRadius * theCameraInfo.focalLength
					/ ballPercept.radiusInImage);
	Vector3f rotatedCameraToBall = theCameraMatrix.rotation * cameraToBall;
	const Vector3f sizeBasedCenterOnField = theCameraMatrix.translation
			+ rotatedCameraToBall;
	const Vector3f bearingBasedCenterOnField = theCameraMatrix.translation
			- rotatedCameraToBall
					* ((theCameraMatrix.translation.z()
							- theFieldDimensions.ballRadius)
							/ rotatedCameraToBall.z());

	if (rotatedCameraToBall.z() < 0) {
		ballPercept.positionOnField.x() = bearingBasedCenterOnField.x();
		ballPercept.positionOnField.y() = bearingBasedCenterOnField.y();
	} else {
		ballPercept.positionOnField.x() = sizeBasedCenterOnField.x();
		ballPercept.positionOnField.y() = sizeBasedCenterOnField.y();
	}
	return true;
}

bool BallPerceptor2017::checkBallOnField(BallPercept2017 ball) const {
//   Not sure about self-localization => Do not use it to check ball position
//  if(theRobotPose.validity < 1.f)
//  {
//    return true;
//  }
//   Check, if the computed ball position is still on the carpet
//  Pose2f currentRobotPose = theRobotPose + theOdometer.odometryOffset;
//  Vector2f absoluteBallPosition = (currentRobotPose * ball.relativePositionOnField);
//  return ((fabs(absoluteBallPosition.x()) < theFieldDimensions.xPosOpponentFieldBorder + 300.f) &&
//          (fabs(absoluteBallPosition.y()) < theFieldDimensions.yPosLeftFieldBorder + 300.f));
}

//Cascade Functions
bool BallPerceptor2017::checkBallSpot(const BallSpot& ballSpot) {
//   Calculate an approximation of the radius based on bearing distance of the ball spot
//  const Vector2i& spot = ballSpot.position;
//  Vector2f correctedStart = theImageCoordinateSystem.toCorrected(spot);
//  Vector3f cameraToStart(theCameraInfo.focalLength, theCameraInfo.opticalCenter.x() - correctedStart.x(), theCameraInfo.opticalCenter.y() - correctedStart.y());
//  Vector3f unscaledField = theCameraMatrix.rotation * cameraToStart;
//  if(unscaledField.z() >= 0.f)
//  {
//    return false;  Above horizon
//  }
//  const float scaleFactor = (theCameraMatrix.translation.z() - theFieldDimensions.ballRadius) / unscaledField.z();
//  cameraToStart *= scaleFactor;
//  unscaledField *= scaleFactor;
//  cameraToStart.y() += cameraToStart.y() > 0 ? -theFieldDimensions.ballRadius : theFieldDimensions.ballRadius;
//  cameraToStart /= scaleFactor;
//  approxRadius = std::abs(theImageCoordinateSystem.fromCorrectedApprox(Vector2i((theCameraInfo.opticalCenter.x() - cameraToStart.y()), (theCameraInfo.opticalCenter.y() - cameraToStart.z()))).x() - spot.x());
//  
//  return true;
}

bool BallPerceptor2017::checkRegionSizes(const BallSpot& ballSpot) {
	return ballSpot.shouldR * 2 / (2 * approxRadius) <= 1.8f
			&& ballSpot.shouldR * 2 / (2 * approxRadius) <= 1.8f;
}

bool BallPerceptor2017::searchEdgePoints(const BallSpot& ballSpot,
		const ColorRGBA& color) {
//  int currentIndex = 0;
//  int ballSpotYBoundary = theTJArkVision.FieldBorder[ballSpot.position.x()];
//  ballSpotYBoundary = std::max(0, 2);
//  validBallPoints = 0;
//  for (float angle = 0; angle <= pi * 2; angle += 0.4f)
//  {
//    int actualX = 0, actualY = 0;
//    int numberOfGreenPixels = 0;
//    BallPoint newBallPoint;
//    ballPoints[currentIndex] = newBallPoint;
//    for (float actualLength = 1; actualLength < approxRadius * 2 + 9; actualLength++)
//    {
//      actualX = ballSpot.position.x() + int(cos(angle) * actualLength);
//      actualY = ballSpot.position.y() + int(sin(angle) * actualLength);
//      if (actualX <= 0 || actualY <= ballSpotYBoundary || actualY >= imageHeigth || actualX >= imageWidth)
//        break;
//      const Image::Pixel pPixel = getPixel(actualY, actualX);
//      if (theColorTable[pPixel].is(ColorClasses::green))
//      {
//        numberOfGreenPixels++;
//        if (numberOfGreenPixels >= approxRadius / 4.f)
//        {
//          newBallPoint.isValid = true;
//          newBallPoint.point.x() = ballSpot.position.x() + int(cos(angle) * (actualLength - numberOfGreenPixels) + 0.5f);
//          newBallPoint.point.y() = ballSpot.position.y() + int(sin(angle) * (actualLength - numberOfGreenPixels) + 0.5f);
//          ballPoints[currentIndex] = newBallPoint;
//          validBallPoints++;
//          break;
//        }
//      }
//    }
//    
//    currentIndex++;
//  }
//  
//  if (validBallPoints <= 6)
//  {
//    return false;
//  }
//  
//  return true;
}

bool BallPerceptor2017::isBallFromPoints() {
//  float Mx = 0, My = 0, Mxx = 0, Myy = 0, Mxy = 0, Mz = 0, Mxz = 0, Myz = 0;
//  
//  for (const BallPoint* ballPoint = ballPoints, *end = ballPoints + sizeof(ballPoints) / sizeof(*ballPoints); ballPoint < end; ++ballPoint)
//    if (ballPoint->isValid)
//    {
//      float x = static_cast<float>(ballPoint->point.x());
//      float y = static_cast<float>(ballPoint->point.y());
//      float xx = x * x;
//      float yy = y * y;
//      float z = xx + yy;
//      Mx += x;
//      My += y;
//      Mxx += xx;
//      Myy += yy;
//      Mxy += x * y;
//      Mz += z;
//      Mxz += x * z;
//      Myz += y * z;
//    }
//  
//  // Construct and solve matrix
//  // Result will be center and radius of ball in theImage.
//  Matrix3f M =(Matrix3f() << Vector3f(Mxx, Mxy, Mx), Vector3f(Mxy, Myy, My),Vector3f(Mx, My, static_cast<float>(validBallPoints))).finished();
//  Vector3f v(-Mxz, -Myz, -Mz);
//  Vector3f BCD;
//  BCD = M.colPivHouseholderQr().solve(v);
//  
//  center.x() = BCD[0] * -0.5f;
//  center.y() = BCD[1] * -0.5f;
//  float radicand = BCD[0] * BCD[0] / 4.0f + BCD[1] * BCD[1] / 4.0f - BCD[2];
//  if (radicand <= 3.f)
//  {
//    return false;
//  }
//  radius = std::sqrt(radicand);
//  return true;
}

bool BallPerceptor2017::checkNewRadius() {
	return true;
//  return radius / approxRadius >= 0.6 && radius / approxRadius <= 1.8;
}

bool BallPerceptor2017::checkBallNear(BallPercept2017& ballPercept) {
//  for (const BallPercept& ball : balls) {
//    if ((center - ball.positionInImage).norm() < ball.radiusInImage) {
//      return false;
//    }
//  }
//  return true;
}

bool BallPerceptor2017::isRobotNear() {
//  float centerX = theCameraInfo.camera == CameraInfo::upper ? center.x() / 2 : center.x();
//  float centerY = theCameraInfo.camera == CameraInfo::upper ? center.y() / 2 : center.y();
//
//  for (const PlayersPercept::Player& player : thePlayersPercept.players)
//    {
//      if (centerX > player.x1FeetOnly && centerX < player.x2FeetOnly && centerY > player.y1
//          && centerY < player.y2)
//      {
//        return true;
//      }
//    }
	return false;
}

bool BallPerceptor2017::searchValidEdges() {
//  int validEdges = 0;
//  for (float angle = 0; angle <= pi * 2; angle += 0.4f)
//  {
//    unsigned char lastY = 0xff;
//    bool edge = false;
//    int actualX;
//    int actualY;
//    for (float actualLength = radius * 0.8f; actualLength < radius * 1.2f; actualLength++)
//    {
//      actualX = int(center.x() + cos(angle) * actualLength + 0.5f);
//      actualY = int(center.y() + sin(angle) * actualLength + 0.5f);
//      
//      if (actualX <= 0 || actualY <= 0 || actualY >= imageHeigth || actualX >= imageWidth)
//        break;
//      
//      const Image::Pixel pPixel = getPixel(actualY, actualX);
//      if (lastY != 0xff)
//      {
//        if (lastY != theColorTable[pPixel].colors)
//        {
//          edge = true;
//          validEdges++;
//          break;
//        }
//      }
//      
//      else if(theColorTable[pPixel].is(ColorClasses::green))
//        break;
//      
//      lastY = theColorTable[pPixel].colors;
//    }
//  }
//  
//  if (validEdges < 11)
//  {
//    return false;
//  }
//  
//  return true;
}

bool BallPerceptor2017::checkGreenInside() {
//  float numberOfGreenPixels = 0;
//  float totalPixels = 0;
//  int numOfTransitions = 0;
//  bool body = theCameraInfo.camera == CameraInfo::lower;
//  
//  for (float angle = 0; angle <= pi * 2; angle += 0.4f)
//  {
//    unsigned char lastY = 0xff;
//    for (float actualLength = 1; actualLength < radius - 1; actualLength++)
//    {
//      double actualX = center.x() + cos(angle) * actualLength;
//      double actualY = center.y() + sin(angle) * actualLength;
//      
//      if (actualX < 0 || actualY < 0 || actualY > imageHeigth || actualX > imageWidth)
//        break;
//      
//      if(body && ! theBodyContour.isValidPoint(Vector2i((int)actualX, (int)actualY)))
//        break;
//      
//      totalPixels++;
//      const Image::Pixel pPixel = getPixel(int(actualY + 0.5f) , int(actualX + 0.5f));
//      
//      if (theColorTable[pPixel].is(ColorClasses::green))
//      {
//        numberOfGreenPixels++;
//      }
//
//      else
//      {
//        if (lastY != 0xff)
//        {
//          if (lastY != theColorTable[pPixel].colors)
//          {
//            numOfTransitions++;
//          }
//        }
//        lastY = theColorTable[pPixel].colors;
//      }
//    }
//  }
//  
//  if (numOfTransitions >= 0.2f *radius)
//    ballValidity = true;
//  
//  if (numberOfGreenPixels/totalPixels < 0.2)
//    return true;
//  
//  return false;
}

bool BallPerceptor2017::checkPentagons() {
	regions.clear();
	float gap = radius + radius / 3.f;
	int step = int(gap / 10.f + 0.5f);
	for (int i = 0; i < 20; i++) {
		Vector2i init((int) (center.x() - gap + step * i),
				int(center.y() - gap));
		Vector2i end(int(center.x() - gap + step * i), int(center.y() + gap));
		scanLine(Vector2i(0, 2), init, end);
	}
	for (int i = 0; i < 20; i++) {
		Vector2i init((int) (center.x() - gap),
				int(center.y() - gap + step * i));
		Vector2i end(int(center.x() + gap), int(center.y() - gap + step * i));
		scanLine(Vector2i(2, 0), init, end);
	}
	mergeRegions();

	int numOfPentagons = createPentagons();

	if ((center - pentagonsCenter / float(numOfPentagons)).norm()
			> radius * 1.f)
		return false;

	for (int j = 0; j < numOfPentagons; j++) {
		if (pentagons[j].valid) {
			if ((center
					- Vector2f((pentagons[j].center.x()),
							(pentagons[j].center.y()))).norm() > radius) {
				return false;
			}

		}
	}

	if (numOfPentagons <= 1) {
		return false;
	}

	int numOfUnions = validatePentagons();
	if (numOfUnions < numOfPentagons - 1) {
		return false;
	}

	return true;
}

//Pentagon Functions
void BallPerceptor2017::scanLine(Vector2i step, Vector2i init, Vector2i end) {
	while (init.x() < 0 || init.x() > imageWidth || init.y() < 0
			|| init.y() > imageHeigth) {
		init += step;
		if (init.x() > end.x() || init.y() > end.y())
			return;
	}

	bool hasInit = false;
	Vector2i init2, end2, origin = init;
	bool horizontal = step.y() == 0;
	int thre1 = greenAround; //theFieldColor.seedY+5;
	int thre2 = greenAround + 30; //theFieldColor.seedY+30;
	if (theCameraInfo.camera == CameraInfo::lower) {
		thre1 = theFieldColor.seedY+20;
		thre2 = theFieldColor.seedY+60;
	}
	int last_color = 1;
	unsigned int tempPixel = theECImage.grayscaled[origin.y()][origin.x()];
	if (tempPixel < thre2 && origin.y() < center.y()) {
		last_color = 0;
	} else if (tempPixel < thre1 && origin.y() >= center.y()) {
		last_color = 0;
	} else if (tempPixel > thre2 && origin.y() < center.y()) {
		last_color = 1;
	} else if (tempPixel > thre1 && origin.y() >= center.y()) {
		last_color = 1;
	}
	if ((origin.x() - center.x()) * (origin.x() - center.x())
			+ (origin.y() - center.y()) * (origin.y() - center.y())
			> radius * radius) {
		last_color = 1;
	}
//	else {
//		if (theECImage.colored[origin.y()][origin.x()]
//				== FieldColors::Color::green)
//			last_color = 100;
//	}
	int lastY = theECImage.grayscaled[origin.y()][origin.x()];
	int comp = int(2.f * radius / 3.f + 0.5f);
	int min = int(radius / 5.f + 0.5f);
	while (origin.x() <= end.x() && origin.y() <= end.y()) {
		if (origin.x() < 0 || origin.x() > imageWidth || origin.y() < 0
				|| origin.y() > imageHeigth)
			return;
		int origin_color = 1;
		if (theECImage.grayscaled[origin.y()][origin.x()] < thre2
				&& origin.y() < center.y()) {
			origin_color = 0;
		} else if (theECImage.grayscaled[origin.y()][origin.x()] < thre1
				&& origin.y() >= center.y()) {
			origin_color = 0;
		} else if (theECImage.grayscaled[origin.y()][origin.x()] > thre2
				&& origin.y() < center.y()) {
			origin_color = 1;
		} else if (theECImage.grayscaled[origin.y()][origin.x()] > thre1
				&& origin.y() >= center.y()) {
			origin_color = 1;
		}
//
		if ((origin.x() - center.x()) * (origin.x() - center.x())
				+ (origin.y() - center.y()) * (origin.y() - center.y())
				> radius * radius) {
			origin_color = 1;
		}
//    else
//    {
//		if(theECImage.colored[origin.y()][origin.x()] == FieldColors::Color::green)
//			origin_color = 100;
//    }
		if (std::abs(last_color - origin_color) == 1) {
			if (last_color == 1) {
				hasInit = true;
				init2 = origin;
				DOT2("module:BallPerceptor2017:zdots", origin.x() * 2,
						origin.y() * 2, ColorRGBA::green, ColorRGBA::green);
			} else {
				end2 = origin;
				DOT2("module:BallPerceptor2017:zdots", origin.x() * 2,
						origin.y() * 2, ColorRGBA::red, ColorRGBA::red);
				if (hasInit
						&& ((horizontal && end2.x() - init2.x() < comp
								&& end2.x() - init2.x() > min)
								|| (!horizontal && end2.y() - init2.y() < comp
										&& end2.y() - init2.y() > min))) {
					LINE2("module:BallPerceptor:image", init2.x(), init2.y(),
							end2.x(), end2.y(), 1, Drawings::solidPen,
							ColorRGBA::violet);
					Region r = { horizontal, init2, end2, -1 };
					regions.push_back(r);
				}
				hasInit = false;
			}
		}
		last_color = origin_color;

		origin += step;
	}
}

void BallPerceptor2017::mergeRegions() {
	pentagons.clear();
	int index = 0;
	if (regions.size() < 2) {
		return;
	}
	for (std::vector<Region>::iterator i = regions.begin();
			i != regions.end() - 1; i++) {
		for (std::vector<Region>::iterator j = i + 1; j != regions.end(); j++) {
			if (i->areIntersected(*j)) {
				if (i->index == -1 && j->index == -1) {
					pentagons.push_back(Pentagon());
					i->index = index;
					j->index = index++;
				} else if (i->index == -1)
					i->index = j->index;
				else if (j->index == -1)
					j->index = i->index;
				else
					changeIndex(j->index, i->index);
			}
		}
	}
}

void BallPerceptor2017::changeIndex(int from, int to) {
	for (Region& region : regions) {
		if (region.index == from) {
			region.index = to;
		}
	}
}

int BallPerceptor2017::createPentagons() {
	for (auto& region : regions) {
		if (region.index < 0) {
			continue;
		}
		pentagons[region.index].addRegion(region);

	}
	int num = 0;
	pentagonsCenter = Vector2f();
	for (Pentagon& pentagon : pentagons) {
		if (pentagon.points == 0) {
			pentagon.valid = false;
			continue;
		}
		pentagon.setCenter();
		pentagon.isPentagon(radius * radius / 2.f, radius);
//    if(((pentagon.center.x() - center.x()) + (pentagon.center.x() - center.x())) > ((radius * 0.5) + (radius * 0.5)))
//    {
//    	pentagon.valid = false;
//    }
		if (pentagon.valid) {
			pentagonsCenter += Vector2f((pentagon.center.x()),
					(pentagon.center.y()));
			num++;
			LINE2("module:BallPerceptor2017:image", pentagon.width.x(),
					pentagon.center.y(), pentagon.width.y(),
					pentagon.center.y(), 1, Drawings::solidPen, ColorRGBA::red);
			LINE2("module:BallPerceptor2017:image", pentagon.center.x(),
					pentagon.heigth.x(), pentagon.center.x(),
					pentagon.heigth.y(), 1, Drawings::solidPen, ColorRGBA::red);
			CROSS2("module:BallPerceptor2017:image", pentagon.center.x(),
					pentagon.center.y(), 1, 1, Drawings::solidPen,
					ColorRGBA::blue);
		}
	}
	return num;
}

int BallPerceptor2017::validatePentagons() {
	int num = 0;
	for (std::vector<Pentagon>::iterator i = pentagons.begin();
			i != pentagons.end(); i++) {
		if (!i->valid) {
			continue;
		}
		for (std::vector<Pentagon>::iterator j = i; j != pentagons.end(); j++) {
			if (!j->valid) {
				continue;
			}
			if (i != j) {
				float dist = float((i->center - j->center).norm());
				if (dist < radius + radius * 1.f
						&& dist > radius - radius * 1.f) {
					num++;
					LINE2("module:BallPerceptor2017:image", i->center.x(),
							i->center.y(), j->center.x(), j->center.y(), 1,
							Drawings::solidPen, ColorRGBA::red);
				}
			}
		}
	}
	return num;
}

//Debug Functions
void BallPerceptor2017::drawBall(const Vector2f& pos) const {
	CIRCLE2("module:BallPerceptor2017:image", pos.x(), pos.y(), radius, 0.5,
			Drawings::solidPen, ColorRGBA::black, Drawings::solidPen,
			ColorRGBA(255, 128, 64, 100));
}

bool BallPerceptor2017::drawError(BallSpot ballSpot, std::string message) {
	CROSS2("module:BallPerceptor2017:image", ballSpot.position.x(),
			ballSpot.position.y(), 1, 1, Drawings::solidPen, ColorRGBA::black);
	DRAWTEXT2("module:BallPerceptor2017:image", ballSpot.position.x() + 3,
			ballSpot.position.y() + 2, 5, ColorRGBA::black, message);
	return false;
}

bool BallPerceptor2017::showRegionSizes(const BallSpot& ballSpot) {
//  float heightLength = ballSpot.shouldR;
//  float widthLength = ballSpot.shouldR;
//  drawError(ballSpot, "checkRegionSizes");
//  LINE2("module:BallPerceptor2017:image", ballSpot.position.x(), ballSpot.position.y(), ballSpot.position.x() + widthLength, ballSpot.position.y(), 1, Drawings::solidPen, ColorRGBA::violet);
//  LINE2("module:BallPerceptor2017:image", ballSpot.position.x(), ballSpot.position.y(), ballSpot.position.x() - widthLength, ballSpot.position.y(), 1, Drawings::solidPen, ColorRGBA::violet);
//  LINE2("module:BallPerceptor2017:image", ballSpot.position.x(), ballSpot.position.y(), ballSpot.position.x(), ballSpot.position.y() + heightLength, 1, Drawings::solidPen, ColorRGBA::violet);
//  LINE2("module:BallPerceptor2017:image", ballSpot.position.x(), ballSpot.position.y(), ballSpot.position.x(), ballSpot.position.y() - heightLength, 1, Drawings::solidPen, ColorRGBA::violet);
//  return false;
}

//Scale Functions
void BallPerceptorScaler2017::scaleInput() {
  typedef BallPerceptor2017Base B;
  theCameraInfo = B::theCameraInfo;
  theBallSpots = B::theBallSpots;

  //Do not copy "table"
  theImageCoordinateSystem.rotation = B::theImageCoordinateSystem.rotation;
  theImageCoordinateSystem.invRotation = B::theImageCoordinateSystem.invRotation;
  theImageCoordinateSystem.origin = B::theImageCoordinateSystem.origin;
  theImageCoordinateSystem.offset = B::theImageCoordinateSystem.offset;
  theImageCoordinateSystem.a = B::theImageCoordinateSystem.a;
  theImageCoordinateSystem.b = B::theImageCoordinateSystem.b;

//  if (theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
  if (1)
  {
    theCameraInfo.width *= 1;
    theCameraInfo.height *= 1;
    theCameraInfo.opticalCenter *= 1.f;
    theCameraInfo.focalLength *= 1.f;
    theCameraInfo.focalLengthInv /= 1.f;
    theCameraInfo.focalLenPow2 *= 1.f;

    theImageCoordinateSystem.origin *= 1.f;
    theImageCoordinateSystem.b *= 1.f;

  }
  theImageCoordinateSystem.setCameraInfo(theCameraInfo);
}

void BallPerceptorScaler2017::scaleOutput(BallPercept& ball) const {
//  if (theCameraInfo.camera == CameraInfo::upper && IS_FULL_SIZE)
//  {
//    ball.positionInImage *= 0.5f;
//    ball.radiusInImage *= 0.5f;
//  }
}

BallPerceptor2017::Ball BallPerceptor2017::fitBall(const Vector2f& center,
		const float searchR, Vector2f& ballColor) {
//  BallPerceptor2017::Ball pBall = newBall(5000, 5000, 0, false, 0);
//  float sr = 1.f*searchR;
//  int factor = theCameraInfo.camera == CameraInfo::upper? 2:1;
//  point_2d allPoints[NUM_STAR_SCANLINE_CANDIDATES];
//  float edge = searchEdges(theImage, point_2d(center.x(),center.y()), allPoints, ballColor, sr * factor);
//  float diameterGuess = guessDiameter(allPoints);
//
//
//  for (int i = 0; i < NUM_STAR_SCANLINE_CANDIDATES; ++i)
//  {
//    DOT("module:BallPerceptor2017:edgePoints", allPoints[i].x / factor, allPoints[i].y / factor,
//        ColorRGBA::yellow, ColorRGBA::yellow);
//  }
//  BallPerceptor2017::circle circle = ransacCircle(allPoints,
//                                              0.2f + diameterGuess * 0.05f);
//
//  if (circle.q < 12)
//    return pBall;
//  if (circle.r > 120 || circle.r < 1)
//    return pBall;
//  return newBall(circle.x, circle.y, circle.r, true, circle.q);

}

BallPerceptor2017::Ball BallPerceptor2017::newBall(float x, float y,
		float radius, bool found, float q) {
//  BallPerceptor2017::Ball b;
//  b.x = x;
//  b.y = y;
//  b.radius = radius;
//  b.found = found;
//  b.q = q;
//  return b;
}

BallPerceptor2017::circle BallPerceptor2017::newCircle(float x, float y,
		float r, float q) {
//  BallPerceptor2017::circle c;
//  c.x = x;
//  c.y = y;
//  c.r = r;
//  c.q = q;
//  return c;
}

//BallPerceptor2017::circle BallPerceptor2017::ransacCircle(
//    point_2d points[NUM_STAR_SCANLINE_CANDIDATES], float maxDistEdge)
//{
////  float max = 0;
////  BallPerceptor2017::circle bestCircle = newCircle(0, 0, 0, 0);
////  for (int i = 0; i < NUM_STAR_SCANLINE_CANDIDATES; i++)
////  {
////    //modell:
////    int p1 = i;
////    if (points[p1].x == -1)
////      continue;
////    int p2 = (i + NUM_STAR_SCANLINES / 4) % (NUM_STAR_SCANLINES * 2);
////    if (points[p2].x == -1)
////      continue;
////    int p3 = (i + NUM_STAR_SCANLINES / 2) % (NUM_STAR_SCANLINES * 2);
////    if (points[p3].x == -1)
////      continue;
////    BallPerceptor2017::circle c = getCircle(points[p1].x, points[p1].y,
////                                        points[p2].x, points[p2].y,
////                                        points[p3].x, points[p3].y);
////
////    //bewerten:
////    float sum = 0;
////    for (int j = 0; j < NUM_STAR_SCANLINE_CANDIDATES; j++)
////    {
////      float distx = (c.x - points[j].x);
////      float disty = (c.y - points[j].y);
////      float r = sqrtf(distx * distx + disty * disty);
////      float distR = fabsf(r - c.r);
////      if (distR < maxDistEdge)
////        sum++;
////    }
////    if (sum > max)
////    {
////      max = sum;
////      bestCircle = c;
////    }
////  }
////  bestCircle.q = max;
////  return bestCircle;
//}

BallPerceptor2017::circle BallPerceptor2017::getCircle(float x1, float y1,
		float x2, float y2, float x3, float y3) {
//  float f2 = x3 * x3 - x3 * x2 - x1 * x3 + x1 * x2 + y3 * y3 - y3 * y2 - y1 * y3
//      + y1 * y2;
//  float g2 = x3 * y1 - x3 * y2 + x1 * y2 - x1 * y3 + x2 * y3 - x2 * y1;
//  float m = 0;
//  if (g2 != 0)
//    m = (f2 / g2);
//
//  float c = (m * y2) - x2 - x1 - (m * y1);
//  float d = (m * x1) - y1 - y2 - (x2 * m);
//  float e = (x1 * x2) + (y1 * y2) - (m * x1 * y2) + (m * x2 * y1);
//
//  float x = (c / 2);
//  float y = (d / 2);
//  float s = (((x) * (x)) + ((y) * (y)) - e);
//  float r = pow(s, .5f);
//  return newCircle(-x, -y, r, 0);
}

//float BallPerceptor2017::searchEdges(const Image& img, point_2d pos,point_2d points[NUM_STAR_SCANLINES * 2],Vector2f& ballColor,float searchR)
//{
////  int edge = 0;
////  int skipMax = searchR/5;
////  float cosin[32] = {1.f, 0.98079f, 0.92388f, 0.83147f, 0.70711f, 0.55557f, 0.38268f, 0.19509f, 0.f, -0.19509f, -0.38268f, -0.55557f, -0.70711f, -0.83147f, -0.92388f, - 0.98079f, -1.f, -0.98079f, -0.92388f, -0.83147f, -0.70711f, -0.55557f, -0.38268f, -0.19509f, 0.f, 0.19509f, 0.38268f, 0.55557f, 0.70711f, 0.83147f, 0.92388f, 0.98079f};
////  float sin[32] = {0.f, 0.19509f, 0.38268f, 0.55557f, 0.70711f, 0.83147f, 0.92388f, 0.98079f, 1.f, 0.98079f, 0.92388f, 0.83147f, 0.70711f, 0.55557f, 0.38268f, 0.19509f, 0.f, -0.19509f, -0.38268f, -0.55557f, -0.70711f, -0.83147f, -0.92388f, - 0.98079f, -1.f, -0.98079f, -0.92388f, -0.83147f, -0.70711f, -0.55557f, -0.38268f, -0.19509f};
////  skipMax = skipMax<1?1:skipMax;
////  skipMax = skipMax>5?5:skipMax;
//////  for (float angle = 0; angle < pi * 2; angle += (pi / NUM_STAR_SCANLINES))
////  for(int i = 0; i < NUM_STAR_SCANLINES * 2; i++)
////  {
////	int skip = 0;
////    int actualX;
////    int actualY;
////    bool borderFound = false;
////    for (float actualLength = searchR * 0.5f; actualLength < searchR * 2.f; actualLength++)
////    {
//////      actualX = int(pos.x + cos(angle) * actualLength + 0.5f);
//////      actualY = int(pos.y + sin(angle) * actualLength + 0.5f);
////    	actualX = int(pos.x + cosin[i] * actualLength + 0.5f);
////    	actualY = int(pos.y + sin[i] * actualLength + 0.5f);
////
////      if (actualX <= 0 || actualY <= 0 || actualY >= theCameraInfo.height || actualX >= theCameraInfo.width)
////        break;
////
////      int meanH = (theTJArkVision.minH+theTJArkVision.maxH)>>1;
////      int meanS = (theTJArkVision.minS+theTJArkVision.maxS)>>1;
////      Image::Pixel ss;
////      theColorTable.getHSV(getPixel(actualY, actualX),ss);
////      int diffGreen = sqrt((ss.h-meanH)*(ss.h-meanH)+(ss.s-meanS)*(ss.s-meanS));
////      int diffBall = sqrt((ss.h-ballColor.x())*(ss.h-ballColor.x())+(ss.s-ballColor.y())*(ss.s-ballColor.y()));
////      if (diffBall<diffGreen)
//////      ColorTable::Colors pixelColor = determineColor(ss);
//////      if( pixelColor.is(ColorClasses::white) || pixelColor.is(ColorClasses::black))
//////      if(!(ss.h > theTJArkVision.minH -10 && ss.h < theTJArkVision.maxH + 10 && ss.s > theTJArkVision.minS -10 && ss.s < theTJArkVision.maxS + 10))
//////      if(ss.h < theTJArkVision.minH || ss.s <  100)
////      {
////    	  skip--;
////          if (skip<0)
////             skip = 0;
////          continue;
////      }
////      else
////      {
////          skip ++;
////          if (skip<=skipMax)
////            continue;
//////          actualX = int(pos.x + cos(angle) *(actualLength - skip) +0.5f);
//////          actualY = int(pos.y + sin(angle) *(actualLength - skip) +0.5f);
////          	actualX = int(pos.x + cosin[i] *(actualLength - skip) +0.5f);
////            actualY = int(pos.y + sin[i] *(actualLength - skip) +0.5f);
////      }
////      borderFound = true;
////      break;
////    }
////    if(borderFound)
////    {
////    	float actualR = Vector2f(actualX - pos.x, actualY - pos.y).norm();
////    	if(actualR > 0.5 * searchR && actualR < 2 * searchR)
////    		points[edge] = newPoint2D(actualX, actualY);
////    	else
////    		points[edge] = newPoint2D(-1, -1);
//////    	std::cout<<"edge--->"<<edge<<"--->"<<actualX<<","<<actualY<<std::endl;
////    }
////    else
////    {
////    	points[edge] = newPoint2D(-1, -1);
////    }
////    edge ++;
////  }
////
////  DRAWTEXT("module:BallPerceptor2017:var", pos.x, pos.y,10,ColorRGBA::magenta,"vaar="<<edge);
////
////  return edge;
//}

//float BallPerceptor2017::guessDiameter(point_2d points[NUM_STAR_SCANLINES])
//{
////  int sum = 0;
////  for (int i = 0; i < 8; i++)
////  {
////    int dx = points[i].x - points[i + 8].x;
////    int dy = points[i].y - points[i + 8].y;
////    int dist = dx * dx + dy * dy;
////    sum += dist;
////  }
////  return sqrtf(sum >> 3);
//}

bool BallPerceptor2017::classifyBalls2(BallPerceptor2017::Ball & b) {
	//FIXME calculate green pixels inside circle, using average Y of green outside the cicle as BW thre
	int meanY = 0;
	int meanBlack = 0;
	int meanWhite = 0;
	int blackCnt = 0;
	int whiteCnt = 0;
	int greenCnt = 0;
	int total = 0;
	int totalOutCircle = 0;
	int totalInCircle = 0;
	int varY = 0;
	float ratio = 0;
	float ratioGreen = 0;
	float ratioTotal = 0;
	float whitePercent = 0;
	float greenInCircle = 0;
	int greenCount = 0;
	int nonWhiteCount = 0;
//	int avgGreen = 0;
//	countAround(Vector2i(b.x, b.y), b.radius, greenCount, nonWhiteCount,
//			avgGreen);
//	greenAround = avgGreen;
//	float greenAroundRatio = 0.f;
//	greenAroundRatio = greenCount / 80.f;
	int thre1 = greenAround; //theFieldColor.seedY+5;
	int thre2 = greenAround + 30; //theFieldColor.seedY+30;
//	if (theCameraInfo.camera == CameraInfo::lower) {
//		thre1 = theFieldColor.seedY + 5;
//		thre2 = theFieldColor.seedY + 30;
//	}
	int step = 1;
	Vector2f cen(b.x, b.y);
	if (b.x < 2 || b.y < 2)
		return false;
	if (b.radius < 2)
		return false;
	ASSERT(b.radius > 1);
	ASSERT(b.x > 0 && b.y > 0);
  if(b.radius > 30)
  {
	  step = 2;
  }
  else if(b.radius > 60)
  {
	  step = 3;
  }
  else
  {
	  step = 1;
  }
	for (int x = std::max(0, b.x - b.radius);
			x < std::min(theECImage.colored.width, b.x + b.radius); x += step) {
		for (int y = std::max(0, b.y - b.radius);
				y < std::min(theECImage.colored.height, b.y + b.radius); y +=
						step) {

			Vector2f cur(x, y);
			if ((cur - cen).norm() >= b.radius) {
				totalOutCircle++;
				if (theECImage.colored[y][x] == FieldColors::Color::green)
					greenCnt++;
				continue;
			}
			total++;
			if (theECImage.colored[y][x] == FieldColors::Color::green) {
				greenInCircle++;
				continue;
			}
			DRAWTEXT("module:BallPerceptor2017:blackwhiteball", b.x, b.y - 40, 10,
							ColorRGBA::gray, "contrast"<<fabs(meanWhite - meanBlack)/meanWhite);
			totalInCircle++;
			meanY += theECImage.grayscaled[y][x];
			if (theECImage.grayscaled[y][x] < thre2 && y < std::max(0, b.y)) {
				DOT("module:BallPerceptor2017:blackwhiteball", cur.x(), cur.y(),
						ColorRGBA::black, ColorRGBA::black);
				blackCnt++;
				meanBlack += theECImage.grayscaled[y][x];
			} else if (theECImage.grayscaled[y][x] < thre1
					&& y >= std::max(0, b.y)) {
				DOT("module:BallPerceptor2017:blackwhiteball", cur.x(), cur.y(),
						ColorRGBA::black, ColorRGBA::black);
				blackCnt++;
				meanBlack += theECImage.grayscaled[y][x];
			} else if (theECImage.grayscaled[y][x] > thre2
					&& y < std::max(0, b.y)) {
				DOT("module:BallPerceptor2017:blackwhiteball", cur.x(), cur.y(),
						ColorRGBA::white, ColorRGBA::white);
				whiteCnt++;
				meanWhite += theECImage.grayscaled[y][x];
			} else if (theECImage.grayscaled[y][x] > thre1
					&& y >= std::max(0, b.y)) {
				DOT("module:BallPerceptor2017:blackwhiteball", cur.x(), cur.y(),
						ColorRGBA::white, ColorRGBA::white);
				whiteCnt++;
				meanWhite += theECImage.grayscaled[y][x];
			}
//			if(theECImage.colored[y][x] == FieldColors::Color::black)
//			{
//				DOT("module:BallPerceptor2017:blackwhiteball", cur.x(), cur.y(),
//						ColorRGBA::black, ColorRGBA::black);
//				blackCnt++;
//				meanBlack += theECImage.grayscaled[y][x];
//			}
//			else if(theECImage.colored[y][x] == FieldColors::Color::white)
//			{
//				DOT("module:BallPerceptor2017:blackwhiteball", cur.x(), cur.y(),
//						ColorRGBA::white, ColorRGBA::white);
//				whiteCnt++;
//				meanWhite += theECImage.grayscaled[y][x];
//			}
		}
	}

	if (totalInCircle == 0)
		return false;
	ASSERT(totalInCircle != 0);
	ASSERT(totalOutCircle != 0);
	ASSERT(total != 0);
	meanY /= totalInCircle;
	greenInCircle /= total;
	ratioGreen = float(greenCnt) / float(totalOutCircle);
	ratioTotal = float(totalInCircle) / float(total);
	if (blackCnt == 0 || whiteCnt == 0) {
		meanBlack = 0;
		meanWhite = 0;
		ratio = 0;
		return false;
	} else {
		meanBlack /= blackCnt;
		meanWhite /= whiteCnt;
		ratio = float(std::min(blackCnt, whiteCnt))
				/ float(std::max(blackCnt, whiteCnt));
		whitePercent = float(whiteCnt) / float(totalInCircle);
	}
	for (int x = std::max(0, b.x - b.radius);
			x < std::min(theECImage.colored.width, b.x + b.radius); x += step) {
		for (int y = std::max(0, b.y - b.radius);
				y < std::min(theECImage.colored.height, b.y + b.radius); y +=
						step) {
			if (theECImage.colored[y][x] == FieldColors::Color::green)
				continue;
			Vector2f cur(x, y);
			if ((cur - cen).norm() > b.radius)
				continue;
			varY += (theECImage.grayscaled[y][x] - meanY)
					* (theECImage.grayscaled[y][x] - meanY);
		}
	}
	varY /= totalInCircle;

	DRAWTEXT("module:BallPerceptor2017:colorCounter", b.x, b.y - 40, 10,
				ColorRGBA::gray, "contrast"<<fabs(meanWhite - meanBlack)/meanWhite);
	DRAWTEXT("module:BallPerceptor2017:colorCounter", b.x, b.y - 30, 10,
			ColorRGBA::gray, "ratioTotal"<<(ratioTotal));
	DRAWTEXT("module:BallPerceptor2017:colorCounter", b.x, b.y - 20, 10,
			ColorRGBA::cyan, "varY="<<varY);
	DRAWTEXT("module:BallPerceptor2017:colorCounter", b.x, b.y - 10, 10,
			ColorRGBA::orange, "greenInCircle="<<greenInCircle);
	DRAWTEXT("module:BallPerceptor2017:colorCounter", b.x, b.y - 0, 10,
			ColorRGBA::white, "ratio="<<ratio);
	DRAWTEXT("module:BallPerceptor2017:colorCounter", b.x, b.y + 20, 10,
			ColorRGBA::white, "whitePercent="<<whitePercent);
//	DRAWTEXT("module:BallPerceptor2017:colorCounter", b.x, b.y + 30, 10,
//			ColorRGBA::white, "greenAround="<<greenAroundRatio);

//  if ((meanWhite-meanBlack)<30)
//    return false;
//  if (varY<300)
//    return false;
//  if (ratio<0.09f)
//    return false;
//  if (whitePercent<0.5f)
//    return false;
//  if (ratioTotal<0.2f)
//    return false;

#ifdef GRASS_LAND
//  float Score = tansig(ratio,0.3f)*0.2f+tansig(ratioTotal,0.95f)*0.4f+tansig(ratioGreen,0.5f)*0.4f;//0625good
	float Score = tansig(ratio,0.3f)*0.2f+tansig(ratioTotal,0.95f)*0.4f+tansig(ratioGreen,0.5f)*0.4f;//0625good
#else
	float Score = tansig(ratio, 0.3f) * 0.2f + tansig(ratioTotal, 1.2f) * 0.4f
			+ tansig(ratioGreen, 0.5f) * 0.4f;
#endif
	int varYThre = (meanWhite - meanBlack) * 10 + 100; //originally 100
	if (theCameraInfo.camera == CameraInfo::lower) {
		varYThre += 200;
	}
	DRAWTEXT("module:BallPerceptor2017:colorCounter", b.x, b.y + 10, 10,
			ColorRGBA::gray, "Score="<<Score);
//  DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y-20, 10,
//              ColorRGBA::cyan, "Green="<<ratioGreen);
//  DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y-10, 10,
//              ColorRGBA::orange, "GrayP="<<ratioTotal);
//  DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y-0, 10,
//              ColorRGBA::white, "White="<<ratio<<"  Perce="<<whitePercent);
//  DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y+10, 10,
//              ColorRGBA::white, "MeanB="<<meanBlack-theTJArkVision.seedY<<"  MeanW="<<meanWhite);
//  DRAWTEXT("module:BallPerceptor:colorCounter", b.x, b.y+20, 10,
//              ColorRGBA::white, "MeanY="<<meanY<<"  DvarY="<<varY-varYThre);

// just for test
//   float th = otsuThreshold(b);
//  DRAWTEXT("module:BallPerceptor:thre", b.x, b.y+15, 10,
//              ColorRGBA::cyan, "thre="<<th);
//  if (varY<varYThre)
//    return false;
//    if ((meanWhite-meanBlack)<30)
//    return false;
	if (greenInCircle > 0.3)
		return false;
//	if (varY < 300)
//		return false;
	if ((theCameraInfo.camera == CameraInfo::upper && (whitePercent < 0.5f || whitePercent > 0.9)) || (theCameraInfo.camera == CameraInfo::lower && (whitePercent < 0.5f || whitePercent > 0.85)))
		return false;
	if ((Score < 0.75f && theCameraInfo.camera == CameraInfo::upper)
			|| (Score < 0.85f && theCameraInfo.camera == CameraInfo::lower))
		return false;

	if (ratioTotal < 0.75f) //0.68
		return false;
	if (ratioTotal < 0.68f)
		return false;
	if (Score < 0.75f)
		return false;
  if (ratio<0.2f)//originally 0.03
    return false;
//	if (greenAroundRatio < 0.6)
//		return false;

	return true;
}

float BallPerceptor2017::tansig(float per, float ref) {
	float x = (per) / ref;
	x = x < 0 ? -x : x;
	x = x > 1 ? 1 : x;
	return 1.f / (1 + exp(-4 * (2 * x - 1)));
}

int BallPerceptor2017::myThreshold1(int centerX, int centerY, int radius) {
	int Im_hist[256];
	float prob[66] = { 0 };
	float maxVar = 0;
	int thresh = 0;
//	for(int y = std::max(0,centerY - radius); y < std::min(centerY + radius, theCameraInfo.width); y++)
//	{
//		for(int x = std::max(0,centerX - radius); x < std::min(centerX + radius,theCameraInfo.height); x++)
//		{
//
//		}
//	}
//	if(ballVar < 30)
//	{
//		return thresh;
//	}
//	for(int i = 1; i < 66; i++)
//	{
//		prob[i] = static_cast<float>(Im_hist[i]) / static_cast<float>(pixelInCircle);
////		std::cout<<i<<"   "<<prob[i]<<std::endl;
//	}
//	for(int th = 191; th < 256; th++)
//	{
//		float w0 = 0;
//		float w1 = 0;
//		float av0 = 0;
//		float av1 = 0;
//		float u0 = 0;
//		float u1 = 0;
//		float var = 0;
//		for(int i = 1; i < 256 - 190; i++)
//		{
//			if(i + 190 <= th)
//			{
//				w0 += prob[i];
//				av0 += static_cast<float>(i + 190) * prob[i];
//			}
//			else
//			{
//				w1 += prob[i];
//				av1 += static_cast<float>(i + 190) * prob[i];
//			}
//		}
//		u0 = av0 / w0;
//		u1 = av1 / w1;
//		var = w0 * w1 *(u1 - u0) * (u1 - u0);
//		if(var > maxVar)
//		{
//			maxVar = var;
//			thresh = th;
//		}
//	}
//	return thresh;
}

void BallPerceptor2017::save_bw(Vector2f c, float radius) {
//	float sample[900];
//	float r = radius;
//	  Vector2f pos = c;
//	  int x = static_cast<int>(pos.x() / 2 - r);
//	  int y = static_cast<int>(pos.y() / 2 - r);
//	  int tempI, tempJ;
//	  int thre1 = theTJArkVision.seedY+5;
//	    int thre2 = theTJArkVision.seedY+30;
//	  float imageScale = r / 15;  // (2 * r) / 30, 30 is dest size
//	  float image[30][30];
//	  for (int i = 0; i != 30; ++i)
//	  {
//		for (int j = 0; j != 30; ++j)
//		{
//		  tempI = y + static_cast<int>(i * imageScale);
//		  tempJ = x + static_cast<int>(j * imageScale);
//		  if (tempI >= theImage.height)
//			tempI = theImage.height - 1;
//		  if (tempJ >= theImage.width)
//			tempJ = theImage.width - 1;
//		  if (tempI < 0)
//			tempI = 0;
//		  if (tempJ < 0)
//			tempJ = 0;
//		  if (getPixel(tempI,tempJ).y < thre2 && tempI < c.y())
//		{
//			  sample[i * 30 + j] =
//			  					(static_cast<double>(0) / 255);
//		}
//		else if(getPixel(tempI,tempJ).y < thre1 && tempI >= c.y())
//		{
//			sample[i * 30 + j] =
//						  					(static_cast<double>(0) / 255);
//		}
//		else if(getPixel(tempI,tempJ).y > thre2  &&tempI < c.y())
//		{
//			sample[i * 30 + j] =
//						  					(static_cast<double>(255) / 255);
//		}
//		else if(getPixel(tempI,tempJ).y > thre1  && tempI >= c.y())
//		{
//			sample[i * 30 + j] =
//									  					(static_cast<double>(255) / 255);
//		}
//
//		}
//	  }
//	  saveImageText(sample, 900);
//	  std::cout<<"sampleing"<<std::endl;
}
void BallPerceptor2017::countAround(const Vector2i& center, float radius,
		int& greenCount, int& nonWhiteCount, int& avgGreenY) const {
	greenCount = nonWhiteCount = 0;
	avgGreenY = 0;
	std::vector<float> greenCheckRadiusRatios;
	greenCheckRadiusRatios.push_back(1.5);
	greenCheckRadiusRatios.push_back(1.75);
	for (float radiusRatio : greenCheckRadiusRatios)
		countAtRadius(center, radius * radiusRatio, greenCount, nonWhiteCount,
				avgGreenY);
	if (greenCount != 0)
		avgGreenY /= greenCount;
	else
		avgGreenY = theFieldColor.seedY;
	DRAWTEXT("module:BallPerceptor2017:around", center.x(), center.y(), 10,
			ColorRGBA::yellow, "recommended threshold = " << avgGreenY);
}

void BallPerceptor2017::countAtRadius(const Vector2i& center, float radius,
		int& greenCount, int& nonWhiteCount, int& avgGreenY) const {
	std::function<bool(int, int)> isInside;
	if (center.x() - radius > 0
			&& center.x() + radius + 1 < theECImage.colored.width
			&& center.y() - radius > 0
			&& center.y() + radius + 1 < theECImage.colored.height)
		isInside = [&](int, int) -> bool {return true;};
	else
		isInside = [&](int x, int y) -> bool
		{
			return x >= 0 && x < theECImage.colored.width &&
			y >= 0 && y < theECImage.colored.height;
		};

	const int numberOfGreenChecks = 40;
	const float epsilon = pi2 / static_cast<float>(numberOfGreenChecks);
	float dx = -radius;
	float dy = 0.f;
	for (int i = 0; i < numberOfGreenChecks; ++i) {
		int x = center.x() + static_cast<int>(std::round(dx));
		int y = center.y() + static_cast<int>(std::round(dy));
		if (isInside(x, y)) {
			const FieldColors::Color color = theECImage.colored[y][x];
			if (color == FieldColors::green) {
				++greenCount;
				++nonWhiteCount;
				avgGreenY += theECImage.grayscaled[y][x];
				DOT("module:BallPerceptor2017:around", x, y, ColorRGBA::green,
						ColorRGBA::green);
			} else if (color != FieldColors::white) {
				++nonWhiteCount;
				DOT("module:B:around", x, y, ColorRGBA::red, ColorRGBA::red);
			} else
				DOT("module:BallPerceptor2017:around", x, y, ColorRGBA::yellow,
						ColorRGBA::yellow);
		}
		dx -= dy * epsilon;
		dy += dx * epsilon;
	}

}

//bool BallPerceptor2017::checkInnerGreen(Vector2i center, float r) const
//{
//
//}


bool BallPerceptor2017::classifyBalls3(BallPerceptor2017::Ball & b)
{
	int greenCnt = 0;
	int totalInCircle = 0;
	float greenInCircle = 0;
	int step = 1;
	int thre1 = greenAround; //theFieldColor.seedY+5;
	int thre2 = greenAround + 30; //theFieldColor.seedY+30;
	int meanY = 0;
	int blackCnt = 0;
	int meanBlack = 0;
	int whiteCnt = 0;
	int meanWhite = 0;
	int varY = 0;
	Vector2f cen(b.x, b.y);
	if (b.x < 2 || b.y < 2)
		return false;
	if (b.radius < 2)
		return false;
	ASSERT(b.radius > 1);
	ASSERT(b.x > 0 && b.y > 0);
	int total = 0;
	if(b.radius > 30)
	{
	  step = 2;
	}
	else if(b.radius > 60)
	{
	  step = 3;
	}
	else
	{
	  step = 1;
	}
	for (int x = std::max(0, b.x - b.radius);
			x < std::min(theECImage.colored.width, b.x + b.radius); x += step)
	{
		for (int y = std::max(0, b.y - b.radius);
				y < std::min(theECImage.colored.height, b.y + b.radius); y +=
						step) {

			Vector2f cur(x, y);
			if ((cur - cen).norm() >= b.radius) {
				continue;
			}
			total++;
			if (theECImage.colored[y][x] == FieldColors::Color::green) {
				greenInCircle++;
				continue;
			}
			totalInCircle++;
			meanY += theECImage.grayscaled[y][x];
			if (theECImage.grayscaled[y][x] < thre2 && y < std::max(0, b.y)) {
				DOT("module:BallPerceptor2017:blackwhiteball", cur.x(), cur.y(),
						ColorRGBA::black, ColorRGBA::black);
				blackCnt++;
				meanBlack += theECImage.grayscaled[y][x];
			} else if (theECImage.grayscaled[y][x] < thre1
					&& y >= std::max(0, b.y)) {
				DOT("module:BallPerceptor2017:blackwhiteball", cur.x(), cur.y(),
						ColorRGBA::black, ColorRGBA::black);
				blackCnt++;
				meanBlack += theECImage.grayscaled[y][x];
			} else if (theECImage.grayscaled[y][x] > thre2
					&& y < std::max(0, b.y)) {
				DOT("module:BallPerceptor2017:blackwhiteball", cur.x(), cur.y(),
						ColorRGBA::white, ColorRGBA::white);
				whiteCnt++;
				meanWhite += theECImage.grayscaled[y][x];
			} else if (theECImage.grayscaled[y][x] > thre1
					&& y >= std::max(0, b.y)) {
				DOT("module:BallPerceptor2017:blackwhiteball", cur.x(), cur.y(),
						ColorRGBA::white, ColorRGBA::white);
				whiteCnt++;
				meanWhite += theECImage.grayscaled[y][x];
			}
			}
		}
	if(totalInCircle == 0 || total == 0)
		return false;
	meanY /= totalInCircle;
	for (int x = std::max(0, b.x - b.radius);
			x < std::min(theECImage.colored.width, b.x + b.radius); x += step) {
		for (int y = std::max(0, b.y - b.radius);
				y < std::min(theECImage.colored.height, b.y + b.radius); y +=
						step) {
			if (theECImage.colored[y][x] == FieldColors::Color::green)
				continue;
			Vector2f cur(x, y);
			if ((cur - cen).norm() > b.radius)
				continue;
			varY += (theECImage.grayscaled[y][x] - meanY)
					* (theECImage.grayscaled[y][x] - meanY);
		}
	}
	varY /= totalInCircle;
	greenInCircle /= total;
	DRAWTEXT("module:BallPerceptor2017:colorCounter", b.x, b.y - 10, 10,
				ColorRGBA::orange, "greenInCircle="<<greenInCircle);
	DRAWTEXT("module:BallPerceptor2017:colorCounter", b.x, b.y - 20, 10,
					ColorRGBA::orange, "varY="<<varY);
	if(greenInCircle > 0.08)
		return false;
	if(varY <= 800)
		return false;
	return true;
}













