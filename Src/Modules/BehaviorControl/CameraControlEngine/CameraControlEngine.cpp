/**
 * @file CameraControlEngine.cpp
 * @author Andreas Stolpmann
 * (Based on an old version by Felix Wenk)
 */

#include "CameraControlEngine.h"
#include "Tools/Motion/InverseKinematic.h"
#include <iostream>

MAKE_MODULE(CameraControlEngine, behaviorControl);

CameraControlEngine::CameraControlEngine()
{
  panBounds.min = theHeadLimits.minPan();
  panBounds.max = theHeadLimits.maxPan();
}

void CameraControlEngine::update(HeadAngleRequest& headAngleRequest)
{
  Vector2a panTiltUpperCam;
  Vector2a panTiltLowerCam;

  if(theHeadMotionRequest.mode == HeadMotionRequest::panTiltMode)
  {
    panTiltUpperCam.x() = theHeadMotionRequest.pan;
    panTiltUpperCam.y() = theHeadMotionRequest.tilt + theRobotDimensions.getTiltNeckToCamera(false);
    panTiltLowerCam.x() = theHeadMotionRequest.pan;
    panTiltLowerCam.y() = theHeadMotionRequest.tilt + theRobotDimensions.getTiltNeckToCamera(true);
  }
  else
  {
    Vector3f hip2Target;
    if(theHeadMotionRequest.mode == HeadMotionRequest::targetMode)
      hip2Target = theHeadMotionRequest.target;
    else
      hip2Target = theTorsoMatrix.inverse() * theHeadMotionRequest.target;

    calculatePanTiltAngles(hip2Target, false, panTiltUpperCam);
    calculatePanTiltAngles(hip2Target, true, panTiltLowerCam);

//    if (theHeadMotionRequest.mode == HeadMotionRequest::targetOnGroundMode)
//    {
//    	headAngleRequest.tilt -= 30_deg;
//    }
//    if(theHeadMotionRequest.cameraControlMode == HeadMotionRequest::autoCamera)
//      panTiltLowerCam.y() = defaultTilt;
  }

  if(panTiltUpperCam.x() < panBounds.min)
  {
    panTiltUpperCam.x() = panBounds.min;
    panTiltLowerCam.x() = panBounds.min;
  }
  else if(panTiltUpperCam.x() > panBounds.max)
  {
    panTiltUpperCam.x() = panBounds.max;
    panTiltLowerCam.x() = panBounds.max;
  }

  Rangea tiltBoundUpperCam = theHeadLimits.getTiltBound(panTiltUpperCam.x());
  Rangea tiltBoundLowerCam = theHeadLimits.getTiltBound(panTiltLowerCam.x());

  adjustTiltBoundToShoulder(panTiltUpperCam.x(), false, tiltBoundUpperCam);
  adjustTiltBoundToShoulder(panTiltLowerCam.x(), true, tiltBoundLowerCam);

  bool lowerCam = false;
  headAngleRequest.pan = panTiltUpperCam.x(); // Pan is the same for both cams

  if(theHeadMotionRequest.cameraControlMode == HeadMotionRequest::upperCamera)
  {
    headAngleRequest.tilt = panTiltUpperCam.y();
    lowerCam = false;
  }
  else if(theHeadMotionRequest.cameraControlMode == HeadMotionRequest::lowerCamera)
  {
    headAngleRequest.tilt = panTiltLowerCam.y();
    lowerCam = true;
  }
  else
  {
//	  std::cout<<"pantilt outside---->"<<toDegrees(panTiltUpperCam.y())<<","<<toDegrees(panTiltLowerCam.y())<<std::endl;
    if(theHeadMotionRequest.mode != HeadMotionRequest::panTiltMode)
    {
//    	std::cout<<"tiltBoundLowerCam.max---->"<<toDegrees(tiltBoundLowerCam.max)<<","<<tiltBoundLowerCam.min<<std::endl;
//      if(panTiltLowerCam.y() < tiltBoundLowerCam.min)
//      {
//        headAngleRequest.tilt = panTiltUpperCam.y();
//        lowerCam = false;
//      }
//      else if(panTiltUpperCam.y() < moveHeadThreshold)
//      {
//    	  std::cout<<"moveHeadThreshold"<<toDegrees(moveHeadThreshold)<<std::endl;
//        headAngleRequest.tilt = defaultTilt - std::abs(moveHeadThreshold - panTiltUpperCam.y());
//        lowerCam = false;
//      }
//      else
//      {
//        headAngleRequest.tilt = panTiltLowerCam.y();
//        lowerCam = true;
//      }
    	const float ballDistance = sqrt(sqr(theHeadMotionRequest.target.x()) + sqr(theHeadMotionRequest.target.y()));
    	if(ballDistance > 700.f)
    	{
    		headAngleRequest.tilt = panTiltUpperCam.y();
    		lowerCam = false;
    	}
    	else if(ballDistance < 600.f)
    	{
    		headAngleRequest.tilt = panTiltLowerCam.y();
    		lowerCam = true;
    	}
    	else
    	{
    		headAngleRequest.tilt = lastUseLowerCamera? panTiltLowerCam.y():panTiltUpperCam.y();
    		lowerCam = lastUseLowerCamera;
    	}
    	lastUseLowerCamera = lowerCam;
    }
    else
    {
      headAngleRequest.tilt = panTiltUpperCam.y();
      lowerCam = true;
    }
  }

  if(lowerCam)
    headAngleRequest.tilt = tiltBoundLowerCam.limit(headAngleRequest.tilt);
  else
    headAngleRequest.tilt = tiltBoundUpperCam.limit(headAngleRequest.tilt);

  if(theHeadMotionRequest.mode == HeadMotionRequest::panTiltMode)
  {
    if(theHeadMotionRequest.tilt == JointAngles::off)
      headAngleRequest.tilt = JointAngles::off;
    if(theHeadMotionRequest.pan == JointAngles::off)
      headAngleRequest.pan = JointAngles::off;
  }
//  std::cout<<"headAngletilt----->"<<toDegrees(headAngleRequest.tilt)<<std::endl;
  headAngleRequest.speed = theHeadMotionRequest.speed;
  headAngleRequest.stopAndGoMode = theHeadMotionRequest.stopAndGoMode;
}

void CameraControlEngine::calculatePanTiltAngles(const Vector3f& hip2Target, bool lowerCamera, Vector2a& panTilt) const
{
  InverseKinematic::calcHeadJoints(hip2Target, pi_2, theRobotDimensions, lowerCamera, panTilt, theCameraCalibration);
}

void CameraControlEngine::adjustTiltBoundToShoulder(const Angle pan, const bool lowerCamera, Range<Angle>& tiltBound) const
{
  Limbs::Limb shoulder = pan > 0_deg ? Limbs::shoulderLeft : Limbs::shoulderRight;
  const Vector3f& shoulderVector = theRobotModel.limbs[shoulder].translation;
  RobotCameraMatrix rcm(theRobotDimensions, pan, 0.0f, theCameraCalibration, !lowerCamera);
  Vector3f intersection = Vector3f::Zero();
  if(theHeadLimits.intersectionWithShoulderEdge(rcm, shoulderVector, intersection))
  {
    Vector2a intersectionPanTilt;
    calculatePanTiltAngles(intersection, lowerCamera, intersectionPanTilt);
    if(intersectionPanTilt.y() < tiltBound.max) // if(tilt smaller than upper bound)
      tiltBound.max = intersectionPanTilt.y();
  }
}
