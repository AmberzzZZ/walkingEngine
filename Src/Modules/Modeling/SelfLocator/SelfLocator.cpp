/**
 * @file SelfLocator.cpp
 *
 * Implements a class that performs self-localization
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */
 
#include "SelfLocator.h"
#include "Platform/SystemCall.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Probabilistics.h"
#include "Tools/Math/Eigen.h"
#include <algorithm>
#include <iostream>

using namespace std;

//#define SHOW_TIME

SelfLocator::SelfLocator() : perceptRegistration(theCameraMatrix, theCirclePercept, theFieldDimensions, theFrameInfo,
      theGoalFeature, theGoalFrame,theGoalPercept2017, theFieldLineIntersections,
      theFieldLines, theMidCircle, theMidCorner, theMotionInfo, theOuterCorner,
      theOwnSideModel, thePenaltyArea,
      thePenaltyMarkPercept, goalAssociationMaxAngle,
      goalAssociationMaxAngularDistance, goalFrameIsPerceivedAsLines,
      lineAssociationCorridor, centerCircleAssociationDistance,
      penaltyMarkAssociationDistance, intersectionAssociationDistance,
      globalPoseAssociationDistance),
  lastTimeFarFieldBorderSeen(0), lastTimeJumpSound(0), timeOfLastReturnFromPenalty(0),
  nextSampleNumber(0), idOfLastBestSample(-1), averageWeighting(0.5f), lastAlternativePoseTimestamp(0)
{
  //              //
  //  2        5  //
  //  3        4  //
  //  1           //
  //     Goal     //
  walkInPositions.clear();
  Vector2f pos1(-3000.f, theFieldDimensions.yPosLeftSideline);
  Vector2f pos2(-1000.f, theFieldDimensions.yPosLeftSideline);
  Vector2f pos3(-2000.f, theFieldDimensions.yPosLeftSideline);
  Vector2f pos4(-2500.f, theFieldDimensions.yPosRightSideline);
  Vector2f pos5(-1500.f, theFieldDimensions.yPosRightSideline);
  Vector2f centerOfGoal(theFieldDimensions.xPosOwnGroundline, 0.f);
  Vector2f pos1ToGoal = centerOfGoal - pos1;
//  Vector2f pos2ToGoal = centerOfGoal - pos2;
  Vector2f pos3ToGoal = centerOfGoal - pos3;
  Vector2f pos4ToGoal = centerOfGoal - pos4;
//  Vector2f pos5ToGoal = centerOfGoal - pos5;
  float angle1 = std::atan2(pos1ToGoal.y(), pos1ToGoal.x());
//  float angle2 = std::atan2(pos2ToGoal.y(), pos2ToGoal.x());
  float angle3 = std::atan2(pos3ToGoal.y(), pos3ToGoal.x());
  float angle4 = std::atan2(pos4ToGoal.y(), pos4ToGoal.x());
//  float angle5 = std::atan2(pos5ToGoal.y(), pos5ToGoal.x());
  Vector2f centerOfCircle(0.f, 0.f);
  Vector2f pos1ToCircle = centerOfCircle - pos1;
  Vector2f pos2ToCircle = centerOfCircle - pos2;
//  Vector2f pos3ToCircle = centerOfCircle - pos3;
  Vector2f pos4ToCircle = centerOfCircle - pos4;
  Vector2f pos5ToCircle = centerOfCircle - pos5;
//  float angle1 = std::atan2(pos1ToCircle.y(), pos1ToCircle.x());
  float angle2 = std::atan2(pos2ToCircle.y(), pos2ToCircle.x());
//  float angle3 = std::atan2(pos3ToCircle.y(), pos3ToCircle.x());
//  float angle4 = std::atan2(pos4ToCircle.y(), pos4ToCircle.x());
  float angle5 = std::atan2(pos5ToCircle.y(), pos5ToCircle.x());
  walkInPositions.push_back(Pose2f(angle1, pos1)); // Robot 1
  walkInPositions.push_back(Pose2f(angle2, pos2)); // Robot 2
  walkInPositions.push_back(Pose2f(angle3, pos3)); // Robot 3
  walkInPositions.push_back(Pose2f(angle4, pos4)); // Robot 4
  walkInPositions.push_back(Pose2f(angle5, pos5)); // Robot 5


  nextWalkInPoseNumber = 0;
  nextReturnFromPenaltyIsLeft = true;

  // Create sample set with samples at the typical walk-in positions
  samples = new SampleSet<UKFSample>(numberOfSamples);
  for(int i = 0; i < samples->size(); ++i)
    samples->at(i).init(getNewPoseAtWalkInPosition(), defaultPoseDeviation, nextSampleNumber++, 0.5f);
  penaltyPositions.clear();
  penaltyPositions.push_back(Pose2f(-pi_2, -2000.f, 3000.f));
  penaltyPositions.push_back(Pose2f(-pi_2, -2750.f, 3000.f));
  penaltyPositions.push_back(Pose2f(-pi_2, -3500.f, 3000.f));
  penaltyPositions.push_back(Pose2f(pi_2, -2000.f, -3000.f));
  penaltyPositions.push_back(Pose2f(pi_2, -2750.f, -3000.f));
  penaltyPositions.push_back(Pose2f(pi_2, -3500.f, -3000.f));
}

SelfLocator::~SelfLocator()
{
  delete samples;
}

void SelfLocator::update(RobotPose& robotPose)
{
#ifndef NDEBUG
  ASSERT(!std::isnan(theOdometer.distanceWalked));
  ASSERT(!std::isnan(static_cast<float>(theOdometer.odometryOffset.rotation)));
  ASSERT(!std::isnan(theOdometer.odometryOffset.translation.x()));
  ASSERT(!std::isnan(theOdometer.odometryOffset.translation.y()));
  ASSERT(!std::isnan(theOwnSideModel.largestXPossible));
  for(const FieldLines::Line& l : theFieldLines.lines)
  {
    ASSERT(!std::isnan(l.alpha));
    ASSERT(!std::isnan(l.d));
    ASSERT(!std::isnan(l.first.x()));
    ASSERT(!std::isnan(l.last.y()));
  }
  if(theFrameInfo.getTimeSince(theCirclePercept.lastSeen) < 10)
  {
    ASSERT(!std::isnan(theCirclePercept.pos.x()));
    ASSERT(!std::isnan(theCirclePercept.pos.y()));
  }
  if(thePenaltyMarkPercept.timeLastSeen == theFrameInfo.time && theFrameInfo.time != 0)
  {
    ASSERT(!std::isnan(thePenaltyMarkPercept.positionOnField.x()));
    ASSERT(!std::isnan(thePenaltyMarkPercept.positionOnField.y()));
  }
  if(theGoalPercept2017.timeWhenLastSeen == theFrameInfo.time && theFrameInfo.time != 0)
  {
	  for(const auto GoalPost:theGoalPercept2017.goalPosts)
	  {
		  ASSERT(!std::isnan(GoalPost.locationOnField.x()));
		  ASSERT(!std::isnan(GoalPost.locationOnField.y()));
	  }
  }

  if(theFieldBoundary.isValid)
  {
    for(const Vector2f& v : theFieldBoundary.boundaryOnField)
    {
      ASSERT(!std::isnan(v.x()));
      ASSERT(!std::isnan(v.y()));
    }
  }
#endif

  /* Initialize variable(s) */
  sampleSetHasBeenResetted = false;
  /*if(theMotionRequest.motion == MotionRequest::specialAction &&
     (theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::keeperJumpLeftBack ||
      theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::keeperJumpLeftSim))
  {
    lastTimeKeeperJumped = static_cast<int>(theFrameInfo.time);
  }*/
  const Pose3f invCameraMatrix = theCameraMatrix.inverse();
  for(int i = 0; i < numberOfSamples; ++i)
    samples->at(i).prepare(robotRotationDeviationInStand, robotRotationDeviation, theMotionInfo, invCameraMatrix);

  Pose2f propagatedRobotPose = robotPose + theOdometer.odometryOffset;
  lastRobotPose = lastRobotPose + theOdometer.odometryOffset;

  /* Modify sample set according to certain changes of the game state:
   *  - Reset mirror in SET
   *  - Handling of penalty positions
   *  - ...
   */
  handleGameStateChanges(propagatedRobotPose);

  /* Move all samples according to the current odometry.
   */
  STOPWATCH("SelfLocator:motionUpdate")
  {
    motionUpdate();
  }
  lastTimeStampe = high_resolution_clock::now();
  /* Integrate perceptions
   *  - goal posts, lines, corners ...
   * and compute validity of each sample
   */
  STOPWATCH("SelfLocator:sensorUpdate")
  {
    sensorUpdate();
  }

  /* Check and plot current weightings */
  float minWeighting = 2.f;
  float maxWeighting = -1.f;
  float weightingSum = 0.f;
  for(int i = 0; i < numberOfSamples; ++i)
  {
    samples->at(i).computeWeightingBasedOnValidity(baseValidityWeighting);
    const float w = samples->at(i).weighting;
    weightingSum += w;
    if(w > maxWeighting)
      maxWeighting = w;
    if(w < minWeighting)
      minWeighting = w;
  }
  averageWeighting = weightingSum / numberOfSamples;
  PLOT("module:SelfLocator:minWeighting", minWeighting);
  PLOT("module:SelfLocator:maxWeighting", maxWeighting);
  PLOT("module:SelfLocator:averageWeighting", averageWeighting);

  /* Handle mirror information from SideConfidence */
  handleSideConfidence();

  /* Detect mirrored samples and put them back
   * - IS THIS USEFUL ???!?!?
   */
  //if(theGameInfo.state == STATE_PLAYING && theGameInfo.secondaryState != STATE2_PENALTYSHOOT &&
  //  !theSideConfidence.mirror && !sampleSetHasBeenResetted)
  //{
  //  for(int i = 0; i < numberOfSamples; ++i)
  //  {
  //    const Pose2f samplePose =  samples->at(i).getPose();
  //    if(isMirrorCloser(samplePose, robotPose))
  //      samples->at(i).mirror();
  //  }
  //}

  /* Actually: Hacks and workaround :-)
   *  - Treat keeper rotation problem
   */
//  domainSpecificSituationHandling();

  /* Particle filter resampling step
   *  - not executed during penalty shootout!
   */
  if(theGameInfo.secondaryState != STATE2_PENALTYSHOOT)
    resampling();

  /* Fill the RobotPose representation based on the current sample set
   */
  computeModel(robotPose);

  /* Replace a sample, if necessary
   *   This step is done at the end to make sure that the new sample
   *   gets the full motion and sensor update steps before being involved
   *   in any pose computations.
   */
  if(sensorResetting(robotPose))
  {
    PLOT("module:SelfLocator:sampleResetting", 6.f);
  }
  else
  {
    PLOT("module:SelfLocator:sampleResetting", 0.f);
  }

  /* Finally, update internal variables, debug and draw stuff.
   */
  DEBUG_RESPONSE("module:SelfLocator:templates_only")
  {
    if(theAlternativeRobotPoseHypothesis.isValid)
    {
      for(int i = 0; i < numberOfSamples; ++i)
      {
        UKFSample newSample;
        if(theOwnSideModel.stillInOwnSide)
          newSample.init(getNewPoseBasedOnObservations(true, lastRobotPose), defaultPoseDeviation, nextSampleNumber++, 0.5f);
        else
          newSample.init(getNewPoseBasedOnObservations(false, lastRobotPose), defaultPoseDeviation, nextSampleNumber++, 0.5f);
        samples->at(i) = newSample;
      }
    }
  }

  if((lastRobotPose.translation - robotPose.translation).norm() > positionJumpNotificationDistance &&
     theGameInfo.state == STATE_PLAYING && !theSideConfidence.mirror)
  {
    if(theFrameInfo.getTimeSince(lastTimeJumpSound) > 1337)
    {
      SystemCall::playSound("jump.wav");
      lastTimeJumpSound = theFrameInfo.time;
    }
    ANNOTATION("SelfLocator", "Robot position has jumped!");
  }

  lastRobotPose = robotPose;
  MODIFY("representation:RobotPose", robotPose);

  draw(robotPose);
  DEBUG_DRAWING("origin:Odometry", "drawingOnField")
  {
    Pose2f origin = robotPose + theOdometryData.inverse();
    ORIGIN("origin:Odometry", origin.translation.x(), origin.translation.y(), origin.rotation);
  }
}

void SelfLocator::update(SelfLocalizationHypotheses& selfLocalizationHypotheses)
{
  selfLocalizationHypotheses.hypotheses.resize(numberOfSamples);
  for(int i = 0; i < numberOfSamples; ++i)
  {
    SelfLocalizationHypotheses::Hypothesis& h = selfLocalizationHypotheses.hypotheses[i];
    h.pose = samples->at(i).getPose();
    Matrix3f cov = samples->at(i).getCov();
    h.xVariance = cov(0, 0);
    h.yVariance = cov(1, 1);
    h.xyCovariance = cov(1, 0);
    h.rotVariance = cov(2, 2);
  }
}

void SelfLocator::computeModel(RobotPose& robotPose)
{
  UKFSample& result = getMostValidSample();
  Pose2f resultPose = result.getPose();
  // Override side information for testing on one side of a field only
  if(alwaysAssumeOpponentHalf && resultPose.translation.x() < 0)
  {
    resultPose = Pose2f(pi) + resultPose;
  }
  robotPose = resultPose;
  Matrix3f cov = result.getCov();
  robotPose.deviation = sqrt(std::max(cov(0, 0), cov(1, 1)));
  robotPose.validity = result.validity;
  robotPose.covariance = cov;
  if(robotPose.validity >= validityThreshold)
    robotPose.validity = 1.f;
  else
    robotPose.validity *= (1.f / validityThreshold);
  if(theMidCircle.isValid || theMidCorner.isValid || theGoalFrame.isValid || theOuterCorner.isValid ||
     thePenaltyArea.isValid || theGoalFeature.isValid)
  {
    robotPose.timeOfLastConsideredFieldFeature = theFrameInfo.time;
  }
  idOfLastBestSample = result.id;
}

void SelfLocator::motionUpdate()
{
  // This is a nasty workaround but should help us in cases of bad/slow assistant referees:
  // If the robot returns from a penalty and is not walking, any rotation (that might come from
  // the z-axis gyro as the robot is turned too late!) is ignored!
  float odometryRotation = theOdometer.odometryOffset.rotation;
  if(theFrameInfo.getTimeSince(timeOfLastReturnFromPenalty) < 10000 && theMotionInfo.motion != MotionRequest::walk)
    odometryRotation = 0.f;

  const float transX = theOdometer.odometryOffset.translation.x();
  const float transY = theOdometer.odometryOffset.translation.y();
  const float dist = theOdometer.odometryOffset.translation.norm();
  const float angle = abs(odometryRotation);
  const float angleWeightNoise = theMotionInfo.motion == MotionRequest::walk ? movedAngleWeightRotationNoise : movedAngleWeightRotationNoiseNotWalking;

  // precalculate rotational error that has to be adapted to all samples
  const float rotError = max(minRotationNoise, max(dist * movedDistWeightRotationNoise, angle * angleWeightNoise));

  // precalculate translational error that has to be adapted to all samples
  const float transXError = max(minTranslationNoise,
                                max(abs(transX * majorDirTransWeight),
                                    abs(transY * minorDirTransWeight)));
  const float transYError = max(minTranslationNoise,
                                max(abs(transY * majorDirTransWeight),
                                    abs(transX * minorDirTransWeight)));

  // update samples
  for(int i = 0; i < numberOfSamples; ++i)
  {
    const Vector2f transOffset((transX - transXError) + (2 * transXError) * Random::uniform(),
                               (transY - transYError) + (2 * transYError) * Random::uniform());
    const float rotationOffset = odometryRotation + Random::uniform(-rotError, rotError);

    samples->at(i).motionUpdate(Pose2f(rotationOffset, transOffset), filterProcessDeviation, odometryDeviation, odometryRotationDeviation);
  }

#ifdef SHOW_TIME
    std::chrono::high_resolution_clock::time_point nowT = high_resolution_clock::now();
    duration<double> dTotal = duration_cast<duration<double>>(nowT - lastTimeStampe);
  	std::cout << "selfLocator=" << dTotal.count() * 1000 << "ms" << std::endl;
#endif
}

void SelfLocator::sensorUpdate()
{
  // Do not update during certain motions as the percepts might have a low quality
  if(!(theMotionRequest.motion == MotionRequest::walk || theMotionInfo.isStanding()))
    return;
  // In the penalty shootout, the goalkeeper should not perform any real localization
  if((theGameInfo.secondaryState == STATE2_PENALTYSHOOT) && (Global::getSettings().isGoalkeeper))
    return;

  // Integrate Landmarks and lines
  bool useLines = true;
  bool useLandmarks = true;
  bool usePoses = true;
  MODIFY("module:SelfLocator:useLines", useLines);
  MODIFY("module:SelfLocator:useLandmarks", useLandmarks);
  MODIFY("module:SelfLocator:usePoses", usePoses);
  for(int i = 0; i < numberOfSamples; ++i)
  {
    const Pose2f samplePose = samples->at(i).getPose();
    perceptRegistration.update(samplePose, registeredPercepts, robotRotationDeviationInStand, robotRotationDeviation);
    if(usePoses)
      for(const auto& pose : registeredPercepts.poses)
        samples->at(i).updateByPose(pose, theCameraMatrix, theFieldDimensions);
    if(useLandmarks)
      for(const auto& landmark : registeredPercepts.landmarks)
        samples->at(i).updateByLandmark(landmark);
    if(useLines)
      for(const auto& line : registeredPercepts.lines)
        samples->at(i).updateByLine(line);
    float numerator = 0.f;
    float denominator = 0.f;
    if(registeredPercepts.totalNumberOfPerceivedLines && useLines)
    {
      numerator = static_cast<float>(registeredPercepts.lines.size()) / static_cast<float>(registeredPercepts.totalNumberOfPerceivedLines);
      denominator = 1.f;
    }
    if(registeredPercepts.totalNumberOfPerceivedLandmarks && useLandmarks)
    {
      numerator += 3.f * (static_cast<float>(registeredPercepts.landmarks.size()) / static_cast<float>(registeredPercepts.totalNumberOfPerceivedLandmarks));
      denominator += 3.f;
    }
    if(registeredPercepts.totalNumberOfPerceivedPoses && usePoses)
    {
      numerator += 5.f * (static_cast<float>(registeredPercepts.poses.size()) / static_cast<float>(registeredPercepts.totalNumberOfPerceivedPoses));
      denominator += 5.f;
    }
    if(denominator != 0.f)
    {
      const float currentValidity = numerator / denominator;
      samples->at(i).updateValidity(numberOfConsideredFramesForValidity, currentValidity);
    }
  }

  // Apply OwnSideModel:
  if(theGameInfo.secondaryState != STATE2_PENALTYSHOOT)
  {
    for(int i = 0; i < numberOfSamples; ++i)
    {
      if(samples->at(i).getPose().translation.x() > theOwnSideModel.largestXPossible ||
         (Global::getSettings().isGoalkeeper && samples->at(i).getPose().translation.x() > 0.f))
        samples->at(i).invalidate();
    }
  }

  // Check, if sample is still on the carpet
  for(int i = 0; i < numberOfSamples; ++i)
  {
    const Vector2f& position = samples->at(i).getPose().translation;
    if(!theFieldDimensions.isInsideCarpet(position))
      samples->at(i).invalidate();
  }
}

bool SelfLocator::sensorResetting(const RobotPose& robotPose)
{
  if(theGameInfo.secondaryState == STATE2_PENALTYSHOOT) // Don't replace samples in penalty shootout
    return false;
  if(theSideConfidence.mirror)                          // Don't replace samples in mirror cycle
    return false;
  if(timeOfLastReturnFromPenalty != 0 && theFrameInfo.getTimeSince(timeOfLastReturnFromPenalty) < 20000)
    return false;

  // Can we perform the resetting of a sample?
  if(theAlternativeRobotPoseHypothesis.isValid &&
     theAlternativeRobotPoseHypothesis.numOfContributingObservations >= minNumberOfObservationsForResetting &&
     theAlternativeRobotPoseHypothesis.timeOfLastPerceptionUpdate > lastAlternativePoseTimestamp)
  {
    // Should we perform the resetting of a sample?
    const Pose2f mirrorPose = Pose2f(pi) + theAlternativeRobotPoseHypothesis.pose;
//    std::cout<<"robotpose----->"<<robotPose.translation.x()<<","<<robotPose.translation.y()<<","<<robotPose.rotation<<std::endl;
//    std::cout<<"theAlternativeRobotPoseHypothesis----->"<<theAlternativeRobotPoseHypothesis.pose.translation.x()<<","<<theAlternativeRobotPoseHypothesis.pose.translation.y()<<","<<theAlternativeRobotPoseHypothesis.pose.rotation<<std::endl;
//    std::cout<<"mirrorPose---->"<<mirrorPose.translation.x()<<","<<mirrorPose.translation.y()<<","<<mirrorPose.rotation<<std::endl;
    const float xDev = std::abs(robotPose.translation.x() - theAlternativeRobotPoseHypothesis.pose.translation.x());
    const float yDev = std::abs(robotPose.translation.y() - theAlternativeRobotPoseHypothesis.pose.translation.y());
    const float rDev = std::abs(Angle::normalize(robotPose.rotation - theAlternativeRobotPoseHypothesis.pose.rotation));
    const float xDevMirror = std::abs(robotPose.translation.x() - mirrorPose.translation.x());
    const float yDevMirror = std::abs(robotPose.translation.y() - mirrorPose.translation.y());
    const float rDevMirror = std::abs(Angle::normalize(robotPose.rotation - mirrorPose.rotation));
    const float xDeviation = std::min(xDev, xDevMirror);
    const float yDeviation = std::min(yDev, yDevMirror);
    const float rDeviation = std::min(rDev, rDevMirror);
//    std::cout<<"dev---->"<<xDev<<","<<yDev<<","<<rDev<<std::endl;
//    std::cout<<"mirror dev---->"<<xDevMirror<<","<<yDevMirror<<","<<rDevMirror<<std::endl;
//	const float xDeviation = xDev;
//	const float yDeviation = yDev;
//	const float rDeviation = rDev;
//    std::cout<<"Deviation------>"<<xDeviation<<","<<yDeviation<<","<<rDeviation<<std::endl;
    if(xDeviation > translationalDeviationForResetting || yDeviation > translationalDeviationForResetting ||
       rDeviation > rotationalDeviationForResetting)
    {
      float resettingValidity = max(0.5f, averageWeighting); // TODO: Recompute?
      int worstSampleIdx = 0;
      float worstSampleValidity = samples->at(0).validity;
      for(int i = 1; i < numberOfSamples; ++i)
      {
        if(samples->at(i).validity < worstSampleValidity)
        {
          worstSampleIdx = i;
          worstSampleValidity = samples->at(i).validity;
        }
      }
      UKFSample newSample;
      if(theOwnSideModel.stillInOwnSide || Global::getSettings().isGoalkeeper)
        newSample.init(getNewPoseBasedOnObservations(true, lastRobotPose), defaultPoseDeviation, nextSampleNumber++, resettingValidity);
      else
        newSample.init(getNewPoseBasedOnObservations(false, lastRobotPose), defaultPoseDeviation, nextSampleNumber++, resettingValidity);
//      newSample.init(theAlternativeRobotPoseHypothesis.pose, defaultPoseDeviation, nextSampleNumber++, resettingValidity);
      samples->at(worstSampleIdx) = newSample;
      std::cout<<"sensorreset!!!!!"<<std::endl;
      lastAlternativePoseTimestamp = theAlternativeRobotPoseHypothesis.timeOfLastPerceptionUpdate;
      return true;
    }
  }
  return false;
}

void SelfLocator::resampling()
{
  if(averageWeighting == 0.f)
    return;
  // actual resampling step:
  UKFSample* oldSet = samples->swap();
  const float weightingBetweenTwoDrawnSamples = averageWeighting;
  float nextPos(Random::uniform() * weightingBetweenTwoDrawnSamples);
  float currentSum(0);

  // resample:
  int replacements(0);
  int j(0);
  for(int i = 0; i < numberOfSamples; ++i)
  {
    currentSum += oldSet[i].weighting;
    int replicationCount(0);
    while(currentSum > nextPos && j < numberOfSamples)
    {
      samples->at(j) = oldSet[i];
      if(replicationCount) // An old sample becomes copied multiple times: we need new identifier for the new instances
      {
        samples->at(j).id = nextSampleNumber++;
        replacements++;
      }
      replicationCount++;
      j++;
      nextPos += weightingBetweenTwoDrawnSamples;
    }
  }
  int missingSamples = numberOfSamples - j;
  // fill up missing samples (could happen in rare cases) with new poses:
  for(; j < numberOfSamples; ++j)
  {
    const Pose2f pose = getNewPoseBasedOnObservations(false, lastRobotPose);
    samples->at(j).init(pose, defaultPoseDeviation, nextSampleNumber++, averageWeighting);
    ANNOTATION("SelfLocator", "Missing sample! Current number of samples: " << j);
  }
  ASSERT(allSamplesIDsAreUnique());
  PLOT("module:SelfLocator:missingSamples", missingSamples);
  PLOT("module:SelfLocator:sampleReplacements", replacements);
}

void SelfLocator::handleSideConfidence()
{
  if(!theSideConfidence.mirror || sampleSetHasBeenResetted || Global::getSettings().isDropInGame)
    return;
  for(int i = 0; i < numberOfSamples; ++i)
  {
    samples->at(i).mirror();
  }
  SystemCall::playSound("theFlippingChicken.wav");
  ANNOTATION("SelfLocator", "Mirrrrrrooaaaarred!");
}

void SelfLocator::handleGameStateChanges(const Pose2f& propagatedRobotPose)
{
	static int penalty_begin = 0;
  if(theGameInfo.secondaryState == STATE2_PENALTYSHOOT)
  {
    // penalty shoot: if game state switched to playing reset samples to start position
    if((theCognitionStateChanges.lastGameState != STATE_PLAYING && theGameInfo.state == STATE_PLAYING) ||
       (theCognitionStateChanges.lastPenalty != PENALTY_NONE && theRobotInfo.penalty == PENALTY_NONE))
    {
      if(theGameInfo.kickOffTeam == theOwnTeamInfo.teamNumber)
      {
        //striker pose (1 meter behind the penalty spot, looking towards opponent goal)
        for(int i = 0; i < samples->size(); ++i)
          samples->at(i).init(Pose2f(0.f, theFieldDimensions.xPosPenaltyStrikerStartPosition, 0.f), defaultPoseDeviation, nextSampleNumber++, 1.f);
        sampleSetHasBeenResetted = true;
      }
      else
      {
        //goalie pose (in the center of the goal, looking towards the field's center)
        for(int i = 0; i < samples->size(); ++i)
          samples->at(i).init(Pose2f(0.f, theFieldDimensions.xPosOwnGroundline, 0.f), defaultPoseDeviation, nextSampleNumber++, 1.f);
        sampleSetHasBeenResetted = true;
      }
    }
  }
  // If a penalty is over AND we are in SET or when the robot has been lifted during SET, reset samples to manual positioning line positions
  else if((theOwnSideModel.returnFromGameControllerPenalty || theOwnSideModel.returnFromManualPenalty || theOwnSideModel.manuallyPlaced) && theGameInfo.state == STATE_SET)
  {
    for(int i = 0; i < samples->size(); ++i)
    {
      samples->at(i).init(getNewPoseAtManualPlacementPosition(), defaultPoseDeviation, nextSampleNumber++, 0.5f);
    }
    sampleSetHasBeenResetted = true;
    timeOfLastReturnFromPenalty = theFrameInfo.time;
  }
  // If a penalty is over, reset samples to reenter positions
//  else if(theOwnSideModel.returnFromGameControllerPenalty || theOwnSideModel.returnFromManualPenalty)
//  {
//    for(int i = 0; i < samples->size(); ++i)
//    {
//      samples->at(i).init(getNewPoseReturnFromPenaltyPosition(), defaultPoseDeviation, nextSampleNumber++, 0.5f);
//    }
//    sampleSetHasBeenResetted = true;
//    timeOfLastReturnFromPenalty = theFrameInfo.time;
//  }
  // I am clearly in the opponent's half and will be placed manually
  else if(theGameInfo.state == STATE_SET && propagatedRobotPose.translation.x() > 100.f)
  {
    for(int i = 0; i < samples->size(); ++i)
    {
      samples->at(i).init(getNewPoseAtManualPlacementPosition(), defaultPoseDeviation, nextSampleNumber++, 0.5f);
    }
    sampleSetHasBeenResetted = true;
  }
  // Normal game is about to start: We start on the sidelines looking at our goal: (this is for checking in TeamCom)
  else if(theCognitionStateChanges.lastGameState != STATE_INITIAL && theGameInfo.state == STATE_INITIAL)
  {
    for(int i = 0; i < samples->size(); ++i)
    {
      samples->at(i).init(getNewPoseAtWalkInPosition(), defaultPoseDeviation, nextSampleNumber++, 0.5f);
    }
    sampleSetHasBeenResetted = true;
  }
  // Normal game really starts: We start on the sidelines looking at our goal: (this is for actual setup)
  else if(theCognitionStateChanges.lastGameState == STATE_INITIAL && theGameInfo.state == STATE_READY)
  {
    for(int i = 0; i < samples->size(); ++i)
    {
      samples->at(i).init(getNewPoseAtWalkInPosition(), defaultPoseDeviation, nextSampleNumber++, 0.5f);
    }
    sampleSetHasBeenResetted = true;
  }
//	  std::cout<<"penalty----->"<<(theOwnSideModel.returnFromGameControllerPenalty || theOwnSideModel.returnFromManualPenalty)<<std::endl;
	if((theCognitionStateChanges.lastPenalty != PENALTY_NONE && theRobotInfo.penalty == PENALTY_NONE))
	{
		std::cout<<"penalty!!!!!"<<std::endl;
//		penalty_time = high_resolution_clock::now();
//		penalty_begin = 1;
//		Pose2f returnPose;
//		if(theTeamBallModel.isValid)
//		{
//			if(theTeamBallModel.position.y() > 0.f)
//			{
//				returnPose = Pose2f(pi_2, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosRightSideline);
//				returnFromLeft = false;
//			}
//			else
//			{
//				returnPose = Pose2f(-pi_2, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosLeftSideline);
//				returnFromLeft = true;
//			}
//		}
//		else
//		{
//			returnPose = Pose2f(pi_2, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosRightSideline);
//			returnFromLeft = false;
//		}
//		for(int i = 0; i < samples->size(); ++i)
//		{
//		  samples->at(i).init(returnPose, defaultPoseDeviation, nextSampleNumber++, 0.5f);
//		}
//		sampleSetHasBeenResetted = true;
		for(int i = 0; i < 6; i++)
		{
			samples->at(i * 2).init(penaltyPositions[i], defaultPoseDeviation, nextSampleNumber++, 0.5f);
			samples->at(i * 2 + 1).init(penaltyPositions[i], defaultPoseDeviation, nextSampleNumber++, 0.5f);
//			std::cout<<"penaltyposition--->"<<penaltyPositions[i].translation.x()<<penaltyPositions[i].translation.y()<<penaltyPositions[i].rotation<<std::endl;
		}
		sampleSetHasBeenResetted = true;
		lastTimeReset = theFrameInfo.time;
	}

//	if(penalty_begin != 0)
//	{
//		Pose2f returnPose;
//		std::chrono::high_resolution_clock::time_point nowT = high_resolution_clock::now();
//		duration<double> dTotal = duration_cast<duration<double>>(nowT - penalty_time);
//		if(dTotal.count() * 1000 >= 4000)
//		{
//			std::cout<<"averageWeight--->"<<averageWeighting<<std::endl;
//			if(averageWeighting < 0.4)
//			{
//				std::cout<<"mirror!!!!!!!!!!!!!!1"<<std::endl;
//				if(returnFromLeft)
//				{
//					returnPose = Pose2f(pi_2, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosRightSideline);
//				}
//				else
//				{
//					returnPose = Pose2f(-pi_2, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosLeftSideline);
//				}
//				for(int i = 0; i < samples->size(); ++i)
//				{
//				  samples->at(i).init(returnPose, defaultPoseDeviation, nextSampleNumber++, 0.5f);
//				}
//				sampleSetHasBeenResetted = true;
//			}
//			penalty_begin = 0;
//		}
//	}

  if(sampleSetHasBeenResetted)
  {
    idOfLastBestSample = -1;
  }
}

UKFSample& SelfLocator::getMostValidSample()
{
  float validityOfLastBestSample = -1.f;
  UKFSample* lastBestSample = 0;
  if(idOfLastBestSample != -1)
  {
    for(int i = 0; i < numberOfSamples; ++i)
    {
      if(samples->at(i).id == idOfLastBestSample)
      {
        validityOfLastBestSample = samples->at(i).validity;
        lastBestSample = &(samples->at(i));
        break;
      }
    }
  }
  UKFSample* returnSample = &(samples->at(0));
  float maxValidity = -1.f;
  float minVariance = 0.f; // Initial value does not matter
  for(int i = 0; i < numberOfSamples; ++i)
  {
    const float val = samples->at(i).validity;
    if(val > maxValidity)
    {
      maxValidity = val;
      minVariance = samples->at(i).getVarianceWeighting();
      returnSample = &(samples->at(i));
    }
    else if(val == maxValidity)
    {
      float variance = samples->at(i).getVarianceWeighting();
      if(variance < minVariance)
      {
        maxValidity = val;
        minVariance = variance;
        returnSample = &(samples->at(i));
      }
    }
  }
  if(lastBestSample && returnSample->validity <= validityOfLastBestSample * 1.1) // Bonus for stability
    return *lastBestSample;
  else
    return *returnSample;
}

void SelfLocator::domainSpecificSituationHandling()
{
//	std::cout<<"domainspec11111"<<std::endl;
  // Currently, this method only handles the case that the
  // keeper is turned by 180 degrees but assumes to stand correctly
  if(!theRobotInfo.number == 1 || theGameInfo.state != STATE_PLAYING || theGameInfo.secondaryState == STATE2_PENALTYSHOOT || !theCameraMatrix.isValid)
    return;
  // The robot is in its penalty area and assumes to look at the opponent half
  // and guards its goal.
  if(theRobotPose.translation.x() < theFieldDimensions.xPosOwnPenaltyArea &&
     abs(theRobotPose.translation.y()) < theFieldDimensions.yPosLeftPenaltyArea &&
     abs(theRobotPose.rotation) < 45_deg &&
	 theRobotInfo.number == 1)
  {
//	  std::cout<<"domainspec222222"<<std::endl;
//	  SystemCall::playSound("theMirrorCow.wav");
    // Calculate distance to furthest point on field boundary
    float maxDistance = 0.f;
    for(const Vector2f& p : theFieldBoundary.boundaryOnField)
    {
      if(p.norm() > maxDistance)
        maxDistance = p.norm();
    }
    if(maxDistance > goalieFieldBorderDistanceThreshold)
      lastTimeFarFieldBorderSeen = theFrameInfo.time;

    // It has not seen important stuff for a long time but the field border appears
    // to be close. This means that it is probably twisted:
    if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > goalieNoPerceptsThreshold &&
       theFrameInfo.getTimeSince(theCirclePercept.lastSeen) > goalieNoPerceptsThreshold &&
       theFrameInfo.getTimeSince(thePenaltyMarkPercept.timeLastSeen) > goalieNoPerceptsThreshold &&
       lastTimeFarFieldBorderSeen != 0 &&
       theFrameInfo.getTimeSince(lastTimeFarFieldBorderSeen) > goalieNoFarFieldBorderThreshold)
    {
      ANNOTATION("SelfLocator", "Goalie Twist!");
      SystemCall::playSound("theMirrorCow.wav");
      for(int i = 0; i < numberOfSamples; ++i)
        samples->at(i).twist();
    }
  }
}

void SelfLocator::draw(const RobotPose& robotPose)
{
  // Visualizes effect of current function's parameters
  DEBUG_DRAWING("module:SelfLocator:isMirrorCloser", "drawingOnField")
  {
    const float stepSize = 500.f;
    const float degStepSize = 30.f;
    const float length = stepSize / 3.f;
    for(float x = theFieldDimensions.xPosOwnFieldBorder; x <= theFieldDimensions.xPosOpponentFieldBorder; x += stepSize)
    {
      for(float y = theFieldDimensions.yPosRightFieldBorder; y <= theFieldDimensions.yPosLeftFieldBorder; y += stepSize)
      {
        for(float rot = 0.f; rot < 360.f; rot += degStepSize)
        {
          Pose2f samplePose(Angle::fromDegrees(rot), x, y);
          ColorRGBA col = isMirrorCloser(samplePose, robotPose) ? ColorRGBA(255, 0, 0) : ColorRGBA(0, 0, 0);
          Vector2f vec(length, 0.f);
          vec.rotate(Angle::fromDegrees(rot));
          LINE("module:SelfLocator:isMirrorCloser", x, y, x + vec.x(), y + vec.y(), 20, Drawings::solidPen, col);
        }
      }
    }
  }
}

bool SelfLocator::allSamplesIDsAreUnique()
{
  for(int i = 0; i < numberOfSamples - 1; ++i)
  {
    for(int j = i + 1; j < numberOfSamples; ++j)
    {
      if(samples->at(i).id == samples->at(j).id)
        return false;
    }
  }
  return true;
}

Pose2f SelfLocator::getNewPoseBasedOnObservations(bool forceOwnHalf, const Pose2f& lastRobotPose) const
{
  const Pose2f& p = theAlternativeRobotPoseHypothesis.pose;
  if(forceOwnHalf)
  {
    if(p.translation.x() <= 0.f)
      return p;
    else
      return Pose2f(pi) + p;
  }
  else
  {
    if(isMirrorCloser(p, lastRobotPose))
      return Pose2f(pi) + p;
    else
      return p;
  }
}

Pose2f SelfLocator::getNewPoseReturnFromPenaltyPosition()
{
  if(nextReturnFromPenaltyIsLeft)
  {
    nextReturnFromPenaltyIsLeft = false;
    return Pose2f(-pi_2, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosLeftSideline);
  }
  else
  {
    nextReturnFromPenaltyIsLeft = true;
    return Pose2f(pi_2, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosRightSideline);
  }
}

Pose2f SelfLocator::getNewPoseAtWalkInPosition()
{
  if(theRobotInfo.number >= 1 && theRobotInfo.number <= 5 && !Global::getSettings().isDropInGame)
  {
    const unsigned index = theRobotInfo.number - 1;
    return walkInPositions[index];
  }
  else
  {
    Pose2f result = walkInPositions[nextWalkInPoseNumber];
    nextWalkInPoseNumber = (nextWalkInPoseNumber + 1) % walkInPositions.size();
    return result;
  }
}

Pose2f SelfLocator::getNewPoseAtManualPlacementPosition()
{
  // Goalie
  if(Global::getSettings().isGoalkeeper)
  {
    return Pose2f(0.f, theFieldDimensions.xPosOwnGroundline, 0.f);
  }
  else
  {
    float x = theFieldDimensions.xPosOwnPenaltyArea + 100.f;
    float minY = theFieldDimensions.yPosRightSideline + 750.f;
    float y = minY + Random::uniform(2 * std::abs(minY));
    return Pose2f(0.f, x, y);
  }
}

bool SelfLocator::isMirrorCloser(const Pose2f& currentPose, const Pose2f& lastPose) const
{
  const Vector2f& translation = currentPose.translation;
  Vector2f rotationWeight(std::max(theFieldDimensions.yPosLeftSideline * 1.1f - std::min(std::abs(translation.x()), std::abs(lastPose.translation.x())), 0.f), 0);
  Vector2f opponentGoal(theFieldDimensions.xPosOpponentGoalPost, 0.f);
  const Vector2f rotation = Pose2f(Geometry::angleTo(currentPose, opponentGoal)) * rotationWeight;
  const Vector2f lastRotation = Pose2f(Geometry::angleTo(lastPose, opponentGoal)) * rotationWeight;
  bool result = (lastPose.translation - translation).norm() + (lastRotation - rotation).norm() >
                (lastPose.translation + translation).norm() + (lastRotation + rotation).norm();
  return result;
}

void SelfLocator::HandleLowValidity(Pose2f robotpose)
{
	std::cout<<"handling low validity"<<std::endl;
	for(int i = 0; i < 6; i++)
	{
		Pose2f resetPose = Pose2f(robotpose.rotation + 180_deg, robotpose.translation.x(), robotpose.translation.y());
		resetPose.rotation.normalize();
		samples->at(i).init(resetPose, defaultPoseDeviation, nextSampleNumber++, 0.7f);
//		samples->at(i * 2 + 1).init(resetPose, defaultPoseDeviation, nextSampleNumber++, 0.5f);
//		std::cout<<"penaltyposition--->"<<penaltyPositions[i].translation.x()<<penaltyPositions[i].translation.y()<<penaltyPositions[i].rotation<<std::endl;
	}
	for(int i = 6; i < 12; i++)
	{
		Pose2f resetPose = Pose2f(robotpose.rotation, robotpose.translation.x(), robotpose.translation.y());
		resetPose.rotation.normalize();
		samples->at(i).init(resetPose, defaultPoseDeviation, nextSampleNumber++, 0.7f);
//		samples->at(i * 2 + 1).init(resetPose, defaultPoseDeviation, nextSampleNumber++, 0.5f);
//		std::cout<<"penaltyposition--->"<<penaltyPositions[i].translation.x()<<penaltyPositions[i].translation.y()<<penaltyPositions[i].rotation<<std::endl;
	}

	sampleSetHasBeenResetted = true;
	lastTimeReset = theFrameInfo.time;
}

MAKE_MODULE(SelfLocator, modeling)
