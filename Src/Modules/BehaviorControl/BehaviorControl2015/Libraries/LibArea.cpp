#include "../LibraryBase.h"

namespace Behavior2015
{
#include "LibArea.h"

  LibArea::LibArea()
//    :fieldCoverageTarget(Vector2f (0.f, 0.f)),
//    globalPatrolTarget(Vector2f (0.f, 0.f)),
//    fieldCoverageTargetVaild(false),
//    globalPatrolTargetValid(false)
  {
  }

  void LibArea::preProcess()
  {
    //@TODO
    /* This 4 vars don't work @wzf */
//    fieldCoverageTarget = theFieldCoverage.worstTarget.target;
//    fieldCoverageTargetVaild = theFieldCoverage.worstTarget.isValid;
//    globalPatrolTarget = theGlobalFieldCoverage.patrolTarget;
//    globalPatrolTargetValid = theGlobalFieldCoverage.patrolTargetValid;
  }
  void LibArea::postProcess()
  {

  }

//@TODO rewrite don't use the number such as 3000 or 1000, using the variable.
  bool LibArea::nearOppGoalLeft(float x, float y)
  {
    return ((y > -x + 5300) && (x < 4600) && (y < 3100));
  }

  bool LibArea::nearOppGoalRight(float x, float y)
  {
    return ((y < x - 5300) && (x < 4600) && (y > -3100));
  }

  bool LibArea::nearOppGoalMiddle(float x, float y)
  {
    return ((y < -x + 5300) && (y > x - 5300) && x > 800);
  }
  
//  int LibArea::minDistanceToFieldBorderPoint()
//  {
//      int minX=0;
//      int minY=0;
//
//      for (int x = 0;x<theImage.width;++x)
//      {
//          if (theTJArkVision.FieldBorder[x]>minY)
//          {
//              minY = theTJArkVision.FieldBorder[x];
//              minX = x;
//          }
//      }
//      return minX;
//  }
//  int LibArea::maxDistanceToFieldBorderPoint()
//  {
//      int maxX=0;
//      int maxY=theImage.height;
//
//      for (int x = 0;x<theImage.width;++x)
//      {
//          if (theTJArkVision.FieldBorder[x]<maxY)
//          {
//              maxY = theTJArkVision.FieldBorder[x];
//              maxX = x;
//          }
//      }
//      return maxX;
//  }
//  float LibArea::minDistanceToFieldBorder()
//  {
//      //找到FieldBorder最低的点
//    int minX = LibArea::minDistanceToFieldBorderPoint();
//    int minY = theTJArkVision.FieldBorder[minX];
//      //计算距离
//    Vector2i PosImg(minX,minY);
//    Vector2f corrected = theImageCoordinateSystem.toCorrected(PosImg);
//    Vector2i RealPos;
//    RealPos.x() = static_cast<int>(corrected.x());
//    RealPos.y() = static_cast<int>(corrected.y());
//    Vector2f position;
//    float distance=0;
//    if (!Transformation::imageToRobotWithCameraRotation(RealPos,theCameraMatrix, theCameraInfo,position))
//    {
//        return 0.f;
//    }
//    else
//    {
//        distance = position.norm();
//    }
//    return distance;
//  }
//  
//  float LibArea::maxDistanceToFieldBorder()
//  {
//    int maxX = LibArea::maxDistanceToFieldBorderPoint();
//    int maxY = theTJArkVision.FieldBorder[maxX];
//      //计算距离
//    Vector2i PosImg(maxX,maxY);
//    Vector2f corrected = theImageCoordinateSystem.toCorrected(PosImg);
//    Vector2i RealPos;
//    RealPos.x() = static_cast<int>(corrected.x());
//    RealPos.y() = static_cast<int>(corrected.y());
//    Vector2f position;
//    float distance=0;
//    if (!Transformation::imageToRobotWithCameraRotation(RealPos,theCameraMatrix, theCameraInfo,position))
//    {
//        return 0.f;
//    }
//    else
//    {
//        distance = position.norm();
//    }
//    return distance;
//  }
//  LibArea::TurnSuggest LibArea::turnSuggest()
//  {
//      int minX = LibArea::minDistanceToFieldBorderPoint();
//      int maxX = LibArea::maxDistanceToFieldBorderPoint();
//      float minD = LibArea::minDistanceToFieldBorder();
//      float maxD = LibArea::maxDistanceToFieldBorder();
//      if (minD < 700 && maxD < 2000)
//      {
//          if (minX-maxX<=-80)
//              return LibArea::TurnSuggest::TURN_RIGHT;
//          if (abs(maxD-minD)<300)
//              return LibArea::TurnSuggest::TURN_OVER; 
//          if (minX-maxX>=80)
//              return LibArea::TurnSuggest::TURN_LEFT;
//      }
//      else
//          return LibArea::TurnSuggest::NONE;
//      return LibArea::TurnSuggest::NONE;
//  }

}
