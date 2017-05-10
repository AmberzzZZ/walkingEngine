#include "GoalPerceptor2017.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include "Modules/Perception/ImagePreprocessors/ext_math.h"
#include <iostream>
#include <chrono>

//#define SHOW_TIME

using namespace std::chrono; 

GoalPerceptor2017::GoalPerceptor2017() 
{ 
	goalPosts.reserve(15); 
	goalSideSpots.reserve(100); 
	colorSegments.reserve(20); 
}
void GoalPerceptor2017::update(GoalPercept2017 &goalPercept)
{
//
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor2017:goalPostCandidates", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor2017:scanForGoalPostBottom","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor2017:goalPostLinesFinal","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor2017:widthScan","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor2017:checkSize","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor2017:goalSpots","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor2017:spotConnections","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor2017:horizon","drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:GoalPerceptor2017:checkCrossBar","drawingOnImage");
  goalPercept.numberOfGoalPosts = 0;
  goalPercept.goalPosts.clear();
//  execute(false,goalPercept);
#ifdef SHOW_TIME
  high_resolution_clock::time_point startT,endT;
  startT = high_resolution_clock::now();
#endif
  if (!theCameraMatrix.isValid)
	  return;
  execute(goalPercept);

//  
  mergeGoalPostInfo(goalPercept);
  goalPercept.numberOfGoalPosts = static_cast<int>(goalPercept.goalPosts.size());
#ifdef SHOW_TIME
  endT = high_resolution_clock::now();
  duration<double> dTotal = duration_cast<duration<double>>(endT-startT);
  std::cout << "[Processing Time] GoalDetectorProvider----->" << dTotal.count()*1000 <<"ms"<< std::endl;
#endif
  
}
//
void GoalPerceptor2017::execute(GoalPercept2017 &goalPercept)
{
  goalPosts.clear();
  goalPostLinesFinal.clear();
  goalPercept.goalPosts.clear();
  goalSideSpots.clear();

  imageWidth = theECImage.colored.width;
  imageHeight = theECImage.colored.height;

  scanForGoalSegments();
  connectGoalSpots();
  createLinesFromSpots();
//  std::cout << "goalPostLinesFinal.numberOfG="<<goalPercept.numberOfGoalPosts<<'\n';
  createGoalPostsFromLines();
  drawGoalPostCandidates();
//std::cout << "goalPercept.numberOfGoalPosts="<<goalPercept.numberOfGoalPosts<<'\n';
  // verify goal posts by looking for goal post base and add some sanity checks, also decide left/right gp
  verifyGoalPosts(goalPercept);

  goalPercept.numberOfGoalPosts = static_cast<int>(goalPercept.goalPosts.size());

}

void GoalPerceptor2017::scanForGoalSegments()
{
  const CameraMatrix &cameraMatrix = (CameraMatrix&)theCameraMatrix;
  const CameraInfo &cameraInfo = (CameraInfo&)theCameraInfo;
  
  Geometry::Line horizon = Geometry::calculateHorizon(cameraMatrix,cameraInfo);
  
  int scanLineDistance = imageHeight/48;//48
  int goalScanStartY = std::max((int)horizon.base.y()-scanLineDistance*3, 4);
  int goalScanEndY = std::min(std::max(goalScanStartY+scanLineDistance*numberOfScanLines, imageHeight/5), imageHeight-scanLineDistance-4);

  int yPos = goalScanStartY;
  while (yPos <= goalScanEndY)
  {
    runSegmentScanLineGauss(yPos);
    yPos += scanLineDistance;
  }
}

void GoalPerceptor2017::runSegmentScanLineGauss(const int &yPos)
{
  const Image &image = (Image&)theImage;

  const int xStep = std::max(1, imageWidth / 640); // TODO: enough?
  Image::Pixel p = image.getFullSizePixel(yPos,4);
  yBuffer.fill(theECImage.grayscaled[yPos][4]);
  int lastY = getGauss(yBuffer);
  int newY = lastY;
  bool gradientUp = false;
  bool gradientDown = false;
  bool wasUp = false;
  bool wasDown = false;
  int gradientStart = 4;
  int yDiff = 0;

  for (int xPos = 4; xPos <= imageWidth - 4 - xStep; xPos += xStep)
  {
//	  DOT("module:GoalPerceptor2017:goalPostCandidates",xPos,yPos,ColorRGBA::blue,ColorRGBA::blue);
    p = image.getFullSizePixel(yPos,xPos);
    yBuffer.push_front(theECImage.grayscaled[yPos][xPos]);
    newY = getGauss(yBuffer);
    yDiff = newY - lastY;
    //cbDiff = getDiffSum(cbBuffer, p.cb - cbBuffer[0]);
    //crDiff = getDiffSum(crBuffer, p.cr - crBuffer[0]);

    gradientUp = (yDiff > gradientMinDiff);
    //|| (cbDiff > gradientMinDiff) 
    //|| (crDiff > gradientMinDiff);
    gradientDown = (yDiff < -gradientMinDiff);
    //|| (cbDiff < -gradientMinDiff) 
    //|| (crDiff < -gradientMinDiff);
    yBuffer.push_front(theECImage.grayscaled[yPos][xPos]);
    //cbBuffer.add(p.cb);
    //crBuffer.add(p.cr);
    if ((wasUp && !gradientUp) || (wasDown && !gradientDown))
    {
      int length = std::max(xPos - gradientStart, 2);
      GoalSideSpot gs;
      gs.cb = p.cb;
      gs.cr = p.cr;
      gs.y = p.y;
      gs.angle = getAngle(xPos - xStep, yPos, length, image);
      gs.xPos = xPos - length / 2;
      gs.yPos = yPos;
      gs.nextSpot = NULL;
      goalSideSpots.push_back(gs);
    }
    if (!wasUp && gradientUp)
    {
      wasUp = true;
      gradientStart = xPos;
    }
    if (!wasDown && gradientDown)
    {
      wasDown = true;
      gradientStart = xPos;
    }
    wasUp = gradientUp;
    wasDown = gradientDown;
  }
}

void GoalPerceptor2017::runSegmentScanLine(const int &yPos)
{
  // TODO: not only last y diff, use sum as in width scan and increase min diff in params
  const Image &image = (Image&)theImage;
  
  const int xStep = std::max(1,imageWidth/640); // TODO: enough?
  int yDiff = 0;
  // unused variables
  //int cbDiff = 0;
  //int crDiff = 0;
  Image::Pixel p = image.getFullSizePixel(yPos,4);
  yBuffer.fill(theECImage.grayscaled[yPos][4]);
  //cbBuffer.fill(p.cb);
  //crBuffer.fill(p.cr);
  bool gradientUp = false;
  bool gradientDown = false;
  bool wasUp = false;
  bool wasDown = false;
  int gradientStart = 4;
  
  for (int xPos = 4; xPos <= imageWidth - 4 - xStep; xPos += xStep)
  {
    p = image.getFullSizePixel(yPos,xPos);
    
    yDiff = getDiffSum(yBuffer, theECImage.grayscaled[yPos][xPos] - yBuffer[0]);
    //cbDiff = getDiffSum(cbBuffer, p.cb - cbBuffer[0]);
    //crDiff = getDiffSum(crBuffer, p.cr - crBuffer[0]);
    
    gradientUp = (yDiff > gradientMinDiff);
      //|| (cbDiff > gradientMinDiff) 
      //|| (crDiff > gradientMinDiff);
    gradientDown = (yDiff < -gradientMinDiff);
      //|| (cbDiff < -gradientMinDiff) 
      //|| (crDiff < -gradientMinDiff);
    yBuffer.push_front(theECImage.grayscaled[yPos][xPos]);
    //cbBuffer.add(p.cb);
    //crBuffer.add(p.cr);
    if ((wasUp && !gradientUp) || (wasDown && !gradientDown))
    {
      int length = std::max(xPos - gradientStart,2);
      GoalSideSpot gs;
      gs.cb = p.cb;
      gs.cr = p.cr;
      gs.y = p.y;
      gs.angle = getAngle(xPos-xStep,yPos,length,image);
      gs.xPos = xPos-length/2;
      gs.yPos = yPos;
      gs.nextSpot = NULL;
      goalSideSpots.push_back(gs);
    }
    if (!wasUp && gradientUp)
    {
      wasUp = true;
      gradientStart = xPos;
    }
    if (!wasDown && gradientDown)
    {
      wasDown = true;
      gradientStart = xPos;
    }
    wasUp = gradientUp;
    wasDown = gradientDown;
  }
}

void GoalPerceptor2017::connectGoalSpots()
{
  // TODO: check if best spot is connected always
  int scanLineDistance = imageHeight/48; // TODO: check48
  std::vector<GoalSideSpot>::const_iterator end = goalSideSpots.end();
  std::vector<GoalSideSpot>::iterator spot = goalSideSpots.begin();
    
  for (; spot != end; ++spot)
  {
    spot->used = false;
    Geometry::Line testLine;
    testLine.base = Vector2f((float)spot->xPos,(float)spot->yPos);
    std::vector<GoalSideSpot>::iterator testSpot = spot;
    for (; testSpot != end; ++testSpot)
    {
      if (testSpot->yPos - spot->yPos == scanLineDistance
        && abs(testSpot->xPos - spot->xPos) < scanLineDistance) //TODO: param
      {
        testLine.direction = Vector2f((float)testSpot->xPos,(float)testSpot->yPos) - testLine.base;
        float lineAngle = Angle::normalize(testLine.direction.angle() + ((spot->angle < 0) ? pi : 0));
        if (std::abs(lineAngle - spot->angle) < 0.2f && std::abs(lineAngle - testSpot->angle) < 0.2f) //TODO: param
        {
          LINE("module:GoalPerceptor2017:spotConnections",
            spot->xPos,spot->yPos,
            testSpot->xPos,testSpot->yPos,
            2,Drawings::solidPen,ColorRGBA::black);
          spot->nextSpot = &(*testSpot);
          break;
        }
      }
    }
  }
}

void GoalPerceptor2017::createLinesFromSpots()
{
  // TODO: check if last or first of connected points differs from rest 
  //       and is not needed (could lead to imprecise goal post line)
  std::vector<GoalSideSpot>::const_iterator end = goalSideSpots.end();
  std::vector<GoalSideSpot>::iterator spot = goalSideSpots.begin();
  
  for (; spot != end; ++spot)
  {
    Geometry::Line testLine;
    testLine.base = Vector2f((float)spot->xPos,(float)spot->yPos);
    int noSpots = 1;
    float baseAngle = spot->angle;
    float avgAngle = baseAngle;
    GoalSideSpot *nextSpot = spot->nextSpot;
    while (nextSpot && !nextSpot->used)
    {
      noSpots++;
      avgAngle += nextSpot->angle;
      testLine.direction = Vector2f((float)nextSpot->xPos,(float)nextSpot->yPos);
      nextSpot = nextSpot->nextSpot;
    }
    if (noSpots > 2) // TODO: param
    {
      avgAngle /= noSpots;
      testLine.direction -= testLine.base;
      float lineAngle = Angle::normalize(testLine.direction.angle() + ((spot->angle < 0) ? pi : 0));
      if (std::abs(lineAngle - avgAngle) < 0.2f)
      {
        nextSpot = spot->nextSpot;
        while (nextSpot)
        {
          nextSpot->used = true;
          nextSpot = nextSpot->nextSpot;
        }
        goalPostLinesFinal.push_back(testLine);
        LINE("module:GoalPerceptor2017:goalPostLinesFinal",
          testLine.base.x(),testLine.base.y(),
          testLine.base.x() + testLine.direction.x(),
          testLine.base.y() + testLine.direction.y(),
          3,Drawings::solidPen,ColorRGBA::yellow);
      }
    }
  } 
}

void GoalPerceptor2017::createGoalPostsFromLines()
{
  // TODO: near goal posts have very different upper/lower width..
  std::vector<Geometry::Line>::const_iterator end = goalPostLinesFinal.end();
  std::vector<Geometry::Line>::iterator line = goalPostLinesFinal.begin();
  const int maxWidth = imageWidth/3;

  goalPosts.clear();

  for (; line != end; ++line)
  {
    std::vector<Geometry::Line>::iterator otherLine = goalPostLinesFinal.begin();
    for (; otherLine != end; ++otherLine)
    {
      int xDiff = (int)(otherLine->base.x() - line->base.x());
      if (xDiff <= 0)
	  {
//		  std::cout << "dead in xDiff ):" << '\n'; 
        continue;
	  }
      if (std::abs(line->base.y() - otherLine->base.y()) > 0.1f)
      {

        bool swap = otherLine->base.y() > line->base.y();
        const Geometry::Line &lowerLine = (swap ? *(otherLine) : *(line));
        const Geometry::Line &upperLine = (swap ? *(line) : *(otherLine));
        Vector2f dir = lowerLine.direction;
        dir.normalize(upperLine.base.y()-lowerLine.base.y());
        Vector2f newBase = lowerLine.base + dir;
        xDiff = (int)std::abs(upperLine.base.x()-newBase.x());
      }
      
      if (xDiff > 0 && xDiff < maxWidth && std::abs(line->direction.angle() - otherLine->direction.angle()) < 0.3)
      {
        GoalPost gp;
        gp.bottomFound = false;
        gp.foundTop = false;
        gp.foundLineAtBottom = false;
        gp.topWidth = (float)xDiff;
        gp.bottomWidth = (float)xDiff;
        gp.direction = line->direction;
        gp.startInImage = line->base;
        gp.endInImage = line->base + line->direction;
        gp.validity = 0.f;
        goalPosts.push_back(gp);
      }
    }
  }
}

void GoalPerceptor2017::verifyGoalPosts(GoalPercept2017 &goalPercept)
{
  // TODO: check validity calculations
  const CameraMatrix &cameraMatrix = (CameraMatrix&)theCameraMatrix;
  const CameraInfo &cameraInfo = theCameraInfo;

  std::vector<GoalPost>::iterator gp = goalPosts.begin();
  for (;gp != goalPosts.end();)
  {
    // first check for goal post width - TODO: still necessary?
    // get goal color through width scan
    // then check for bottom of the possible goal posts
    if (!verifyWidth || scanForGoalPostWidth((*gp), 5 * (imageHeight / 120)))
    {
      if (!verifyWidth)
        (*gp).validity = 0.4f;
      if (scanForGoalPostBottom(*gp))
      {
        (*gp).foundTop = scanForGoalPostTop(*gp);
      
        (*gp).bottomFound = true;
        if (verifyBySize(*gp, useWorldModel))
        {
          Vector2f pImage((*gp).endInImage.x() + (*gp).bottomWidth*0.5f,(*gp).endInImage.y());
          CIRCLE("module:GoalPerceptor2017:scanForGoalPostBottom",pImage.x(),pImage.y(),5,2,Drawings::solidPen,ColorRGBA::yellow,Drawings::solidBrush,ColorRGBA::black);
          if (Transformation::imageToRobot(pImage,cameraMatrix,cameraInfo,(*gp).locationOnField))
          {
            float gpWidth = ((*gp).topWidth+(*gp).bottomWidth)/2;
            if ((std::abs(1 - Geometry::getDistanceBySize(cameraInfo,100.f,gpWidth)/(*gp).locationOnField.norm()) < 0.2f) ||
              (std::abs(1 - Geometry::getDistanceBySize(cameraInfo,100.f,gpWidth+2)/(*gp).locationOnField.norm()) < 0.2f) ||
              (std::abs(1 - Geometry::getDistanceBySize(cameraInfo, 800.f, (*gp).endInImage.y() - (*gp).startInImage.y()) / (*gp).locationOnField.norm()) < 0.2f)) // TODO: param, ->validity
              (*gp).validity += 0.3f; //TODO: param
            else
              (*gp).validity -= 0.1f; //TODO: param
          }
          else
            (*gp).validity = 0.f;
        }
        else
          (*gp).validity = 0.f;
      }
	  
      else if (acceptWithoutBase)
      {
        (*gp).foundTop = scanForGoalPostTop(*gp);
      
        (*gp).validity -= 0.1f; //TODO: param
        (*gp).bottomFound = false;
        if (verifyBySize(*gp, !((*gp).foundTop)))
        {
          // TODO: update distance (later?) with distance of left/right gp
          //       if top bar is found
          // have to calculate position through size
          float distanceBySize = Geometry::getDistanceBySize(cameraInfo,100.f,std::max((*gp).topWidth,(*gp).bottomWidth));
          Vector2f angles(0,0);
          Geometry::calculateAnglesForPoint(Vector2f((*gp).startInImage.x()+(*gp).topWidth*0.5f,(float)(*gp).startInImage.y()),cameraMatrix,cameraInfo,angles);
          float angleToGoal = angles.x();
          (*gp).locationOnField.x() = (std::cos(angleToGoal)*distanceBySize);
          (*gp).locationOnField.y() = (std::sin(angleToGoal)*distanceBySize);
          (*gp).validity += 0.3f; // TODO
        }
        else
          (*gp).validity = 0.f;
      }
//	  DRAWTEXT("module:GoalPerceptor2017:goalPostCandidates",(*gp).startInImage.x(),(*gp).startInImage.y(),15,ColorRGBA::magenta,(*gp).validity);
    }
    

    if ((*gp).validity > minValidity 
      && (!useGoalColor || abs((*gp).avgY - expectedGoalY) < 60)
      && ((*gp).endInImage.y()-(*gp).startInImage.y()) > imageHeight/12) // TODO: param
    {
      bool newGPUseless = false; //already found or improbable (too far away)
      std::vector<GoalPercept2017::GoalPost>::iterator goalPost = goalPercept.goalPosts.begin();
      while (goalPost != goalPercept.goalPosts.end())
      {
        float goalPostDistance = goalPost->locationOnField.norm();
        int newGPCenterX = (int)((*gp).endInImage.x() + (*gp).startInImage.x() + (*gp).topWidth) / 2;
        int gpCenterX = (goalPost->bottomInImage.x() + goalPost->topInImage.x()) / 2;
        if (abs(newGPCenterX - gpCenterX) < 2 * std::max((float)goalPost->topWidth, (*gp).topWidth))
        {
          if (goalPost->validity < (*gp).validity)
            goalPost = goalPercept.goalPosts.erase(goalPost);
          else
          {
            newGPUseless = true;
            break;
          }
        }
        else if (std::abs((*gp).locationOnField.norm() - goalPostDistance) > 3000 && (*gp).bottomFound && goalPost->bottomFound)
        {
          if ((*gp).locationOnField.norm() < goalPostDistance)
            goalPost = goalPercept.goalPosts.erase(goalPost);
          else
          {
            newGPUseless = true;
            break;
          }
        }
        else
          goalPost++;
      }
      if (!newGPUseless)
      {
        GoalPercept2017::GoalPost newGP;
        newGP.bottomFound = (*gp).bottomFound;
        newGP.bottomWidth = (*gp).bottomWidth;
        newGP.fromUpper = (theCameraInfo.camera==CameraInfo::upper)?true:false;
        newGP.foundLineAtBottom = (*gp).foundLineAtBottom;
        // determine side of goal post, if possible
//        newGP.goalPostSide = 1;//((*gp).foundTop ? scanForGoalPostSide((*gp)) : GoalPercept2017::GoalPost::unknownPost);
//		newGP.goalPostSide = GoalPercept2017::GoalPost::unknownPost;
        newGP.topWidth = (*gp).topWidth;
        newGP.locationOnField.x() = (*gp).locationOnField.x();
        newGP.locationOnField.y() = (*gp).locationOnField.y();
        newGP.validity = (*gp).validity;
        // TODO:
        if ((*gp).foundTop)
        {
          newGP.topInImage.x() = (int)((*gp).startInImage.x() + (*gp).topWidth / 2);
          newGP.topInImage.y() = (int)((*gp).startInImage.y());
        }
        else
        {
          Vector2f pImage = newGP.topInImage.cast<float>();
          if (!Transformation::robotToImage(
            Vector3f((float)newGP.locationOnField.x(), (float)newGP.locationOnField.y(),
              theFieldDimensions.goalHeight), cameraMatrix, cameraInfo, pImage))
          {
            pImage.x() = (*gp).startInImage.x();
            pImage.y() = 2.f; // top of image
          }

          newGP.topInImage = pImage.cast<int>();
        }
        if (newGP.bottomFound)
        {
          newGP.bottomInImage.x() = (int)((*gp).endInImage.x() + (*gp).bottomWidth / 2);
          newGP.bottomInImage.y() = (int)((*gp).endInImage.y());
        }
        else
        {
          int oldY = (int)(*gp).endInImage.y();
          Vector2f posImage;
          //TODO why is the return value not used?
          if (!Transformation::robotToImage(Vector2f(newGP.locationOnField.cast<float>()), cameraMatrix, cameraInfo, posImage))
          {
            posImage = (*gp).startInImage;
          }
          newGP.bottomInImage = posImage.cast<int>();
          if ((abs(oldY - newGP.bottomInImage.y()) > newGP.bottomWidth))
          {
            gp = goalPosts.erase(gp);
            continue;
          }
        }
        // cross bar check
	  float distanceOnField = newGP.locationOnField.norm();
	  float heightByDistance = Geometry::getSizeByDistance(theCameraInfo, 800, distanceOnField);
	  Vector2i bot(newGP.bottomInImage);
	  Vector2i top(newGP.topInImage);
	  top.y() = bot.y()-heightByDistance;
	  CIRCLE("module:GoalPerceptor2017:checkCrossBar", bot.x(), bot.y(), newGP.bottomWidth, 2,
			  Drawings::solidPen, ColorRGBA::red, Drawings::solidBrush, ColorRGBA::green);
	  CIRCLE("module:GoalPerceptor2017:checkCrossBar", top.x(), top.y(), newGP.topWidth, 2,
				  Drawings::solidPen, ColorRGBA::blue, Drawings::solidBrush, ColorRGBA::gray);
	  LINE("module:GoalPerceptor2017:checkCrossBar", bot.x(), bot.y(), top.x(), top.y(),
			  3, Drawings::solidPen, ColorRGBA::gray);

	  const int minWid = 40;
	  const int maxWid = 80;
	  const int minHei = 20;
	  const int maxHei = 40;
	  int wid = 5*newGP.topWidth;
	  if (wid>maxWid) wid = maxWid;
	  if (wid<minWid) wid = minWid;
	  int hei = 3*newGP.topWidth;
	  if (hei>maxHei) hei = maxHei;
	  if (hei<minHei) hei = minHei;

	  RECTANGLE("module:GoalPerceptor2017:checkCrossBar",
	  			  top.x()-wid, top.y()-hei,
	  			  top.x()+wid,top.y()+hei,
	  			  2,Drawings::solidPen,ColorRGBA::orange);
	  std::vector<GoalSideSpot> checkSpots;
	  std::vector<GoalSideSpot> checkLeft;
	  std::vector<GoalSideSpot> checkRight;
	  checkSpots.clear();
	  checkLeft.clear();
	  checkRight.clear();



	  const int xStart = ext_math::clamp(1, top.x()-wid, theECImage.colored.width-1);//5*newGP.topWidth;
	  const int xEnd = ext_math::clamp(1, top.x()+wid, theECImage.colored.width-1);//5*newGP.topWidth;
	  const int yStart = ext_math::clamp(1, top.y()-hei, theECImage.colored.height-1);//3*newGP.topWidth;
	  const int yEnd = ext_math::clamp(1, top.y()+hei, theECImage.colored.height-1);//3*newGP.topWidth;
	  const int xStep = 8;//newGP.topWidth;
	  for (int xPos = xStart; xPos<=xEnd; xPos += xStep)
	  {
		  if (std::abs(xPos-top.x())<newGP.topWidth)
			  continue;
//		  const int yStart = top.y()-3*newGP.topWidth;
//		  const int yEnd = top.y()+3*newGP.topWidth;
		  scanCrossbar(xPos,yStart,yEnd);
	  }
	  for (GoalSideSpot & s : goalSideSpots)
	  {
		  if (s.xPos<xStart)
			  continue;
		  if (s.xPos>xEnd)
			  continue;
		  if (s.yPos<yStart)
			  continue;
		  if (s.yPos>yEnd)
			  continue;
		  if (std::abs(s.xPos-top.x())<newGP.topWidth)
			  continue;
		  checkSpots.push_back(s);
		  if (s.xPos<top.x())
			  checkLeft.push_back(s);
		  else
			  checkRight.push_back(s);
	  }
	  float totalSpots = static_cast<float>(checkSpots.size());
	  float maxAngle = 1000;
	  int maxNum = 0;
	  int dir = 0; //0:no-direction, 1:right, -1:left
	  while(!checkLeft.empty())
	  {
	  float pre = checkLeft.back().angle;
	  float cur = 1000;
	  int num = 0;
	  checkLeft.pop_back();
	  if (fabs(pre)>0.8f)
			  continue;
	  for (int i = 0; i < static_cast<int>(checkLeft.size());++i)
	  {

		  cur = checkLeft[i].angle;
		  if (fabs(fabs(cur)-fabs(pre))<0.1f)
		  {
			  num++;
		  }
	  }
	  if (num>maxNum)
	  {
		  maxNum = num;
		  maxAngle = pre;
		  dir = -1;
	  }
	  }
	  while(!checkRight.empty()){
		  float pre = checkRight.back().angle;
		  float cur = 1000;
		  int num = 0;
		  checkRight.pop_back();
		  if (fabs(pre)>0.8f)
			  continue;
		  for (int i = 0; i < static_cast<int>(checkRight.size());++i)
		  {

			  cur = checkRight[i].angle;
			  if (fabs(fabs(cur)-fabs(pre))<0.1f)
			  {
				  num++;
			  }
		  }
		  if (num>maxNum)
		  {
			  maxNum = num;
			  maxAngle = pre;
			  dir = 1;
		  }
	  }
	  float percent = maxNum/totalSpots;
	  bool accept = true;
	  DRAWTEXT("module:GoalPerceptor2017:checkCrossBar", top.x(), top.y()-10, 10, ColorRGBA::violet, maxAngle);
	  DRAWTEXT("module:GoalPerceptor2017:checkCrossBar", top.x(), top.y(), 10, ColorRGBA::cyan, percent);
	  if (maxAngle<1.f && percent>0.09f)
	  {
	  if (dir>0.5)
	  {
		  DRAWTEXT("module:GoalPerceptor2017:checkCrossBar", top.x(), top.y()+10, 10, ColorRGBA::yellow, "LEFT");
		 newGP.goalPostSide = GoalPercept2017::GoalPost::leftPost;
	  }
	  else if (dir<-0.5)
	  {
		  DRAWTEXT("module:GoalPerceptor2017:checkCrossBar", top.x(), top.y()+10, 10, ColorRGBA::yellow, "RIGHT");
		 newGP.goalPostSide = GoalPercept2017::GoalPost::rightPost;
	  }
	  }
	  else
	  {
		  accept = false;
		 DRAWTEXT("module:GoalPerceptor2017:checkCrossBar", top.x(), top.y()+10, 10, ColorRGBA::yellow, "ABORT");
	  }

        // END TODO
        if (accept)
        {
        	goalPercept.goalPosts.push_back(newGP);
            ++gp;
        }
        else
        {
        	gp = goalPosts.erase(gp);
        }
      }
      else
        gp = goalPosts.erase(gp);
    }
    else
      gp = goalPosts.erase(gp);
  }

}

void GoalPerceptor2017::mergeGoalPostInfo(GoalPercept2017 &goalPercept)
{
  // TODO: finish function
  // TODO: if bottom was not found -> try to get distance by distance to other goal post + topbar?
  int numGoalPosts = static_cast<int>(goalPercept.goalPosts.size());
//  if (numGoalPosts < 2)
//    return;

  std::vector<GoalPercept2017::GoalPost>::iterator gp = goalPercept.goalPosts.begin();
  std::vector<GoalPercept2017::GoalPost>::iterator gpOther = goalPercept.goalPosts.begin();
  
  // checking for goal posts with correct distance on field
  const float maxDist = (float)(theFieldDimensions.yPosLeftGoal/4);
  while (gp != goalPercept.goalPosts.end())
  {
    gpOther = gp;
    gpOther++;
    while (gpOther != goalPercept.goalPosts.end())
    {
      float dist = (gp->locationOnField-gpOther->locationOnField).norm();
      // TODO: one of the gps has side info, validity change to params
      if (std::abs(dist-2*theFieldDimensions.yPosLeftGoal) < maxDist
        && std::abs(gp->topWidth-gpOther->topWidth) < std::max<float>(gp->topWidth/10,3.f))
      {
        gp->validity = std::min(gp->validity+0.1f,1.f);
        gpOther->validity = std::min(gp->validity+0.1f,1.f);
      }
      gpOther++;
    }
    gp++;
  }

  // TODO: trust lower cam goal posts more than upper cam goalposts

  // not too many goal posts! TODO..
  while (goalPercept.goalPosts.size() > 4)
  {
    unsigned minID = 0;
    float minVal = 1.f;
    for (unsigned int i = 0; i < goalPercept.goalPosts.size(); i++)
    {
      if (goalPercept.goalPosts[i].validity < minVal)
      {
        minVal = goalPercept.goalPosts[i].validity;
        minID = i;
      }
    }
    goalPercept.goalPosts.erase(goalPercept.goalPosts.begin()+minID);
  }
  
//  // cross bar check
//  for (GoalPercept2017::GoalPost & gp : goalPercept.goalPosts)
//  {
//	  Vector2f onField;
//	  if (!Transformation::imageToRobot(gp.bottomInImage, theCameraMatrix, theCameraInfo, onField))
//		continue;
//	  gp.locationOnField = onField;
//	  float distanceOnField = onField.norm();
//	  float heightByDistance = Geometry::getSizeByDistance(theCameraInfo, 800, distanceOnField);
//	  Vector2i bot(gp.bottomInImage);
//	  Vector2i top(gp.topInImage);
//	  top.y() = bot.y()-heightByDistance;
//	  CIRCLE("module:GoalPerceptor2017:checkCrossBar", bot.x(), bot.y(), gp.bottomWidth, 2,
//			  Drawings::solidPen, ColorRGBA::red, Drawings::solidBrush, ColorRGBA::green);
//	  CIRCLE("module:GoalPerceptor2017:checkCrossBar", top.x(), top.y(), gp.topWidth, 2,
//	  			  Drawings::solidPen, ColorRGBA::blue, Drawings::solidBrush, ColorRGBA::gray);
//	  LINE("module:GoalPerceptor2017:checkCrossBar", bot.x(), bot.y(), top.x(), top.y(),
//			  3, Drawings::solidPen, ColorRGBA::yellow);
//	  RECTANGLE("module:GoalPerceptor2017:checkCrossBar",
//			  top.x()-5*gp.topWidth, top.y()-3*gp.topWidth,
//			  top.x()+5*gp.topWidth,top.y()+3*gp.topWidth,
//			  2,Drawings::solidPen,ColorRGBA::orange);
//
//	  std::vector<GoalSideSpot> checkSpots;
//	  std::vector<GoalSideSpot> checkLeft;
//	  std::vector<GoalSideSpot> checkRight;
//	  checkSpots.clear();
//	  checkLeft.clear();
//	  checkRight.clear();
//	  const int xStart = top.x()-5*gp.topWidth;
//	  const int xEnd = top.x()+5*gp.topWidth;
//	  const int yStart = top.y()-3*gp.topWidth;
//	  const int yEnd = top.y()+3*gp.topWidth;
//	  	  const int xStep = gp.topWidth;
//	  	  for (int xPos = xStart; xPos<=xEnd; xPos += xStep)
//	  	  {
//	  		  if (std::abs(xPos-top.x())<gp.topWidth)
//	  			  continue;
//	  		  const int yStart = top.y()-3*gp.topWidth;
//	  		  const int yEnd = top.y()+3*gp.topWidth;
//	  		  scanCrossbar(xPos,yStart,yEnd);
//	  	  }
//	  for (GoalSideSpot & s : goalSideSpots)
//	  {
//		  if (s.xPos<xStart)
//			  continue;
//		  if (s.xPos>xEnd)
//			  continue;
//		  if (s.yPos<yStart)
//			  continue;
//		  if (s.yPos>yEnd)
//			  continue;
//		  if (std::abs(s.xPos-top.x())<gp.topWidth)
//		  	  continue;
//		  checkSpots.push_back(s);
//		  if (s.xPos<top.x())
//			  checkLeft.push_back(s);
//		  else
//			  checkRight.push_back(s);
//	  }
//	  float totalSpots = static_cast<float>(checkSpots.size());
//	  float maxAngle = 1000;
//	  int maxNum = 0;
//	  int dir = 0; //0:no-direction, 1:right, -1:left
//	  while(!checkLeft.empty())
//	  {
//	  float pre = checkLeft.back().angle;
//	  float cur = 1000;
//	  int num = 0;
//	  checkLeft.pop_back();
//	  if (fabs(pre)>0.8f)
//	  		  continue;
//	  for (int i = 0; i < static_cast<int>(checkLeft.size());++i)
//	  {
//
//		  cur = checkLeft[i].angle;
//		  if (fabs(fabs(cur)-fabs(pre))<0.1f)
//		  {
//			  num++;
//		  }
//	  }
//	  if (num>maxNum)
//	  {
//		  maxNum = num;
//		  maxAngle = pre;
//		  dir = -1;
//	  }
//	  }
//	  while(!checkRight.empty()){
//	  	  float pre = checkRight.back().angle;
//	  	  float cur = 1000;
//	  	  int num = 0;
//	  	  checkRight.pop_back();
//	  	  if (fabs(pre)>0.8f)
//	  		  continue;
//	  	  for (int i = 0; i < static_cast<int>(checkRight.size());++i)
//	  	  {
//
//	  		  cur = checkRight[i].angle;
//	  		  if (fabs(fabs(cur)-fabs(pre))<0.1f)
//	  		  {
//	  			  num++;
//	  		  }
//	  	  }
//	  	  if (num>maxNum)
//	  	  {
//	  		  maxNum = num;
//	  		  maxAngle = pre;
//	  		  dir = 1;
//	  	  }
//	  }
//	  DRAWTEXT("module:GoalPerceptor2017:checkCrossBar", top.x(), top.y()-10, 10, ColorRGBA::violet, maxAngle);
//	  DRAWTEXT("module:GoalPerceptor2017:checkCrossBar", top.x(), top.y(), 10, ColorRGBA::cyan, maxNum/totalSpots);
//	  if (dir>0.5)
//	  {
//		  DRAWTEXT("module:GoalPerceptor2017:checkCrossBar", top.x(), top.y()+10, 10, ColorRGBA::yellow, "LEFT");
//	  }
//	  else if (dir<-0.5)
//	  {
//		  DRAWTEXT("module:GoalPerceptor2017:checkCrossBar", top.x(), top.y()+10, 10, ColorRGBA::yellow, "RIGHT");
//	  }


//  }

  numGoalPosts = static_cast<int>(goalPercept.goalPosts.size());
  if (numGoalPosts==2)
  {
	  if (goalPercept.goalPosts[0].bottomInImage.x()<goalPercept.goalPosts[1].bottomInImage.x())
	  {
		  goalPercept.goalPosts[0].goalPostSide = GoalPercept2017::GoalPost::leftPost;
		  goalPercept.goalPosts[1].goalPostSide = GoalPercept2017::GoalPost::rightPost;
	  }
	  else
	  {
		  goalPercept.goalPosts[1].goalPostSide = GoalPercept2017::GoalPost::leftPost;
		  goalPercept.goalPosts[0].goalPostSide = GoalPercept2017::GoalPost::rightPost;
	  }
  }

}

bool GoalPerceptor2017::scanForGoalPostBottom(GoalPost &gp)
{
  // needs correct avgCr and avgCb values!
   
  Vector2f direction(gp.endInImage.x()-gp.startInImage.x(),gp.endInImage.y()-gp.startInImage.y());
  Vector2f startPoint(gp.startInImage.x()+gp.topWidth*0.5f,gp.startInImage.y());
  if (!verifyWidth)
  {
    // TODO
    gp.avgCb = expectedGoalCb;
    gp.avgCr = expectedGoalCr;
    gp.avgY = expectedGoalY;
  }
  int length = scanSameColor(
    startPoint,
    direction,
    (int)(1+(gp.topWidth/40)),
    gp.avgCb,gp.avgCr,gp.avgY);
//  std::cout<<"length="<<length<<'\n';
  if (!useGoalColor || 
    (abs(gp.avgCb-expectedGoalCb) < maxCbDiffOnGoal &&
    abs(gp.avgCr-expectedGoalCr) < maxCrDiffOnGoal))
  {
    gp.endInImage.y() = gp.startInImage.y() + length;
    gp.endInImage.x() = (gp.startInImage + direction.normalize((float)length)).x();
    if (!scanForGreen(gp.endInImage))
      return false;
    LINE("module:GoalPerceptor2017:scanForGoalPostBottom",
      gp.startInImage.x()+gp.topWidth*0.5f,
      gp.startInImage.y(),
      gp.endInImage.x()+gp.bottomWidth*0.5f,
      gp.endInImage.y(),
      3,Drawings::solidPen,ColorRGBA::yellow);
    Vector2f base = gp.startInImage+Vector2f(gp.topWidth*0.5f,0);
    Vector2f dir = (gp.endInImage+Vector2f(gp.bottomWidth*0.5f,0))-base;
    Geometry::Line goalLine(base,dir);
    const std::vector<LinesPercept::Line> lineVector = theLinesPercept.lines;
    for (unsigned i = 0; i < lineVector.size(); i++)
    {
      // TODO
//      if (upper != lineVector[i].fromUpper)
//        continue;
      Geometry::Line fieldLine(lineVector[i].firstImg,Vector2i(lineVector[i].firstImg-lineVector[i].lastImg));
      Vector2f intersection(-1.f,-1.f);
      if (Geometry::checkIntersectionOfLines(fieldLine.base,fieldLine.base+fieldLine.direction,goalLine.base,goalLine.base+goalLine.direction)
        && Geometry::getIntersectionOfLines(fieldLine,goalLine,intersection)
        && intersection.y() < gp.endInImage.y() && intersection.y() > 5) // not out of image
      {
        CROSS("module:GoalPerceptor2017:scanForGoalPostBottom",
          intersection.x(),intersection.y(),
          5,2,
          Drawings::solidPen,ColorRGBA::blue);
        gp.endInImage = intersection;
        gp.endInImage.x() -= gp.bottomWidth*0.5f;
        gp.validity = std::min(1.f,gp.validity+0.3f);
        gp.foundLineAtBottom = true;
        break;
      }
    }
    return true;
  }
  else
    gp.validity = 0.f;
  return false;
}

bool GoalPerceptor2017::scanForGoalPostTop(GoalPost &gp)
{
  Vector2f direction(-gp.endInImage.x()+gp.startInImage.x(),-gp.endInImage.y()+gp.startInImage.y());
  if (std::abs(direction.y()) < 1.f)
    return false;
  Vector2f startPoint(gp.startInImage.x()+gp.topWidth*0.5f,gp.startInImage.y());
  int length = std::abs(scanSameColor(
    startPoint,
    direction,
    (int)(1+(gp.topWidth/40)),
    gp.avgCb,gp.avgCr,gp.avgY));
  // not a sure gp upper end found
  if (gp.startInImage.y()-length < 6)
    return false;
  else
    gp.startInImage += direction.normalize(static_cast<float>(length));
  return true;
}

GoalPercept2017::GoalPost::GoalPostSide GoalPerceptor2017::scanForGoalPostSide(
    const GoalPost &gp)
{
  if (!gp.foundTop)
    return GoalPercept2017::GoalPost::unknownPost;
  int minGoalColorCount = std::max<int>(static_cast<int>(gp.topWidth/3),3);
  Vector2i scanDir(0,1);
  int maxWidth = (int)(gp.topWidth);
  Vector2i startPoint((int)(gp.startInImage.x()+0.5f*gp.topWidth),
    (int)(gp.startInImage.y()-0.5f*gp.topWidth));
  Vector2i pointStep(2*(int)gp.topWidth,0);
  
  if (checkForGoalPostColor(startPoint+pointStep,scanDir,maxWidth,gp.avgCb,gp.avgCr,gp.avgY) >= minGoalColorCount
    && checkForGoalPostColor(startPoint+pointStep*2,scanDir,maxWidth,gp.avgCb,gp.avgCr,gp.avgY) >= minGoalColorCount)
    return GoalPercept2017::GoalPost::leftPost;
  else if (checkForGoalPostColor(startPoint-pointStep,scanDir,maxWidth,gp.avgCb,gp.avgCr,gp.avgY) >= minGoalColorCount
    && checkForGoalPostColor(startPoint-pointStep*2,scanDir,maxWidth,gp.avgCb,gp.avgCr,gp.avgY) >= minGoalColorCount)
    return GoalPercept2017::GoalPost::rightPost;

  return GoalPercept2017::GoalPost::unknownPost;
}

bool GoalPerceptor2017::verifyBySize(GoalPost &gp, const bool useWorldModel)
{
  const CameraInfo &cameraInfo = theCameraInfo;
  const CameraMatrix &cameraMatrix = (CameraMatrix&)theCameraMatrix;
  
  if (theCameraInfo.camera==CameraInfo::upper && std::max(gp.topWidth, gp.bottomWidth) > imageWidth / 6)
    return false;
  float expectedMinDistance = 150.f;
  float expectedMaxDistance = 12000.f;
  if (useWorldModel) // TODO: check for head position (isPostVisible..)
  {
    Vector2f robotPose(theRobotPose.translation);
    Vector2f closePost(sgn(theRobotPose.translation.x())*theFieldDimensions.xPosOpponentGroundline,theFieldDimensions.yPosLeftGoal);
    bool foundClosePost = false;
    bool foundFarPost = false;
    if (std::abs(Transformation::fieldToRobot(theRobotPose,closePost).angle() - theJointAngles.angles[Joints::headYaw]) < 0.6f)
    {
      float dist = Transformation::fieldToRobot(theRobotPose,closePost).norm()/5.f;
      expectedMinDistance = std::max(150.f,(robotPose-closePost).norm() - dist);
      expectedMaxDistance = std::min(10000.f,(robotPose-closePost).norm() + dist);
      foundClosePost = true;
    }
    else if (std::abs(Transformation::fieldToRobot(theRobotPose,Vector2f(closePost.x(),-closePost.y())).angle() - theJointAngles.angles[Joints::headYaw]) < 0.6f)
    {
      closePost = Vector2f(closePost.x(),-closePost.y());
      float dist = Transformation::fieldToRobot(theRobotPose,closePost).norm()/5.f;
      expectedMinDistance = std::max(150.f,(robotPose-closePost).norm() - dist);
      if (!foundClosePost)
        expectedMaxDistance = std::min(10000.f,(robotPose-closePost).norm() + dist);
      else
        expectedMaxDistance = std::min(10000.f,std::max((robotPose-closePost).norm() + dist,expectedMaxDistance));
      foundClosePost = true;
    }
    if (!foundClosePost)
      expectedMinDistance = 100000.f;

    Vector2f farPost = -closePost;
    if (std::abs(Transformation::fieldToRobot(theRobotPose,farPost).angle() - theJointAngles.angles[Joints::headYaw]) < 0.6)
    {
      float dist = Transformation::fieldToRobot(theRobotPose,farPost).norm()/5.f;
      expectedMaxDistance = std::min(10000.f,(robotPose-farPost).norm() + dist);
      if (!foundClosePost)
        expectedMinDistance = std::max(150.f,(robotPose-farPost).norm() - dist);
      foundFarPost = true;
    }
    if (std::abs(Transformation::fieldToRobot(theRobotPose,Vector2f(farPost.x(),-farPost.y())).angle() - theJointAngles.angles[Joints::headYaw]) < 0.6)
    {
      farPost = Vector2f(farPost.x(),-farPost.y());
      float dist = Transformation::fieldToRobot(theRobotPose,farPost).norm()/5.f;
      if (!foundFarPost)
      {
        expectedMaxDistance = std::min(10000.f,std::max(expectedMaxDistance,(robotPose-farPost).norm() + dist));
        if (!foundClosePost)
          expectedMinDistance = std::max(150.f,std::min(expectedMinDistance,(robotPose-farPost).norm() - dist));
      }
      else
      {
        expectedMaxDistance = std::min(10000.f,(robotPose-farPost).norm() + dist);
        if (!foundClosePost)
          expectedMinDistance = std::max(150.f,(robotPose-farPost).norm() - dist);
      }
      foundFarPost = true;
    }
    if (!foundFarPost && !foundClosePost)
      expectedMaxDistance = 0.f;
  }
  float pixelSize = std::max(gp.topWidth,gp.bottomWidth);
  float realSize = 100.f;
  float distanceBySize = Geometry::getDistanceBySize(cameraInfo,realSize,pixelSize);

  if (gp.bottomFound)
  {
    Vector2f onField;
    if (!Transformation::imageToRobot(gp.endInImage, cameraMatrix, cameraInfo, onField))
      return false;
    float distanceOnField = onField.norm();
    float sizeByDistance = Geometry::getSizeByDistance(cameraInfo, 100, distanceOnField);
    float heightByDistance = Geometry::getSizeByDistance(cameraInfo, 800, distanceOnField);
    float ratio = std::max(sizeByDistance, gp.bottomWidth) / std::min(sizeByDistance, gp.bottomWidth);
	float ratioWidth = std::fabs(gp.bottomWidth-sizeByDistance) / sizeByDistance;
	float ratioHeight = std::fabs((gp.endInImage-gp.startInImage).norm()-heightByDistance)/heightByDistance;
	DRAWTEXT("module:GoalPerceptor2017:checkSize",gp.startInImage.x(),gp.startInImage.y(),10,ColorRGBA::magenta,ratioWidth);
	DRAWTEXT("module:GoalPerceptor2017:checkSize",gp.startInImage.x(),gp.startInImage.y()+10,10,ColorRGBA::cyan,ratioHeight);
	DRAWTEXT("module:GoalPerceptor2017:checkSize",gp.startInImage.x(),gp.startInImage.y()-10,10,ColorRGBA::black,ratio);
//	if (ratioHeight>0.5)
//		return false;
    if (ratio > 1.5 && std::abs(sizeByDistance - gp.bottomWidth) > 3)
      return false;
  }

  if (!gp.bottomFound && distanceBySize > 1000) // TODO: param
    return false;
  if (gp.bottomFound && gp.foundTop)
  {
    realSize = 800.f;
    pixelSize = (gp.startInImage-gp.endInImage).norm();
    float distanceByHeight = Geometry::getDistanceBySize(cameraInfo,realSize,pixelSize);
    if (distanceByHeight < 2000 || std::abs(distanceByHeight - distanceBySize) > distanceBySize / 4 || distanceByHeight > expectedMaxDistance)
      gp.validity -= 0.1f;
  }
  
  return (distanceBySize > expectedMinDistance && distanceBySize < expectedMaxDistance);
}

bool GoalPerceptor2017::scanForGoalPostWidth(
    GoalPost &gp, 
    const int yStep)
{
  const int maxNumberOfScans = 5;
  float xStep = Vector2f(
    (gp.endInImage.x()-gp.startInImage.x()),
    (gp.endInImage.y()-gp.startInImage.y())).normalize((float)yStep).x();
  int yStart = (int)gp.startInImage.y();
  float xStart = gp.startInImage.x();
  int maxY = std::min<int>(imageHeight - 4, yStart + 4*yStep);
  float width[maxNumberOfScans];
  int startX[maxNumberOfScans];
  for (int i = 0; i < maxNumberOfScans; i++)
    width[i] = -1;
  int validityPoints = 0;
  int fails = 0;
  //float avgWidth = 0.f;
  //short noValidScans = 0;

  for (int noScan = 0; yStart <= maxY; noScan++)
  {
    width[noScan] = scanForGoalPostWidthAt(Vector2i((int)xStart,yStart),
      gp.topWidth,
      startX[noScan],
      gp.avgCb,
      gp.avgCr,
      gp.avgY);
	  
    if (width[noScan] <= 0)
    {
      DEBUG_RESPONSE("debug drawing:module:GoalPerceptor2017:widthScan")
      {
        if (theCameraInfo.camera==CameraInfo::upper)
          CROSS("module:GoalPerceptor2017:widthScan",
            startX[noScan], yStart,
            yStep / 2,
            3, Drawings::solidPen, ColorRGBA::black);
      }
      DEBUG_RESPONSE("debug drawing:module:GoalPerceptor2017:widthScan")
      {
        if (theCameraInfo.camera==CameraInfo::lower)
          CROSS("module:GoalPerceptor2017:widthScan",
            startX[noScan], yStart,
            yStep / 2,
            3, Drawings::solidPen, ColorRGBA::black);
      }
      fails++;
    }
    else
    {
      DEBUG_RESPONSE("debug drawing:module:GoalPerceptor2017:widthScan")
      {
        if (theCameraInfo.camera==CameraInfo::upper)
          LINE("module:GoalPerceptor2017:widthScan",
            startX[noScan], yStart,
            startX[noScan] + width[noScan], yStart,
            3, Drawings::solidPen, ColorRGBA::yellow);
      }
      DEBUG_RESPONSE("debug drawing:module:GoalPerceptor2017:widthScan")
      {
        if (theCameraInfo.camera==CameraInfo::lower)
          LINE("module:GoalPerceptor2017:widthScan",
            startX[noScan], yStart,
            startX[noScan] + width[noScan], yStart,
            3, Drawings::solidPen, ColorRGBA::yellow);
      }
      validityPoints++;
      //avgWidth += width[noScan];
      // TODO: better bottom width somewhere else!
      gp.bottomWidth = width[noScan]; // best we have right now
      gp.endInImage.x() = static_cast<float>(startX[noScan]);
      gp.endInImage.y() = static_cast<float>(yStart);
      //noValidScans++;
    }
    yStart += yStep;
    xStart += xStep;
  }
  gp.topWidth = width[0];
  gp.startInImage.x() = static_cast<float>(startX[0]);
  /*avgWidth /= noValidScans;
  int widthDiffSum = 0;
  for (short i = 0; i < maxNumberOfScans; i++)
    if (width[i] > 0) 
      widthDiffSum += abs(avgWidth-width[i]);*/
  // TODO: hard coded values..
  gp.validity = (float)validityPoints/(1.5f*(float)maxNumberOfScans);
  return (validityPoints > fails && width[0] > 0);
}

float GoalPerceptor2017::scanForGoalPostWidthAt(
    const Vector2i &startPoint,
    const float &expectedWidth,
    int &realStartX,
    int &avgCb, int &avgCr, int &avgY)
{
  // TODO: check/remove hard coded values
  // TODO: check for goal color?
  //       -> problem: edge very near to real goal edge is detected as one..
  // TODO: use jump in color too
  const Image &image = (Image&)theImage;

  int maxColorDiff = std::max(maxCbDiffOnGoal,maxCrDiffOnGoal);
  MODIFY("module:GoalPerceptor2017:maxColorDiff",maxColorDiff);
  int oldCb = avgCb;
  int oldCr = avgCr;
  int oldY = avgY;
  colorSegments.clear();
  colorSegments.push_back(ColorSegment());
  int baseX = std::max(startPoint.x() - 3,4);
  float width = 0.f;
  int colorCount = 0;
  Image::Pixel p = image.getFullSizePixel(startPoint.y(),baseX);
  int lastY = theECImage.grayscaled[startPoint.y()][baseX];
  int lastYDiff = 0;
  int startX = baseX;
  float startXFinal = (float)startX;
  int endX = baseX;
  float endXFinal = (float)endX;
  int imageX = baseX;
  bool lastYDiffBig = false;
  // unused variable
  //bool lastColorDiffBig = false;
  int maxX = startPoint.x()+(int)expectedWidth + std::max<int>(3,static_cast<int>(expectedWidth/6.f));
  int yDiffSum = 0;
  if (image.isOutOfImage(maxX,startPoint.y(),4))
  {
    return -1;
  }

  int segmentNo = 0;
  int lastColorDiff = 0;
  int lastCb = p.cb;
  int lastCb2 = p.cb;
  int lastCr2 = p.cr;
  int lastCr = p.cr;

  int yDiff = 0;
  int colorDiff = 0;

  for (; imageX < maxX; imageX++)
  {
    p = image.getFullSizePixel(startPoint.y(),imageX);
    yDiff = (theECImage.grayscaled[startPoint.y()][imageX] - lastY);
    colorDiff = std::abs(p.cr - lastCr - lastCr + lastCr2) + std::abs(p.cb - lastCb - lastCb + lastCb2);
    yDiffSum = (sgn(yDiff) != sgn(lastYDiff)) ? 0 : yDiffSum + yDiff;

    if ((std::abs(yDiff+lastYDiff) > 40 && std::abs(lastYDiff) <= 40) // TODO: params
      || std::abs(yDiffSum) > 40
      || colorDiff > 40
      || (lastYDiffBig && std::abs(yDiff+lastYDiff) < 20 && std::abs(yDiff) > 20))
    {
      yDiffSum = 0;
      if (!lastYDiffBig)
      {
        if (startX != baseX)
          endX = imageX;
        else
          startX = imageX;
      }
      lastYDiffBig = true;
    }
    else
    {
      if (lastYDiffBig && endX == baseX && startXFinal < startX+0.1f)
      {
        startXFinal = 0.5f*(float)(startX+imageX);
        avgCb = 0;
        avgCr = 0;
        avgY = 0;
        colorCount = 0;
        colorSegments[segmentNo].length = 0;
        colorSegments[segmentNo].avgCb = 0;
        colorSegments[segmentNo].avgCr = 0;
      }
      lastYDiffBig = false;
      if (endX != baseX)
      {
        if (expectedWidth-(endX-startX) < std::max(5.f,expectedWidth/3))
          break;
        else
          endX = baseX;
      }
    }
    if (startX != baseX && endX == baseX)
    {
      lastColorDiff = std::abs(p.cr-lastCr-lastCr+lastCr2)+std::abs(p.cb-lastCb-lastCb+lastCb2);
      if (colorSegments[segmentNo].length > 0
        && (lastColorDiff > maxColorDiff || std::abs(yDiff + lastYDiff) > 50))
      {
        colorSegments[segmentNo].avgCb /= colorSegments[segmentNo].length;
        colorSegments[segmentNo].avgCr /= colorSegments[segmentNo].length;
        colorSegments[segmentNo].avgY /= colorSegments[segmentNo].length;
        colorSegments.push_back(ColorSegment());
        segmentNo++;
      }
      colorSegments[segmentNo].avgCb += p.cb;
      colorSegments[segmentNo].avgCr += p.cr;
      colorSegments[segmentNo].avgY += p.y;;
      colorSegments[segmentNo].length++;
      avgCr += p.cr;
      avgCb += p.cb;
      avgY += p.y;
	  if (theECImage.colored[startPoint.y()][imageX]==FieldColors::Color::green)//!!!
//      if (fieldColor.isPixelFieldColor(p.y/,p.cb,p.cr))
        return -1;
      colorCount++;
    }
    lastYDiff = yDiff;
    lastY = p.y;
    lastCb2 = lastCb;
    lastCr2 = lastCr;
    lastCb = p.cb;
    lastCr = p.cr;
  }
  realStartX = (int)startXFinal; //TODO: float?
  endXFinal = endX + startXFinal - startX;
  width = endXFinal - startXFinal;
  if (width <= 0.1f || ((std::abs(1.f-width/expectedWidth) > 0.3) && std::abs(width-expectedWidth) >= 3.f))
  {
    avgCb = oldCb;
    avgCr = oldCr;
    avgY = oldY;
    return -1.f;
  }
  if (colorCount == 0)
    return -1;
  avgCb /= colorCount;
  avgCr /= colorCount;
  avgY /= colorCount;
  if (oldCb > 0)
  {
    avgCb = (avgCb+oldCb)/2;
    avgCr = (avgCr+oldCr)/2;
    avgY = (avgY+oldY)/2;
  }
  if (colorSegments[segmentNo].length > 0)
  {
    colorSegments[segmentNo].avgCb /= colorSegments[segmentNo].length;
    colorSegments[segmentNo].avgCr /= colorSegments[segmentNo].length;
  }
  const int minLength = 3;
  int lastID = -1;
  for (int i = 0; i <= segmentNo; i++)
  {
    if (colorSegments[i].length > minLength)
    {
      if (lastID < 0)
      {
        if (abs(colorSegments[i].avgCb-avgCb) < maxCbDiffOnGoal 
          && abs(colorSegments[i].avgCr-avgCr) < maxCrDiffOnGoal)
        {
          lastID = i;
        }
        else
          return -1;
      }
      else
      {
        if (abs(colorSegments[i].avgCb-colorSegments[lastID].avgCb) < maxCbDiffOnGoal 
          && abs(colorSegments[i].avgCr-colorSegments[lastID].avgCr) < maxCrDiffOnGoal
          && abs(colorSegments[i].avgY - colorSegments[lastID].avgY) < 50)
        {
          lastID = i;
        }
        else
          return -1;
      }
    }
  }
  return width;
}

int GoalPerceptor2017::scanSameColor(
  const Vector2f &startPoint, 
  const Vector2f &direction,
  const int &stepSize,
  int &avgCb, int &avgCr, int &avgY)
{
  const Image &image = (Image&)theImage;

  float length = 0.f;
  int count = 0;
  Vector2f scanStep = direction;
  scanStep.normalize((float)stepSize);
  Vector2f checkPoint = startPoint;
  Vector2f lastGoodPoint = startPoint;
  if (image.isOutOfImage(checkPoint.x(),checkPoint.y(),3))
    return 0;
  Image::Pixel p = image.getFullSizePixel((unsigned int)checkPoint.y(),(unsigned int)checkPoint.x());
  int lastCb = p.cb;
  int lastCr = p.cr;
  int lastY = p.y;
  int avgCbNew = 0;
  int avgCrNew = 0;
  int avgYNew = 0;
  int wrongColorCount = 0;
  int wrongColorDiffCount = 0;
  int goodColorCount = 0;
  // TODO: check for color diff sum

  // TODO: too much params?
  while (!image.isOutOfImage(checkPoint.x(),checkPoint.y(),3)
    && wrongColorCount < 3 && wrongColorDiffCount < std::max(3,count/5)) //TODO: param
  {
    p = image.getFullSizePixel((unsigned int)checkPoint.y(),(unsigned int)checkPoint.x());
    if (std::abs(p.cb - avgCb) > maxCbDiffOnGoal
      || std::abs(p.cr - avgCr) > maxCrDiffOnGoal
      || (std::abs(p.y - avgY) > 60)
      || (useGoalColor && (std::abs(p.cb - expectedGoalCb) > maxCbDiffOnGoal
        || std::abs(p.cr - expectedGoalCr) > maxCrDiffOnGoal
        || ((abs(p.y - expectedGoalY) > 60) && std::abs(p.y - avgY) > 20)))
        || theECImage.colored[(unsigned int)checkPoint.y()][(unsigned int)checkPoint.x()]==FieldColors::Color::green)//!!!!!!!!!!
    {
      wrongColorCount++;
      wrongColorDiffCount = 0;
      goodColorCount = 0;
    }
    else if (std::abs(p.cb - lastCb) > maxCbDiffOnGoal
      || std::abs(p.cr - lastCr) > maxCrDiffOnGoal)
      wrongColorDiffCount++;
    else
    {
      avgCbNew += p.cb;
      avgCrNew += p.cr;
      avgYNew += p.y;
      count++;
      length+=stepSize;
      goodColorCount++;
      lastGoodPoint = checkPoint;
      if (goodColorCount > 5)
        wrongColorCount = 0;
    }
    checkPoint+=scanStep;
    lastCb = p.cb;
    lastCr = p.cr;
    lastY = p.y;
  }
  if (count == 0)
    return 1;
  checkPoint -= scanStep;
  length -= stepSize;
  avgCb = (avgCb + avgCbNew/count)/2;
  avgCr = (avgCr + avgCrNew/count)/2;
  avgY = (avgY + avgYNew/count)/2;
  return (int)lastGoodPoint.y() - (int)startPoint.y();
}

bool GoalPerceptor2017::scanForGreen(const Vector2f &basePoint)
{
  const Image &image = (Image&)theImage;
  int fieldColorCount = 0;
  for (float angle = -pi_4; angle < 0.8f; angle += pi_4)
  {
    Vector2f direction(0.f,2.f);
    direction.rotate(angle);
    Vector2f scanPoint(basePoint+direction*5.f);
    for (int count = 0; count < 4; count++)
    {
      if (image.isOutOfImage(scanPoint.x(),scanPoint.y(),3))
        return false;
//      Image::Pixel p = image.getFullSizePixel((unsigned int)scanPoint.y(),(unsigned int)scanPoint.x());
      fieldColorCount += (theECImage.colored[(unsigned int)scanPoint.y()][(unsigned int)scanPoint.x()]==FieldColors::Color::green);//!!!!!!!
      scanPoint += direction;
    }
  }
  return fieldColorCount > 3; //TODO ?
}

int GoalPerceptor2017::checkForGoalPostColor(
  const Vector2i &startPoint, 
  const Vector2i &direction, 
  const int &width,
  const int &optCb, const int &optCr, const int &optY)
{
  const Image &image = (Image&)theImage;

  const int maxDiffCb = std::max(optCb/10,maxCbDiffOnGoal);
  const int maxDiffCr = std::max(optCr/10,maxCrDiffOnGoal);
  const int maxDiffY = std::max(optY/6,60);
  int goalColorCount = 0;
  int i = 0;
  int minGoalColorCount = std::max(3,(int)width/6);
  
  Vector2i checkPoint(startPoint.x(),startPoint.y());
  while (!image.isOutOfImage(checkPoint.x(),checkPoint.y(),3) &&
    i < width &&
    goalColorCount <= minGoalColorCount)
  {
    checkPoint += direction;
    i++;
    Image::Pixel p = image.getFullSizePixel((unsigned int)checkPoint.y(),(unsigned int)checkPoint.x());
    if (abs(p.cr - optCr) < maxDiffCr && abs(p.cb - optCb) < maxDiffCb
      && (abs(p.y - optY) < maxDiffY || p.y > optY))
    {
      if (goalColorCount == 0)
      {
        float angle = getAngle(checkPoint.x(),checkPoint.y(),2,image);
        if (std::abs(std::abs(angle)-pi_2) > 0.2f)
          goalColorCount--;
      }
      goalColorCount++;
    }
  }
  return goalColorCount;
}  

void GoalPerceptor2017::drawGoalPostCandidates()
{
  ColorRGBA baseColor(0,0,0);
  std::vector<GoalPost>::const_iterator gp = goalPosts.begin();
  std::vector<GoalPost>::const_iterator end = goalPosts.end();
  for (;gp != end; ++gp)
  {
    baseColor.b += 70;
    baseColor.b %= 255;
    baseColor.g += 20;
    baseColor.g %= 255;
    baseColor.r += 100;
    baseColor.r %= 255;
        
    LINE("module:GoalPerceptor2017:goalPostCandidates",
      gp->startInImage.x(),
      gp->startInImage.y(),
      gp->startInImage.x()+ gp->topWidth,
      gp->startInImage.y(),
      3,Drawings::solidPen,baseColor);
    LINE("module:GoalPerceptor2017:goalPostCandidates",
      gp->startInImage.x()+ gp->topWidth,
      gp->startInImage.y(),
      gp->endInImage.x(),
      gp->endInImage.y(),
      3,Drawings::solidPen,baseColor);
    LINE("module:GoalPerceptor2017:goalPostCandidates",
      gp->endInImage.x(),
      gp->endInImage.y(),
      gp->endInImage.x() - gp->topWidth,
      gp->endInImage.y(),
      3,Drawings::solidPen,baseColor);
    LINE("module:GoalPerceptor2017:goalPostCandidates",
      gp->endInImage.x() - gp->topWidth,
      gp->endInImage.y(),
      gp->startInImage.x(),
      gp->startInImage.y(),
      3,Drawings::solidPen,baseColor);
    LINE("module:GoalPerceptor2017:goalPostCandidates",
      gp->endInImage.x(),
      gp->endInImage.y(),
      gp->startInImage.x(),
      gp->startInImage.y(),
      2,Drawings::solidPen,baseColor);
    LINE("module:GoalPerceptor2017:goalPostCandidates",
      gp->startInImage.x() + gp->topWidth,
      gp->startInImage.y(),
      gp->endInImage.x() - gp->topWidth,
      gp->endInImage.y(),
      2,Drawings::solidPen,baseColor);
    DRAWTEXT("module:GoalPerceptor2017:goalPostCandidates",
      gp->startInImage.x() + gp->topWidth,
      gp->startInImage.y(),
	  10,
	  ColorRGBA::red,
	  gp->direction.angle());
  }
}

bool GoalPerceptor2017::transformImageToField(Vector2i& positionInImage, Vector2f& positionOnField)
{
  Vector2f corrected = theImageCoordinateSystem.toCorrected(positionInImage);
  return Transformation::imageToRobot(corrected, theCameraMatrix, theCameraInfo, positionOnField);
}

void GoalPerceptor2017::scanCrossbar(const int & xPos, const int & yStart, const int & yEnd)
{
	const Image &image = (Image&)theImage;

//	  const int xStep = std::max(1, imageWidth / 640); // TODO: enough?
	  const int yStep = 1;
	  Image::Pixel p = image.getFullSizePixel(yStart,xPos);
	  yBuffer.fill(theECImage.grayscaled[yStart][xPos]);
	  int lastY = getGauss(yBuffer);
	  int newY = lastY;
	  bool gradientUp = false;
	  bool gradientDown = false;
	  bool wasUp = false;
	  bool wasDown = false;
	  int gradientStart = yStart;
	  int yDiff = 0;

	  for (int yPos = yStart; yPos <= yEnd - yStep; yPos += yStep)
	  {
	//	  DOT("module:GoalPerceptor2017:goalPostCandidates",xPos,yPos,ColorRGBA::blue,ColorRGBA::blue);
	    p = image.getFullSizePixel(yPos,xPos);
	    yBuffer.push_front(theECImage.grayscaled[yPos][xPos]);
	    newY = getGauss(yBuffer);
	    yDiff = newY - lastY;
	    //cbDiff = getDiffSum(cbBuffer, p.cb - cbBuffer[0]);
	    //crDiff = getDiffSum(crBuffer, p.cr - crBuffer[0]);

	    gradientUp = (yDiff > gradientMinDiff);
	    //|| (cbDiff > gradientMinDiff)
	    //|| (crDiff > gradientMinDiff);
	    gradientDown = (yDiff < -gradientMinDiff);
	    //|| (cbDiff < -gradientMinDiff)
	    //|| (crDiff < -gradientMinDiff);
	    yBuffer.push_front(theECImage.grayscaled[yPos][xPos]);
	    //cbBuffer.add(p.cb);
	    //crBuffer.add(p.cr);
	    if ((wasUp && !gradientUp) || (wasDown && !gradientDown))
	    {
	      int length = std::max(yPos - gradientStart, 2);
	      GoalSideSpot gs;
	      gs.cb = p.cb;
	      gs.cr = p.cr;
	      gs.y = p.y;
	      gs.angle = getAngle(xPos, yPos - yStep, length, image);
	      gs.xPos = xPos - length / 2;
	      gs.yPos = yPos;
	      gs.nextSpot = NULL;
	      goalSideSpots.push_back(gs);
	    }
	    if (!wasUp && gradientUp)
	    {
	      wasUp = true;
	      gradientStart = yPos;
	    }
	    if (!wasDown && gradientDown)
	    {
	      wasDown = true;
	      gradientStart = yPos;
	    }
	    wasUp = gradientUp;
	    wasDown = gradientDown;
	  }
}


MAKE_MODULE(GoalPerceptor2017, perception)
