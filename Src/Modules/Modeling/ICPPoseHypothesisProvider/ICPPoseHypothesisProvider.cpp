/**
 * @file ICPPoseHypothesisProvider.cpp
 *
 * Implementation of a module that uses recent field feature obervations
 * and combines them to an alternative pose of the robot.
 *
 * @author Zeng Zhiying
 */

#include "ICPPoseHypothesisProvider.h"
#include <algorithm>
#include <iostream>

MAKE_MODULE(ICPPoseHypothesisProvider, modeling)

ICPPoseHypothesisProvider::ICPPoseHypothesisProvider()
{
	// Initialize list of relevant field lines
	for(size_t i = 0, count = theFieldDimensions.fieldLines.lines.size(); i < count; ++i)
	{
		const FieldDimensions::LinesTable::Line& fieldLine = theFieldDimensions.fieldLines.lines[i];
		if(!fieldLine.isPartOfCircle && (fieldLine.to - fieldLine.from).norm() > 300.f)
		{
			FieldLine relevantFieldLine;
			relevantFieldLine.start = fieldLine.from;
			relevantFieldLine.end = fieldLine.to;
			relevantFieldLine.dir = relevantFieldLine.end - relevantFieldLine.start;
			relevantFieldLine.dir.normalize();
			relevantFieldLine.length = (fieldLine.to - fieldLine.from).norm();
			relevantFieldLine.vertical = std::abs(fieldLine.from.y() - fieldLine.to.y()) < 0.001f;
			if(relevantFieldLine.vertical)
				verticalFieldLines.push_back(relevantFieldLine);
			else
				horizontalFieldLines.push_back(relevantFieldLine);
		}
	}
	// Initialize corner lists:
	// X
	xIntersections.push_back(Vector2f(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.centerCircleRadius));
	xIntersections.push_back(Vector2f(theFieldDimensions.xPosHalfWayLine, -theFieldDimensions.centerCircleRadius));
	// T
	tIntersections = xIntersections;
	tIntersections.push_back(Vector2f(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosRightSideline));
	tIntersections.push_back(Vector2f(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosLeftSideline));
	tIntersections.push_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea));
	tIntersections.push_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightPenaltyArea));
	tIntersections.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftPenaltyArea));
	tIntersections.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightPenaltyArea));
	// L
	lIntersections = tIntersections;
	lIntersections.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline));
	lIntersections.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline));
	lIntersections.push_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightSideline));
	lIntersections.push_back(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline));
	lIntersections.push_back(Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea));
	lIntersections.push_back(Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea));
	lIntersections.push_back(Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosRightPenaltyArea));
	lIntersections.push_back(Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea));
}

ICPPoseHypothesisProvider::~ICPPoseHypothesisProvider()
{

}



void ICPPoseHypothesisProvider::update(ICPPoseHypothesis& icpPoseHypothesis)
{
	int result = ICP_LOST;
	Vector3f covariance;
	result = localise(thePenaltyArea, theMidCircle, theMidCorner, theOuterCorner, theFieldLineIntersections, theFieldLines, thePenaltyMarkPercept, theFrameInfo,
			theRobotPose, covariance, theMotionInfo, theCameraMatrix);
	std::cout<<"result---->"<<result<<std::endl;
	std::cout<<"ICP------->"<<robotpose.translation.x()<<","<<robotpose.translation.y()<<","<<robotpose.rotation<<std::endl;
	if(result != ICP_LOST)
	{
		icpPoseHypothesis.isValid = true;
		icpPoseHypothesis.pose = robotpose;
		icpPoseHypothesis.timeOfLastIterateUpdate = theFrameInfo.time;
		icpPoseHypothesis.isInOwnHalf = robotpose.translation.x() > 0? false:true;
	}
	else
	{
		icpPoseHypothesis.isValid = false;
		icpPoseHypothesis.pose = robotpose;
		icpPoseHypothesis.timeOfLastIterateUpdate = theFrameInfo.time;
		icpPoseHypothesis.isInOwnHalf = robotpose.translation.x() > 0? false:true;
	}
}
int ICPPoseHypothesisProvider::localise(const PenaltyArea& penaltyArea, const MidCircle& midCircle, const MidCorner& midCorner, const OuterCorner& outerCorner,
		const FieldLineIntersections& intersectionsPercept, const FieldLines& linePercept, const PenaltyMarkPercept& penaltyMarkPercept, const FrameInfo frameInfo,
	 								    Pose2f robotPose, Vector3f covariance, const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix)
{
    reset();
    int result = ICP_LOST;
    bool penaltyMarkValid = false;
    robotpose = robotPose;
    if(!penaltyArea.isValid)
    {
    	std::cout<<"penaltyarea!!!"<<std::endl;
        minIterations++;
    }
    if(!midCircle.isValid)
    {
    	std::cout<<"midCircle!!!"<<std::endl;
    	minIterations++;
    }
    if(!midCorner.isValid)
	{
    	std::cout<<"midCorner!!!"<<std::endl;
		minIterations++;
	}
    if(!outerCorner.isValid)
	{
    	std::cout<<"outerCorner!!!"<<std::endl;
		minIterations++;
	}
    if(penaltyMarkPercept.timeLastSeen == frameInfo.time && frameInfo.time != 0)
	{
    	std::cout<<"penaltymark!!!"<<std::endl;
    	penaltyMarkValid = true;
		minIterations++;
	}
    if(intersectionsPercept.intersections.size() == 0)
	{
    	std::cout<<"intersection!!!"<<std::endl;
		minIterations++;
	}
    if(linePercept.lines.size() == 0)
    {
    	std::cout<<"line!!!"<<std::endl;
        minIterations++;
    }
    if(minIterations == 7)
    {
        return result;
    }
    minIterations += penaltyArea.isValid + midCircle.isValid + midCorner.isValid + outerCorner.isValid + penaltyMarkValid + linePercept.lines.size() + intersectionsPercept.intersections.size();

    if(minIterations >= 7)
    {
        minIterations = 7;
    }
	std::cout<<"minIterations---->"<<minIterations<<std::endl;
    preprocessObservation(penaltyArea, midCircle, midCorner,outerCorner,intersectionsPercept, linePercept, penaltyMarkPercept, frameInfo);
    for(int i = 0; i < sourcePoints.size(); i++)
    {
//    	std::cout<<"sourcepoint---->"<<i<<","<<sourcePoints[i].x()<<","<<sourcePoints[i].y()<<std::endl;
//    	std::cout<<"targetpoint---->"<<i<<","<<targetPoints[i].x()<<","<<targetPoints[i].y()<<std::endl;
    }
    associateFeatures(1, motionInfo, cameraMatrix);
    solve(motionInfo, cameraMatrix);

    if(mse < LOCALISED && fabs(robotpose.translation.x()) < 5000 && fabs(robotpose.translation.y() < 3500))
    {
        covariance = cov;
        result = N;
    }
    return result;
}

void ICPPoseHypothesisProvider::reset(void)
{
    iteration = 0;
    mse = 0.f;
    N = 0;
    penaltyAreas.clear();
    midCircles.clear();
    midCorners.clear();
    outerCorners.clear();
    lines.clear();
    penaltymarks.clear();
    intersections.clear();
    minIterations = 0;
}

void ICPPoseHypothesisProvider::preprocessObservation(const PenaltyArea& penaltyArea, const MidCircle& midCircle, const MidCorner& midCorner, const OuterCorner& outerCorner,
		const FieldLineIntersections& intersectionsPercept, const FieldLines& linePercept, const PenaltyMarkPercept& penaltyMarkPercept, const FrameInfo frameInfo)
{
    if(penaltyArea.isValid)
    	penaltyAreas.push_back(penaltyArea);
    if(midCircle.isValid)
    	midCircles.push_back(midCircle);
    if(midCorner.isValid)
    	midCorners.push_back(midCorner);
    if(outerCorner.isValid)
    	outerCorners.push_back(outerCorner);
    Vector2i parallelLineIndex = Vector2i(-1,-1);
    for(int i = 0; i < linePercept.lines.size(); ++i)
    {
		lines.push_back(linePercept.lines[i]);
    }
    if(penaltyMarkPercept.timeLastSeen == frameInfo.time && frameInfo.time != 0)
    {
    	const Vector2f penaltyMarkInWorld = theRobotPose * penaltyMarkPercept.positionOnField;
		Vector2f associatedPenaltyMark = penaltyMarkInWorld.x() <= 0.f ? Vector2f(-3200.f, 0.f) : Vector2f(3200.f, 0.f);
		const float differenceBetweenPerceptAndModel = (penaltyMarkInWorld - associatedPenaltyMark).norm();
		if(differenceBetweenPerceptAndModel < 1500.f)
			penaltymarks.push_back(penaltyMarkPercept.positionOnField);
    }
    for(int i = 0; i < intersectionsPercept.intersections.size(); ++i)
    {
        intersections.push_back(intersectionsPercept.intersections[i]);
    }
}

void ICPPoseHypothesisProvider::associateFeatures(int association, const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix)
{
    /* Matching priority as follows:
   Priority 1: PenaltyArea
   Priority 2: MidCircle
   Priority 3: MidCorner
   Priority 4: OuterConner
   Priority 5: PenaltyMark
   Priority 6: Intersections
   Priority 7: Lines
   */
    sourcePoints.clear();
    targetPoints.clear();
    weights.clear();
    distances.clear();
    targetType.clear();
    N = 0;
    x_known = false;
    y_known = false;
    z_known = false;
    nonLineFeature = false;
    cov.x() = 100.f;
    cov.y() = 100.f;
    cov.z() = 100.f;

    int thisAssociation = 0;

    // Priority 1: PenaltyArea

	if(!penaltyAreas.empty())
	{
		Vector2f source = robotpose * penaltyAreas[0].translation;
		Vector2f target = penaltyAreas[0].getGlobalFeaturePosition().translation;
		float distance = (source - target).norm();
		float weight = 1.f / (penaltyAreas[0].translation.norm() / 1000.f);
		addToPointCloud(source, target, distance, weight, POINT);
		Vector2f center = penaltyAreas[0].translation;
		Matrix2f covar = getCovOfPointInWorld(center, 0.f, motionInfo, cameraMatrix);
		if(cov.x() > covar(0,0))
		{
			cov.x() = covar(0,0);
		}
		if(cov.y() > covar(1,1))
		{
			cov.y() = covar(1,1);
		}
		nonLineFeature = true;
		x_known = true;
		y_known = true;
	}
	if(++thisAssociation == association)
	{
		return;
	}

    //Priority 2: MidCircles
    if(!midCircles.empty())
    {
        Vector2f source = robotpose * midCircles[0].translation;
        Vector2f target = midCircles[0].getGlobalFeaturePosition().translation;
        float distance = (source - target).norm();
        float weight = 1.f / (midCircles[0].translation.norm() / 1000);
        addToPointCloud(source, target, distance, weight, POINT);
        Vector2f center = midCircles[0].translation;
        Matrix2f circleCov = getCovOfCircle(center, 750.f, motionInfo, cameraMatrix);
        if(cov.x() > circleCov(0,0))
        {
            cov.x() = circleCov(0,0);
        }
        if(cov.y() > circleCov(1,1))
        {
            cov.y() = circleCov(1,1);
        }

        nonLineFeature = true;
    }
    if(++thisAssociation == association)
    {
        return;
    }

    //Priority 3: MidCorners
    if(!midCorners.empty())
	{
		Vector2f source = robotpose * midCorners[0].translation;
		Vector2f target = midCorners[0].getGlobalFeaturePosition().translation;
		float distance = (source - target).norm();
		float weight = 1.f / (midCorners[0].translation.norm() / 1000.f);
		addToPointCloud(source, target, distance, weight, POINT);
		Vector2f center = midCorners[0].translation;
		Matrix2f covar = getCovOfPointInWorld(center, 0.f, motionInfo, cameraMatrix);
		if(cov.x() > covar(0,0))
		{
			cov.x() = covar(0,0);
		}
		if(cov.y() > covar(1,1))
		{
			cov.y() = covar(1,1);
		}
		nonLineFeature = true;
		x_known = true;
		y_known = true;
	}
	if(++thisAssociation == association)
	{
		return;
	}

	//Priority 4: OuterCorners
	if(!outerCorners.empty())
	{
		Vector2f source = robotpose * outerCorners[0].translation;
		Vector2f target = outerCorners[0].getGlobalFeaturePosition().translation;
		float distance = (source - target).norm();
		float weight = 1.f / (outerCorners[0].translation.norm() / 1000.f);
		addToPointCloud(source, target, distance, weight, POINT);
		Vector2f center = outerCorners[0].translation;
		Matrix2f covar = getCovOfPointInWorld(center, 0.f, motionInfo, cameraMatrix);
		if(cov.x() > covar(0,0))
		{
			cov.x() = covar(0,0);
		}
		if(cov.y() > covar(1,1))
		{
			cov.y() = covar(1,1);
		}
		nonLineFeature = true;
		x_known = true;
		y_known = true;
	}
	if(++thisAssociation == association)
	{
		return;
	}

    //Priority 5: Penalty Mark
    if(!penaltymarks.empty())
    {
        Vector2f source = robotpose * penaltymarks[0];
        Vector2f target;
        if(source.x() > 0)
        {
            target = Vector2f(3200, 0);
        }
        else
        {
            target = Vector2f(-3200, 0);
        }
        float distance = (source - target).norm();
        float weight = 1.f / (penaltymarks[0].norm() / 1000);
        addToPointCloud(source, target, distance, weight, POINT);
        Vector2f center = penaltymarks[0];
        Matrix2f covar = getCovOfPointInWorld(center, 0.f, motionInfo, cameraMatrix);

        if(cov.x() > covar(0,0))
        {
            cov.x() = covar(0,0);
        }
        if(cov.y() > covar(1,1))
        {
            cov.y() = covar(1,1);
        }

        nonLineFeature = true;
        x_known = true;
        y_known = true;
    }
    if(++thisAssociation == association)
    {
        return;
    }

//    Priority 6: Intersections
    if(!intersections.empty())
    {
        for(int i = 0; i < intersections.size(); ++i)
        {
            Vector2i associatedLine;
            Vector2f source = robotpose * intersections[i].pos;
            Vector2f target;
            if(getAssociatedIntersection(robotpose, intersections[i], target))
            {
                ;
            }
            else
            {
                getCloestCorner(robotpose, intersections[i], target);
            }
            float distance = (source - target).norm();
            float weight = 1.f / (intersections[i].pos.norm() / 1000);
            addToPointCloud(source, target, distance, weight, POINT);
            Vector2f center = intersections[i].pos;
            Matrix2f covar = getCovOfPointInWorld(center, 0.f, motionInfo, cameraMatrix);

            if(cov.x() > covar(0,0))
            {
                cov.x() = covar(0,0);
            }
            if(cov.y() > covar(1,1))
            {
                cov.y() = covar(1,1);
            }
            nonLineFeature = true;
            x_known = true;
            y_known = true;
        }
    }
    if(++thisAssociation == association)
    {
        return;
    }

//    Priority 7: Lines
    if(!lines.empty())
    {
//        std::cout<<"size--->"<<lines.size()<<std::endl;
        for(int i = 0; i < lines.size(); ++i)
        {
//            std::cout<<"i---->"<<i<<std::endl;
            Vector2f source1 = robotpose * lines[i].first;
            Vector2f source2 = robotpose * lines[i].last;
            Vector2f target1, target2;
            const FieldLine* AssociatedLine = getPointerToAssociatedLine(robotpose,lines[i].first, lines[i].last);
//            std::cout<<"index--->"<<index<<std::endl;
            if(AssociatedLine != 0)
            {
                const FieldLine fieldLine = *AssociatedLine;
                float length1 = (source1.x() - fieldLine.start.x()) *fieldLine.dir.x() + (source1.y() - fieldLine.start.y()) * fieldLine.dir.y();
                target1.x() = static_cast<float>(fieldLine.start.x())+static_cast<float>(fieldLine.dir.x()*length1);
                target1.y() = static_cast<float>(fieldLine.start.y())+static_cast<float>(fieldLine.dir.y()*length1);
                float lineLength = (source1-source2).norm();
                float length2 = (source2.x() - fieldLine.start.x()) *fieldLine.dir.x() + (source2.y() - fieldLine.start.y()) * fieldLine.dir.y();
                target2.x() = static_cast<float>(fieldLine.start.x())+static_cast<float>(fieldLine.dir.x()*length2);
                target2.y() = static_cast<float>(fieldLine.start.y())+static_cast<float>(fieldLine.dir.y()*length2);
                Vector2f dir ;
                dir.x() = static_cast<float>(target2.x() - target1.x());
                dir.y() = static_cast<float>(target2.y() - target1.y());
                dir.x() /= dir.norm();
                dir.y() /= dir.norm();
                if(dir.x() * fieldLine.dir.x() + dir.y() * fieldLine.dir.y() > 0)
                {
                    if((target1.x() - fieldLine.start.x()) * (target1.x() - fieldLine.end.x()) + (target1.y() - fieldLine.start.y()) * (target1.y() - fieldLine.end.y()) > 0)
                    {
                        target1.x() = fieldLine.start.x();
                        target1.y() = fieldLine.start.y();
                        target2.x() = fieldLine.start.x() + fieldLine.dir.x() * lineLength;
                        target2.y() = fieldLine.start.y() + fieldLine.dir.y() * lineLength;
                    }
                    else if((target2.x() - fieldLine.start.x()) * (target2.x() - fieldLine.end.x()) + (target2.y() - fieldLine.start.y()) * (target2.y() - fieldLine.end.y()) > 0)
                    {
                        target2.x() = fieldLine.end.x();
                        target2.y() = fieldLine.end.y();
                        target1.x() = fieldLine.end.x() - fieldLine.dir.x() * lineLength;
                        target1.y() = fieldLine.end.y() - fieldLine.dir.y() * lineLength;
                    }
                    else
                    {
                        target2.x() = target1.x() + fieldLine.dir.x() * lineLength;
                        target2.y() = target1.y() + fieldLine.dir.y() * lineLength;
                    }
                }
                if(dir.x() * fieldLine.dir.x() + dir.y() * fieldLine.dir.y() < 0)
                {
                    if((target1.x() - fieldLine.start.x()) * (target1.x() - fieldLine.end.x()) + (target1.y() - fieldLine.start.y()) * (target1.y() - fieldLine.end.y()) > 0)
                    {
                        target1.x() = fieldLine.end.x();
                        target1.y() = fieldLine.end.y();
                        target2.x() = fieldLine.end.x() - fieldLine.dir.x() * lineLength;
                        target2.y() = fieldLine.end.y() - fieldLine.dir.y() * lineLength;
                    }
                    else if((target2.x() - fieldLine.start.x()) * (target2.x() - fieldLine.end.x()) + (target2.y() - fieldLine.start.y()) * (target2.y() - fieldLine.end.y()) > 0)
                    {
                        target2.x() = fieldLine.start.x();
                        target2.y() = fieldLine.start.y();
                        target1.x() = fieldLine.start.x() + fieldLine.dir.x() * lineLength;
                        target1.y() = fieldLine.start.y() + fieldLine.dir.y() * lineLength;
                    }
                    else
                    {
                        target2.x() = target1.x() - fieldLine.dir.x() * lineLength;
                        target2.y() = target1.y() - fieldLine.dir.y() * lineLength;
                    }
                }
//            std::cout<<"ok1--->"<<std::endl;
            dir = lines[i].last - lines[i].first;
            dir.normalize();
//            std::cout<<"ok2--->"<<std::endl;
            float l = (0 - lines[i].first.x()) * dir.x() + (0 - lines[i].first.y()) * dir.y();
            Vector2f projection1 = lines[i].first + dir * l;
            float distance1 = (source1 - target1).norm();
            float distance2 = (source2 - target2).norm();
            float weight = 1.f / (projection1.norm() / 1000);
            TargetType type = fieldLine.vertical? VERT_LINE:HOR_LINE;
            if(type == VERT_LINE)
            {
                y_known = true;
            }
            else
            {
                x_known = true;
            }
            addToPointCloud(source1, target1, distance1, weight, type);
            addToPointCloud(source2, target2, distance2, weight, type);
            Vector2f first = lines[i].first;
            Vector2f last = lines[i].last;
            Vector2f center = (first + last) / 2;
            Matrix2f covar = getCovOfPointInWorld(center, 0.f, motionInfo, cameraMatrix);
            float angleVariance = sqr(atan(sqrt(4.f * covar(0,0) / (first - last).squaredNorm())));
            if(cov.x() > covar(0,0) && !VERT_LINE)
            {
                cov.x() = covar(0,0);
            }
            if(cov.y() > covar(1,1) && VERT_LINE)
            {
                cov.y() = covar(1,1);
            }
            if(cov.z() > angleVariance)
            {
                cov.z() = angleVariance;
            }
           }
        }
    }
    if(++thisAssociation == association)
    {
        return;
    }
    return;
}

void ICPPoseHypothesisProvider::addToPointCloud(Vector2f source, Vector2f target, float dist, float weight, TargetType type)
{
    if(source.x() == target.y() && source.x() == target.y())
    {
        source.x() += 1;
        source.y() += 1;
    }
    sourcePoints.push_back(source);
    targetPoints.push_back(target);
    targetType.push_back(type);
    distances.push_back(dist);
    weights.push_back(weight);
    N = (int)sourcePoints.size();
}

void ICPPoseHypothesisProvider::solve(const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix)
{
    calcMSE();
    iteration = 1;
    bool convergence = false;
    Pose2f bestPose;
    float bestMSE = 3.0e+20;
    while(true)
    {
        iterate();
//        std::cout<<"iteration--->"<<iteration<<std::endl;
        associateFeatures(iteration + 1, motionInfo, cameraMatrix);

        float old_mse = mse;
        calcMSE();
        if(iteration >= minIterations && mse < bestMSE)
        {
            bestMSE = mse;
            bestPose = robotpose;
        }
        if(iteration > minIterations && old_mse - mse < IMPROVEMENT_THRESHOLD)
        {
            if(convergence)
            {
                break;
            }
            else
            {
                convergence = true;
            }
        }
        else
        {
            convergence = false;
        }
        if(iteration >= MAX_ITERATIONS)
        {
            break;
        }
        iteration ++;
    }

    robotpose = bestPose;
    mse = bestMSE;
}

void ICPPoseHypothesisProvider::calcMSE(void)
{
    mse = 0.f;
    float weight_sum = 0.f;
    for(int i = 0; i < N; i++)
    {
        float dist = distances[i];
        mse += (dist * dist) * weights[i];
        weight_sum += weights[i];
    }
    if(N > 0)
    {
        mse /= ((float)N * weight_sum);
    }
}

void ICPPoseHypothesisProvider::iterate(void)
{
    std::cout<<"N------->"<<N<<std::endl;
    for(int i = 0; i < sourcePoints.size(); i++)
    {
    	std::cout<<"source----->"<<i<<","<<sourcePoints[i].x()<<std::endl;
    }
    if(N == 0)
    {
        return;
    }

    if(N == 1)
    {
//        if(distances[0] > 1000)
//        {
//            return;
//        }
//        else if((1 / weights[0] * 1000) > 700)
        {
            float headingDiff = atan2(robotpose.translation.y() - targetPoints[0].y(), robotpose.translation.x() - targetPoints[0].x())
                               -atan2(robotpose.translation.y() - sourcePoints[0].y(), robotpose.translation.x() - sourcePoints[0].x());
//            std::cout<<"-----------------------iterat.normalize.start"<<std::endl;
            Vector2f dir = targetPoints[0] - robotpose.translation;
            dir.normalize();
//            std::cout<<"-----------------------iterat.normalize.end"<<std::endl;
            float dist = (sourcePoints[0] - robotpose.translation).norm();
            sourcePoints[0] = robotpose.translation + dir * dist;
            robotpose.rotation += headingDiff;
            robotpose.rotation.normalize();
        }
    }
    Vector2f source_cen = Vector2f(0,0);
    Vector2f target_cen = Vector2f(0,0);
    for(int i = 0; i < N; i++)
    {
        source_cen += sourcePoints[i];
        target_cen += targetPoints[i];
    }
    source_cen /= N;
    target_cen /= N;

    const int m = N * 3;
    MatrixXf A(m,3);
    MatrixXf b(m,1);
    Vector3f result;
    int Ar = 0, Ac = 0, br = 0;
    return;
    for(int i = 0; i < N; i++)
    {
        float w = weights[i];
        Vector2f p = sourcePoints[i] - source_cen;
        Vector2f q = targetPoints[i] - target_cen;
        TargetType type = targetType[i];

        if(type == POINT)
        {
            //Matrix for matching a point to a point
            A.coeffRef(Ar, Ac++) = w;
            A.coeffRef(Ar, Ac++) = 0;
            A.coeffRef(Ar++, Ac) = -w * p.y();
            Ac = 0;

            A.coeffRef(Ar, Ac++) = 0;
            A.coeffRef(Ar, Ac++) = w;
            A.coeffRef(Ar++, Ac) = w * p.x();
            Ac = 0;

            A.coeffRef(Ar, Ac++) = -w * p.y();
            A.coeffRef(Ar, Ac++) = w * p.x();
            A.coeffRef(Ar++, Ac) = w * (p.x() * p.x() + p.y() * p.y());
            Ac = 0;

            b.coeffRef(br++) = w * (q.x() - p.x());
            b.coeffRef(br++) = w * (q.y() - p.y());
            b.coeffRef(br++) = w * (p.x() * (q.y() - p.y()) - p.y() * (q.x() - p.x()));
        }
        else if(type == VERT_LINE)
        {
            A.coeffRef(Ar, Ac++) = 0;
            A.coeffRef(Ar, Ac++) = 0;
            A.coeffRef(Ar++, Ac) = 0;
            Ac = 0;

            A.coeffRef(Ar, Ac++) = 0;
            A.coeffRef(Ar, Ac++) = w;
            A.coeffRef(Ar++, Ac) = w * p.x();
            Ac = 0;

            A.coeffRef(Ar, Ac++) = 0;
            A.coeffRef(Ar, Ac++) = w * p.x();
            A.coeffRef(Ar++, Ac) = w * (p.x() * p.x());
            Ac = 0;

            b.coeffRef(br++) = 0;
            b.coeffRef(br++) = w * (q.y() - p.y());
            b.coeffRef(br++) = w * (p.x() * (q.y() - p.y()));
        }
        else if(type == HOR_LINE)
        {
            //Matrix for matching a point to a vertical line
            A.coeffRef(Ar, Ac++) = w;
            A.coeffRef(Ar, Ac++) = 0;
            A.coeffRef(Ar++, Ac) = -w * p.y();
            Ac = 0;

            A.coeffRef(Ar, Ac++) = 0;
            A.coeffRef(Ar, Ac++) = 0;
            A.coeffRef(Ar++, Ac) = 0;
            Ac = 0;

            A.coeffRef(Ar, Ac++) = -w * p.y();
            A.coeffRef(Ar, Ac++) = 0;
            A.coeffRef(Ar++, Ac) = w * (p.y() * p.y());
            Ac = 0;

            b.coeffRef(br++) = w * (q.x() - p.x());
            b.coeffRef(br++) = 0;
            b.coeffRef(br++) = w * (- p.y() * (q.x() - p.x()));
            //Matrix for matching a point to a horizontal line

        }
    }

    result = A.colPivHouseholderQr().solve(b);

    Vector2f oldrobot = Vector2f(robotpose.translation.x(), robotpose.translation.y());
    Vector2f oldrobot_to_cen = oldrobot - source_cen;
    Vector2f newrobot = transform(oldrobot_to_cen, result.x(), result.y(), result.z()) + target_cen;

    Vector2f oldsrc = sourcePoints[0];
    Vector2f oldsrc_to_cen = oldsrc - source_cen;
    Vector2f newsrc = transform(oldsrc_to_cen, result.x(), result.y(), result.z()) + target_cen;

    float robotHeadingChg = atan2(newrobot.y() - newsrc.y(), newrobot.x() - newsrc.x())
                          - atan2(oldrobot.y() - oldsrc.y(), oldrobot.x() - oldsrc.x());
    Angle rotation = robotpose.rotation + robotHeadingChg;
    rotation.normalize();

    robotpose = Pose2f(rotation,newrobot.x(), newrobot.y());
}

Vector2f ICPPoseHypothesisProvider::transform(const Vector2f& point, float tx, float ty, float theta)
{
    Vector2f result;
    result.x() = cos(theta) * point.x() - sin(theta) * point.y() + tx;
    result.y() = sin(theta) * point.x() + cos(theta) * point.y() + ty;
    return result;
}


Matrix2f ICPPoseHypothesisProvider::getCovOfPointInWorld(const Vector2f& pointInWorld2, float pointZInWorld, const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix) const
{
  Vector3f unscaledVectorToPoint = cameraMatrix.inverse() * Vector3f(pointInWorld2.x(), pointInWorld2.y(), pointZInWorld);
  const Vector3f unscaledWorld = cameraMatrix.rotation * unscaledVectorToPoint;
  const float h = cameraMatrix.translation.z() - pointZInWorld;
  const float scale = h / -unscaledWorld.z();
  Vector2f pointInWorld(unscaledWorld.x() * scale, unscaledWorld.y() * scale);
  const float distance = pointInWorld.norm();
  Matrix2f rot;
  Vector2f cossin = pointInWorld * (1.f / distance);
  if(distance == 0.f)
    rot = Matrix2f::Identity();
  else
    rot << cossin.x(), -cossin.y(), cossin.y(), cossin.x();
  const Vector2f& robotRotationDeviation = motionInfo.motion == MotionRequest::stand ||
      (motionInfo.motion == MotionRequest::specialAction && motionInfo.specialActionRequest.specialAction == SpecialActionRequest::standHigh)
      ? robotRotationDeviationInStand : robotRotationDeviation;
  Matrix2f cov;
  cov << sqr(h / tan((distance == 0.f ? pi_2 : atan(h / distance)) - robotRotationDeviation.x()) - distance), 0.f,
      0.f, sqr(tan(robotRotationDeviation.y()) * distance);
  return rot * cov * rot.transpose();
}

Matrix2f ICPPoseHypothesisProvider::getCovOfCircle(const Vector2f& circlePos, float centerCircleRadius, const MotionInfo& motionInfo, const CameraMatrix& cameraMatrix) const
{
  float circleDistance = circlePos.norm();
  Vector2f increasedCirclePos = circlePos;
  if(circleDistance < centerCircleRadius * 2.f)
  {
    if(circleDistance < 10.f)
      increasedCirclePos = Vector2f(centerCircleRadius * 2, 0.f);
    else
      increasedCirclePos *= centerCircleRadius * 2.f / circleDistance;
  }
  return getCovOfPointInWorld(increasedCirclePos, 0.f, motionInfo, cameraMatrix);
}

bool ICPPoseHypothesisProvider::getAssociatedIntersection(const Pose2f& robotPose, const FieldLineIntersections::Intersection& intersection, Vector2f& associatedIntersection) const
{
  const std::vector< Vector2f >* corners = &lIntersections;
  if(intersection.type == FieldLineIntersections::Intersection::T)
    corners = &tIntersections;
  else if(intersection.type == FieldLineIntersections::Intersection::X)
    corners = &xIntersections;
  const Vector2f pointWorld = robotPose * intersection.pos;
  const float sqrThresh = intersectionAssociationDistance;
  for(unsigned int i = 0; i < corners->size(); ++i)
  {
    const Vector2f& c = corners->at(i);
    // simple implementation for testing:
    if((pointWorld - c).norm() < sqrThresh)
    {
      associatedIntersection = c;
      return true;
    }
  }
  return false;
}

void ICPPoseHypothesisProvider::getCloestCorner(const Pose2f& robotPose, const FieldLineIntersections::Intersection& intersection, Vector2f& associatedCorner) const
{
    const std::vector< Vector2f >* corners = &lIntersections;
    if(intersection.type == FieldLineIntersections::Intersection::T)
  {
    corners = &tIntersections;
  }
  else if(intersection.type == FieldLineIntersections::Intersection::X)
  {
    corners = &xIntersections;
  }
  const Vector2f pointWorld = robotPose * intersection.pos;
  float minDistance = 10000.f;
  int minIndex = -1;

  for(unsigned int i=0; i < corners->size(); ++i)
  {
    const Vector2f& c = corners->at(i);
    // simple implementation for testing:
    if((pointWorld - c).norm() < minDistance)
    {
        minDistance = (pointWorld - c).norm();
        minIndex = i;
    }
  }
  associatedCorner = corners->at(minIndex);
}

const ICPPoseHypothesisProvider::FieldLine* ICPPoseHypothesisProvider::getPointerToAssociatedLine(const Pose2f robotPose, const Vector2f& start, const Vector2f& end) const
{
  const Vector2f startOnField = robotPose * start;
  const Vector2f endOnField = robotPose * end;
  Vector2f dirOnField = endOnField - startOnField;
  dirOnField.normalize();
  Vector2f orthogonalOnField(dirOnField.y(), -dirOnField.x());
  const float lineLength = (start - end).norm();
  // Throw away short lines on center circle (as they are currently not in the field lines)
  if(lineLength < 500.f)
  {
    const float centerCircleCorridorInner = theFieldDimensions.centerCircleRadius * 0.8f;
    const float centerCircleCorridorOuter = theFieldDimensions.centerCircleRadius * 1.2f;
    const float distStart = startOnField.norm();
    const float distEnd = endOnField.norm();
    if(distStart > centerCircleCorridorInner && distStart < centerCircleCorridorOuter &&
       distEnd > centerCircleCorridorInner && distEnd < centerCircleCorridorOuter)
      return 0;
  }
  const float sqrLineAssociationCorridor = sqr(lineAssociationCorridor);
  Vector2f intersection;
  bool isVertical = std::abs(dirOnField.x()) > std::abs(dirOnField.y());
  const std::vector<FieldLine>& fieldLines = isVertical ? verticalFieldLines : horizontalFieldLines;

  for(unsigned int i = 0; i < fieldLines.size(); ++i)
  {
    const FieldLine& fieldLine = fieldLines[i];
    if(getSqrDistanceToLine(fieldLine.start, fieldLine.dir, fieldLine.length, startOnField) > sqrLineAssociationCorridor ||
       getSqrDistanceToLine(fieldLine.start, fieldLine.dir, fieldLine.length, endOnField) > sqrLineAssociationCorridor)
      continue;
    if(!intersectLineWithLine(startOnField, orthogonalOnField, fieldLine.start, fieldLine.dir, intersection))
      continue;
    if(getSqrDistanceToLine(startOnField, dirOnField, intersection) > sqrLineAssociationCorridor)
      continue;
    if(!intersectLineWithLine(endOnField, orthogonalOnField, fieldLine.start, fieldLine.dir, intersection))
      continue;
    if(getSqrDistanceToLine(startOnField, dirOnField, intersection) > sqrLineAssociationCorridor)
      continue;
    return &fieldLines[i];
  }
  return 0;
}

float ICPPoseHypothesisProvider::getSqrDistanceToLine(const Vector2f& base, const Vector2f& dir, float length, const Vector2f& point) const
{
  float l = (point.x() - base.x()) * dir.x() + (point.y() - base.y()) * dir.y();
  if(l < 0)
    l = 0;
  else if(l > length)
    l = length;
  return ((base + dir * l) - point).squaredNorm();
}

bool ICPPoseHypothesisProvider::intersectLineWithLine(const Vector2f& lineBase1, const Vector2f& lineDir1,
    const Vector2f& lineBase2, const Vector2f& lineDir2, Vector2f& intersection) const
{
  const float h = lineDir1.x() * lineDir2.y() - lineDir1.y() * lineDir2.x();
  if(h == 0.f)
    return false;
  float scale = ((lineBase2.x() - lineBase1.x()) * lineDir1.y() - (lineBase2.y() - lineBase1.y()) * lineDir1.x()) / h;
  intersection.x() = lineBase2.x() + lineDir2.x() * scale;
  intersection.y() = lineBase2.y() + lineDir2.y() * scale;
  return true;
}

float ICPPoseHypothesisProvider::getSqrDistanceToLine(const Vector2f& base, const Vector2f& dir, const Vector2f& point) const
{
  const float l = (point.x() - base.x()) * dir.x() + (point.y() - base.y()) * dir.y();
  return ((base + dir * l) - point).squaredNorm();
}
