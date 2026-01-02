/**
* This file is part of ORB-SLAM3
*
* Ground Plane Detector for Height-based Scale Estimation
* Implementation
*/

#include "GroundPlaneDetector.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace ORB_SLAM3
{

GroundPlaneDetector::GroundPlaneDetector(float cameraHeight,
                                         int minInliers,
                                         float ransacThreshold,
                                         int maxIterations)
    : mCameraHeight(cameraHeight)
    , mMinInliers(minInliers)
    , mRansacThreshold(ransacThreshold)
    , mMaxIterations(maxIterations)
{
}

GroundPlaneDetector::Plane GroundPlaneDetector::DetectGroundPlane(
    const std::vector<MapPoint*>& mapPoints,
    const Eigen::Vector3f& cameraCenter,
    const Eigen::Vector3f& cameraYAxis)
{
    Plane bestPlane;
    bestPlane.confidence = 0.0f;
    bestPlane.inliers = 0;

    if(mapPoints.size() < 3)
    {
        return bestPlane;
    }

    // Filter ground candidate points: only points in positive Y direction of camera frame
    std::vector<Eigen::Vector3f> candidatePoints = FilterGroundCandidates(mapPoints, cameraCenter, cameraYAxis);

    if(candidatePoints.size() < 3)
    {
        return bestPlane;
    }

    // RANSAC plane fitting: only accept planes parallel to xz plane (normal parallel to camera Y-axis)
    Plane detectedPlane;
    if(FitPlaneRANSAC(candidatePoints, detectedPlane, cameraYAxis))
    {
        return detectedPlane;
    }

    return bestPlane;
}

float GroundPlaneDetector::EstimateScale(const Plane& plane,
                                        const Eigen::Vector3f& cameraCenter)
{
    if(plane.inliers < mMinInliers || plane.confidence < 0.3f)
    {
        std::cout << "Plane not valid: inliers < mMinInliers or confidence < 0.3f" << std::endl;
        return -1.0f;
    }

    // Calculate distance from camera to plane
    float slamDistance = CameraToPlaneDistance(cameraCenter, plane);

    if(slamDistance <= 0.0f || std::isnan(slamDistance) || std::isinf(slamDistance))
    {
        std::cout << "Slam distance not valid: <= 0.0f or nan or inf" << std::endl;
        return -1.0f;
    }

    // Calculate scale factor
    float scale = mCameraHeight / slamDistance;

    return scale;
}

float GroundPlaneDetector::EstimateScaleFromMapPoints(
    const std::vector<MapPoint*>& mapPoints,
    const Eigen::Vector3f& cameraCenter)
{
    // Default: assume Y-up coordinate system (world Y-axis = camera Y-axis)
    Eigen::Vector3f cameraYAxis(0.0f, 1.0f, 0.0f);
    
    // Detect ground plane
    Plane plane = DetectGroundPlane(mapPoints, cameraCenter, cameraYAxis);

    // Estimate scale from plane
    return EstimateScale(plane, cameraCenter);
}

float GroundPlaneDetector::FilterScaleMedian(const std::vector<float>& scaleFactors)
{
    if(scaleFactors.empty())
    {
        return -1.0f;
    }

    std::vector<float> validScales;
    for(float scale : scaleFactors)
    {
        if(scale > 0.0f)
        {
            validScales.push_back(scale);
        }
    }

    if(validScales.empty())
    {
        return -1.0f;
    }

    std::sort(validScales.begin(), validScales.end());
    size_t n = validScales.size();
    
    if(n % 2 == 0)
    {
        return (validScales[n/2 - 1] + validScales[n/2]) / 2.0f;
    }
    else
    {
        return validScales[n/2];
    }
}

float GroundPlaneDetector::FilterScaleMovingAverage(
    const std::vector<float>& scaleFactors,
    int windowSize)
{
    if(scaleFactors.empty())
    {
        return -1.0f;
    }

    std::vector<float> validScales;
    for(float scale : scaleFactors)
    {
        if(ValidateScale(scale) && scale > 0.0f)
        {
            validScales.push_back(scale);
        }
    }

    if(validScales.empty())
    {
        return -1.0f;
    }

    windowSize = std::min(windowSize, (int)validScales.size());
    int startIdx = std::max(0, (int)validScales.size() - windowSize);
    
    float sum = 0.0f;
    int count = 0;
    for(int i = startIdx; i < (int)validScales.size(); i++)
    {
        sum += validScales[i];
        count++;
    }

    if(count == 0)
    {
        return -1.0f;
    }

    return sum / count;
}

bool GroundPlaneDetector::ValidateScale(float scale, float minScale, float maxScale)
{
    if(scale <= 0.0f || std::isnan(scale) || std::isinf(scale))
    {
        return false;
    }

    return (scale >= minScale && scale <= maxScale);
}

bool GroundPlaneDetector::FitPlaneRANSAC(
    const std::vector<Eigen::Vector3f>& points,
    Plane& plane,
    const Eigen::Vector3f& cameraYAxis)
{
    if(points.size() < 3)
    {
        return false;
    }

    int bestInliers = 0;
    Plane bestPlane;

    // RANSAC iterations
    float degree_threshold = 3.0f;
    float cos_threshold = std::cos(degree_threshold * M_PI / 180.0f);
    for(int iter = 0; iter < mMaxIterations; iter++)
    {
        // Randomly select 3 points
        int idx1 = rand() % points.size();
        int idx2 = rand() % points.size();
        int idx3 = rand() % points.size();

        if(idx1 == idx2 || idx2 == idx3 || idx1 == idx3)
        {
            continue;
        }

        // Fit plane from 3 points
        Plane candidatePlane = FitPlaneFrom3Points(points[idx1], points[idx2], points[idx3]);

        // Validate: plane normal should be parallel to camera Y-axis (plane parallel to xz plane)
        // Ground plane should be parallel to camera's xz plane
        float dotProduct = std::abs(candidatePlane.normal.dot(cameraYAxis));
        // Allow tolerance: normal should be nearly parallel to camera Y-axis (dot product close to 1)
        // Threshold: angle < 5 degrees means dot product > 0.9962
        // cos(5°) ≈ 0.9962
        
        if(dotProduct < cos_threshold)
        {
            continue; // Skip this plane candidate (not parallel to xz plane)
        }

        // Ensure normal points in the same direction as camera Y-axis (positive Y direction)
        if(candidatePlane.normal.dot(cameraYAxis) < 0.0f)
        {
            // Flip normal to point in positive Y direction
            candidatePlane.normal = -candidatePlane.normal;
            candidatePlane.d = -candidatePlane.d;
        }

        // Count inliers
        int inliers = 0;
        for(const auto& point : points)
        {
            float dist = PointToPlaneDistance(point, candidatePlane);
            if(dist < mRansacThreshold)
            {
                inliers++;
            }
        }

        // Update best plane
        if(inliers > bestInliers)
        {
            bestInliers = inliers;
            bestPlane = candidatePlane;
            bestPlane.inliers = inliers;
            bestPlane.confidence = (float)inliers / points.size();
        }

        // Early termination if we found enough inliers
        if(bestInliers >= mMinInliers && bestPlane.confidence > 0.5f)
        {
            break;
        }
    }

    if(bestInliers < mMinInliers)
    {
        return false;
    }

    // Refine plane by removing outliers and refitting
    // Iterative refinement: remove outliers and refit plane with remaining inliers
    const int maxRefinementIterations = 30;
    Plane refinedPlane = bestPlane;
    
    for(int refineIter = 0; refineIter < maxRefinementIterations; refineIter++)
    {
        // Collect inliers based on current plane
        std::vector<Eigen::Vector3f> inlierPoints;
        for(const auto& point : points)
        {
            float dist = PointToPlaneDistance(point, refinedPlane);
            if(dist < mRansacThreshold)
            {
                inlierPoints.push_back(point);
            }
        }
        
        // Need at least 3 points to fit a plane
        if(inlierPoints.size() < 3)
        {
            break;
        }
        
        // Refit plane using all inliers (least squares)
        Plane newPlane = FitPlaneFromInliers(inlierPoints, cameraYAxis);
        
        // Validate new plane
        float dotProduct = std::abs(newPlane.normal.dot(cameraYAxis));
        if(dotProduct < cos_threshold)  // Still need to be parallel to camera Y-axis
        {
            break;  // Invalid plane, keep previous one
        }
        
        // Ensure normal points in positive Y direction
        if(newPlane.normal.dot(cameraYAxis) < 0.0f)
        {
            newPlane.normal = -newPlane.normal;
            newPlane.d = -newPlane.d;
        }
        
        // Check if improvement is significant
        int newInliers = 0;
        for(const auto& point : points)
        {
            float dist = PointToPlaneDistance(point, newPlane);
            if(dist < mRansacThreshold)
            {
                newInliers++;
            }
        }
        
        // If new plane has similar or more inliers, use it
        if(newInliers >= refinedPlane.inliers)
        {
            refinedPlane = newPlane;
            refinedPlane.inliers = newInliers;
            refinedPlane.confidence = (float)newInliers / points.size();
        }
        else
        {
            // No improvement, stop refinement
            break;
        }
    }
    
    plane = refinedPlane;
    return true;
}

GroundPlaneDetector::Plane GroundPlaneDetector::FitPlaneFrom3Points(
    const Eigen::Vector3f& p1,
    const Eigen::Vector3f& p2,
    const Eigen::Vector3f& p3)
{
    Plane plane;

    // Calculate two vectors in the plane
    Eigen::Vector3f v1 = p2 - p1;
    Eigen::Vector3f v2 = p3 - p1;

    // Normal vector = cross product
    plane.normal = v1.cross(v2);
    float norm = plane.normal.norm();

    // Check if points are collinear
    if(norm < 1e-6f)
    {
        plane.normal = Eigen::Vector3f(0, 1, 0); // Default: Y-up
        plane.d = 0.0f;
        plane.inliers = 0;
        plane.confidence = 0.0f;
        return plane;
    }

    // Normalize normal vector
    plane.normal.normalize();

    // Calculate d: ax + by + cz + d = 0
    // d = -(ax + by + cz) for any point on the plane
    plane.d = -plane.normal.dot(p1);

    plane.inliers = 0;
    plane.confidence = 0.0f;

    return plane;
}

GroundPlaneDetector::Plane GroundPlaneDetector::FitPlaneFromInliers(
    const std::vector<Eigen::Vector3f>& inlierPoints,
    const Eigen::Vector3f& cameraYAxis)
{
    Plane plane;
    
    if(inlierPoints.size() < 3)
    {
        plane.normal = Eigen::Vector3f(0, 1, 0);
        plane.d = 0.0f;
        plane.inliers = 0;
        plane.confidence = 0.0f;
        return plane;
    }
    
    // Calculate centroid of inlier points
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for(const auto& point : inlierPoints)
    {
        centroid += point;
    }
    centroid /= inlierPoints.size();
    
    // Build covariance matrix for least squares plane fitting
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for(const auto& point : inlierPoints)
    {
        Eigen::Vector3f centered = point - centroid;
        covariance += centered * centered.transpose();
    }
    covariance /= inlierPoints.size();
    
    // Find eigenvector corresponding to smallest eigenvalue (normal direction)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(covariance);
    if(eigenSolver.info() != Eigen::Success)
    {
        // Fallback to default
        plane.normal = Eigen::Vector3f(0, 1, 0);
        plane.d = 0.0f;
        plane.inliers = 0;
        plane.confidence = 0.0f;
        return plane;
    }
    
    // Smallest eigenvalue corresponds to normal direction
    Eigen::Vector3f normal = eigenSolver.eigenvectors().col(0);
    normal.normalize();
    
    // Ensure normal points in positive Y direction (same as cameraYAxis)
    if(normal.dot(cameraYAxis) < 0.0f)
    {
        normal = -normal;
    }
    
    plane.normal = normal;
    plane.d = -normal.dot(centroid);
    plane.inliers = inlierPoints.size();
    plane.confidence = 1.0f;  // All points are inliers by definition
    
    return plane;
}

float GroundPlaneDetector::PointToPlaneDistance(
    const Eigen::Vector3f& point,
    const Plane& plane)
{
    // Distance = |ax + by + cz + d| / sqrt(a^2 + b^2 + c^2)
    // Since normal is normalized, distance = |ax + by + cz + d|
    float dist = std::abs(plane.normal.dot(point) + plane.d);
    return dist;
}

float GroundPlaneDetector::CameraToPlaneDistance(
    const Eigen::Vector3f& cameraCenter,
    const Plane& plane)
{
    return PointToPlaneDistance(cameraCenter, plane);
}

std::vector<Eigen::Vector3f> GroundPlaneDetector::FilterGroundCandidates(
    const std::vector<MapPoint*>& mapPoints,
    const Eigen::Vector3f& cameraCenter,
    const Eigen::Vector3f& cameraYAxis)
{
    std::vector<Eigen::Vector3f> candidates;

    // Get camera Y coordinate in world coordinate system
    float cameraY = cameraCenter.y();

    for(MapPoint* pMP : mapPoints)
    {
        if(!pMP || pMP->isBad())
        {
            continue;
        }

        // Get world coordinates of map point
        Eigen::Vector3f pos = pMP->GetWorldPos();
        
        // Filter criteria:
        // 1. Map point's Y coordinate in world coordinates should be greater than camera's Y coordinate
        // 2. Distance should be reasonable
        if(pos.y() > cameraY+0.05)
        {
            candidates.push_back(pos);
        }
    }

    return candidates;
}

} // namespace ORB_SLAM3

