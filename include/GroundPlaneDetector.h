/**
* This file is part of ORB-SLAM3
*
* Ground Plane Detector for Height-based Scale Estimation
* Detects ground plane from map points and estimates scale factor
*/

#ifndef GROUNDPLANEDETECTOR_H
#define GROUNDPLANEDETECTOR_H

#include <vector>
#include <Eigen/Dense>

namespace ORB_SLAM3
{

class MapPoint;
class KeyFrame;

/**
 * Ground Plane Detector
 * Detects ground plane from map points and estimates scale factor
 * based on known camera height
 */
class GroundPlaneDetector
{
public:
    /**
     * Plane representation: ax + by + cz + d = 0
     * Normal vector: [a, b, c], distance: d
     */
    struct Plane
    {
        Eigen::Vector3f normal;  // Normal vector (normalized)
        float d;                 // Distance from origin
        int inliers;             // Number of inliers
        float confidence;        // Confidence score (0-1)
    };

    /**
     * Constructor
     * @param cameraHeight Known camera height from ground (meters)
     * @param minInliers Minimum number of inliers for valid plane detection
     * @param ransacThreshold Distance threshold for RANSAC (meters)
     * @param maxIterations Maximum RANSAC iterations
     */
    GroundPlaneDetector(float cameraHeight = 1.65f, 
                        int minInliers = 30,
                        float ransacThreshold = 0.1f,
                        int maxIterations = 1000);

    /**
     * Detect ground plane from map points
     * @param mapPoints Vector of map points
     * @param cameraCenter Camera center position in world coordinates
     * @param cameraYAxis Camera's positive Y-axis direction in world coordinates (normalized)
     * @return Detected plane (if successful)
     */
    Plane DetectGroundPlane(const std::vector<MapPoint*>& mapPoints,
                           const Eigen::Vector3f& cameraCenter,
                           const Eigen::Vector3f& cameraYAxis);

    /**
     * Estimate scale factor from ground plane
     * @param plane Detected ground plane
     * @param cameraCenter Camera center position
     * @return Scale factor (> 0 if successful, -1 if failed)
     */
    float EstimateScale(const Plane& plane, 
                       const Eigen::Vector3f& cameraCenter);

    /**
     * Estimate scale factor directly from map points and camera center
     * @param mapPoints Vector of map points
     * @param cameraCenter Camera center position
     * @return Scale factor (> 0 if successful, -1 if failed)
     */
    float EstimateScaleFromMapPoints(const std::vector<MapPoint*>& mapPoints,
                                    const Eigen::Vector3f& cameraCenter);

    /**
     * Filter scale factors using median filter
     * @param scaleFactors Vector of scale factors
     * @return Filtered scale factor
     */
    static float FilterScaleMedian(const std::vector<float>& scaleFactors);

    /**
     * Filter scale factors using moving average
     * @param scaleFactors Vector of scale factors
     * @param windowSize Window size for moving average
     * @return Filtered scale factor
     */
    static float FilterScaleMovingAverage(const std::vector<float>& scaleFactors,
                                         int windowSize = 5);

    /**
     * Validate scale factor
     * @param scale Scale factor to validate
     * @param minScale Minimum valid scale
     * @param maxScale Maximum valid scale
     * @return True if scale is valid
     */
    static bool ValidateScale(float scale, float minScale = 0.1f, float maxScale = 10.0f);

    /**
     * Calculate distance from point to plane
     * @param point 3D point
     * @param plane Plane
     * @return Distance
     */
    float PointToPlaneDistance(const Eigen::Vector3f& point, const Plane& plane);

    /**
     * Set camera height
     */
    void SetCameraHeight(float height) { mCameraHeight = height; }

    /**
     * Get camera height
     */
    float GetCameraHeight() const { return mCameraHeight; }

private:
    /**
     * RANSAC plane fitting
     * @param points 3D points
     * @param plane Output plane
     * @param cameraYAxis Camera's positive Y-axis direction in world coordinates (normalized)
     * @return True if plane is found
     */
    bool FitPlaneRANSAC(const std::vector<Eigen::Vector3f>& points, Plane& plane,
                       const Eigen::Vector3f& cameraYAxis);

    /**
     * Fit plane from 3 points using least squares
     * @param p1 First point
     * @param p2 Second point
     * @param p3 Third point
     * @return Plane
     */
    Plane FitPlaneFrom3Points(const Eigen::Vector3f& p1,
                              const Eigen::Vector3f& p2,
                              const Eigen::Vector3f& p3);

    /**
     * Fit plane from multiple inlier points using least squares
     * @param inlierPoints Vector of inlier points
     * @param cameraYAxis Camera's positive Y-axis direction (for validation)
     * @return Plane
     */
    Plane FitPlaneFromInliers(const std::vector<Eigen::Vector3f>& inlierPoints,
                              const Eigen::Vector3f& cameraYAxis);

    /**
     * Calculate distance from camera center to plane
     * @param cameraCenter Camera center position
     * @param plane Plane
     * @return Distance
     */
    float CameraToPlaneDistance(const Eigen::Vector3f& cameraCenter, const Plane& plane);

    /**
     * Filter map points for ground detection
     * Select points that are likely to be on the ground
     * @param mapPoints Input map points
     * @param cameraCenter Camera center position
     * @param cameraYAxis Camera's positive Y-axis direction in world coordinates (normalized)
     * @return Filtered map points
     */
    std::vector<Eigen::Vector3f> FilterGroundCandidates(
        const std::vector<MapPoint*>& mapPoints,
        const Eigen::Vector3f& cameraCenter,
        const Eigen::Vector3f& cameraYAxis);

    float mCameraHeight;      // Known camera height (meters)
    int mMinInliers;          // Minimum inliers for valid detection
    float mRansacThreshold;   // RANSAC distance threshold (meters)
    int mMaxIterations;       // Maximum RANSAC iterations
};

} // namespace ORB_SLAM3

#endif // GROUNDPLANEDETECTOR_H

