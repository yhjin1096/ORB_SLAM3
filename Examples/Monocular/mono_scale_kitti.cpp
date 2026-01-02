/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<signal.h>

#include<opencv2/core/core.hpp>
#include<pangolin/pangolin.h>

#include"System.h"
#include"KeyFrame.h"
#include"GroundPlaneDetector.h"
#include"MapPoint.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

// Collect map points observed in keyframes from startIdx to endIdx
vector<ORB_SLAM3::MapPoint*> CollectLocalMapPoints(
    const vector<ORB_SLAM3::KeyFrame*>& allKeyFrames,
    int startIdx, int endIdx);

// Ground plane information for visualization
struct GroundPlaneInfo
{
    vector<Eigen::Vector3f> groundPoints;
    ORB_SLAM3::GroundPlaneDetector::Plane plane;
    bool isValid;
};

// Find ground points and plane for a keyframe using local map points
GroundPlaneInfo FindGroundPoints(
    ORB_SLAM3::KeyFrame* pKF,
    const vector<ORB_SLAM3::MapPoint*>& localMapPoints,
    ORB_SLAM3::GroundPlaneDetector& groundDetector);

// Draw coordinate axes
void DrawCoordinateAxes();

// Draw all map points
void DrawMapPoints(const vector<ORB_SLAM3::MapPoint*>& mapPoints);

// Draw all keyframe centers and trajectory
void DrawKeyFrames(const vector<ORB_SLAM3::KeyFrame*>& keyFrames);

// Draw ground points
void DrawGroundPoints(const vector<Eigen::Vector3f>& groundPoints);

// Draw ground plane
void DrawGroundPlane(const ORB_SLAM3::GroundPlaneDetector::Plane& plane,
                     const vector<Eigen::Vector3f>& groundPoints,
                     const Eigen::Vector3f& keyFrameCenter);

// Highlight current keyframe center
void DrawCurrentKeyFrame(ORB_SLAM3::KeyFrame* pKF);

// Visualize SLAM with ground points navigation
void VisualizeSLAMWithNavigation(
    const vector<ORB_SLAM3::KeyFrame*>& allKeyFrames,
    const vector<ORB_SLAM3::MapPoint*>& allMapPoints,
    const vector<GroundPlaneInfo>& groundPlaneInfoPerKeyFrame);

// Signal handler를 위한 전역 변수
ORB_SLAM3::System* pSLAM = nullptr;
bool b_continue_session = true;

void exit_loop_handler(int s){
    cout << endl << "Interrupt signal received. Saving trajectory and shutting down..." << endl;
    b_continue_session = false;
}

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
    pSLAM = &SLAM;
    float imageScale = SLAM.GetImageScale();

    // Signal handler 등록 (Ctrl+C 처리)
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    double t_resize = 0.f;
    double t_track = 0.f;

    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Ctrl+C 체크
        if(!b_continue_session)
            break;

        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],cv::IMREAD_UNCHANGED); //,cv::IMREAD_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe,vector<ORB_SLAM3::IMU::Point>(), vstrImageFilenames[ni]);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Get all keyframes and map points
    vector<ORB_SLAM3::KeyFrame*> allKeyFrames = SLAM.GetAllKeyFrames();
    vector<ORB_SLAM3::MapPoint*> allMapPoints = SLAM.GetAllMapPoints();
    cout << "Total keyframes: " << allKeyFrames.size() << endl;
    cout << "Total map points: " << allMapPoints.size() << endl;

    // Ground plane detector initialization (KITTI camera height: 1.65m)
    ORB_SLAM3::GroundPlaneDetector groundDetector(1.65f, 30, 0.1f, 1000);
    
    // Find ground points and planes for each keyframe
    vector<GroundPlaneInfo> groundPlaneInfoPerKeyFrame;
    const int maxPreviousKeyFrames = 20;
    
    cout << endl << "Finding ground points for each keyframe..." << endl;
    for(size_t i = 0; i < allKeyFrames.size(); i++)
    {
        ORB_SLAM3::KeyFrame* pKF = allKeyFrames[i];
        if(pKF->isBad())
        {
            GroundPlaneInfo info;
            info.isValid = false;
            groundPlaneInfoPerKeyFrame.push_back(info);
            continue;
        }

        // Determine the range of keyframes to consider (max 20 previous keyframes)
        int startIdx = std::max(0, (int)i - maxPreviousKeyFrames);
        
        // Collect local map points
        vector<ORB_SLAM3::MapPoint*> localMapPoints = 
            CollectLocalMapPoints(allKeyFrames, startIdx, i);
        
        if(localMapPoints.size() < 30)
        {
            GroundPlaneInfo info;
            info.isValid = false;
            groundPlaneInfoPerKeyFrame.push_back(info);
            continue;
        }

        // Find ground points and plane
        GroundPlaneInfo info = FindGroundPoints(pKF, localMapPoints, groundDetector);
        groundPlaneInfoPerKeyFrame.push_back(info);
        
        // cout << "KeyFrame " << i << ": Found " << info.groundPoints.size() 
        //      << " ground points (range: " << startIdx << "~" << i << ")" << endl;
    }

    // Visualize with navigation
    VisualizeSLAMWithNavigation(allKeyFrames, allMapPoints, groundPlaneInfoPerKeyFrame);

    // Calculate scale factors from ground planes
    cout << endl << "Calculating scale factors from ground planes..." << endl;
    vector<float> scaleFactors;
    
    for(size_t i = 0; i < allKeyFrames.size(); i++)
    {
        ORB_SLAM3::KeyFrame* pKF = allKeyFrames[i];
        if(pKF->isBad())
        {
            continue;
        }
        
        if(i >= groundPlaneInfoPerKeyFrame.size() || !groundPlaneInfoPerKeyFrame[i].isValid)
        {
            continue;
        }
        
        const GroundPlaneInfo& info = groundPlaneInfoPerKeyFrame[i];
        Eigen::Vector3f cameraCenter = pKF->GetCameraCenter();
        
        // Estimate scale from ground plane
        float scale = groundDetector.EstimateScale(info.plane, cameraCenter);
        std::cout << "Scale: " << scale << std::endl;
        if(scale > 0.0f)
        {
            scaleFactors.push_back(scale);
            // cout << "KeyFrame " << i << ": Scale = " << scale << endl;
        }
    }
    
    // Filter scale factors using median filter
    float finalScale = ORB_SLAM3::GroundPlaneDetector::FilterScaleMedian(scaleFactors);
    
    if(finalScale > 0.0f)
    {
        cout << endl << "Final scale factor (median): " << finalScale << endl;
        cout << "Applying scale to trajectory..." << endl;
    }
    else
    {
        cout << endl << "Warning: Could not calculate valid scale factor. Using scale = 1.0" << endl;
        finalScale = 1.0f;
    }
    
    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    int processedImages = vTimesTrack.size();
    for(int ni=0; ni<processedImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    if(processedImages > 0)
    {
        cout << "median tracking time: " << vTimesTrack[processedImages/2] << endl;
        cout << "mean tracking time: " << totaltime/processedImages << endl;
    }
    cout << "Processed " << processedImages << " images" << endl;
 
    // Save camera trajectory with scale applied
    SLAM.SaveKeyFrameTrajectoryTUM("/root/ORB_SLAM3/results/KFTrajectory_scale.txt", finalScale);
    cout << "KeyFrame trajectory saved to: KeyFrameTrajectory_scale.txt (scale = " << finalScale << ")" << endl;

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}

vector<ORB_SLAM3::MapPoint*> CollectLocalMapPoints(
    const vector<ORB_SLAM3::KeyFrame*>& allKeyFrames,
    int startIdx, int endIdx)
{
    std::set<ORB_SLAM3::MapPoint*> observedMapPoints;
    
    for(int j = startIdx; j <= endIdx; j++)
    {
        if(j < 0 || j >= (int)allKeyFrames.size() || allKeyFrames[j]->isBad())
            continue;
        
        std::set<ORB_SLAM3::MapPoint*> kfMapPoints = allKeyFrames[j]->GetMapPoints();
        for(ORB_SLAM3::MapPoint* pMP : kfMapPoints)
        {
            if(pMP && !pMP->isBad())
            {
                observedMapPoints.insert(pMP);
            }
        }
    }
    
    return vector<ORB_SLAM3::MapPoint*>(observedMapPoints.begin(), observedMapPoints.end());
}

GroundPlaneInfo FindGroundPoints(
    ORB_SLAM3::KeyFrame* pKF,
    const vector<ORB_SLAM3::MapPoint*>& localMapPoints,
    ORB_SLAM3::GroundPlaneDetector& groundDetector)
{
    GroundPlaneInfo info;
    info.isValid = false;
    
    // Get camera center and Y-axis
    Eigen::Vector3f cameraCenter = pKF->GetCameraCenter();
    Sophus::SE3f pose = pKF->GetPose();
    Eigen::Matrix3f R = pose.rotationMatrix();
    Eigen::Vector3f cameraYAxis = R.col(1).normalized();

    // Detect ground plane
    ORB_SLAM3::GroundPlaneDetector::Plane plane = 
        groundDetector.DetectGroundPlane(localMapPoints, cameraCenter, cameraYAxis);
    
    if(plane.inliers >= 30 && plane.confidence >= 0.3f)
    {
        // Extract ground inlier points
        for(ORB_SLAM3::MapPoint* pMP : localMapPoints)
        {
            if(pMP && !pMP->isBad())
            {
                Eigen::Vector3f pos = pMP->GetWorldPos();
                float dist = groundDetector.PointToPlaneDistance(pos, plane);
                if(dist < 0.1f) // Threshold for inliers
                {
                    info.groundPoints.push_back(pos);
                }
            }
        }
        
        info.plane = plane;
        info.isValid = true;
    }
    
    return info;
}

void DrawCoordinateAxes()
{
    glLineWidth(3);
    glBegin(GL_LINES);
    // X axis - red
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(0, 0, 0);
    glVertex3f(1, 0, 0);
    // Y axis - green
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 1, 0);
    // Z axis - blue
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 1);
    glEnd();
}

void DrawMapPoints(const vector<ORB_SLAM3::MapPoint*>& mapPoints)
{
    if(mapPoints.empty())
        return;
        
    glPointSize(2.0f);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0); // Black for map points
    
    for(const auto& pMP : mapPoints)
    {
        if(pMP && !pMP->isBad())
        {
            Eigen::Vector3f pos = pMP->GetWorldPos();
            glVertex3f(pos.x(), pos.y(), pos.z());
        }
    }
    glEnd();
}

void DrawKeyFrames(const vector<ORB_SLAM3::KeyFrame*>& keyFrames)
{
    if(keyFrames.empty())
        return;
    
    // Draw keyframe centers
    glPointSize(5.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 0.0); // Red for keyframe centers
    
    for(const auto& pKF : keyFrames)
    {
        if(pKF && !pKF->isBad())
        {
            Eigen::Vector3f center = pKF->GetCameraCenter();
            glVertex3f(center.x(), center.y(), center.z());
        }
    }
    glEnd();

    // Draw keyframe trajectory
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(0.0, 1.0, 1.0); // Cyan for trajectory
    
    for(size_t i = 0; i < keyFrames.size() - 1; i++)
    {
        if(keyFrames[i] && !keyFrames[i]->isBad() && 
           keyFrames[i+1] && !keyFrames[i+1]->isBad())
        {
            Eigen::Vector3f center1 = keyFrames[i]->GetCameraCenter();
            Eigen::Vector3f center2 = keyFrames[i+1]->GetCameraCenter();
            
            glVertex3f(center1.x(), center1.y(), center1.z());
            glVertex3f(center2.x(), center2.y(), center2.z());
        }
    }
    glEnd();
}

void DrawGroundPoints(const vector<Eigen::Vector3f>& groundPoints)
{
    if(groundPoints.empty())
        return;
        
    glPointSize(5.0f);
    glBegin(GL_POINTS);
    glColor3f(0.0, 1.0, 0.0); // Green for ground points
    
    for(const auto& pos : groundPoints)
    {
        glVertex3f(pos.x(), pos.y(), pos.z());
    }
    glEnd();
}

void DrawGroundPlane(const ORB_SLAM3::GroundPlaneDetector::Plane& plane,
                     const vector<Eigen::Vector3f>& groundPoints,
                     const Eigen::Vector3f& keyFrameCenter)
{
    if(groundPoints.empty() || plane.normal.norm() < 0.001f)
        return;
    
    // Calculate extent from ground points for plane size
    Eigen::Vector3f minPt = groundPoints[0];
    Eigen::Vector3f maxPt = groundPoints[0];
    for(const auto& pt : groundPoints)
    {
        minPt = minPt.cwiseMin(pt);
        maxPt = maxPt.cwiseMax(pt);
    }
    
    // Calculate extent (size of the plane)
    Eigen::Vector3f extent = (maxPt - minPt) * 0.5f;
    extent *= 1.2f; // Extend by 20%
    
    // Find two orthogonal vectors on the plane
    Eigen::Vector3f normal = plane.normal.normalized();
    Eigen::Vector3f u, v;
    
    // Find a vector perpendicular to normal
    if(std::abs(normal.x()) < 0.9f)
    {
        u = Eigen::Vector3f(1.0f, 0.0f, 0.0f).cross(normal).normalized();
    }
    else
    {
        u = Eigen::Vector3f(0.0f, 1.0f, 0.0f).cross(normal).normalized();
    }
    v = normal.cross(u).normalized();
    
    // Calculate plane center: project keyframe center onto the plane
    // For plane ax + by + cz + d = 0, distance from point to plane is |ax + by + cz + d|
    // Project keyframe center onto plane
    float distToPlane = plane.normal.dot(keyFrameCenter) + plane.d;
    Eigen::Vector3f planeCenter = keyFrameCenter - distToPlane * normal;
    
    // Create 4 corners of a rectangle on the plane
    Eigen::Vector3f corner1 = planeCenter - extent.x() * u - extent.z() * v;
    Eigen::Vector3f corner2 = planeCenter + extent.x() * u - extent.z() * v;
    Eigen::Vector3f corner3 = planeCenter + extent.x() * u + extent.z() * v;
    Eigen::Vector3f corner4 = planeCenter - extent.x() * u + extent.z() * v;
    
    // Draw plane as a semi-transparent quad
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.0, 1.0, 0.0, 0.3); // Green with transparency
    glBegin(GL_QUADS);
    glVertex3f(corner1.x(), corner1.y(), corner1.z());
    glVertex3f(corner2.x(), corner2.y(), corner2.z());
    glVertex3f(corner3.x(), corner3.y(), corner3.z());
    glVertex3f(corner4.x(), corner4.y(), corner4.z());
    glEnd();
    
    // Draw plane border
    glLineWidth(2.0f);
    glColor3f(0.0, 0.8, 0.0); // Darker green for border
    glBegin(GL_LINE_LOOP);
    glVertex3f(corner1.x(), corner1.y(), corner1.z());
    glVertex3f(corner2.x(), corner2.y(), corner2.z());
    glVertex3f(corner3.x(), corner3.y(), corner3.z());
    glVertex3f(corner4.x(), corner4.y(), corner4.z());
    glEnd();
}

void DrawCurrentKeyFrame(ORB_SLAM3::KeyFrame* pKF)
{
    if(!pKF || pKF->isBad())
        return;
        
    glPointSize(8.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0, 0.0, 1.0); // Magenta for current keyframe
    Eigen::Vector3f center = pKF->GetCameraCenter();
    glVertex3f(center.x(), center.y(), center.z());
    glEnd();
}

void VisualizeSLAMWithNavigation(
    const vector<ORB_SLAM3::KeyFrame*>& allKeyFrames,
    const vector<ORB_SLAM3::MapPoint*>& allMapPoints,
    const vector<GroundPlaneInfo>& groundPlaneInfoPerKeyFrame)
{
    // Create Pangolin window
    pangolin::CreateWindowAndBind("SLAM Visualization", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(-0, -0.7, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    // Create UI panel for buttons (left side, 200 pixels wide)
    pangolin::CreatePanel("ui")
        .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(200));

    // Add button and display variables
    pangolin::Var<bool> buttonNext("ui.Next KeyFrame", false, false);
    pangolin::Var<int> currentKFDisplay("ui.Current KeyFrame", 0, 0, 0);
    pangolin::Var<int> totalKFDisplay("ui.Total KeyFrames", allKeyFrames.size(), 0, 0);
    
    // Create Interactive View in window (right side, leaving space for UI)
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -1024.0f/768.0f)
            .SetHandler(&handler);
    
    // Current keyframe index for visualization
    int currentKFIdx = 0;
    
    // Main visualization loop
    while(!pangolin::ShouldQuit())
    {
        // Update current keyframe display
        currentKFDisplay = currentKFIdx;
        
        // Set white background
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);

        // Draw all elements
        DrawCoordinateAxes();
        DrawMapPoints(allMapPoints);
        DrawKeyFrames(allKeyFrames);
        
        // Draw current keyframe's ground points and plane
        if(currentKFIdx >= 0 && currentKFIdx < (int)groundPlaneInfoPerKeyFrame.size() &&
           currentKFIdx < (int)allKeyFrames.size() && 
           allKeyFrames[currentKFIdx] && !allKeyFrames[currentKFIdx]->isBad())
        {
            const GroundPlaneInfo& info = groundPlaneInfoPerKeyFrame[currentKFIdx];
            if(info.isValid)
            {
                Eigen::Vector3f keyFrameCenter = allKeyFrames[currentKFIdx]->GetCameraCenter();
                DrawGroundPlane(info.plane, info.groundPoints, keyFrameCenter);
                DrawGroundPoints(info.groundPoints);
            }
        }

        // Highlight current keyframe center
        if(currentKFIdx >= 0 && currentKFIdx < (int)allKeyFrames.size())
        {
            DrawCurrentKeyFrame(allKeyFrames[currentKFIdx]);
        }

        // Check if button is pressed
        if(buttonNext)
        {
            currentKFIdx++;
            if(currentKFIdx >= (int)allKeyFrames.size())
            {
                currentKFIdx = 0; // Loop back to start
            }
            buttonNext = false; // Reset button state
            currentKFDisplay = currentKFIdx;
            if(currentKFIdx < (int)groundPlaneInfoPerKeyFrame.size())
            {
                cout << "Switched to KeyFrame " << currentKFIdx << " (" 
                     << groundPlaneInfoPerKeyFrame[currentKFIdx].groundPoints.size() << " ground points)" << endl;
            }
        }

        pangolin::FinishFrame();
    }

    pangolin::DestroyWindow("SLAM Visualization");
    cout << "Viewer closed." << endl;
}
