#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include "navigation.h"
#include "fc_serial.h"
#include "main.h"
#include "PID.h"
using namespace std;

bool autopilot = false;

PID xPid = {75,0.1,5,10,-10,5,-5};
PID yPid = {75,0.1,5,10,-10,5,-5};

void setAutopilot(bool enabled){
    autopilot = enabled;
}

double cameraMatrixArray[3][3] = {
    {586.1899962688001, 0, 318.059252104379},
    {0, 584.1750645338378, 234.6106522131543},
    {0, 0, 1}};
double distCoeffsArray[5] = {0.1399191037477049, -0.4173075385395212, -0.0006774708587260779, 0.0003085555507675579, 0.4408805610934874};

double bxs = 0.085, bys = 0.165, bzs = 0.06;
double bxo = -0.0425, byo = -0.0825, bzo = 0;

cv::Vec3d targetOffset = cv::Vec3d(0,-0.09,0);

vector<cv::Point3d> boxPoints = {
    cv::Point3d(bxo, byo, bzo),
    cv::Point3d(bxo + bxs, byo, bzo),
    cv::Point3d(bxo + bxs, byo + bys, bzo),
    cv::Point3d(bxo, byo + bys, bzo),
    cv::Point3d(bxo, byo, bzo + bzs),
    cv::Point3d(bxo + bxs, byo, bzo + bzs),
    cv::Point3d(bxo + bxs, byo + bys, bzo + bzs),
    cv::Point3d(bxo, byo + bys, bzo + bzs),
    cv::Point3d(-0.01, -0.01, bzo),
    cv::Point3d(0.01, 0.01, bzo),
    cv::Point3d(0.01, -0.01, bzo),
    cv::Point3d(-0.01, 0.01, bzo)};

vector<pair<int, int>> crossLines = {
    {8, 9},
    {10, 11}};

vector<pair<int, int>> boxLines = {
    //bottom size
    {0, 1},
    {1, 2},
    {2, 3},
    {3, 0},
    //top size
    {4, 5},
    {5, 6},
    {6, 7},
    {7, 4},
    //top to bottom joints
    {0, 4},
    {1, 5},
    {2, 6},
    {3, 7}};

cv::Ptr<cv::aruco::Dictionary> dictionary;
cv::Ptr<cv::aruco::Board> board;
cv::Ptr<cv::aruco::DetectorParameters> parameters;
cv::Mat distCoeffs, cameraMatrix;

cv::Vec3f prevTarget = {0,0,0};

void navigationInit()
{
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    //board = cv::aruco::GridBoard::create(2, 2, 0.03, 0.01, dictionary);
    std::vector<std::vector<cv::Point3f>> objPoints;
    float markerSize = 0.03;
    float markerInterval = 0.04;
    int countX = 2, countY = 2;
    float offsetX = -0.035, offsetY = -0.075, offsetZ = 0;
    for (int j = 0; j < countY; j++)
    {
        for (int i = 0; i < countX; i++)
        {
            float ox = i * markerInterval + offsetX;
            float oy = j * markerInterval + offsetY;
            float oz = 0 + offsetZ;
            std::vector<cv::Point3f> marker = {
                cv::Point3f(ox + 0, oy + 0, oz + 0),
                cv::Point3f(ox + markerSize, oy + 0, oz + 0),
                cv::Point3f(ox + markerSize, oy + markerSize, oz + 0),
                cv::Point3f(ox + 0, oy + markerSize, oz + 0)};
            objPoints.push_back(marker);
        }
    }
    vector<int> ids = {0, 1, 2, 3};
    board = cv::aruco::Board::create(objPoints, dictionary, ids);
    parameters = cv::aruco::DetectorParameters::create();
    distCoeffs = cv::Mat(1, 5, CV_64F, distCoeffsArray);
    cameraMatrix = cv::Mat(3, 3, CV_64F, cameraMatrixArray);

    init_serial();
}

cv::Vec3d rotate(cv::Vec3d tvec, cv::Vec3d rvec)
{
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    cv::Mat tmat2 = rmat * cv::Mat(tvec);
    cv::Vec3d tvec2 = cv::Vec3d((double *)tmat2.data);
    return tvec2;
}

cv::Vec3d rotate(cv::Vec3d tvec, cv::Mat rmat)
{
    cv::Mat tmat2 = rmat * cv::Mat(tvec);
    cv::Vec3d tvec2 = cv::Vec3d((double *)tmat2.data);
    return tvec2;
}

cv::Mat quatToMat(cv::Vec4d q)
{
    double w = q[0], x = q[1], y = q[2], z = q[3];
    double mat[9] =
        {1-2*y*y-2*z*z, 2*x*y-2*w*z,   2*x*z+2*w*y,
        2*x*y+2*w*z,    1-2*x*x-2*z*z, 2*y*z-2*w*x,
        2*x*z-2*w*y,    2*y*z+2*w*x,   1-2*x*x-2*y*y};
    cv::Mat rmat = cv::Mat(3, 3, CV_64F, mat);
    return rmat.clone();
}
cv::Vec3d quatToRvec(cv::Vec4d q)
{
    cv::Mat mat = quatToMat(q);
    cv::Vec3d rvec;
    cv::Rodrigues(mat, rvec);
    return rvec;
}

void detectPosition(cv::Mat frame, float fps)
{
    //detect markers
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

    if (markerIds.size() == 0)
        return;

    //calculate box position
    cv::Vec3d boxRvec, boxTvec;
    int valid = cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, cameraMatrix, distCoeffs, boxRvec, boxTvec);
    if (valid == 0)
        return;

    //draw box
    std::vector<cv::Point2d> projectedBoxPoints;
    cv::projectPoints(boxPoints, boxRvec, boxTvec, cameraMatrix, distCoeffs, projectedBoxPoints);
    for (pair<int, int> line : boxLines)
    {
        cv::Point2d pt1 = projectedBoxPoints[line.first];
        cv::Point2d pt2 = projectedBoxPoints[line.second];
        cv::line(frame, pt1, pt2, cv::Scalar(0, 0, 255), 2);
    }
    for (pair<int, int> line : crossLines)
    {
        cv::Point2d pt1 = projectedBoxPoints[line.first];
        cv::Point2d pt2 = projectedBoxPoints[line.second];
        cv::line(frame, pt1, pt2, cv::Scalar(0, 255, 0), 2);
    }

    //calculate drone rotation
    cv::Vec3d droneRvec;
    droneRvec = quatToRvec(fc_getRotation());
    {
        float a = droneRvec[0];
        droneRvec[0] = droneRvec[1];
        droneRvec[1] = a;
    }
    cv::Mat droneRmat;
    cv::Rodrigues(droneRvec,droneRmat);

    //calclualte rotation around z axis
    cv::Mat zRmat;
    {
        cv::Vec3d v = cv::Vec3d(1, 0, 0);
        v = rotate(v, droneRvec);
        double len = sqrt(v[0] * v[0] + v[1] * v[1]);
        double sinA = v[1] / len;
        double cosA = v[0] / len;
        double r_array[9] = {
            cosA, -sinA, 0,
            sinA, cosA, 0,
            0, 0, 1};
        cv::Mat(3, 3, CV_64F, r_array).copyTo(zRmat);
    }

    //calculate camera rotation
    cv::Mat cameraRmat;
    {
        double a = getCameraAngle();
        a = a / 180 * CV_PI;
        double sinA = sin(a);
        double cosA = cos(a);
        double r_array[9] = {
            1, 0, 0,
            0, cosA, -sinA,
            0, sinA, cosA};
        cv::Mat(3, 3, CV_64F, r_array).copyTo(cameraRmat);
        cameraRmat = cameraRmat * droneRmat;
    }

    //calculate drone position
    cv::Vec3d droneTvec = rotate(boxTvec, cameraRmat.t());
    //calculate vector to target

    cv::Vec3d target;
    target[0] = droneTvec[0];
    target[1] = droneTvec[1];
    target[2] = droneTvec[2];
    target = rotate(target, zRmat);
    target += targetOffset;
    //draw vector to target
    cv::line(frame, cv::Point(80, 400), cv::Point(80 + target[0] * 200, 400 + target[1] * 200), cv::Scalar(255, 0, 0), 2);

    //print coordinates
    stringstream outString;
    outString << "box:    ";
    outString << boxTvec;
    cv::putText(frame, outString.str(), cv::Point(10, 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    outString.str(std::string());
    outString << "drone:  ";
    outString << droneTvec;
    cv::putText(frame, outString.str(), cv::Point(10, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    outString.str(std::string());
    outString << "target: ";
    outString << target;
    cv::putText(frame, outString.str(), cv::Point(10, 100), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));

    if(autopilot){
        float dTime = 1/fps;
        float controlRoll = xPid.update(dTime,target[0],(target[0]-prevTarget[0])*fps,0);
        float controlPitch = yPid.update(dTime,target[1],(target[1]-prevTarget[1])*fps,0);
        sendControl(0,controlPitch,-controlRoll,0);
    }

    prevTarget = target;
}