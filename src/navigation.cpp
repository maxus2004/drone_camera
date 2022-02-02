#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include "navigation.h"

using namespace std;

double cameraMatrixArray[3][3] = {
    {586.1899962688001, 0, 318.059252104379},
    {0, 584.1750645338378, 234.6106522131543},
    {0, 0, 1}};
double distCoeffsArray[5] = {0.1399191037477049, -0.4173075385395212, -0.0006774708587260779, 0.0003085555507675579, 0.4408805610934874};

double bxs = 0.085, bys = 0.165, bzs = 0.06;
double bxo = -0.0425, byo = -0.0825, bzo = 0;

vector<cv::Point3d> boxPoints = {
    cv::Point3d(bxo, byo, bzo),
    cv::Point3d(bxo + bxs, byo, bzo),
    cv::Point3d(bxo + bxs, byo + bys, bzo),
    cv::Point3d(bxo, byo + bys, bzo),
    cv::Point3d(bxo, byo, bzo + bzs),
    cv::Point3d(bxo + bxs, byo, bzo + bzs),
    cv::Point3d(bxo + bxs, byo + bys, bzo + bzs),
    cv::Point3d(bxo, byo + bys, bzo + bzs),
    cv::Point3d(-0.01,-0.01,bzo),
    cv::Point3d(0.01,0.01,bzo),
    cv::Point3d(0.01,-0.01,bzo),
    cv::Point3d(-0.01,0.01,bzo)};

vector<pair<int, int>> crossLines = {
    {8, 9},
    {10, 11}
};

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
}

cv::Vec3d rotate(cv::Vec3d tvec, cv::Vec3d rvec){
    cv::Mat rmat;
    cv::Rodrigues(rvec,rmat);
    cv::Mat tmat2 = rmat*cv::Mat(tvec);
    cv::Vec3d tvec2 = cv::Vec3d((double*)tmat2.data);
    return tvec2;
}

void detectPosition(cv::Mat frame)
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
        cv::line(frame, pt1, pt2, cv::Scalar(0, 0, 255), 4);
    }
    for (pair<int, int> line : crossLines)
    {
        cv::Point2d pt1 = projectedBoxPoints[line.first];
        cv::Point2d pt2 = projectedBoxPoints[line.second];
        cv::line(frame, pt1, pt2, cv::Scalar(0, 255, 0), 4);
    }

    //calculate drone position
    cv::Vec3d droneRvec, droneTvec;
    droneTvec = rotate(boxTvec,-boxRvec);

    //print coordinates
    stringstream boxPosString;
    boxPosString << "box:   x: ";
    boxPosString << boxTvec[0];
    boxPosString << " y: ";
    boxPosString << boxTvec[1];
    boxPosString << " z: ";
    boxPosString << boxTvec[2];
    cv::putText(frame, boxPosString.str(), cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
    stringstream dronePosString;
    dronePosString << "drone: x: ";
    dronePosString << droneTvec[0];
    dronePosString << " y: ";
    dronePosString << droneTvec[1];
    dronePosString << " z: ";
    dronePosString << droneTvec[2];
    cv::putText(frame, dronePosString.str(), cv::Point(10, 40), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255));
}