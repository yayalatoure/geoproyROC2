//
// Created by lalo on 30-08-18.
//

#ifndef NEUROOBJORIENTED_FOOT_H
#define NEUROOBJORIENTED_FOOT_H

#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <sstream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv/cv.h>
#include <QRect>
#include <QDir>

#include "geoproyectiva.h"


using namespace std;
using namespace cv;

typedef struct {

    ////// Frames //////
    cv::Mat processFrame;
    cv::Mat maskConvexPoly;
    cv::Mat segmentedFrame;
    cv::Mat resultFrame;

    //// Segmentation Labels ////
    cv::Mat labelsFrame;
    cv::Mat labels2Frame;

    //// Foot Boxes ////
    map<int, Rect> footBoxes;
    vector<Rect>   footBoxesVector;

    map<int, Rect> blobBoxes;
    map<int, Rect> tempBoxes;
    map<int, Rect> segmLowerBoxes;
    vector<Rect>   segmLowerBoxesVector;

    cv::Rect   segmLowerBox;
    cv::Rect   segmLowerBoxFL;
    cv::Point  lowPointFloor;
    double     segmCutPercent;

    //// Foots to Objetive Matching ////
    cv::Rect  leftRectFoot;
    cv::Rect  rightRectFoot;
    cv::Point leftFoot;
    cv::Point rightFoot;


} ImageBoxes;


class foot {

    public:
        //// Constructor ////
        explicit foot(bool start);

        //// Segmentation & ROI (footBoxes) ////
        void maskConvexPoly(geoproy GeoProy);
        void segmentation(cv::Ptr<cv::BackgroundSubtractorMOG2> mog);

        void firstTimeLowerBox(geoproy GeoProy);

        void getBlobsBoxes(cv::Mat labels, std::map<int, cv::Rect> &bboxes);
        void orderVectorBoxes(std::map<int, cv::Rect> &bboxes, vector<Rect> &vectorBoxes);
        void distanceFilterBoxes();
        void findLowerBox();
        void getLowerBox();
        void zoneDetectionG3(geoproy GeoProy);
        void linearFunctionPosYG3();
        void areasideFilter(std::map<int, cv::Rect> &bboxes);
        void getFeetBoxes(geoproy GeoProy);
        void leftrightBoxes();

        //// Kalman Filter ////
        void kalmanInit(int pie);
        void kalmanPredict(int pie, int dT);
        void kalmanResetStep(int pie);
        void kalmanUpdate(int pie);
        void stepPrecision(int pie);

        //// Measure Error ////
        double distance(cv::Point center_kalman, cv::Point center_measured);
        void measureError1Np(int pie);

        //// Generate Template ////
        void generateTemplateNp();
        //// Partial Occlusion ////
        void matchingScorePocc();
        //// Occlusion Type ////
        void occlusionType();
        //// Max Candidates Points Vector ////
        void maxCandidatesPocc();
        //// Select Matching Score ////
        void matchingSelectPocc();
        //// Proyect Measure-Box ////
        void proyectBoxes();


        //// Objetive Matching ////
        void logCSVInit();
        void logInitFrame();
        void logMatchingFrame();
        void logMatchingCenterFrame();
        void logMatchingError1Frame();
        void logMatchingError2Frame();
        void logEndVideo();
        void askObjetives(geoproy GeoProy);
        void centerOutCountFlag(geoproy GeoProy);
        void stateMachine(geoproy &GeoProy);

        //// Drawing Result ////
        void drawingResults();
        void paintRectangles(cv::Mat &img, std::map<int, cv::Rect> &bboxes, cv::Scalar color);
        void paintRectanglesVector(vector<Rect> &vectorBoxes, cv::Scalar color);

        //// Clear Variables ////
        void clearVariables();


    public:

        //// Reset MOG2 ////
        double learningrate;

        //// CSV Writer Atributes
        ofstream *ofStream;

        string frame;
        int limit;

        //// Int Atributes ////
        int Right = 1;
        int Left = 2;
        int rowsIm = 480; int colsIm = 640;
        int platformZone = 2;
        int objetive = 0;
        int stopCount = 0;
        int sequenceCount = 0;
        int countCenterOut = 0;

        //// Bool Atributes ////
        bool start = false;
        bool stopFlagCenter = false;
        bool stop = false;
        bool firstTimeLBFlag = true;
        bool found;
        bool distFilterBoxesFlag = false;
        bool Reset_R;
        bool Reset_L;
        bool step_R;
        bool step_L;
        bool occlusion;
        bool totalOccR;
        bool totalOccL;
        bool init = true;
        bool center, object;
        bool betweenFromCenter, betweenFromObjet;
        bool foundMatchR = false;
        bool foundMatchL = false;
        bool centerFlagWasOut = false;
        bool centerFlagIsIn = true;
        bool paint = false;
        bool paintTarget = false;


        //// Image & Boxes Atributes ////
        ImageBoxes frameAct, frameAnt;

        //// Kalman Atributes ////
        unsigned int type = CV_32F;
        int stateSize = 6, measSize  = 4, contSize = 0;
        int notFoundCount = 0;
        cv::KalmanFilter kf_R = cv::KalmanFilter(stateSize, measSize, contSize, type);
        cv::KalmanFilter kf_L = cv::KalmanFilter(stateSize, measSize, contSize, type);
        cv::Mat state_R = cv::Mat(stateSize, 1, type);
        cv::Mat state_L = cv::Mat(stateSize, 1, type);
        cv::Mat meas_R = cv::Mat(measSize, 1, type);
        cv::Mat meas_L = cv::Mat(measSize, 1, type);

        //// Rectangle & Kalman Center Atributes ////
        cv::Rect  predRect_R;
        cv::Rect  predRect_L;
        cv::Point centerKalman_L, centerMeasured_L;
        cv::Point centerKalman_R, centerMeasured_R;

        //// Errors Normal Detection ////
        double errorNpAct1_R, errorNpAnt1_R; // |Measured position - Kalman predicition|
        double errorNpAct1_L, errorNpAnt1_L;
        double errorNpAct2, errorNpAnt2; // |Measured position - Kalman correction|

        //// PARTIAL OCCLUSION ////

        //// Occlusion Atributes ////
        cv::Point occlusionCorner;
        int offsetR = 2; // offset de template derecho
        int offsetL = 2; // offset de template izquierdo

        //// Maximum Candidates ////
        cv::Mat centroidsR; // maximos locales matching R
        cv::Mat centroidsL; // maximos locales matching L
        vector<Point> maxLocR, maxLocL;
        //// Max Matching Points ////
        cv::Point maxlocSelectedR;
        cv::Point maxlocSelectedL;

        //// Colors ////
        static cv::Scalar blue;
        static cv::Scalar green;
        static cv::Scalar red;
        static cv::Scalar cyan;
        static cv::Scalar ivory;
        static cv::Scalar blueviolet;
        static cv::Scalar orange;




};




#endif //NEUROOBJORIENTED_FOOT_H
