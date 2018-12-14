//
// Created by lalo on 11-10-18.
//



#include <QtGui/QPainter>
#include "geoproyectiva.h"
#include "foot.h"

#include <cstdlib>

//#include <math.h>
#define pi  3.141592653589793238462643383279502884L /* pi */

//// CONSTRUCTOR ////
geoproy::geoproy(bool start) {
    this -> start = true;
}

void geoproy::readCalibFile(string fileName) {

    if(fileName == "")
        return;

    cv::FileStorage fs(fileName, cv::FileStorage::READ);
    FileNode calib = fs["calibration"];
    FileNodeIterator it = calib.begin(), it_end = calib.end();

    //// iterate through a sequence using FileNodeIterator
    for( ; it != it_end; ++it ) {
        (*it)["Homography"] >> homography;
    }

    homographyInv = homography.inv();
    fs.release();

//    cout << "Homography: \n" << geoproyTest.homography << "\n" << endl;
//    cout << "Homography Inv: \n" << geoproyTest.homographyInv << "\n" << endl;

}

void geoproy::genCalibPointsSuelo() {

    calibPointsFloor.clear();
    cv::Point2f p;
    int step = 200;

    p.x = -step; p.y = -step;
    calibPointsFloor[1] = p;
    p.x = 0;     p.y = -step;
    calibPointsFloor[2] = p;
    p.x = step;  p.y = -step;
    calibPointsFloor[3] = p;
    p.x = -step; p.y = 0;
    calibPointsFloor[4] = p;
    p.x = 0;     p.y = 0;
    calibPointsFloor[5] = p;
    p.x = step;  p.y = 0;
    calibPointsFloor[6] = p;
    p.x = -step; p.y = step;
    calibPointsFloor[7] = p;
    p.x = 0;     p.y = step;
    calibPointsFloor[8] = p;
    p.x = step;  p.y = step;
    calibPointsFloor[9] = p;

    genCalibPointsImage();

}

void geoproy::genCalibPointsImage(){

    std::map<int, cv::Point2f>::iterator it, it_end = calibPointsFloor.end();
    int index;

    for(it=calibPointsFloor.begin(); it!=it_end; it++) {
        index = it->first;
        cv::Point2f &p = it->second;
        calibPointsImage[index] = transformFloor2Image(p, homography);
    }

}

void geoproy::genCalibPointsCorner() {

    calibPointsCornerFloor.clear();
    cv::Point2f p;
    int step = 350;

    p.x = -step; p.y = -step;
    calibPointsCornerFloor[1] = p;
    p.x = step; p.y = -step;
    calibPointsCornerFloor[2] = p;
    p.x = step;  p.y = step;
    calibPointsCornerFloor[3] = p;
    p.x = -step; p.y = step;
    calibPointsCornerFloor[4] = p;

    roiConvexPoly.clear();
    for (int i = 1; i <= 4; ++i) {
        calibPointsCornerImage[i] = transformFloor2Image(calibPointsCornerFloor[i], homography);
        roiConvexPoly.push_back(calibPointsCornerImage[i]);
    }

}

cv::Point geoproy::transformFloor2Image(cv::Point2f p, cv::Mat &H) {

    cv::Mat pin(3, 1, CV_64FC1); // NOLINT
    pin.at<double>(0,0) = p.x;
    pin.at<double>(1,0) = p.y;
    pin.at<double>(2,0) = 1;

    cv::Mat res = H*pin;
    cv::Point pout;

    pout.x = int(res.at<double>(0,0)/res.at<double>(2,0));
    pout.y = int(res.at<double>(1,0)/res.at<double>(2,0));

    return pout;
}

void geoproy::drawRectangleRed(QPainter &pnt, Point2f &p, cv::Mat &H){

    cv::Point point1, point2;
    int hside = 50;
    pnt.setPen(QColor(255,255,0));

    point1 = transformFloor2Image(cv::Point2f(p.x-hside, p.y-hside), H);
    point2 = transformFloor2Image(cv::Point2f(p.x+hside, p.y-hside), H);
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

    point1 = transformFloor2Image(cv::Point2f(p.x+hside, p.y-hside), H);
    point2 = transformFloor2Image(cv::Point2f(p.x+hside, p.y+hside), H);
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

    point1 = transformFloor2Image(cv::Point2f(p.x+hside, p.y+hside), H);
    point2 = transformFloor2Image(cv::Point2f(p.x-hside, p.y+hside), H);
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

    point1 = transformFloor2Image(cv::Point2f(p.x-hside, p.y+hside), H);
    point2 = transformFloor2Image(cv::Point2f(p.x-hside, p.y-hside), H);
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

}

void geoproy::drawRectangleBlue(QPainter &pnt, cv::Mat &H){

    cv::Point point1, point2;
    pnt.setPen(QColor(255,255,0));

    point1 = calibPointsCornerImage[1];
    point2 = calibPointsCornerImage[2];
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

    point1 = calibPointsCornerImage[2];
    point2 = calibPointsCornerImage[3];
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

    point1 = calibPointsCornerImage[3];
    point2 = calibPointsCornerImage[4];
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

    point1 = calibPointsCornerImage[4];
    point2 = calibPointsCornerImage[1];
    pnt.drawLine(QPoint(point1.x, point1.y), QPoint(point2.x, point2.y));

}

void geoproy::addCalibPoints(QImage &image) {

    std::map<int, cv::Point2f>::iterator it1, it1_end = calibPointsFloor.end();
    int index;
    QPoint p_im;
    QPainter pnt;
    pnt.begin(&image);
    cv::Point pout;
    cv::Mat &H = homography;

    for(it1=calibPointsFloor.begin(); it1!=it1_end; it1++) {
        index = it1->first;
        cv::Point2f &p = it1->second;

        pout = transformFloor2Image(p, H);
        p_im = QPoint(pout.x, pout.y);

        //Draw circle
        pnt.setPen(QColor(255,0,0));
        pnt.drawEllipse(p_im, 3, 3);

        //Draw rectangle
        drawRectangleRed(pnt, p, H);

        //Draw text
        pnt.setPen(QColor(0,255,0));
        pnt.drawText(QPoint(p_im.x() + 5, p_im.y()), QString::number(index));

    }

    drawRectangleBlue(pnt, H);

    pnt.end();
}

void geoproy::generateSequence(int seedIn){

    int rnd;

    bool used[tplays];
    memset(used, false, tplays*sizeof(bool));

    srand(static_cast<unsigned int>(seedIn));

    for (int i = 0; i < numPlays ; i++) {
        rnd = rand()%(tplays-i); //NOLINT
        for(int j=0, k=0; j<tplays; j++) {
            if(used[j]) continue;

            if(k == rnd) {
                plays[i] = j+1;
                used[j] = true;
                break;
            }
            k++;
        }
    }
}

void geoproy::playsToObjetives() {

    for (int i = 0; i < numPlays; ++i) {
        switch(plays[i]) {
            case 1 :
                objetivesG2[i] = 9;
                objetivesG3[i] = 1;
                break;
            case 2 :
                objetivesG2[i] = 2;
                objetivesG3[i] = 8;
                break;
            case 3 :
                objetivesG2[i] = 1;
                objetivesG3[i] = 9;
                break;
            case 4 :
                objetivesG2[i] = 3;
                objetivesG3[i] = 7;
                break;
            case 5 :
                objetivesG2[i] = 4;
                objetivesG3[i] = 6;
                break;
            case 6 :
                objetivesG2[i] = 8;
                objetivesG3[i] = 2;
                break;
            case 7 :
                objetivesG2[i] = 4;
                objetivesG3[i] = 6;
                break;
            case 8 :
                objetivesG2[i] = 6;
                objetivesG3[i] = 4;
                break;
            case 9 :
                objetivesG2[i] = 9;
                objetivesG3[i] = 1;
                break;
            case 10 :
                objetivesG2[i] = 1;
                objetivesG3[i] = 9;
                break;
            case 11 :
                objetivesG2[i] = 6;
                objetivesG3[i] = 4;
                break;
            case 12 :
                objetivesG2[i] = 7;
                objetivesG3[i] = 3;
                break;
            default :
                break;
        }
    }
}

void geoproy::paintMatchOrError(Mat &image, int objetive, cv::Scalar color){

    cv::Point p = calibPointsFloor[objetive];

    const Point* ppt;
    int hside = 50;
    cv::Point rook_points[1][4];

    rook_points[0][0] = transformFloor2Image(cv::Point2f(p.x-hside, p.y-hside), homography);
    rook_points[0][1] = transformFloor2Image(cv::Point2f(p.x+hside, p.y-hside), homography);
    rook_points[0][2] = transformFloor2Image(cv::Point2f(p.x+hside, p.y+hside), homography);
    rook_points[0][3] = transformFloor2Image(cv::Point2f(p.x-hside, p.y+hside), homography);

    ppt = {rook_points[0]};

    fillConvexPoly(image, ppt, 4, color, 8, 0);

}




void geoproy::genCirclePoints(int objetive){

    cv::Mat &H = homography;
    cv::Point2f p;

    for (int i = 0; i < circlenumPoints; ++i) {
        p.x = calibPointsFloor[objetive].x + static_cast<float>(radio * cos((pi / (circlenumPoints/2.0)) * i));
        p.y = calibPointsFloor[objetive].y + static_cast<float>(radio * sin((pi / (circlenumPoints/2.0)) * i));
        circlePointsFloor[i] = p;
        circlePointsImage[i] = transformFloor2Image(p, H);
    }

}

void geoproy::paintCircles(QImage &image){

    QPainter pnt;
    pnt.begin(&image);;
    cv::Point p1, p2;
    pnt.setPen(QColor(255,255,0));

    for (int j = 1; j <= 9 ; ++j) {
        if(j != 5) {
            genCirclePoints(j);
            for (int i = 0; i < circlenumPoints; ++i) {
                p1 = circlePointsImage[i];
                p2 = circlePointsImage[(i + 1) % circlenumPoints];
                pnt.setPen(QColor(255, 255, 0));
                pnt.drawLine(QPoint(p1.x, p1.y), QPoint(p2.x, p2.y));
            }
        }
    }

}


















