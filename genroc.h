

#ifndef GEOPROYECTIVA_GENROC_H
#define GEOPROYECTIVA_GENROC_H

#include "foot.h"
#include "geoproyectiva.h"


using namespace std;
using namespace cv;



class genroc {

public:

    //// Constructor ////
    explicit genroc(bool start);

    //// PUBLIC METHODS ////
    void logcsvOpen();
    void calibVideoSelector();
    void getVideo();
    void firstLog();
    void logcsvClose();
    void algorithm();


public:

    //// ATRIBUTES ////

    //// Bool Atributes ////
    bool start;

    //// Varthreshold ////
    int varThreshold;

    //// Video Get Atributes ////

    bool stopLoopFlag = false;

    int dT = 1;
    int seed;
    int limit = 10;
    int digits = 5;
    int count_cal = 0;
    int count_test = 0;
    int numVideo = 0;
    int numCalibVideo = 0;

    char ch = 0;

    size_t pos;
    string pathVideos;
    string pathCalib;
    string path_cal;
    string path_test;
    string calfileName;
    vector<String> filenames_cal, filenames_test;
    string substring;


    //// CSV Writer Atributes ////
    string fileNameCSV;
    ofstream ofStreamGenroc;

    //// Imshow Atributes ////
    Mat geopro;
    QImage qedit;


    //// MOG2 ////
    //// Grabacion 3
    int  nmixtures = 3;
    int  historyMOG = 200;
    bool bShadowDetection = true;
    double learningRate = 0.005;
    double backgroundRatio = 0.6;


};



#endif //GEOPROYECTIVA_GENROC_H
