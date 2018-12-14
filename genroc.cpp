

#include "genroc.h"
#include "foot.h"



//// CONSTRUCTOR ////
genroc::genroc(bool start) {
    this -> start = true;
}

//// OBJETIVE MATCHING ////
void genroc::logcsvOpen(){

    fileNameCSV = "/home/lalo/Desktop/Data/Login/varTreshold"+ to_string(varThreshold)+".csv";
    ofStreamGenroc.open(fileNameCSV);

    ofStreamGenroc << "Video" << "," << "InitFrame" << "," << "FinalFrame" << "," << "TestType" << "," << "Seed" << ",";
    ofStreamGenroc << "#Plays" << "," << "VarTreshold" << "," << "TotalErrors" << ",";

    ofStreamGenroc <<"P1"<<","<<"E1.1"<<","<<"E2.1"<<","<<"C1"<<",";
    ofStreamGenroc <<"P2"<<","<<"E1.2"<<","<<"E2.2"<<","<<"C2"<<",";
    ofStreamGenroc <<"P3"<<","<<"E1.3"<<","<<"E2.3"<<","<<"C3"<<",";
    ofStreamGenroc <<"P4"<<","<<"E1.4"<<","<<"E2.4"<<","<<"C4"<<",";
    ofStreamGenroc <<"P5"<<","<<"E1.5"<<","<<"E2.5"<<","<<"C5"<<",";
    ofStreamGenroc <<"P6"<<","<<"E1.6"<<","<<"E2.6"<<","<<"C6"<<",";
    ofStreamGenroc <<"P7"<<","<<"E1.7"<<","<<"E2.7"<<","<<"C7"<<",";
    ofStreamGenroc <<"P8"<<","<<"E1.8"<<","<<"E2.8"<<","<<"C8"<<",";
    ofStreamGenroc <<"P9"<<","<<"E1.9"<<","<<"E2.9"<<","<<"C9"<<",";
    ofStreamGenroc <<"P10"<<","<<"E1.10"<<","<<"E2.10"<<","<<"C10" << "\n";

}

void genroc::calibVideoSelector(){

    switch(numVideo) {
        case 1 :
            numCalibVideo = 1;
            seed = 362369073;
            break;
        case 2 :
            numCalibVideo = 2;
            seed = 660978730;
            break;
        case 3 :
            numCalibVideo = 2;
            seed = 868094992;
            break;
        case 4 :
            numCalibVideo = 2;
            seed = 2025060938;
            break;
        case 5 :
            numCalibVideo = 2;
            seed = 56646810;
            break;
        case 6 :
            numCalibVideo = 3;
            seed = 291078514;
            break;
        case 7 :
            numCalibVideo = 3;
            seed = 73832449;
            break;
        case 8 :
            numCalibVideo = 4;
            seed = 254894743;
            break;
        case 9 :
            numCalibVideo = 4;
            seed = 424457088;
            break;
        default :
            break;
    }

}

void genroc::getVideo() {
    count_test = 0;
    count_cal  = 0;
    pathVideos = "/home/lalo/Desktop/Data/Videos/Video";
    pathCalib  = "/home/lalo/Desktop/Data/Calib/Video";

    calibVideoSelector();

    path_test = pathVideos + to_string(numVideo) + "/*.jpg";
    path_cal  = pathCalib + to_string(numCalibVideo)+"/*.jpg";
    calfileName = pathCalib + to_string(numCalibVideo) + "/default_calib.yml";

    cout << "\n" << "Path Video: " << endl;
    cout << path_test << endl;
//    cout << "Path Video Calibracion" << endl;
//    cout << path_cal << endl;
//    cout << "Path Archivo Calibracion" << endl;
//    cout << calfileName << endl;

    glob(path_test, filenames_test);
    glob(path_cal , filenames_cal);

    pos = filenames_test[count_test].find(".jpg");

}

void genroc::firstLog(){

    ofStreamGenroc <<to_string(numVideo)<<","<<"Init"<<","<<"Final"<<","<<"Cognitive"<<","<<seed<<","<<"10"<<",";
    ofStreamGenroc <<to_string(varThreshold)<<","<<"None"<<",";

}

void genroc::logcsvClose(){
    ofStreamGenroc.close();
}

void genroc::algorithm(){

    Mat img_cal, img_test;
    foot Foot(false);
    geoproy geoproyTest(true);

    cv::Ptr<cv::BackgroundSubtractorMOG2> mog = cv::createBackgroundSubtractorMOG2(historyMOG, varThreshold,
                                                bShadowDetection);
    mog->setNMixtures(nmixtures);
    mog->setBackgroundRatio(backgroundRatio);
    mog->setShadowValue(0);
    mog->setShadowThreshold(0.3);
    mog->setDetectShadows(true);
    Foot.learningrate = learningRate;

    geoproyTest.readCalibFile(calfileName);
    geoproyTest.genCalibPointsSuelo();
    geoproyTest.genCalibPointsCorner();
    geoproyTest.generateSequence(seed);
    geoproyTest.playsToObjetives();

    Foot.maskConvexPoly(geoproyTest);
    Foot.limit = 0;
    Foot.ofStream = &ofStreamGenroc;

    QImage edit;
    cv::Mat geopro, img;

    stopLoopFlag = false;

    while(!stopLoopFlag){   //(ch != 'q' && ch != 'Q' && !stopLoopFlag) {  //


        //// Transfer Frame Structure ////
        Foot.frameAnt = Foot.frameAct;

        ////////// Frame Acquisition /////////
        if (count_cal < limit) {
            Foot.frameAct.processFrame.release();
            Foot.frameAct.processFrame = imread(filenames_cal[count_cal], CV_LOAD_IMAGE_COLOR);
            substring = filenames_cal[count_cal].substr(pos - digits);
            //cout << substring << endl;
        } else {
            Foot.start = true;
            Foot.frameAct.processFrame.release();
            Foot.frameAct.processFrame = imread(filenames_test[count_test], CV_LOAD_IMAGE_COLOR);
            if (!Foot.frameAct.processFrame.data){
                stopLoopFlag = true;
                break;
            }
            substring = filenames_test[count_test].substr(pos - digits);
            Foot.frame = substring;
            //cout << substring << endl;
        }

        if (Foot.frameAct.processFrame.data && Foot.start) {

            //// Low Step Flag ////
            Foot.step_R = false;
            Foot.step_L = false;

            //// Clone process frame to result frame ////
            Foot.frameAct.resultFrame = Foot.frameAct.processFrame.clone();

            Foot.segmentation(mog);

            if (Foot.firstTimeLBFlag){
                Foot.firstTimeLowerBox(geoproyTest);
                Foot.firstTimeLBFlag = false;
            }

            Foot.getLowerBox();

            Foot.getFeetBoxes(geoproyTest);

            Foot.occlusion = bool(Foot.frameAct.footBoxes.size() <= 1);

            //// Kalman Filter ////
            Foot.kalmanPredict(Foot.Right, dT);
            Foot.kalmanPredict(Foot.Left, dT);

            //// Kalman Update ////
            Foot.kalmanUpdate(Foot.Right);
            Foot.kalmanUpdate(Foot.Left);

            //// Measure Error ////
            Foot.measureError1Np(Foot.Right);
            Foot.measureError1Np(Foot.Left);

            //// Kalman Reset Step ////
            Foot.kalmanResetStep(Foot.Right);
            Foot.kalmanResetStep(Foot.Left);

            Foot.askObjetives(geoproyTest);

            if (!Foot.stop){
                //// Matching Objetives State Machine////
                Foot.stateMachine(geoproyTest);
            }

            Foot.frameAct.processFrame.copyTo(Foot.frameAct.resultFrame);
            Foot.drawingResults();

            img = Foot.frameAct.resultFrame.clone();
            edit = QImage((uchar*) img.data, img.cols, img.rows, int(img.step), QImage::Format_RGB888);
            geoproyTest.addCalibPoints(edit);
            geoproyTest.paintCircles(edit);
            geopro = cv::Mat(edit.height(), edit.width(), CV_8UC3, (uchar*) edit.bits(), static_cast<size_t>(edit.bytesPerLine())); // NOLINT

        }else{
            if(Foot.frameAct.processFrame.data){
                Foot.segmentation(mog);
            }
        }

//        //// SHOW IMAGES ////
//        if (Foot.frameAct.processFrame.data && Foot.start) {
//            imshow("geoProy", geopro);
//            imshow("Segment", Foot.frameAct.segmentedFrame);
//            //imshow("Result", Foot.frameAct.processFrame);
//            ch = char(cv::waitKey(0));
//        }
//
//        else if(Foot.frameAct.processFrame.data){
//            imshow("Segment", Foot.frameAct.segmentedFrame);
//            ch = char(cv::waitKey(0));
//        }

        count_cal++;
        count_test++;
    }
    ch = 0;

}
