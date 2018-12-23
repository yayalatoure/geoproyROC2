

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

    ofStreamGenroc <<"Frame1"<<","<<"Target1"<<","<<"Frame2"<<","<<"Target2"<<",";
    ofStreamGenroc <<"Frame3"<<","<<"Target3"<<","<<"Frame4"<<","<<"Target4"<<",";
    ofStreamGenroc <<"Frame5"<<","<<"Target5"<<","<<"Frame6"<<","<<"Target6"<<",";
    ofStreamGenroc <<"Frame7"<<","<<"Target7"<<","<<"Frame8"<<","<<"Target8"<<",";
    ofStreamGenroc <<"Frame9"<<","<<"Target9"<<","<<"Frame10"<<","<<"Target10"<<",";
    ofStreamGenroc <<"Frame11"<<","<<"Target11"<<","<<"Frame12"<<","<<"Target12"<<",";
    ofStreamGenroc <<"Frame13"<<","<<"Target13"<<","<<"Frame14"<<","<<"Target14"<<",";
    ofStreamGenroc <<"Frame15"<<","<<"Target15"<<","<<"Frame16"<<","<<"Target16"<<",";
    ofStreamGenroc <<"Frame17"<<","<<"Target17"<<","<<"Frame18"<<","<<"Target18"<<",";
    ofStreamGenroc <<"Frame19"<<","<<"Target19"<<","<<"Frame20"<<","<<"Target20"<<"," << "\n";

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

    ofStreamGenroc <<to_string(numVideo)<<","<<"Init"<<","<<"Final"<<","<<"Cognitive"<<","<<to_string(seed)<<","<<"10"<<",";
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

    while(ch != 'q' && ch != 'Q' && !stopLoopFlag) {  //


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
            Foot.centerOutCountFlag(geoproyTest);

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

        //// SHOW IMAGES ////
        if (Foot.frameAct.processFrame.data && Foot.start) {
            //imshow("geoProy", geopro);
//            imshow("Segment", Foot.frameAct.segmentedFrame);
//            //imshow("Result", Foot.frameAct.processFrame);
            //ch = char(cv::waitKey(0));
        }

//        else if(Foot.frameAct.processFrame.data){
//            imshow("Segment", Foot.frameAct.segmentedFrame);
//            ch = char(cv::waitKey(0));
//        }

        count_cal++;
        count_test++;
    }

    Foot.logEndVideo();

}
