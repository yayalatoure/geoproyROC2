//    //// Video Original
//    QString fileName = "/home/lalo/Desktop/Data_Videos/VideoOriginal/CALIB/default_calib1.yml";
//    string path_cal  = "/home/lalo/Dropbox/Proyecto IPD441/Data/Videos/1_CAMARA/CALIBRACION01/*.jpg";
//    string path_test = "/home/lalo/Dropbox/Proyecto IPD441/Data/Videos/1_CAMARA/TEST01/*.jpg";
//    int teststart = 195;
//    int count_test = teststart+145, count_cal = 0, limit = 5;
//    int seed = 85062514;


//    //// UPLA Grabacion 1
//    string path_cal  = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion1/CAL_Test1/*.jpg";
//    QString fileName = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion1/default_calib.yml";
//
////    //// Player1
////    string path_test = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion1/Player1/*.jpg";
////    int count_test = 467, count_cal = 0, limit = 5;
////    int seed = 1170201133;
//
////    //// Player2
////    string path_test = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion1/Player2/*.jpg";
////    int count_test = 1210, count_cal = 0, limit = 5;
////    int seed = 176218894;
//
//    //// Player3
//    string path_test = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion1/Player3/*.jpg";
//    int teststart = 165+145;
//    int count_test = teststart, count_cal = 0, limit = 5;
//    int seed = 463094935;


    //// UPLA Grabacion 2
    string path_cal  = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion2/CALIB/CAL1/*.jpg";
    QString fileName = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion2/CALIB/default_calib_g2_1.yml";

//    //// Player 1
//    string path_test = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion2/DATA1/Data1Player1/*.jpg";
//    int count_test = 676, count_cal = 0, limit = 5;
//    int seed = 6001;

//    //// Player 5
//    string path_test = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion2/DATA1/Data1Player5/*.jpg";
//    int count_test = 509-30, count_cal = 0, limit = 30;
//    int seed = 6004;

void readCalibFile(QString fileName);

void geoproy::readCalibFile(QString fileName)

void foot::distanceFilterBoxes(vector<Rect> &vectorBoxes){

    cv::Point boxActPoint, boxAntPointR, boxAntPointL;
    double distanceR, distanceL;
    int distanceThreshold = 100;

    vector<Rect> vectorOut;

    boxAntPointR.x = frameAnt.footRight.x;
    boxAntPointR.y = frameAnt.footRight.y;

    boxAntPointL.x = frameAnt.footLeft.x;
    boxAntPointL.y = frameAnt.footLeft.y;


    for (int i = 0; i < vectorBoxes.size() ; ++i) {

        boxActPoint.x = vectorBoxes[i].x + vectorBoxes[i].width/2;
        boxActPoint.y = vectorBoxes[i].y + vectorBoxes[i].height;

        distanceR = distance(boxActPoint, boxAntPointR);
        distanceL = distance(boxActPoint, boxAntPointL);

        if((distanceR < distanceThreshold) || (distanceL < distanceThreshold)){
            vectorOut.push_back(vectorBoxes[i]);
        }
    }

    vectorBoxes.clear();
    vectorBoxes = vectorOut;



}

    //// Player 6
    string path_test = "/home/lalo/Desktop/Data_Videos/UPLAGrabacion2/DATA1/Data1Player6/*.jpg";
    int teststart = 353;
    int count_test = teststart-30, count_cal = 0, limit = 30;
    int seed = 6005;


    fileName = "/home/lalo/Dropbox/Proyecto IPD441/NeuroMisil_Lalo/NeuroMisil/Logging/pasos_result.csv";
    ofstream ofStream(fileName);
    size_t pos = filenames_test[count_test].find(".jpg");
    ofStream << "Frame" << "," << "CX_Paso" << "," << "CY_Paso" << "," << "W_Paso" << "." << "H_Paso" << "," << "Pie" << "\n";

    fileout << substring << "," << r.center().x() << "," << r.center().y() << "," <<
    (*img_out).fboxes[pie].height <<","<<  (*img_out).fboxes[pie].width <<","<<"Rigth"<<"\n";





