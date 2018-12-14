//
// Created by lalo on 30-08-18.
//

#include "foot.h"


//// CONSTRUCTOR ////
foot::foot(bool start) {
    this -> start = false;
    Reset_R = false;
    Reset_L = false;
    step_R = false;
    step_L = false;
    occlusion = false;
}

//// COLORS ////
cv::Scalar foot::blue(255, 0, 0); // NOLINT
cv::Scalar foot::green(0, 255, 0); // NOLINT
cv::Scalar foot::red(0, 0, 255); // NOLINT

cv::Scalar foot::cyan(255, 255, 0); // NOLINT
cv::Scalar foot::ivory(240, 255, 255); // NOLINT
cv::Scalar foot::blueviolet(226, 43, 138); // NOLINT
cv::Scalar foot::orange(0, 165, 255);


//// Euclidean Distance Between ////
double foot::distance(cv::Point center_kalman, cv::Point center_measured) {
    double dx = 0, dy = 0, result = 0;
    dx = pow(center_kalman.x - center_measured.x, 2);
    dy = pow(center_kalman.y - center_measured.y, 2);
    result = sqrt(dx + dy);
    return result;
}

//// SEGMENTATION AND FOOT BOXES ////

//// Draw Foot Rectangles from Measurement ////
void foot::paintRectangles(cv::Mat &img, std::map<int, cv::Rect> &bboxes, cv::Scalar color){

    std::map<int, cv::Rect>::iterator it, it_end = bboxes.end();
    int i = 0;
    for(it = bboxes.begin(); it != it_end; it++) {
        i += 1;
        cv::rectangle(img, it->second, color, 2);
//        if (i == 2)
//            break;
    }

}

void foot::paintRectanglesVector(vector<Rect> &vectorBoxes, cv::Scalar color){

    for(int i = 0; i < vectorBoxes.size(); i++) {
        cv::rectangle(frameAct.processFrame, vectorBoxes[i], color, 2);
//        if (i == 1)
//            break;
    }

}

//// Convex Polygon Platform Mask ////
void foot::maskConvexPoly(geoproy GeoProy){

    Mat mask = Mat(rowsIm, colsIm, CV_8UC1, Scalar(0)); // NOLINT
    approxPolyDP(GeoProy.roiConvexPoly, GeoProy.roiConvexPoly, 1.0, true);
    fillConvexPoly(mask, &GeoProy.roiConvexPoly[0], (int)GeoProy.roiConvexPoly.size(), 255, 8, 0);
    frameAct.maskConvexPoly = mask.clone();

}

//// Segmentation and Foot Boxes ////
void foot::segmentation(cv::Ptr<cv::BackgroundSubtractorMOG2> mog){

    cv::Mat processMasked, foreGround, labels, stats, centroids;

/*
//    //// Grabacion 3
//    double backgroundRatio = 0.6;
//    double learningRate = 0.005; ////0.005q
//    int    nmixtures = 3;
//    int historyMOG = 200;
//    bool bShadowDetection = true;
//    static cv::Ptr<cv::BackgroundSubtractorMOG2> mog = cv::createBackgroundSubtractorMOG2(history, varThreshold, true);
//    mog->setNMixtures(nmixtures);
//    mog->setBackgroundRatio(backgroundRatio);
//    mog->setShadowValue(0);
//    mog->setShadowThreshold(0.3);
//    mog->setDetectShadows(true);
*/

    //// Start Segmentation ////
    //// Convex Polygon Mask ////
    frameAct.processFrame.copyTo(processMasked, frameAct.maskConvexPoly);

    mog->apply(processMasked, foreGround, 2*learningrate);
    cv::dilate(foreGround, foreGround, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 6))); ////(4,6)
    cv::erode(foreGround, foreGround, cv::getStructuringElement(cv::MORPH_RECT,  cv::Size(3, 3))); ////(4,6)
    cv::connectedComponentsWithStats(foreGround, labels, stats, centroids, 4, CV_32S);

    frameAct.segmentedFrame  =  foreGround.clone();
    frameAct.labelsFrame = labels.clone();

}

//// Order to BoxesVector ////
struct byArea {
    bool operator () (const Rect &a, const Rect &b) {
        return a.width*a.height > b.width*b.height ;
    }
};

//// First Time Box Ant ////
void foot::firstTimeLowerBox(geoproy GeoProy){
    cv::Point2f pin, pout;
    pin.x = 0; pin.y = 0;
    int w = 30, h = 40;

    pout = GeoProy.transformFloor2Image(pin, GeoProy.homography); // NOLINT
    frameAnt.segmLowerBox.x = int(pout.x) - w/2;
    frameAnt.segmLowerBox.y = int(pout.y) - h;
    frameAnt.segmLowerBox.width  = w;
    frameAnt.segmLowerBox.height = h;

}


//// Grabacion 3 ////
void foot::getBlobsBoxes(cv::Mat labels, std::map<int, cv::Rect> &bboxes) {

    int ro = labels.rows, co = labels.cols;
    int label, x, y;

    bboxes.clear();

    for (int j = 0; j < ro; ++j) {
        for (int i = 0; i < co; ++i) {
            label = labels.at<int>(j, i);
            if (label > 0) {                    // Not Background?
                if (bboxes.count(label) == 0) { // New label
                    cv::Rect r(i, j, 1, 1);
                    bboxes[label] = r;
                } else {                       // Update rect
                    cv::Rect &r = bboxes[label];
                    x = r.x + r.width - 1;
                    y = r.y + r.height - 1;
                    if (i < r.x) r.x = i;
                    if (i > x) x = i;
                    if (j < r.y) r.y = j;
                    if (j > y) y = j;
                    r.width = x - r.x + 1;
                    r.height = y - r.y + 1;
                }
            }
        }
    }

}

void foot::orderVectorBoxes(std::map<int, cv::Rect> &bboxes, vector<Rect> &vectorBoxes){

    for(unsigned int j=0; j < bboxes.size(); j++){   //NOLINT
        vectorBoxes.push_back(bboxes[j]);
    }

    std::sort(vectorBoxes.begin(), vectorBoxes.end(), byArea());

};

void foot::distanceFilterBoxes(){

    cv::Point boxActPoint, boxAntPoint;
    double distanceToLowerBox;
    int distanceThreshold = 75;

    vector<Rect> vectorOut;

    vectorOut.clear();

    //cv::rectangle(frameAct.processFrame, frameAnt.segmLowerBox, orange, 3);

    boxAntPoint.x = frameAnt.segmLowerBox.x + frameAnt.segmLowerBox.width/2;
    boxAntPoint.y = frameAnt.segmLowerBox.y + frameAnt.segmLowerBox.height/2;


    for (auto &i : frameAct.segmLowerBoxesVector) {
        boxActPoint.x = i.x + i.width/2;
        boxActPoint.y = i.y + i.height/2;

        distanceToLowerBox = distance(boxActPoint, boxAntPoint);

        if(distanceToLowerBox < distanceThreshold){
            vectorOut.push_back(i);
            //cv::rectangle(frameAct.processFrame, i, green, 2);
            //cv::circle(frameAct.processFrame, boxActPoint, 3, green, -1);
        }else{
            //cv::rectangle(frameAct.processFrame, i, red, 2);
            //cv::circle(frameAct.processFrame, boxActPoint, 3, red, -1);
        }
    }

    //cv::circle(frameAct.processFrame, boxAntPoint, 3, blue, -1);

    frameAct.segmLowerBoxesVector.clear();
    frameAct.segmLowerBoxesVector = vectorOut;

}

void foot::findLowerBox(){

    Rect ROI (0, 0, 1, 1);
    frameAct.segmLowerBox = ROI;

    cv::Point point1, point2;
    double result = 0;
    int threshold = 50;

    if(!frameAct.segmLowerBoxesVector.empty()) {

        point1 = (frameAct.segmLowerBoxesVector[0].br()+frameAct.segmLowerBoxesVector[0].tl())/2;
        point2 = (frameAct.segmLowerBoxesVector[1].br()+frameAct.segmLowerBoxesVector[1].tl())/2;

        result = distance(point1, point2);

        point1.x = frameAct.segmLowerBoxesVector[0].x + frameAct.segmLowerBoxesVector[0].width/2;
        point1.y = frameAct.segmLowerBoxesVector[0].y ;
        point2.x = frameAct.segmLowerBoxesVector[1].x + frameAct.segmLowerBoxesVector[1].width/2;
        point2.y = frameAct.segmLowerBoxesVector[1].y;

        if (result < threshold) {
            frameAct.segmLowerBox.x = std::min(frameAct.segmLowerBoxesVector[0].x , frameAct.segmLowerBoxesVector[1].x);
            frameAct.segmLowerBox.y = std::min(point1.y, point2.y);
            frameAct.segmLowerBox.width = std::max(frameAct.segmLowerBoxesVector[0].x + frameAct.segmLowerBoxesVector[0].width,
                                                   frameAct.segmLowerBoxesVector[1].x + frameAct.segmLowerBoxesVector[1].width)
                                                 - frameAct.segmLowerBox.x;
            frameAct.segmLowerBox.height = std::max(point1.y + frameAct.segmLowerBoxesVector[0].height, point2.y +
                                           frameAct.segmLowerBoxesVector[1].height) - frameAct.segmLowerBox.y;
        }else{
            frameAct.segmLowerBox.x = frameAct.segmLowerBoxesVector[0].x;
            frameAct.segmLowerBox.y = frameAct.segmLowerBoxesVector[0].y;
            frameAct.segmLowerBox.width = frameAct.segmLowerBoxesVector[0].width;
            frameAct.segmLowerBox.height = frameAct.segmLowerBoxesVector[0].height;
        }
    }

//    distFilterBoxesFlag = true;
}

void foot::getLowerBox() {

    frameAct.segmLowerBoxes.clear();
    frameAct.segmLowerBoxesVector.clear();

    getBlobsBoxes(frameAct.labelsFrame, frameAct.segmLowerBoxes);
    orderVectorBoxes(frameAct.segmLowerBoxes, frameAct.segmLowerBoxesVector);

//    if (distFilterBoxesFlag)
    distanceFilterBoxes();

    findLowerBox();

}

void foot::zoneDetectionG3(geoproy GeoProy){

    cv::Point2f lowPointImage;

    lowPointImage.x = frameAct.segmLowerBox.x + frameAct.segmLowerBox.width/2; // NOLINT
    lowPointImage.y = frameAct.segmLowerBox.y + frameAct.segmLowerBox.height;

    frameAct.lowPointFloor = GeoProy.transformFloor2Image(lowPointImage, GeoProy.homographyInv); // NOLINT

    if (frameAct.lowPointFloor.y >= 50) {
        platformZone = 1;
    }else if (frameAct.lowPointFloor.y < 50 && frameAct.lowPointFloor.y > -100){
        platformZone = 2;
    } else {
        platformZone = 3;
    }

}

void foot::linearFunctionPosYG3(){


    int h = frameAct.segmLowerBox.height;
    int y = frameAct.lowPointFloor.y;
    int newHeight;
    double slope, intercept;

    //MinHeight
    int  hsizeMin = 35;

    //Zone1
    double percentInit1 = 25.0;
    double percentFin1 = 0.0;
    double yMin1 = 50;
    double yMax1 = 300;

    //Zone2
    double percentInit2 = 60;
    double percentFin2 = percentInit1;
    double yMin2 = -100.0;
    double yMax2 = 50.0;

    //Zone3
    double percentInit3 = 70;
    double percentFin3 = percentInit2;
    double yMin3 = -300.0;
    double yMax3 = -100.0;

    switch(platformZone) {
        case 1 :
            slope = (percentFin1 - percentInit1)/(yMax1 - yMin1);
            intercept = percentInit1 - yMin1*slope;
            break;
        case 2 :
            slope = (percentFin2 - percentInit2)/(yMax2 - yMin2);
            intercept = percentInit2 - slope*yMin2;
            break;
        default :
            slope = (percentFin3 - percentInit3)/(yMax3 - yMin3);
            intercept = percentInit3 - slope*yMin3;
    }

    if(h > hsizeMin){
        frameAct.segmCutPercent = slope*y + intercept;
    }else{
        frameAct.segmCutPercent = 0;
    }

    newHeight = int (h * ((100 - frameAct.segmCutPercent)/100));

    frameAct.segmLowerBoxFL = frameAct.segmLowerBox;
    frameAct.segmLowerBoxFL.y += (h - newHeight);
    frameAct.segmLowerBoxFL.height = newHeight;

}

void foot::areasideFilter(std::map<int, cv::Rect> &bboxes){

    std::map<int, cv::Rect> areaFiltered, sideFilteredH, sideFilteredW;
    int thresholdArea = 80;
    int thresholdSideH = 8;
    int thresholdSideW = 5;

    for (int i = 0; i < bboxes.size() ; ++i) {
        if (bboxes[i].area() > thresholdArea)
            areaFiltered[i] = bboxes[i];
    }

    for (int j = 0; j < areaFiltered.size() ; ++j) {
        if (areaFiltered[j].height > thresholdSideH)
            sideFilteredH[j] = areaFiltered[j];
    }

    for (int k = 0; k < sideFilteredH.size(); ++k) {
        if (sideFilteredH[k].width > thresholdSideW)
            sideFilteredW[k] = sideFilteredH[k];
    }

    bboxes.clear();
    for (int l = 0; l < sideFilteredW.size(); ++l) {
        bboxes[l] = sideFilteredW[l];
    }

//    cout << bboxes.size() << endl;
}

void foot::leftrightBoxes(){

    int Right = 1;
    int Left  = 2;
    auto boxesSize = frameAct.footBoxes.size();

    switch(boxesSize) {
        case 1 :

            break;
        case 2 :
            frameAct.rightRectFoot = frameAct.footBoxes[Right];
            frameAct.leftRectFoot = frameAct.footBoxes[Right];
            centerMeasured_R.x = frameAct.rightRectFoot.x + frameAct.rightRectFoot.width / 2;
            centerMeasured_R.y = frameAct.rightRectFoot.y + frameAct.rightRectFoot.height;
            centerMeasured_L.x = frameAct.leftRectFoot.x + frameAct.leftRectFoot.width / 2;
            centerMeasured_L.y = frameAct.leftRectFoot.y + frameAct.leftRectFoot.height;
            break;
        case 3 :
            frameAct.rightRectFoot = frameAct.footBoxes[Right];
            frameAct.leftRectFoot = frameAct.footBoxes[Left];
            centerMeasured_R.x = frameAct.rightRectFoot.x + frameAct.rightRectFoot.width / 2;
            centerMeasured_R.y = frameAct.rightRectFoot.y + frameAct.rightRectFoot.height;
            centerMeasured_L.x = frameAct.leftRectFoot.x + frameAct.leftRectFoot.width / 2;
            centerMeasured_L.y = frameAct.leftRectFoot.y + frameAct.leftRectFoot.height;
            break;
        default :
            frameAct.rightRectFoot = frameAct.footBoxes[Right];
            frameAct.leftRectFoot = frameAct.footBoxes[Left];
            centerMeasured_R.x = frameAct.rightRectFoot.x + frameAct.rightRectFoot.width / 2;
            centerMeasured_R.y = frameAct.rightRectFoot.y + frameAct.rightRectFoot.height;
            centerMeasured_L.x = frameAct.leftRectFoot.x + frameAct.leftRectFoot.width / 2;
            centerMeasured_L.y = frameAct.leftRectFoot.y + frameAct.leftRectFoot.height;
            break;
    }

}

void foot::getFeetBoxes(geoproy GeoProy){

    zoneDetectionG3(std::move(GeoProy));
    linearFunctionPosYG3();

    Mat mask = Mat(frameAct.processFrame.size(), CV_8UC1, Scalar(0)); // NOLINT
    rectangle(mask, frameAct.segmLowerBoxFL, Scalar(255), CV_FILLED);

    Mat fgROI = Mat::zeros(frameAct.processFrame.size(), CV_8U);

    //// copia fg a fgROI donde mask es distinto de cero.
    frameAct.segmentedFrame.copyTo(fgROI, mask);

//    imshow("fgROI", fgROI);

    //// aplica componentes conectados otra vez.
    cv::connectedComponents(fgROI, frameAct.labels2Frame, 8, CV_32S);

    frameAct.footBoxes.clear();
    frameAct.footBoxesVector.clear();


    getBlobsBoxes(frameAct.labels2Frame, frameAct.footBoxes);
    areasideFilter(frameAct.footBoxes);


    //orderVectorBoxes(frameAct.footBoxes, frameAct.footBoxesVector);

    leftrightBoxes();

}


//// KALMAN FILTER ////

//// Kalman Initialization////
void foot::kalmanInit(int pie){

    cv::KalmanFilter kf;
    if(pie == Right) {
        kf = kf_R;
    }else{
        kf = kf_L;
    }

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 1e-2f;// 5.0f
    kf.processNoiseCov.at<float>(21) = 1e-2f;// 5.0f
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-2));

}

//// Kalman Prediction ////
void foot::kalmanPredict(int pie, int dT){

    cv::KalmanFilter *kf;
    cv::Mat *state;
    cv::Rect *predRect;
    cv::Point *centerKalman;

    if(pie == Right) {
        kf = &kf_R;
        state = &state_R;
        predRect = &predRect_R;
        centerKalman = &centerKalman_R;
    }else{
        kf = &kf_L;
        state = &state_L;
        predRect = &predRect_L;
        centerKalman = &centerKalman_L;
    }

    /////// Prediction ///////
    *state = (*kf).predict();

    ////// Predicted Rect Red //////
    (*predRect).width = static_cast<int>((*state).at<float>(4));
    (*predRect).height = static_cast<int>((*state).at<float>(5));
    (*predRect).x = static_cast<int>((*state).at<float>(0) - (*state).at<float>(4)/2);
    (*predRect).y = static_cast<int>((*state).at<float>(1) - (*state).at<float>(5));

    //// Predicted Point ////
    (*centerKalman).x = static_cast<int>((*state).at<float>(0));
    (*centerKalman).y = static_cast<int>((*state).at<float>(1));

    (*kf).transitionMatrix.at<float>(2) = dT;
    (*kf).transitionMatrix.at<float>(9) = dT;

}

//// Error Measure ////
void foot::measureError1Np(int pie){
    if (pie == Right){
        errorNpAct1_R = distance(centerKalman_R, centerMeasured_R);
    }else{
        errorNpAct1_L = distance(centerKalman_L, centerMeasured_L);
    }
}

//// Reset Kalman ////
void foot::kalmanResetStep(int pie){

    double error;
    bool reset;

    if (pie == Right){
            Reset_R = abs(errorNpAct1_R) > 2.5;//2
            reset = Reset_R;
    }else{
            Reset_L = abs(errorNpAct1_L) > 2.5;//2
            reset = Reset_L;
    }

    //if(!occlusion & !reset){
    if(!reset){
        if (pie == Right){
            error = (errorNpAct1_R); //+ errorNpAnt1_R)/2;
            if(abs(error) < 2.8) { //2.2
                step_R = true;
                stepPrecision(Right);
            }
            errorNpAnt1_R = errorNpAct1_R;
        }else{
            error = (errorNpAct1_L); //  + errorNpAnt1_L)/2;
            if(abs(error) < 2.8) { //2.2
                step_L = true;
                stepPrecision(Left);
            }
            errorNpAnt1_L = errorNpAct1_L;
        }

    }

}

//// Kalman Correction ////
void foot::kalmanUpdate(int pie){

    cv::KalmanFilter *kf;
    cv::Mat *state;
    cv::Mat *measure;
    bool *reset;

    if(pie == Right) {
        kf = &kf_R;
        state = &state_R;
        measure = &meas_R;
        reset = &Reset_R;
    }else{
        kf = &kf_L;
        state = &state_L;
        measure = &meas_L;
        reset = &Reset_L;
    }

    // Cuando no encuentra caja
    if (frameAct.footBoxes[1].width <= 0){
        notFoundCount++;
        if( notFoundCount >= 100 ){
            found = false;
        }else{
            (*kf).statePost.at<float>(0) = (*state).at<float>(0);
            (*kf).statePost.at<float>(1) = (*state).at<float>(1);//// + (*state).at<float>(5);
            (*kf).statePost.at<float>(2) = (*state).at<float>(2);
            (*kf).statePost.at<float>(3) = (*state).at<float>(3);
            (*kf).statePost.at<float>(4) = (*state).at<float>(4);
            (*kf).statePost.at<float>(5) = (*state).at<float>(5);
        }
    }else{

        if(pie == Right) {
            (*measure).at<float>(0) = frameAct.footBoxes[Right].x + float(frameAct.footBoxes[Right].width) / 2;
            (*measure).at<float>(1) = frameAct.footBoxes[Right].y + float(frameAct.footBoxes[Right].height);
            (*measure).at<float>(2) = (float) frameAct.footBoxes[Right].width;
            (*measure).at<float>(3) = (float) frameAct.footBoxes[Right].height;
        }else if(pie == Left){
            (*measure).at<float>(0) = frameAct.footBoxes[Left].x + float(frameAct.footBoxes[Left].width) / 2;
            (*measure).at<float>(1) = frameAct.footBoxes[Left].y + float(frameAct.footBoxes[Left].height);
            (*measure).at<float>(2) = (float) frameAct.footBoxes[Left].width;
            (*measure).at<float>(3) = (float) frameAct.footBoxes[Left].height;
        }

        if (*reset){ // First detection!
            // >>>> Initialization
            (*kf).errorCovPre.at<float>(0) = 1; // px
            (*kf).errorCovPre.at<float>(7) = 1; // px
            (*kf).errorCovPre.at<float>(14) = 1;
            (*kf).errorCovPre.at<float>(21) = 1;
            (*kf).errorCovPre.at<float>(28) = 1; // px
            (*kf).errorCovPre.at<float>(35) = 1; // px

            (*state).at<float>(0) = (*measure).at<float>(0);
            (*state).at<float>(1) = (*measure).at<float>(1);
            (*state).at<float>(2) = 0;
            (*state).at<float>(3) = 0;
            (*state).at<float>(4) = (*measure).at<float>(2);
            (*state).at<float>(5) = (*measure).at<float>(3);
            // <<<< Initialization

            (*kf).statePost.at<float>(0) = (*state).at<float>(0);
            (*kf).statePost.at<float>(1) = (*state).at<float>(1);
            (*kf).statePost.at<float>(2) = (*state).at<float>(2);
            (*kf).statePost.at<float>(3) = (*state).at<float>(3);
            (*kf).statePost.at<float>(4) = (*state).at<float>(4);
            (*kf).statePost.at<float>(5) = (*state).at<float>(5);

            *reset = false;

        }else{
            (*kf).correct((*measure)); // Kalman Correction
        }
        notFoundCount = 0;
    }

}

////// HACER UNA WEA POR ZONA ////
void foot::stepPrecision(int pie){

    int widthThreshold = 18;

    if (!occlusion) {
        if (pie == Right) {
            if (frameAct.rightRectFoot.width > widthThreshold) {
                frameAct.rightFoot.x = frameAct.rightRectFoot.x + (frameAct.rightRectFoot.width) / 4;
                frameAct.rightFoot.y = frameAct.rightRectFoot.y + (frameAct.rightRectFoot.height *4)/5 ;
            } else {
                frameAct.rightFoot.x = frameAct.rightRectFoot.x + frameAct.rightRectFoot.width / 2;
                frameAct.rightFoot.y = frameAct.rightRectFoot.y + (frameAct.rightRectFoot.height *4)/5;
            }
        } else {
            if (frameAct.leftRectFoot.width > widthThreshold) {
                frameAct.leftFoot.x = frameAct.leftRectFoot.x + (frameAct.leftRectFoot.width * 3) / 4;
                frameAct.leftFoot.y = frameAct.leftRectFoot.y + (frameAct.leftRectFoot.height *4)/5;
            } else {
                frameAct.leftFoot.x = frameAct.leftRectFoot.x + frameAct.leftRectFoot.width / 2;
                frameAct.leftFoot.y = frameAct.leftRectFoot.y + (frameAct.leftRectFoot.height *4)/5;
            }
        }
//        if (platformZone == 1){
//            frameAct.rightFoot.y = frameAct.rightRectFoot.y + (frameAct.rightRectFoot.height *5)/6;
//            frameAct.leftFoot.y = frameAct.leftRectFoot.y + (frameAct.leftRectFoot.height *5)/6;
//        }

    }else{
        if(pie == Right) {
            frameAct.rightFoot.x = frameAct.rightRectFoot.x + frameAct.rightRectFoot.width / 2;
            frameAct.rightFoot.y = frameAct.rightRectFoot.y + (frameAct.rightRectFoot.height *4)/5;
        }else{
            frameAct.leftFoot.x = frameAct.leftRectFoot.x + frameAct.leftRectFoot.width / 2;
            frameAct.leftFoot.y = frameAct.leftRectFoot.y + (frameAct.leftRectFoot.height *4)/5;
        }
    }

}


void foot::logMatchingEvent(int objetive){
    ofstream &fileout = *ofStream;
    std::string::size_type sz;   // alias of size_t
    int i_dec = std::stoi (frame.substr(0,5),&sz) - limit;
    string frameToLog = to_string(i_dec);
    fileout <<frameToLog<<","<<to_string(objetive)<<",";
}

void foot::logEndVideo(){
    ofstream &fileout = *ofStream;
    fileout << "end" << "\n";
}


//// Ask which objetive is near of step given ////
void foot::askObjetives(geoproy GeoProy){

    int totalObjetives = 9;
    double objetiveThreshold = 35;
    double resultDistance;

    cv::Point stepPointR, stepPointL;

//    cout << "Center Out? : " << endl;
//    cout << centerFlagWasOut << endl;

    //// para cada paso R verifico si el punto esta en algun objetivo
    if (step_R){
        stepPointR = geoproy::transformFloor2Image(frameAct.rightFoot, GeoProy.homographyInv);
        for (int i = 1; i <= totalObjetives; ++i) {
            if (i != 5){
                resultDistance = distance(stepPointR, GeoProy.calibPointsFloor[i]);
                if (resultDistance < objetiveThreshold){
                    objetive = i;
                    if (objetive != objetiveAnt){
                        foundMatchR = true;
                        objetiveAnt = objetive;
                        logMatchingEvent(objetive);
                    }
                    break;
                }else{
                    foundMatchR = false;
                }
            }
        }
    }else{
        foundMatchR = false;
    }

    if (step_L){
        stepPointL = geoproy::transformFloor2Image(frameAct.leftFoot, GeoProy.homographyInv);
        for (int i = 1; i <= totalObjetives; ++i) {
            if (i != 5){
                resultDistance = distance(stepPointL, GeoProy.calibPointsFloor[i]);
                if (resultDistance < objetiveThreshold){
                    objetive = i;
                    if (objetive != objetiveAnt){
                        foundMatchL = true;
                        objetiveAnt = objetive;
                        logMatchingEvent(objetive);
                    }
                    break;
                }else{
                    foundMatchL = false;
                }
            }

        }
    }else{
        foundMatchL = false;
    }

    if(step_R) {
        stepPointR = geoproy::transformFloor2Image(frameAct.rightFoot, GeoProy.homographyInv);
        centerFlagIsIn = stepPointR.x > -60 && stepPointR.x < 60 && stepPointR.y > -60 && stepPointR.y < 60;
    }
    if(step_L){
        stepPointL = geoproy::transformFloor2Image(frameAct.leftFoot, GeoProy.homographyInv);
        centerFlagIsIn = stepPointL.x > -60 && stepPointL.x < 60 && stepPointL.y > -60 && stepPointL.y < 60;
    }

    if(!step_R && !step_L){

        stepPointR = geoproy::transformFloor2Image(centerKalman_R, GeoProy.homographyInv);
        stepPointL = geoproy::transformFloor2Image(centerKalman_L, GeoProy.homographyInv);
        centerFlagIsIn = stepPointR.x > -50 && stepPointR.x < 50 && stepPointR.y > -50 && stepPointR.y < 50 &&
                         stepPointL.x > -50 && stepPointL.x < 50 && stepPointL.y > -50 && stepPointL.y < 50;
    }

    if (centerFlagIsIn && centerFlagWasOut){
        centerFlagWasOut = false;
        countCenterOut = 0;
        objetiveAnt = 5;
        logMatchingEvent(5);

    }

}

void foot::centerOutCountFlag(geoproy GeoProy){

    int countCenterOutTh = 10;

    cv::Point2f feetPosFlootR, feetPosFloorL;
    bool centerFlagR, centerFlagL;

    feetPosFlootR = GeoProy.transformFloor2Image(centerMeasured_R, GeoProy.homographyInv); // NOLINT
    feetPosFloorL = GeoProy.transformFloor2Image(centerMeasured_L, GeoProy.homographyInv); // NOLINT

    //// logica inversa -> esta afuera = true
    centerFlagR = ((feetPosFlootR.y < -60 || feetPosFlootR.y > 60) ||
                    (feetPosFlootR.x < -60 || feetPosFlootR.x > 60));

    centerFlagL = ((feetPosFloorL.y < -60 || feetPosFloorL.y > 60) ||
                    (feetPosFloorL.x < -60 || feetPosFloorL.x > 60));

    if(centerFlagL && centerFlagR){
        countCenterOut++;
    }

    centerFlagWasOut = countCenterOut >= countCenterOutTh;
}




//// DRAW RESULTS ////

//// Draw Results to Image ////
void foot::drawingResults() {

    //// ARCHIREVISADO ////
    /*
    cv::rectangle(frameAct.resultFrameR, frameAct.segmLowerBox, cyan, 2);
    cv::circle(frameAct.resultFrameR,(frameAct.segmLowerBox.br()+frameAct.segmLowerBox.tl())/2, 3, cyan, -1);
    cv::circle(frameAct.resultFrameR, (frameAnt.segmLowerBox.br()+frameAnt.segmLowerBox.tl())/2, 3, blue, -1);

    cv::rectangle(frameAct.resultFrameR, frameAct.segmLowerBoxFL, cyan, 2);
    cv::circle(frameAct.resultFrameR,(frameAct.segmLowerBoxFL.br()+frameAct.segmLowerBoxFL.tl())/2, 3, cyan, -1);

    cv::rectangle(frameAct.resultFrameL, frameAct.segmLowerBox, cyan, 2);
    cv::circle(frameAct.resultFrameL,(frameAct.segmLowerBox.br()+frameAct.segmLowerBox.tl())/2, 3, cyan, -1);
    cv::circle(frameAct.resultFrameL, (frameAnt.segmLowerBox.br()+frameAnt.segmLowerBox.tl())/2, 3, blue, -1);

    cv::rectangle(frameAct.resultFrameL, frameAct.segmLowerBoxFL, cyan, 2);
    cv::circle(frameAct.resultFrameL,(frameAct.segmLowerBoxFL.br()+frameAct.segmLowerBoxFL.tl())/2, 3, cyan, -1);
    */

//    cv::rectangle(frameAct.resultFrame, frameAct.segmLowerBox, cyan, 2);
//    cv::circle(frameAct.resultFrame,(frameAct.segmLowerBox.br()+frameAct.segmLowerBox.tl())/2, 3, cyan, -1);
//    cv::circle(frameAct.resultFrame, (frameAnt.segmLowerBox.br()+frameAnt.segmLowerBox.tl())/2, 3, blue, -1);

    cv::rectangle(frameAct.resultFrame, frameAct.segmLowerBoxFL, cyan, 2);
    cv::circle(frameAct.resultFrame,(frameAct.segmLowerBoxFL.br()+frameAct.segmLowerBoxFL.tl())/2, 3, cyan, -1);

    cv::rectangle(frameAct.resultFrame, frameAct.rightRectFoot, green, 2);
    cv::rectangle(frameAct.resultFrame, frameAct.leftRectFoot, green, 2);

    //// Step Detected or Kalman Filter////
    if (step_R) {
        cv::rectangle(frameAct.resultFrame, predRect_R, blue, 2);
        cv::circle(frameAct.resultFrame, frameAct.rightFoot, 2, blue, -1);
    }
//    else{
//        cv::rectangle(frameAct.resultFrame, predRect_R, red, 2);
//        cv::circle(frameAct.resultFrame, centerKalman_R, 2, red, -1);
//    }

    if (step_L) {
        cv::rectangle(frameAct.resultFrame, predRect_L, blue, 2);
        cv::circle(frameAct.resultFrame, frameAct.leftFoot, 2, blue, -1);
    }
//    else{
//        cv::rectangle(frameAct.resultFrame, predRect_L, red, 2);
//        cv::circle(frameAct.resultFrame, centerKalman_L, 2, red, -1);
//    }






    /*
    //// Foots Rectangles ////
    //paintRectangles(frameAct.resultFrame, frameAct.footBoxes, green);

    //// Measured Centers ////
    //cv::circle(frameAct.resultFrame, centerMeasured_R, 3, green, -1);
    //cv::circle(frameAct.resultFrame, centerMeasured_L, 3, green, -1);




//    if(!occlusion) {

        //// Kalman Prediction ////
//        cv::rectangle(frameAct.resultFrame, predRect_R, CV_RGB(255, 0, 0), 2);
//        cv::rectangle(frameAct.resultFrame, predRect_L, CV_RGB(255, 0, 0), 2);
//        cv::circle(frameAct.resultFrame, centerKalman_R, 2, CV_RGB(255, 0, 0), -1);
//        cv::circle(frameAct.resultFrame, centerKalman_L, 2, CV_RGB(255, 0, 0), -1);

        //// Template Boxes Generated in Normal Detection ////
        //paintRectangles(frameAct.resultFrame, frameAct.tempBoxes, blueviolet);


//    }
//    //// Matchscore Partial Occlusion ////
//    }else{
//
//        //// Kalman Prediction ////
//        cv::rectangle(frameAct.resultFrame, predRect_R, CV_RGB(255, 0, 0), 2);
//        cv::rectangle(frameAct.resultFrame, predRect_L, CV_RGB(255, 0, 0), 2);
//        cv::circle(frameAct.resultFrame, centerKalman_R, 2, CV_RGB(255, 0, 0), -1);
//        cv::circle(frameAct.resultFrame, centerKalman_L, 2, CV_RGB(255, 0, 0), -1);
//
//        //// Predicted Boxes ////
//        paintRectangles(frameAct.resultFrame, frameAct.footBoxes, cyan);
//
//
//        namedWindow("Occlusion", WINDOW_AUTOSIZE);
//        namedWindow("TempR", WINDOW_AUTOSIZE);
//        namedWindow("TempL", WINDOW_AUTOSIZE);
//        namedWindow("MatchR", WINDOW_AUTOSIZE);
//        namedWindow("MatchL", WINDOW_AUTOSIZE);
//
//        Size sizeoccBox(frameAct.occlusionFrame.cols*10, frameAct.occlusionFrame.rows*10);
//        Size sizetempBoxR(frameAnt.templateFrameR.cols*10, frameAnt.templateFrameR.rows*10);
//        Size sizetempBoxL(frameAnt.templateFrameL.cols*10, frameAnt.templateFrameL.rows*10);
//        Size sizematchScoreR(frameAct.matchScoreShowR.cols*10, frameAct.matchScoreShowR.rows*10);
//        Size sizematchScoreL(frameAct.matchScoreShowL.cols*10, frameAct.matchScoreShowL.rows*10);
//
//        //// Paint Local Max Points ////
//        for (const auto &i : maxLocR) {
//            circle(frameAct.matchScoreShowR, i, 1, CV_RGB(0,0,255), -1);
//        }
//        for (const auto &i : maxLocL) {
//            circle(frameAct.matchScoreShowL, i, 1, CV_RGB(0,0,255), -1);
//        }
//
//        circle(frameAct.matchScoreShowR, maxlocSelectedR, 1, blueviolet, -1);
//        circle(frameAct.matchScoreShowL, maxlocSelectedL, 1, blueviolet, -1);
//
//        resize(frameAct.occlusionFrame, frameAct.occlusionFrame, sizeoccBox);
//        resize(frameAnt.templateFrameR, frameAnt.templateFrameR, sizetempBoxR);
//        resize(frameAnt.templateFrameL, frameAnt.templateFrameL, sizetempBoxL);
//        resize(frameAnt.tempmaskFrameR, frameAnt.tempmaskFrameR, sizetempBoxR);
//        resize(frameAnt.tempmaskFrameL, frameAnt.tempmaskFrameL, sizetempBoxL);
//        resize(frameAct.matchScoreShowR, frameAct.matchScoreShowR, sizematchScoreR);
//        resize(frameAct.matchScoreShowL, frameAct.matchScoreShowL, sizematchScoreL);
//
//        imshow("Occlusion", frameAct.occlusionFrame);
//        imshow("TempR", frameAnt.tempmaskFrameR);
//        imshow("TempL", frameAnt.tempmaskFrameL);
//        imshow("MatchR", frameAct.matchScoreShowR);
//        imshow("MatchL", frameAct.matchScoreShowL);
//
//
//    }


     */


}

//// Clear Variables ////
void foot::clearVariables(){



}