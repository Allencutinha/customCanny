#include<iostream>
#include"opencv2/opencv.hpp"
#include "custom_canny.h"
using namespace cv;
using namespace std;

void display(string winName, Mat &image){
   namedWindow(winName);
   imshow(winName, image);
}

void displayBuff(string winName, uchar *inImg, ImgProperties){}

int main(int argc, char* argv[]){
    if(argv[1] == NULL){cout<<"canny inputimage lowThreshold highThreshold hysterisis iterations"<<endl;return -1;}
    Mat image = imread(argv[1],0);
    Mat cannyImage;
    image.copyTo(cannyImage);
    ImgProperties imgProp;
    imgProp.height = image.rows;
    imgProp.width = image.cols;
    imgProp.step = image.step;
    imgProp.channels = image.channels();
    customCanny(image.data, cannyImage.data, imgProp);

    display("input",image);
    display("canny", cannyImage);
    waitKey();
    return 0; 
}

