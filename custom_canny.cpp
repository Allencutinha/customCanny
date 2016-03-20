#include<iostream>
#include <math.h>
#include<cstring>
using namespace std;
#include "custom_canny.h"

static int height;
static int width;
static int step;
static int channels;
static float gaussian5[5][5] = {{2, 4, 5, 4, 2},
                         {4, 9,12, 9, 4},
                         {5,12,15,12, 5},
                         {4, 9,12, 9, 4},
                         {2, 4, 5, 4, 2}};
                         
static void customGaussian(uchar * inImg, uchar * blurImg){
    for(int row = 2; row <height-2; row++){
        for(int col = 2; col<width-2; col++){
            float sum = 0;
            for(int i =0; i < 5; i++){
                for(int j =0; j<5;j++){
                    sum = sum + inImg[(col-2+j)+(row-2+j) * step] * gaussian5[i][j];
		        }
            }
	        sum = sum/159.0;
            blurImg[col + row*step] = (uchar)sum;
        }
    }    
}

void hysterisisEdgeTracking(int* Edges, uchar * cannyEdgeBuff){
    int hysterisisIteartions = 50;
    int count = 0;
    while(count < hysterisisIteartions){
        for(int row=0; row<height; row++){
            for(int col=0; col<width; col++){
                int currPixLoc = col+row*step;
                cannyEdgeBuff[currPixLoc] = 0;
                if(Edges[currPixLoc] !=0){
		            for(int i=0; i<3; i++){
		                for(int j=0;j<3;j++){
			                if(Edges[(col-1+j)+(row-1+i)*step] == 255){
			                    cannyEdgeBuff[currPixLoc] = 255;
			                    break;
			                }
		                }
		            }
                }
            } 
        }
	count++;
    }
}
static void nonMaxSuppressionAndDoublethreshold(int * mag, int * dir, int *Edges){
    int lowThreshold = 20;
    int highThreshold = 50;
    //non maximum suppression of edges
    memset(Edges, 0, height*width *sizeof(int));
    for (int row = 2; row < height-2; row++){
        for (int col =2; col<width-2; col++){
            int currPixLoc = col + row * step;
            int currAngle = dir[currPixLoc];
            int currGrad = mag[currPixLoc];
            int neighbour1;
            int neighbour2;
            switch(currAngle){
            case 0:
                neighbour1 = (col - 1)+row*step;
                neighbour2 = (col + 1)+row*step;
            break;
            case 45:
		        neighbour1 = (col - 1)+(row+1)*step;
                neighbour2 = (col + 1)+(row-1)*step;
            break;
            case 90:
		        neighbour1 = (col - 0)+(row - 1)*step;
                neighbour2 = (col + 0)+(row + 1)*step;
            break;
            case 135:
		        neighbour1 = (col - 1)+(row-1)*step;
                neighbour2 = (col + 1)+(row+1)*step;
            break;
            default:
                cout<<"error computing !! neighbours"<<endl;
            }
            if(currGrad > mag[neighbour1] && currGrad > mag[neighbour2]){
		Edges[currPixLoc] = currGrad;
                //double thesholding
                if(Edges[currPixLoc] < lowThreshold){Edges[currPixLoc] = 0;}
                if(Edges[currPixLoc] > highThreshold){Edges[currPixLoc] = 255;}
            }
             
        }  
    }
}

static int sobelY[3][3] = {{-1,-2,-1},{0,0,0},{1,2,1}};
static int sobelX[3][3] = {{-1,0,1},{-2,0,2},{-1,0,1}};

static void computeGradMagnitudeAndDirection(uchar* inImg, int *mag, int*dir){
    int direction[4] = {0,0,0,0};
    for(int row=1; row<height-1; row++){
    	for(int col =1; col<width-2; col++){
            int gX = 0;
            int gY = 0;
            int currpixelLoc = col + row*step;
            for(int i = 0; i<3; i++){
	            for(int j=0; j<3; j++){
		            gX = gX+ inImg[(col-1+j) + (row-1+j)*step] * sobelX[i][j];
		            gY = gY+ inImg[(col-1+j)+(row-1+i)*step] * sobelY[i][j];
	    	    }
            }
            float angleDeg = (atan2(gY,gX) * 180)/3.14;
            //cout<<angleDeg<<endl;
            int magnitude = mag[currpixelLoc] = sqrt(gX*gX+gY*gY);
            if(angleDeg >= 0){
                if(angleDeg <= 22.5){angleDeg=0;}
                else if(angleDeg <= 67.5){angleDeg=45;}
                else if(angleDeg <= 112.5){angleDeg=90;}
                else if(angleDeg <= 167.5){angleDeg=135;}
                else if((int)(angleDeg) <= 180){angleDeg = 0;}
                else{cout<<"error in gradient direction computation  :  "<<angleDeg<<endl;}
            }
            else{
                if(angleDeg >= -22.5){angleDeg = 0;}
                else if(angleDeg >= -67.5){angleDeg = 135;}
                else if(angleDeg >= -112.5){angleDeg = 90;}
                else if(angleDeg >= -167.5){angleDeg = 45;}
                else if(angleDeg >= -180){angleDeg = 0;}
                else{cout<<"error in gradient direction computation  :  "<<angleDeg<<endl;}
            }
            // code to debug -----------------
            switch((int)angleDeg){
              case 0: direction[0]++; break;
              case 45: direction[1]++; break;
              case 90: direction[2]++; break;
              case 135: direction[3]++; break; 
            //---------------------------------
            }
            dir[currpixelLoc] = angleDeg;
        }
    }
    cout<<" 0 : "<<direction[0]<<" 45 : "<<direction[1]<<" 90 : "<<direction[2]<<" 135 : "<<direction[3]<<endl;
}

void customCanny(uchar * inImg, uchar * cannyImg, ImgProperties & imgProp){
    height = imgProp.height;
    width = imgProp.width;
    step = imgProp.step;
    channels = imgProp.channels;
    uchar * blurImg = new uchar[height*width];
    int * dir = new int[height * width];
    int * mag = new int[height * width];
    int * edges = new int [height * width];

    customGaussian(inImg, blurImg);

    computeGradMagnitudeAndDirection(blurImg, mag, dir);
   
    nonMaxSuppressionAndDoublethreshold(mag, dir, edges);

    hysterisisEdgeTracking(edges, cannyImg);
     
    delete[] blurImg;
    delete[] dir;
    delete[] mag; 
    delete[] edges;
}
