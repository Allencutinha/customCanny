#include<iostream>
#include"opencv2/opencv.hpp"
using namespace cv;
using namespace std;
int height, width, step, channel;
void display(string winName, Mat &image){
	namedWindow(winName);
	imshow(winName, image);
}

float gaussian5[5][5] = {{2, 4, 5, 4, 2},
                         {4,9,12,9,4},
                         {5,12,15,12,5},
                         {4,9,12,9,4},
                         {2,4,5,4,2}};
void customGaussian(uchar * inImg, uchar * blurImg){
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

int sobelY[3][3] = {{-1,-2,-1},{0,0,0},{1,2,1}};
int sobelX[3][3] = {{-1,0,1},{-2,0,2},{-1,0,1}};

void computeGradMagnitudeAndDirection(uchar* inImg, int *mag, int*dir){
    Mat dispEdge, dispDirection, dispThinEdge;
    dispEdge.create(height, width, CV_8UC1);
    dispThinEdge.create(height, width, CV_8UC1);
    dispDirection.create(height, width, CV_8UC1);
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
            dispEdge.data[currpixelLoc] = (uchar)((magnitude * 255)/1442);
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
            switch((int)angleDeg){
              case 0: direction[0]++; break;
              case 45: direction[1]++; break;
              case 90: direction[2]++; break;
              case 135: direction[3]++; break;
            }
            dir[currpixelLoc] = angleDeg;
            if(angleDeg*2 > 255){angleDeg = 255;}else{angleDeg = angleDeg*2;}
            dispDirection.data[currpixelLoc] = angleDeg;  
        }
    }
    cout<<" 0 : "<<direction[0]<<" 45 : "<<direction[1]<<" 90 : "<<direction[2]<<" 135 : "<<direction[3]<<endl;
    display("sobel", dispEdge);
    display("direction", dispDirection);

    //non maximum suppression of edges
    int * Edges = new int [height * width];
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
                dispThinEdge.data[currPixLoc] = dispEdge.data[currPixLoc];
            }
        }  
    }
    display("supprion_edges", dispThinEdge);

    // double thresholding , this step can be merged with the previous step
    int lowThreshold = 20;
    int highThreshold = 50;
    for(int row = 0; row<height; row++){
	for(int col = 0; col < width; col++){
	    int currPixLoc = col + row * step;
            if(Edges[currPixLoc] < lowThreshold){Edges[currPixLoc] = 0; dispThinEdge.data[currPixLoc] = 0;}
            if(Edges[currPixLoc] > highThreshold){Edges[currPixLoc] = 255; dispThinEdge.data[currPixLoc] = 255;}
        }
    }
    display("thresholding_", dispThinEdge);

   //hystersis rejecting unconnected weak edges
    int hysterisisIteartions = 50;
    int count = 0;
    uchar * cannyEdgeBuff = new uchar[height * width];
    Mat cannyImage;
    cannyImage.create(height, width, CV_8UC1);  
    while(count < hysterisisIteartions){
        for(int row=0; row<height; row++){
            for(int col=0; col<width; col++){
                int currPixLoc = col+row*step;
                cannyEdgeBuff[currPixLoc] = 0;
                cannyImage.data[currPixLoc] = 0;
                if(Edges[currPixLoc] !=0){
		    for(int i=0; i<3; i++){
		        for(int j=0;j<3;j++){
			    if(Edges[(col-1+j)+(row-1+i)*step] == 255){
			        cannyEdgeBuff[currPixLoc] = 255;
			        cannyImage.data[currPixLoc] = 255;
			        break;
			    }
		        }
		    }
                }
            } 
        }
	count++;
    }
    display("cannyEdges", cannyImage);
}
int main(int argc, char* argv[]){
    if(argv[1] == NULL){cout<<"canny inputimage lowThreshold highThreshold hysterisis iterations"<<endl;return -1;}
    Mat image = imread(argv[1],0);
    height = image.rows;
    width   =image.cols;
    step = image.step;
    channel = image.channels();
    Mat blurImage;
    image.copyTo(blurImage);
    
    customGaussian(image.data, blurImage.data);
    int * mag = new int[height*width];
    int * dir = new int[height*width];
    computeGradMagnitudeAndDirection(blurImage.data, mag, dir);

    display("input",image);
    display("gaussian", blurImage);
    waitKey();
    return 0; 
}
