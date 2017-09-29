											#define HAVE_OPENCV_FLANN
											#define HAVE_OPENCV_IMGPROC
											#define HAVE_OPENCV_VIDEO
											#define HAVE_OPENCV_OBJDETECT
											#define HAVE_OPENCV_CALIB3D
											#define HAVE_OPENCV_ML
											#define HAVE_OPENCV_HIGHGUI
											#define HAVE_OPENCV_CONTRIB
											#define HAVE_OPENCV_PHOTO
											#define HAVE_OPENCV_FEATURES2D


											#include<opencv2/opencv.hpp>
											#include<opencv2/core/core_c.h>
											#include<opencv2/core/core.hpp>


											#include<iostream>

											# include "opencv2/core/core.hpp"
											# include "opencv2/features2d/features2d.hpp"
											# include "opencv2/highgui/highgui.hpp"
											# include "opencv2/calib3d/calib3d.hpp"
											# include "opencv2/nonfree/features2d.hpp"


											#include "opencv2/opencv_modules.hpp"

											using namespace std;
											using namespace cv;






											//#include "wqueue.h"
											//#include "thread.h"
											//#include "imgframe.h"
											#include <iostream>
											#include <fstream>
											#include<string>
											#include <iomanip>

											#include <opencv2/core/core.hpp>


											#include <opencv2/calib3d/calib3d.hpp>
											#include <opencv2/highgui//highgui.hpp>

											#include "dmtx.h"   ////////////////////////////////////////////////dmtx 

											#include <cstdio>
											#include <ctime>



											#include<stdio.h>
											#include<string.h>

											#include<stdio.h>
											#include<sys/time.h>
											#include<unistd.h>






//img_size: width=640, height=480

double width=640;
double height=480;

double unitLen=0;

class Line
{
public:

	Line(double k_, double b_)
	{
		 k=k_;b=b_;
			
		 Point2d P0_, P640_, P_0, P_480;

		 P0_.x=0;
		 P0_.y=b;

		P640_.x=639;
		P640_.y=k*P640_.x+b;

		P_0.x=-b/k;
		P_0.y=0;
		

		P_480.x=(479-b)/k;
		P_480.y=479;

		vector<Point2d>vtP;

		if(  (P0_.x>=0 && P0_.x<width)  && (P0_.y>=0)&&(P0_.y<height)  )
			vtP.push_back(P0_);
		
		if(  (P640_.x>=0 && P640_.x<width)  && (P640_.y>=0)&&(P640_.y<height)  )
			vtP.push_back(P640_);

		if(  (P_0.x>=0 && P_0.x<width)  && (P_0.y>=0)&&(P_0.y<height)  )
			vtP.push_back(P_0);

		if(  (P_480.x>=0 && P_480.x<width)  && (P_480.y>=0)&&(P_480.y<height)  )
			vtP.push_back(P_480);

		if(vtP.size()<2)
		cout<<"size<2"<<endl;
		startp=vtP.at(0);
		endp=vtP.at(1);
		
	
	}

	Line(){}

	Line(Point2d startp_, Point2d endp_)
	{
		startp=startp_;
		endp=endp_;

		k=(endp.y -startp.y)/(endp.x-startp.x);

		b=startp.y - k*startp.x;
	}



	void LinePrint()
	{
		cout<<"line.startp="<<startp<<"  line.endp="<<endp<<endl;
	}	



	Point2d startp;
	Point2d endp;
	double k, b;  // lean rate (slope), cut distance(intercept).
	
};


///////////////

typedef struct Intersect_Surround_
{
	Point2d P;
	Point2d vecfirst;
	Point2d vecsecond;   // right hand order while  "linefirst Cross* linesecond = positive z"  
}Intersect_Sur;




double dotMul_point2d(Point2d& p1, Point2d& p2)
{
	double result;
	result= p1.x*p2.x +p1.y*p2.y;

	return result;
}


void cross_matvec(Mat& a, Mat& b, Mat& result)  // a: 3*1   b:3*1   result:3*1    // correct
{
	Mat a_cross= (Mat_<double>(3,3)<< 0,-a.at<double>(2,0),a.at<double>(1,0),
					  a.at<double>(2,0),0, -a.at<double>(0,0),
					  -a.at<double>(1,0),a.at<double>(0,0),0);

	result=a_cross*b;
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Intersect_Sur IS_12;
						Intersect_Sur IS_14;
						Intersect_Sur IS_34;
						Intersect_Sur IS_23;
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void orderIS(Point2d& intersection_diag, Intersect_Sur& IS)  // Reorder IS
{

		Point2d vec_1=IS.vecfirst;
		Point2d vec_2=IS.vecsecond;

			double diag_Dot_vec_1=dotMul_point2d(intersection_diag,vec_1);
			if(diag_Dot_vec_1 > 0)
				IS.vecfirst=vec_1;	
			else
				IS.vecfirst=-vec_1;	

			

			
			double diag_Dot_vec_2=dotMul_point2d(intersection_diag,vec_2);
			if(diag_Dot_vec_2 > 0)
				IS.vecsecond=vec_2;	
			else
				IS.vecsecond=-vec_2;
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&here align the positive and negtive direction


			double Angle_IS_vecfirst=atan2(IS.vecfirst.y, IS.vecfirst.x)*180/3.14159265;
			double Angle_IS_vecsecond=atan2(IS.vecsecond.y, IS.vecsecond.x)*180/3.14159265;


cout<<"Angle first="<<Angle_IS_vecfirst<<endl;
cout<<"Angle second="<<Angle_IS_vecsecond<<endl;

			if(abs(Angle_IS_vecfirst-Angle_IS_vecsecond) < 180)// normal case // near 180 and near -180 cause wrong, here check
			{	

				if(Angle_IS_vecfirst > Angle_IS_vecsecond)  // switch
				{

					
					Point2d temp=IS.vecsecond;
					IS.vecsecond=IS.vecfirst;		
					IS.vecfirst=temp;
				}
			}    // big bug, if no }, else will trace back to last little if
		
			else  // the round over case

			{
				if(Angle_IS_vecfirst < Angle_IS_vecsecond)  //  in this case, opposite to the above one 
				{

					
					Point2d temp=IS.vecsecond;
					IS.vecsecond=IS.vecfirst;		
					IS.vecfirst=temp;
				}

			}

cout<<"IS.vecfirst="<<IS.vecfirst<<endl;
cout<<"IS.vecsecond="<<IS.vecsecond<<endl;

		
	//here we get a correct IS in good order.
}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


double lenTwoPoints(Point2d& p1,Point2d& p2)
{
	return sqrt ( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) );
}


Point2d nomalizeVec(Point2d p)
{
	Point2d temp;
	double len=sqrt(p.x*p.x+p.y*p.y);
	temp.x=(1.0/len)*p.x;
	temp.y=(1.0/len)*p.y;

	return temp;
}	




//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//globally defined

Intersect_Sur leftCorner;

bool judgeLeftcorner(Intersect_Sur& IS, Mat& bina)
{

	Point2d unit_first= unitLen* nomalizeVec(IS.vecfirst);
	Point2d unit_second= unitLen* nomalizeVec(IS.vecsecond);

	//cout<<"unit_first="<<unit_first<<endl;
	//cout<<"unit_second="<<unit_second<<endl;

	Point2d basep=IS.P+ 2.5*unit_second + 2.5*unit_first;  //drift add here


	int num_first=0;
	for(int i=0;i<12;i++)    // firstline, column scan "L"
		{
			Point2d temp= basep+ i*unit_first;
			if(    bina.at<uchar>( cvRound(temp.y), cvRound(temp.x) ) == 0 )
				num_first++;
		}
	
	int num_second=0;
	for(int i=0;i<12;i++)    // firstline, column scan "L"
		{
			Point2d temp= basep+ i*unit_second;
			if(    bina.at<uchar>( cvRound(temp.y), cvRound(temp.x) ) == 0 )
				num_second++;
		}

	//cout<<"num_first="<<num_first<<endl;
	//cout<<"num_second="<<num_second<<endl;

	if(num_first>=8 && num_second>=8 )
		{
			leftCorner.P=IS.P;
			leftCorner.vecfirst=IS.vecfirst;
			leftCorner.vecsecond=IS.vecsecond;
			cout<<"leftcorner found!"<<endl;

			cout<<"leftCorner.P="<<leftCorner.P<<endl;
			cout<<"leftCorner.vecfirst="<<leftCorner.vecfirst<<endl;
			cout<<"leftCorner.vecsecond="<<leftCorner.vecsecond<<endl;
			return 1;
		}
	else
		return 0;



}

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
/////////////////////////////calculate two lines' intersection point 
 
bool intersectionOf2lines(Line& line1, Line& line2, Point2d& output)  // bool is to check inrange or out of range

{
	output.x= (line1.b- line2.b)/(line2.k - line1.k);
	
	output.y= (line1.b*line2.k - line2.b*line1.k)/(line2.k-line1.k);

	
/////////check range 


	if(output.x <= max(line1.startp.x, line1.endp.x)  && output.x >= min(line1.startp.x, line1.endp.x)  &&
	   output.x <= max(line2.startp.x, line2.endp.x)  && output.x >= min(line2.startp.x, line2.endp.x)  &&
	   output.y <= max(line1.startp.y, line1.endp.y)  && output.y >= min(line1.startp.y, line1.endp.y)  &&
    	   output.y <= max(line2.startp.y, line2.endp.y)  && output.y >= min(line2.startp.y, line2.endp.y)  )     // embedded in the litte range
		 
		return 1;
	else
		return 0;
		
}


//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------

void lineFitting(vector<Point2d>vP, double& k, double& b)
{
	Mat A=(Mat_<double>(2,2)<<0.0,0.0,0.0,0.0);
	Mat d=(Mat_<double>(2,1)<<0.0,0.0);

	Mat solution=(Mat_<double>(2,1)<<0.0,0.0);

	double N= vP.size();

	double sum_x_squ=0;
	double sum_x=0;
	double sum_xy=0;
	double sum_y=0;
	for(int i=0;i<N;i++)
		{
			sum_x_squ+=vP.at(i).x*vP.at(i).x;
			sum_x+=vP.at(i).x;
			sum_xy+=vP.at(i).x*vP.at(i).y;
			sum_y+=vP.at(i).y;
		
		}
	A.at<double>(0,0)= sum_x_squ;
	A.at<double>(0,1)= sum_x;
	A.at<double>(1,0)= sum_x;
	A.at<double>(1,1)= N;
	
	
	d.at<double>(0,0)= sum_xy;
	d.at<double>(1,0)= sum_y;

	solution= A.inv() * d;

	k=solution.at<double>(0,0);
	b=solution.at<double>(1,0);
}

//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------





//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------


//R:Right; L:Left; T: Top; B: Bottom
//only consider the first black points;

void L2RScan(Mat& img, Line& line1, Line& line2, vector<Point2d>& EdgePoints)
{
	int num=0;
	for (int i=0;i<img.rows;i++)
	{
		for(int j=0;j<img.cols;j++)
			{
				if(img.at<uchar>(i,j) > 50)
					{
						Point2d curp=Point2d(j,i);
						EdgePoints.push_back(curp);
						//cout<<(int)(img.at<uchar>(i,j))<<endl;
						num++;
						break;
					}
			
			}
	}

	cout<<"num="<<num<<endl;


/////////////line extract
	vector<Point2d>::iterator iter; 	

	vector<Point2d> line1_points;
	vector<Point2d> line2_points;

	iter=EdgePoints.begin();
	line1_points.assign(iter, iter+30 );

	iter=iter=EdgePoints.end();
	line2_points.assign(iter-30, iter);

	cout<<"fuck"<<endl;
	for(int i=0;i<20;i++)
		cout<<line2_points.at(i)<<endl;

/////fitting line
	double lk1,lb1,lk2,lb2;
	lineFitting(line1_points,  lk1,  lb1);
	lineFitting(line2_points,  lk2,  lb2);

	cout<<"fuck"<<endl;
	line1=Line(lk1,lb1);
	line2=Line(lk2,lb2);
	cout<<line1.startp<<endl;
	cout<<line2.endp<<endl;
		
}

//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------

void R2LScan(Mat& img, Line& line1, Line& line2, vector<Point2d>& EdgePoints)
{
	int num=0;
	for (int i=0;i<img.rows;i++)
	{
		for(int j=img.cols-1;j>=0;j--)
			{
				if(img.at<uchar>(i,j) > 50)
					{
						Point2d curp=Point2d(j,i);
						EdgePoints.push_back(curp);
						//cout<<(int)(img.at<uchar>(i,j))<<endl;
						num++;
						break;
					}
			
			}
	}

	cout<<"num="<<num<<endl;


/////////////line extract
	vector<Point2d>::iterator iter; 	

	vector<Point2d> line1_points;
	vector<Point2d> line2_points;

	iter=EdgePoints.begin();
	line1_points.assign(iter, iter+30 );

	iter=iter=EdgePoints.end();
	line2_points.assign(iter-30, iter);

	cout<<"fuck"<<endl;
	for(int i=0;i<20;i++)
		cout<<line2_points.at(i)<<endl;

/////fitting line
	double lk1,lb1,lk2,lb2;
	lineFitting(line1_points,  lk1,  lb1);
	lineFitting(line2_points,  lk2,  lb2);

	cout<<"fuck"<<endl;
	line1=Line(lk1,lb1);
	line2=Line(lk2,lb2);
	cout<<line1.startp<<endl;
	cout<<line2.endp<<endl;	
}
//void T2BScan(Mat& img, Line& line1, Line& line2)
//void B2TScan(Mat& img, Line& line1, Line& line2)




//=========================================================================================
//==========================================================================================for RS
//==========================================================================================


void reorder(Mat& raw, uchar* zo)
{
	Mat temp(raw.rows,raw.cols,CV_8UC1, Scalar(0) );

	for(int i=0;i<raw.rows;i++)
		for(int j=0;j<raw.cols;j++)
		{
			{
			if(raw.at<uchar>(i,j)==1)
				temp.at<uchar>(10-1-i,j)=16;
			else
				temp.at<uchar>(10-1-i,j)=23;
			}	

			zo[(10-1-i)*raw.rows+j]=temp.at<uchar>(10-1-i,j);
		}
	cout<<"temp="<<endl<<temp<<endl;

	
}



void xiu_RS_decoding(uchar* zeroones, uchar* output)  // 100 0s and 1s as input, output the string
{

	DmtxMessage* msg = dmtxMessageCreate(1,DmtxFormatMatrix);  // sizeIdx=1;


	memcpy(msg->array,zeroones, 100*sizeof(uchar));

	int sizeIdx=1; 
	int fix=-1;
	dmtxDecodePopulatedArray(sizeIdx, msg, fix); // Key

							printf("outputSize=%d \n", msg->outputSize); //output 1

							for(int i=0;i<(int)(msg->outputSize);i++)

							printf("fuck msg_xiu.output(%d)=%c \n", i, msg->output[i]);

	memcpy(output,msg->output, msg->outputSize*sizeof(uchar));
}


//=========================================================================================
//==========================================================================================
//==========================================================================================


//globally defind unit black square length





int main()
{
//^^^^^^^^^^^^^^^^^^^^^^^^^
//read an img
//^^^^^^^^^^^^^^^^^^^^^^^^^
	Mat img=imread("RecTight2_canny.jpg", 0);
	Mat raw=imread("RecTight2.jpg",0);

	imshow("img_raw",img);

	waitKey(0);

	
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//^^^^^^^^^^^^^^^^^^^^^^^^^
//extract left to right edge, line1 and line2
//^^^^^^^^^^^^^^^^^^^^^^^^^
	Line line1, line2;
	vector<Point2d> EdgePoints;
	
	L2RScan(img, line1, line2, EdgePoints);

	Mat oneEdge;
	oneEdge.create(img.size(),img.type());
	oneEdge=Scalar::all(0);

	
	
	for(int i=0;i<EdgePoints.size();i++)
		{
			cout<<EdgePoints.at(i)<<endl;
			Point2d tempP=EdgePoints.at(i);
			oneEdge.at<uchar>( cvRound(tempP.y), cvRound(tempP.x))=255;
		
		}
	

	

	imshow("oneEdge",oneEdge);
	imwrite("oneEdge.jpg",oneEdge);
	waitKey(0);

cout<<"----------------Here we got l2r edge."<<endl;
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	
//^^^^^^^^^^^^^^^^^^^^^^^^^
//find the  l2r corner, intersection of line1 and line2
//^^^^^^^^^^^^^^^^^^^^^^^^^
		
	Mat img_showline(height,width,CV_8UC3,Scalar(0,0,0));
	
	img_showline=Scalar::all(0);
	int thickness=1;
	int linetype=8;
	line(img_showline,line1.startp, line1.endp, Scalar(0,255,0),thickness,linetype);
	line(img_showline,line2.startp, line2.endp, Scalar(255,255,0),thickness,linetype);
	imshow("img_showline",img_showline);	
	imwrite("img_showline.jpg",img_showline);
	waitKey(0);

	
	Point2d output12;
	intersectionOf2lines(line1, line2, output12);// correct  // bool is to check inrange or out of range
	cout<<"intersection="<<output12<<endl;  // correct













//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	Line line3, line4;
	vector<Point2d> EdgePoints34;
	cout<<"fuck 34"<<endl;
	R2LScan(img, line3, line4, EdgePoints34);

	Mat oneEdge34;
	oneEdge34.create(img.size(),img.type());
	oneEdge34=Scalar::all(0);

	cout<<"EdgePoints34.size()="<<EdgePoints34.size()<<endl;
	cout<<"fuck 34333"<<endl;
	cout<<"EdgePoints34.at(1)="<<EdgePoints34.at(1)<<endl;
	for(int i=0;i<EdgePoints34.size();i++)
		{
			cout<<EdgePoints34.at(i)<<endl;
			//cout<<"i="<<i<<endl;
			Point2d tempP=EdgePoints34.at(i);
			oneEdge34.at<uchar>( cvRound(tempP.y), cvRound(tempP.x))=255;
		
		}
	

	cout<<"fuck 34"<<endl;

	imshow("oneEdge34",oneEdge34);
	imwrite("oneEdge34.jpg",oneEdge34);
	waitKey(0);

cout<<"----------------Here we got r2l edge."<<endl;
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	
//^^^^^^^^^^^^^^^^^^^^^^^^^
//find the  l2r corner, intersection of line1 and line2
//^^^^^^^^^^^^^^^^^^^^^^^^^
		
	Mat img_showline34(height,width,CV_8UC3,Scalar(0,0,0));
	
	img_showline34=Scalar::all(0);
	//int thickness=1;
	//int linetype=8;
	line(img_showline34,line3.startp, line3.endp, Scalar(0,255,0),thickness,linetype);
	line(img_showline34,line4.startp, line4.endp, Scalar(255,255,0),thickness,linetype);
	imshow("img_showline34",img_showline34);	
	imwrite("img_showline34.jpg",img_showline34);
	waitKey(0);

	
	Point2d output34;
	intersectionOf2lines(line3, line4, output34);// correct  // bool is to check inrange or out of range
	cout<<"intersection34="<<output34<<endl;  // correct






//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// draw the whole

	Mat img_showline1234(height,width,CV_8UC3,Scalar(0,0,0));

	img_showline1234=Scalar::all(0);
	//int thickness=1;
	//int linetype=8;
	line(img_showline1234,line3.startp, line3.endp, Scalar(255,0,0),thickness,linetype);
	line(img_showline1234,line4.startp, line4.endp, Scalar(0,255,0),thickness,linetype);
	line(img_showline1234,line1.startp, line1.endp, Scalar(255,255,0),thickness,linetype);
	line(img_showline1234,line2.startp, line2.endp, Scalar(0,255,255),thickness,linetype);
	imshow("img_showline1234",img_showline1234);	
	imwrite("img_showline1234.jpg",img_showline1234);
	waitKey(0);

	
	Mat img_r=img.clone();
	line(img_r,line3.startp, line3.endp, Scalar(255),thickness,linetype);
	line(img_r,line4.startp, line4.endp, Scalar(255),thickness,linetype);
	line(img_r,line1.startp, line1.endp, Scalar(255),thickness,linetype);
	line(img_r,line2.startp, line2.endp, Scalar(255),thickness,linetype);
	imshow("img_r",img_r);	
	imwrite("img_r.jpg",img_r);
	waitKey(0);
	


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


//--------------------------------------------------------------------------------Permutate in Order--------------------------------------------------------------------------- 









// calculate intersection 13,24,14,23 , who are parallel pairs
Mat linepair(4,1,CV_8UC1,Scalar(0));


	if(line1.k * line3.k > 0)   // -1  notice
		{
			linepair.at<uchar>(1-1,0)=3-1;
			linepair.at<uchar>(2-1,0)=4-1;
			linepair.at<uchar>(3-1,0)=1-1;
			linepair.at<uchar>(4-1,0)=2-1;
		}

	else

		{
			linepair.at<uchar>(1-1,0)=4-1;
			linepair.at<uchar>(4-1,0)=1-1;
			linepair.at<uchar>(2-1,0)=3-1;
			linepair.at<uchar>(3-1,0)=2-1;
	
		}




cout<<"linepair="<<linepair<<endl;



//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// diagonal line to judge and search left corner

	Point2d vec_l1(1,line1.k);
	Point2d vec_l2(1,line2.k);
	Point2d vec_l3(1,line3.k);
	Point2d vec_l4(1,line4.k);

	Point3d vec3_l1(1,line1.k,0);
	Point3d vec3_l2(1,line2.k,0);
	Point3d vec3_l3(1,line3.k,0);
	Point3d vec3_l4(1,line4.k,0);

	cout<<"vec_l1="<<vec_l1<<endl;
	cout<<"vec_l2="<<vec_l2<<endl;
	cout<<"vec_l3="<<vec_l3<<endl;
	cout<<"vec_l4="<<vec_l4<<endl;

	
	Point2d intersection_12=output12;
	Point2d intersection_34=output34;

	Point2d intersection_14;
	Point2d intersection_24;
	Point2d intersection_13;
	Point2d intersection_23;
	
	

	if(linepair.at<uchar>(1-1,0)==(3-1)) // if 1_3 pair parallel, then 1_4 cross
		{
			Point2d output14;
			intersectionOf2lines(line1, line4, output14);
			intersection_14=output14;

			Point2d output23;
			intersectionOf2lines(line2, line3, output14);
			intersection_23=output23;
		
		}

	else
		{
			Point2d output13;
			intersectionOf2lines(line1, line3, output13);
			intersection_13=output13;

			Point2d output24;
			intersectionOf2lines(line2, line4, output24);
			intersection_24=output24;

			
		}
		


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Assign Intersect_Sur to recover the order of 4 corners 

			Intersect_Sur IS_corner_A;
			Intersect_Sur IS_corner_B;
			Intersect_Sur IS_corner_C;
			Intersect_Sur IS_corner_D;




			if(linepair.at<uchar>(1-1,0)==3-1) // if 1_3 pair parallel, then 1_4 cross, 1_2 cross

			{
						Intersect_Sur IS_12;
						Intersect_Sur IS_14;
						Intersect_Sur IS_34;
						Intersect_Sur IS_23;

						IS_12.P=intersection_12;		IS_12.vecfirst=vec_l1;     IS_12.vecsecond=vec_l2;   
						IS_14.P=intersection_14;   		IS_14.vecfirst=vec_l1;     IS_14.vecsecond=vec_l4;   
						IS_34.P=intersection_34;		IS_34.vecfirst=vec_l3;     IS_34.vecsecond=vec_l4;   
						IS_23.P=intersection_23;		IS_23.vecfirst=vec_l2;     IS_12.vecsecond=vec_l3;     // initialize IS

			



						Point2d intersection_diag_12_34 = intersection_34 - intersection_12;
						Point2d intersection_diag_34_12 = -(intersection_34 - intersection_12);
						Point2d intersection_diag_14_23 = intersection_23 - intersection_14;
						Point2d intersection_diag_23_14 = -(intersection_23 - intersection_14);



			//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& 
			//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& Put IS in order

						orderIS(intersection_diag_12_34, IS_12);
						orderIS(intersection_diag_34_12, IS_34);
						orderIS(intersection_diag_14_23, IS_14);
						orderIS(intersection_diag_23_14, IS_23);

			//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& 

			//&&&&&&&&&&&&&&&&&assign A,B,C,D 
				 IS_corner_A=IS_12;
				 IS_corner_B=IS_14;
				 IS_corner_C=IS_34;
				 IS_corner_D=IS_23;




			//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& 
//double lenTwoPoints(Point2d& p1,Point2d& p2)


			//calculate unit length

						double edgeLength_1= lenTwoPoints(IS_12.P, IS_14.P);  //line1
						double edgeLength_2= lenTwoPoints(IS_12.P, IS_23.P);  //line2
						double edgeLength_3= lenTwoPoints(IS_23.P, IS_34.P);  //line3
						double edgeLength_4= lenTwoPoints(IS_34.P, IS_14.P);  //line4

					
						double aveEdgeLen= 	(edgeLength_1 +edgeLength_2+edgeLength_3+edgeLength_4)/4.0;	
						
						unitLen= aveEdgeLen/16.0; // 16 units in one edge
						
			//end of calculate unit length		
						
			
			}



			else   // if 1_4 pair parallel, then 1_3 cross, 1_2 cross
		
			{

						Intersect_Sur IS_12;
						Intersect_Sur IS_13;
						Intersect_Sur IS_34;
						Intersect_Sur IS_24;

						IS_12.P=intersection_12;		IS_12.vecfirst=vec_l1;     IS_12.vecsecond=vec_l2;   
						IS_13.P=intersection_13;   		IS_13.vecfirst=vec_l1;     IS_13.vecsecond=vec_l3;   
						IS_34.P=intersection_34;		IS_34.vecfirst=vec_l3;     IS_34.vecsecond=vec_l4;   
						IS_24.P=intersection_24;		IS_24.vecfirst=vec_l2;     IS_24.vecsecond=vec_l4;     // initialize IS

			



						Point2d intersection_diag_12_34 = intersection_34 - intersection_12;
						Point2d intersection_diag_34_12 = -(intersection_34 - intersection_12);
						Point2d intersection_diag_13_24 = intersection_24 - intersection_13;
						Point2d intersection_diag_24_13 = -(intersection_24 - intersection_13);



			//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& 
			//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& Put IS in order

						orderIS(intersection_diag_12_34, IS_12);
						orderIS(intersection_diag_34_12, IS_34);
						orderIS(intersection_diag_13_24, IS_13);
						orderIS(intersection_diag_24_13, IS_24);

			//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& 
			//&&&&&&&&&&&&&&&&&assign A,B,C,D 
						 IS_corner_A=IS_12;
						 IS_corner_B=IS_13;
						 IS_corner_C=IS_34;
						 IS_corner_D=IS_24;




			//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& 
			//calculate unit length

						double edgeLength_1= lenTwoPoints(IS_12.P, IS_13.P);  //line1
						double edgeLength_2= lenTwoPoints(IS_12.P, IS_24.P);  //line2
						double edgeLength_3= lenTwoPoints(IS_13.P, IS_34.P);  //line3
						double edgeLength_4= lenTwoPoints(IS_24.P, IS_34.P);  //line4

					
						double aveEdgeLen= 	(edgeLength_1 +edgeLength_2+edgeLength_3+edgeLength_4)/4.0;	
						
						unitLen= aveEdgeLen/16.0; // 16 units in one edge
						
			//end of calculate unit length		
						
						

			//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&  Great! All correct! in Order.


			}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&  Great! All correct! in Order.&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&  Great! All correct! in Order.&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

			




//####################  OK, Let's Find the big monster: left corner #############################################################


// first binarize the raw 

			cout<<"unitLen="<<unitLen<<endl;

			Mat bina;
			bina.create(img.size(),img.type());	

			threshold( raw, bina, 120, 255,CV_THRESH_BINARY);
			
			imshow("bina",bina);

			imwrite("bina.jpg",bina);

			waitKey(0);

//Point2d nomalizeVec(Point2d p)
//bool judgeLeftcorner(Intersect_Sur& IS, Mat& bina)		

// judge Left corner

			judgeLeftcorner( IS_corner_A,  bina);
			judgeLeftcorner( IS_corner_B,  bina);	
			judgeLeftcorner( IS_corner_C,  bina);	
			judgeLeftcorner( IS_corner_D,  bina);	

		

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&  Great! Left corner we got it! His global name is: leftCorner  &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&  Great!  Left corner we got it! &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	






//####################  OK, Let's Find the bits 0s and 1s #############################################################

cout<<"^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^  begin to calculate all the 0s and 1s -------"<<endl;


Mat bina_final=bina.clone();

Mat decodeResult= (Mat_<uchar>(10,10)<<0);

Point2d unit_first= unitLen* nomalizeVec(leftCorner.vecfirst);
Point2d unit_second= unitLen* nomalizeVec(leftCorner.vecsecond);

Point2d codebasep=leftCorner.P+ 3.5*unit_second + 3.5*unit_first;  //drift add here, above is 2.5, here for code is 3.5

for(int fi=0;fi<10;fi++)
	for(int fj=0;fj<10;fj++)
	
	{
		Point2d temp= codebasep + fi*unit_first +fj*unit_second;
	
		if(bina_final.at<uchar>( cvRound(temp.y) , cvRound(temp.x) ) ==255 )
			decodeResult.at<uchar>(fi,fj)=1;
		else
			decodeResult.at<uchar>(fi,fj)=0;	
		
		
	}


cout<<"decodeResult="<<endl<<decodeResult<<endl;




cout<<"==========================fuck out the final fuck up 0s and 1s ============================================"<<endl;







//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


cout<<"==========================Reed Solomn Decoding here============================================"<<endl;

cout<<"==========================Reed Solomn Decoding here============================================"<<endl;

cout<<"==========================Reed Solomn Decoding here============================================"<<endl;
cout<<"==========================Reed Solomn Decoding here============================================"<<endl;
cout<<"==========================Reed Solomn Decoding here============================================"<<endl;
cout<<"==========================Reed Solomn Decoding here============================================"<<endl;


uchar zeroones_after_formsg[100]={0};


reorder( decodeResult, zeroones_after_formsg);

uchar output[120]={0};

xiu_RS_decoding( zeroones_after_formsg, output);










cout<<"==========================final decoded story is============================================"<<endl;
cout<<"==========================final decoded story is============================================"<<endl;
cout<<"==========================final decoded story is============================================"<<endl;
cout<<"==========================final decoded story is============================================"<<endl;


for(int ifi=0;ifi<120;ifi++)
	if(output[ifi]!=0)
		cout<<output[ifi];
cout<<endl;
cout<<endl;
cout<<endl;

cout<<"============================================================================================="<<endl;








	return 1;


}






























































