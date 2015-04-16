#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include<vector>
#include <string>
#include<cstring>
#include "bitmap_image.hpp"
#include<conio.h>

using namespace cv;
using namespace std;

char
	// RLFBUD is the face order used for input, so that a correctly oriented
	// piece in the input has its 'highest value' facelet first. The rest of the
	// program uses moves in FBRLUD order.
	*faces="RLFBUD",
	// I use char arrays here cause they can be initialised with a string
	// which is shorter than initialising other arrays.
	// Internally cube uses slightly different ordering to the input so that
	//  orbits of stage 4 are contiguous. Note also that the two corner orbits
	//  are diametrically opposite each other.
	//input:  UF UR UB UL  DF DR DB DL  FR FL BR BL  UFR URB UBL ULF   DRF DFL DLB DBR
	//        A  B  C  D   E  F  G  H   I  J  K  L   M   N   O   P     Q   R   S   T
	//        A  E  C  G   B  F  D  H   I  J  K  L   M   S   N   T     R   O   Q   P
	//intrnl: UF DF UB DB  UR DR UL DL  FR FL BR BL  UFR UBL DFL DBR   DLB DRF URB ULF
	*order="AECGBFDHIJKLMSNTROQP",
	//To quickly recognise the pieces, I construct an integer by setting a bit for each
	// facelet. The unique result is then found on the list below to map it to the correct
	// cubelet of the cube.
	//intrnl: UF DF UB DB  UR DR UL DL  FR FL BR BL  UFR UBL DFL DBR   DLB DRF URB ULF
	//bithash:20,36,24,40, 17,33,18,34, 5, 6, 9, 10, 21, 26, 38, 41,   42, 37, 25, 22
	*bithash="TdXhQaRbEFIJUZfijeYV",
	//Each move consists of two 4-cycles. This string contains these in FBRLUD order.
	//intrnl: UF DF UB DB  UR DR UL DL  FR FL BR BL  UFR UBL DFL DBR   DLB DRF URB ULF
	//        A  B  C  D   E  F  G  H   I  J  K  L   M   N   O   P     Q   R   S   T
	*perm="AIBJTMROCLDKSNQPEKFIMSPRGJHLNTOQAGCEMTNSBFDHORPQ",

	// current cube position
	pos[20],ori[20],val[20],
	// temporaryorary variable used in swap macro
	TEMP,
	// pruning tables, 2 for each phase
	*tables[8];
	// current phase solution
int moves[20],moveamount[20],
	// current phase being searched (0,2,4,6 for phases 1 to 4)
	 phase1=0,
	// Length of pruning tables. (one dummy in phase 1);
	tablesize[]={1,4096,  6561,4096,  256,1536,  13824,576};

// Use very ugly and unsafe macro to swap items instead of classic routine with
//   pointers for the sole reason of brevity
#define SWAP(a,b) TEMP=a;a=b;b=TEMP;
// number 65='A' is often subtracted to convert char ABC... to number 0,1,2,...
#define CHAROFFSET 65



// Cycles 4 pieces in array p, the piece indices given by a[0..3].
void cycle(char*p,char*a){
	SWAP(p[*a-CHAROFFSET],p[a[1]-CHAROFFSET]);
	SWAP(p[*a-CHAROFFSET],p[a[2]-CHAROFFSET]);
	SWAP(p[*a-CHAROFFSET],p[a[3]-CHAROFFSET]);
}

// twists i-th piece a+1 times.
void twist(int i,int a){
	i-=CHAROFFSET;
	ori[i]=(ori[i]+a+1)%val[i];
}


// set cube to solved position
void reset(){
	for( int i=0; i<20; pos[i]=i, ori[i++]=0);
}

// convert permutation of 4 chars to a number in range 0..23
int permtonum(char* p){
	int n=0;
	for ( int a=0; a<4; a++) {
		n*=4-a;
		for( int b=a; ++b<4; )
			if (p[b]<p[a]) n++;
	}
	return n;
}

// convert number in range 0..23 to permutation of 4 chars.
void numtoperm(char* p,int n,int o){
	p+=o;
	p[3]=o;
	for (int a=3; a--;){
		p[a] = n%(4-a) +o;
		n/=4-a;
		for (int b=a; ++b<4; )
			if ( p[b] >= p[a]) p[b]++;
	}
}

// get index of cube position from table t
int getposition(int t){
	int i=-1,n=0;
	switch(t){
	// case 0 does nothing so returns 0
	case 1://edgeflip
		// 12 bits, set bit if edge is flipped
		for(;++i<12;) n+= ori[i]<<i;
		break;
	case 2://cornertwist
		// get base 3 number of 8 digits - each digit is corner twist
		for(i=20;--i>11;) n=n*3+ori[i];
		break;
	case 3://middle edge choice
		// 12 bits, set bit if edge belongs in Um middle slice
		for(;++i<12;) n+= (pos[i]&8)?(1<<i):0;
		break;
	case 4://ud slice choice
		// 8 bits, set bit if UD edge belongs in Fm middle slice
		for(;++i<8;) n+= (pos[i]&4)?(1<<i):0;
		break;
	case 5://tetrad choice, twist and parity
		int corn[8],j,k,l,corn2[4];
		// 8 bits, set bit if corner belongs in second tetrad.
		// also separate pieces for twist/parity determination
		k=j=0;
		for(;++i<8;)
			if((l=pos[i+12]-12)&4){
				corn[l]=k++;
				n+=1<<i;
			}else corn[j++]=l;
		//Find permutation of second tetrad after solving first
		for(i=0;i<4;i++) corn2[i]=corn[4+corn[i]];
		//Solve one piece of second tetrad
		for(;--i;) corn2[i]^=corn2[0];

		// encode parity/tetrad twist
		n=n*6+corn2[1]*2-2;
		if(corn2[3]<corn2[2])n++;
		break;
	case 6://two edge and one corner orbit, permutation
		n=permtonum(pos)*576+permtonum(pos+4)*24+permtonum(pos+12);
		break;
	case 7://one edge and one corner orbit, permutation
		n=permtonum(pos+8)*24+permtonum(pos+16);
		break;
	}
	return n;
}


// sets cube to any position which has index n in table t
void setposition(int t, int n){
	int i=0,j=12,k=0;
	char *corn="QRSTQRTSQSRTQTRSQSTRQTSR";
	reset();
	switch(t){
	// case 0 does nothing so leaves cube solved
	case 1://edgeflip
		for(;i<12;i++,n>>=1) ori[i]=n&1;
		break;
	case 2://cornertwist
		for(i=12;i<20;i++,n/=3) ori[i]=n%3;
		break;
	case 3://middle edge choice
		for(;i<12;i++,n>>=1) pos[i]= 8*n&8;
		break;
	case 4://ud slice choice
		for(;i<8;i++,n>>=1) pos[i]= 4*n&4;
		break;
	case 5://tetrad choice,parity,twist
		corn+=n%6*4;
		n/=6;
		for(;i<8;i++,n>>=1)
			pos[i+12]= n&1 ? corn[k++]-CHAROFFSET : j++;
		break;
	case 6://slice permutations
		numtoperm(pos,n%24,12);n/=24;
		numtoperm(pos,n%24,4); n/=24;
		numtoperm(pos,n   ,0);
		break;
	case 7://corner permutations
		numtoperm(pos,n/24,8);
		numtoperm(pos,n%24,16);
		break;
	}
}


//do a clockwise quarter turn cube move
void domove(int m){
	char *p=perm+8*m, i=8;
	//cycle the edges
	cycle(pos,p);
	cycle(ori,p);
	//cycle the corners
	cycle(pos,p+4);
	cycle(ori,p+4);
	//twist corners if RLFB
	if(m<4)
		for(;--i>3;) twist(p[i],i&1);
	//flip edges if FB
	if(m<2)
		for(i=4;i--;) twist(p[i],0);
}

// calculate a pruning table
void filltable(int ti){
	int n=1,l=1, tl=tablesize[ti];
	// alocate table memory
	char* tb = tables[ti]=new char[tl];
	//clear table
	memset( tb, 0, tl);
	//mark solved position as depth 1
	reset();
	tb[getposition(ti)]=1;

	// while there are positions of depth l
	while(n){
		n=0;
		// find each position of depth l
		for(int i=0;i<tl;i++){
			if( tb[i]==l ){
				//construct that cube position
				setposition(ti,i);
				// try each face any amount
				for( int f=0; f<6; f++){
					for( int q=1;q<4;q++){
						domove(f);
						// get resulting position
						int r=getposition(ti);
						// if move as allowed in that phase, and position is a new one
						if( ( q==2 || f>=(ti&6) ) && !tb[r]){
							// mark that position as depth l+1
							tb[r]=l+1;
							n++;
						}
					}
					domove(f);
				}
			}
		}
		l++;
	}
}


// Pruned tree search. recursive.
bool searchphase(int movesleft, int movesdone,int lastmove){

	// prune - position must still be solvable in the remaining moves available
	if( tables[phase1  ][getposition(phase1  )]-1 > movesleft ||
	    tables[phase1+1][getposition(phase1+1)]-1 > movesleft ) return false;

	// If no moves left to do, we have solved this phase1
	if(!movesleft) return true;

	// not solved. try each face move
	for( int i=6;i--;){
		// do not repeat same face, nor do opposite after DLB.
		if( i-lastmove && (i-lastmove+1 || i|1 ) ){
			moves[movesdone]=i;
			// try 1,2,3 quarter turns of that face
			for(int j=0;++j<4;){
				//do move and remember it
				domove(i);
				moveamount[movesdone]=j;
				//Check if phase1 only allows half moves of this face
				if( (j==2 || i>=phase1 ) &&
					//search on
					searchphase(movesleft-1,movesdone+1,i) ) return true;
			}
			// put face back to original position.
			domove(i);
		}
	}
	// no solution found
	return false;
}


int i1=80;
int i2=240;
int i3=400;

int Pos_x1=220;
int Pos_x2=290;
int Pos_x3=355;

int Pos_y1=315;
int Pos_y2=380;
int Pos_y3=450;


int red_R=255;
int red_G=0;
int red_B=0;


int blue_R=0;
int blue_G=0;
int blue_B=255;


int green_R=0;
int green_G=255;
int green_B=0;


int orange_R=255;
int orange_G=128;
int orange_B=0;


int yellow_R=255;
int yellow_G=255;
int yellow_B=0;



int white_R=255;
int white_G=255;
int white_B=255;








char color(int R, int G , int B)
    {
        if(R==red_R&&G==red_G&&B==red_B)
{
 return 'F';
}
else
if(R==yellow_R&&G==yellow_G&&B==yellow_B)
{
 return 'D';
}
else
if(R==orange_R&&G==orange_G&&B==orange_B)
{
 return 'B';
}
else
if(R==blue_R&&G==blue_G&&B==blue_B)
{
 return 'R';
}
else
if(R==green_R&&G==green_G&&B==green_B)
{
 return 'L';

}
else    {
    return 'x';
}



}

int loadfacedata(int face, char arr[], char* imagename, char col)
    {

        bitmap_image image(imagename);

   if (!image)
   {
      printf("Error - Failed to open: input.bmp\n");
      return 1;
   }

   unsigned char red;
   unsigned char green;



   unsigned char blue;



   const unsigned int height = image.height();
   const unsigned int width  = image.width();

   for (std::size_t j = 0; j < height; ++j)
   {
      for (std::size_t i = 0; i < width; ++i)
      {
         image.get_pixel(i,j,red,green,blue);


         int r=(int)red;
          int g=(int)green;
           int b=(int)blue;

       if((i==Pos_x1||i==Pos_x2||i==Pos_x3)&&(j==Pos_y1||j==Pos_y2||j==Pos_y3))
            {

             if(i==Pos_x1&&j==Pos_y1)
             {

if(col==color(r,g,b))
                 arr[8*face + 0]=color(r,g,b);
             }


             if(i==Pos_x2&&j==Pos_y1)
             {



if(col==color(r,g,b))
                 arr[8*face + 1]=color(r,g,b);
             }


             if(i==Pos_x3&&j==Pos_y1)
             {


if(col==color(r,g,b))
                 arr[8*face + 2]=color(r,g,b);
             }


             if(i==Pos_x3&&j==Pos_y2)
             {
if(col==color(r,g,b))
                 arr[8*face + 3]=color(r,g,b);
             }





             if(i==Pos_x3&&j==Pos_y3)
             {

if(col==color(r,g,b))
                 arr[8*face + 4]=color(r,g,b);
             }


             if(i==Pos_x2&&j==Pos_y3)
             {

if(col==color(r,g,b))
                 arr[8*face + 5]=color(r,g,b);
             }


             if(i==Pos_x1&&j==Pos_y3)
             {

if(col==color(r,g,b))
                 arr[8*face + 6]=color(r,g,b);
             }


             if(i==Pos_x1&&j==Pos_y2)
             {

if(col==color(r,g,b))
                 arr[8*face + 7]=color(r,g,b);
             }

    }

      }
   }


    }

 int main( int argc, char** argv )
 { Mat imgThresholded;


  bool check1=false;
  bool check2=false;
  bool check3=false;
  bool check4=false;
  bool check5=false;


  Mat temp;
  Mat temp1;

    VideoCapture cap(0); //capture the video from web cam

    if ( !cap.isOpened() )  // if not success, exit program
    {
         cout << "Cannot open the web cam" << endl;
         return -1;
    }



  int iLowH = 38;
 int iHighH = 75;

  int iLowS = 100;
 int iHighS = 255;

  int iLowV = 40;
 int iHighV = 255;

 int a=0;
 int b=1;
 int p=1;
 int d=1;
 int e=1;
 int g=1;


    while (true)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video

         if (!bSuccess) //if not success, break loop
        {
             cout << "Cannot read a frame from video stream" << endl;
             break;
        }

    Mat imgHSV;

   cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV



  inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

  //morphological opening (remove small objects from the foreground)
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(9, 7)) );
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(9, 7)) );

   //morphological closing (fill small holes in the foreground)
  dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(9,7)) );
  erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(9, 7)) );

   imshow("Thresholded Image", imgThresholded); //show the thresholded image
  imshow("Original", imgOriginal); //show the original image

if(waitKey(50)=='x')
{

    iLowH = 38;
    iHighH = 75;

    iLowS = 100;
    iHighS = 255;

    iLowV = 40;
    iHighV = 255;

vector<vector<Point>>contours;
vector<Vec4i>hierarchy;
findContours(imgThresholded.clone(),contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
Mat temp1=Mat::zeros(imgThresholded.size(),CV_8UC3);
for(int i=0;i<contours.size();i++)
{
    Scalar color=Scalar(0,255,0);
    drawContours(temp1,contours,i,color,CV_FILLED,8,hierarchy,0,Point());
}
 if(!g)
        {
         imwrite("green.bmp",temp1);

        }
        if(!e)
        {
         imwrite("green1.bmp",temp1);

        }
        if(!d)
        {
         imwrite("green11.bmp",temp1);

        }
        if(!p)
        {
         imwrite("green111.bmp",temp1);
        }
        if(!b)
        {
         imwrite("green1111.bmp",temp1);
        }

        if(!a)
        {
         imwrite("green11111.bmp",temp1);
        }

 check1=true;
 continue;

}
if(check1)
{
  iLowH = 0;
  iHighH = 22;

  iLowS = 100;
  iHighS = 255;

  iLowV = 40;
  iHighV = 255;

   vector<vector<Point>>contours;
vector<Vec4i>hierarchy;
findContours(imgThresholded.clone(),contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
Mat temp1=Mat::zeros(imgThresholded.size(),CV_8UC3);
for(int i=0;i<contours.size();i++)
{
    Scalar color=Scalar(0,0,0);
    drawContours(temp1,contours,i,color,CV_FILLED,8,hierarchy,0,Point());
}
  if(!g)
        {
         imwrite("white.bmp",temp1);

        }
        if(!e)
        {
         imwrite("white1.bmp",temp1);


        }
        if(!d)
        {
         imwrite("white11.bmp",temp1);

         }
        if(!p)
        {
         imwrite("white111.bmp",temp1);

        }
        if(!b)
        {
         imwrite("white1111.bmp",temp1);

        }

        if(!a)
        {
         imwrite("white11111.bmp",temp1);

        }
check1=false;
 check2=true;
 continue;

}
if(check2)
{
  iLowH = 75;
  iHighH = 130;

  iLowS = 100;
  iHighS = 255;

  iLowV = 40;
  iHighV = 255;

vector<vector<Point>>contours;
vector<Vec4i>hierarchy;
findContours(imgThresholded.clone(),contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
Mat temp1=Mat::zeros(imgThresholded.size(),CV_8UC3);
for(int i=0;i<contours.size();i++)
{
    Scalar color=Scalar(0,128,255);
    drawContours(temp1,contours,i,color,CV_FILLED,8,hierarchy,0,Point());
}
 if(!g)
        {
         imwrite("orange.bmp",temp1);

        }
        if(!e)
        {
         imwrite("orange1.bmp",temp1);

        }
        if(!d)
        {
         imwrite("orange11.bmp",temp1);

        }
        if(!p)
        {
         imwrite("orange111.bmp",temp1);
        }
        if(!b)
        {
         imwrite("orange1111.bmp",temp1);
        }

        if(!a)
        {
         imwrite("orange11111.bmp",temp1);
        }
check2=false;
check3=true;
continue;

}
if(check3)
{
  iLowH = 22;
  iHighH = 38;

  iLowS = 100;
  iHighS = 255;

  iLowV = 40;
  iHighV = 255;

  vector<vector<Point>>contours;
vector<Vec4i>hierarchy;
findContours(imgThresholded.clone(),contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
Mat temp1=Mat::zeros(imgThresholded.size(),CV_8UC3);
for(int i=0;i<contours.size();i++)
{
    Scalar color=Scalar(255,0,0);
    drawContours(temp1,contours,i,color,CV_FILLED,8,hierarchy,0,Point());
}

 if(!g)
        {
         imwrite("blue.bmp",temp1);

        }
        if(!e)
        {
         imwrite("blue1.bmp",temp1);

        }
        if(!d)
        {
         imwrite("blue11.bmp",temp1);

        }
        if(!p)
        {
         imwrite("blue111.bmp",temp1);
        }
        if(!b)
        {
         imwrite("blue1111.bmp",temp1);
        }

        if(!a)
        {
         imwrite("blue11111.bmp",temp1);
        }
check3=false;
check4=true;
continue;

}
if(check4)
{
   iLowH = 160;
  iHighH = 179;

iLowS =100;
iHighS = 255;

  iLowV = 40;
 iHighV = 255;


  vector<vector<Point>>contours;
vector<Vec4i>hierarchy;
findContours(imgThresholded.clone(),contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
Mat temp1=Mat::zeros(imgThresholded.size(),CV_8UC3);
for(int i=0;i<contours.size();i++)
{
    Scalar color=Scalar(0,255,255);
    drawContours(temp1,contours,i,color,CV_FILLED,8,hierarchy,0,Point());
}

        if(!g)
        {
         imwrite("yellow.bmp",temp1);

        }
        if(!e)
        {
         imwrite("yellow1.bmp",temp1);

        }
        if(!d)
        {
         imwrite("yellow11.bmp",temp1);

        }
        if(!p)
        {
         imwrite("yellow111.bmp",temp1);
        }
        if(!b)
        {
         imwrite("yellow1111.bmp",temp1);
        }

        if(!a)
        {
         imwrite("yellow11111.bmp",temp1);

        }

check4=false;
check5=true;
continue;

}
if(check5)
{
 iLowH = 38;
 iHighH = 75;

  iLowS = 100;
 iHighS = 255;

 iLowV = 40;
 iHighV = 255;

  vector<vector<Point>>contours;
vector<Vec4i>hierarchy;
findContours(imgThresholded.clone(),contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE,Point(0,0));
Mat temp1=Mat::zeros(imgThresholded.size(),CV_8UC3);
for(int i=0;i<contours.size();i++)
{
    Scalar color=Scalar(0,0,255);
    drawContours(temp1,contours,i,color,CV_FILLED,8,hierarchy,0,Point());
}


        if(!g)
        {
         imwrite("red.bmp",temp1);

        g=1;

        }
        if(!e)
        {
         imwrite("red1.bmp",temp1);

        e=1;
        g=0;
        }
        if(!d)
        {
         imwrite("red11.bmp",temp1);

        d=1;
        e=0;
        }
        if(!p)
        {
         imwrite("red111.bmp",temp1);

        p=1;
        d=0;
        }
        if(!b)
        {
         imwrite("red1111.bmp",temp1);

        b=1;
        p=0;
        }

        if(!a)
        {
         imwrite("red11111.bmp",temp1);

        a=1;
        b=0;
        }
        check5=false;
    continue;
}

    check1=false;
    check2=false;
    check3=false;
    check4=false;



  if(waitKey(30)=='z')
    {
        break;
    }

}
////////////////////////

 char arr[48];

    for(int a = 0 ;a<48 ;a++)
    {

        arr[a]='U';
    }



char* D_R="red.bmp";
char* D_G="green.bmp";
char* D_B="blue.bmp";
char* D_O="orange.bmp";
char* D_Y="yellow.bmp";


char* F_R="red11111.bmp";
char* F_G="green11111.bmp";
char* F_B="blue11111.bmp";
char* F_O="orange11111.bmp";
char* F_Y="yellow11111.bmp";


char* L_R="red11.bmp";
char* L_G="green11.bmp";
char* L_B="blue11.bmp";
char* L_O="orange11.bmp";
char* L_Y="yellow11.bmp";


char* R_R="red111.bmp";
char* R_G="green111.bmp";
char* R_B="blue111.bmp";
char* R_O="orange111.bmp";
char* R_Y="yellow111.bmp";


char* B_R="red1.bmp";
char* B_G="green1.bmp";
char* B_B="blue1.bmp";
char* B_O="orange1.bmp";
char* B_Y="yellow1.bmp";

char* U_R="red1111.bmp";
char* U_G="green1111.bmp";
char* U_B="blue1111.bmp";
char* U_O="orange1111.bmp";
char* U_Y="yellow1111.bmp";


int U=0;
int F=1;
int R=4;
int L=5;
int B=3;
int D=2;

char colR='F';
char colG='L';
char colB='R';
char colO='B';
char colY='D';


loadfacedata(U,arr,U_R,colR);
loadfacedata(U,arr,U_G, colG);
loadfacedata(U,arr,U_B,colB);
loadfacedata(U,arr,U_O,colO);
loadfacedata(U,arr,U_Y,colY);


loadfacedata(F,arr,F_R,colR);
loadfacedata(F,arr,F_G, colG);
loadfacedata(F,arr,F_B,colB);
loadfacedata(F,arr,F_O,colO);
loadfacedata(F,arr,F_Y,colY);

loadfacedata(D,arr,D_R,colR);
loadfacedata(D,arr,D_G, colG);
loadfacedata(D,arr,D_B,colB);
loadfacedata(D,arr,D_O,colO);
loadfacedata(D,arr,D_Y,colY);

loadfacedata(B,arr,B_R,colR);
loadfacedata(B,arr,B_G, colG);
loadfacedata(B,arr,B_B,colB);
loadfacedata(B,arr,B_O,colO);
loadfacedata(B,arr,B_Y,colY);

loadfacedata(R,arr,R_R,colR);
loadfacedata(R,arr,R_G, colG);
loadfacedata(R,arr,R_B,colB);
loadfacedata(R,arr,R_O,colO);
loadfacedata(R,arr,R_Y,colY);

loadfacedata(L,arr,L_R,colR);
loadfacedata(L,arr,L_G, colG);
loadfacedata(L,arr,L_B,colB);
loadfacedata(L,arr,L_O,colO);
loadfacedata(L,arr,L_Y,colY);



cout<<"\n\n";
for(int a=0;a<48 ; a++)
{

    cout<<arr[a];
}

cout<<"\n\n\n\n";
cout<<arr[5]<<arr[9]<<" "<<arr[3]<<arr[33]<<" "<<arr[1]<<arr[29]<<" "<<arr[7]<<arr[41]<<" ";

     cout<<arr[17]<<arr[13]<<" "<<arr[19]<<arr[37]<<" "<<arr[21]<<arr[25]<<" "<<arr[23]<<arr[45]<<" ";

     cout<<arr[11]<<arr[39]<<" "<<arr[15]<<arr[43]<<" "<<arr[27]<<arr[35]<<" "<<arr[31]<<arr[47]<<" ";

     cout<<arr[4]<<arr[10]<<arr[32]<<" ";

cout<<arr[2]<<arr[34]<<arr[28]<<" ";

     cout<<arr[0]<<arr[30]<<arr[40]<<" ";

     cout<<arr[6]<<arr[42]<<arr[8]<<" ";

     cout<<arr[18]<<arr[38]<<arr[12]<<" ";

     cout<<arr[16]<<arr[14]<<arr[44]<<" ";

     cout<<arr[22]<<arr[46]<<arr[24]<<" ";

     cout<<arr[20]<<arr[26]<<arr[36]<<" "<<endl;

char c[] = {arr[5], arr[9], ' ', arr[3], arr[33], ' ', arr[1], arr[29], ' ', arr[7], arr[41], ' ', arr[17], arr[13], ' ',
 arr[19], arr[37], ' ', arr[21], arr[25], ' ',
arr[23], arr[45], ' ', arr[11], arr[39], ' ', arr[15], arr[43], ' ', arr[27], arr[35], ' ', arr[31], arr[47], ' ',
arr[4], arr[10], arr[32], ' ', arr[2], arr[34], arr[28], ' ', arr[0], arr[30], arr[40], ' ', arr[6], arr[42], arr[8], ' ',
arr[18], arr[38], arr[12], ' ',
arr[16], arr[14], arr[44], ' ', arr[22], arr[46], arr[24], ' ', arr[20], arr[26], arr[36], ' ' };

///////////////////////////////

int f,i=0,j=0,k=0,pc,mor;

	// initialise tables
	for(; k<20; k++) val[k]=k<12?2:3;
	for(; j<8; j++) filltable(j);


int z=0; char flag;
	// read input, 20 pieces worth
	for(; i<20; i++){

		f=pc=k=mor=0;
		for(;f<val[i];f++){
			// read input from stdin, or...

			  			  do{flag=c[z]; z++;}
            while(c[z-1]==' ');
			    j=strchr(faces,flag)-faces;
			// ...from command line and get face number of facelet
			//j=strchr(faces,argv[i+1][f])-faces;
			// keep track of principal facelet for orientation
			if(j>k) {k=j;mor=f;}
			//construct bit hash code
			pc+= 1<<j;
		}
		// find which cubelet it belongs, i.e. the label for this piece
		for(f=0; f<20; f++)
			if( pc==bithash[f]-64 ) break;
		// store piece
		pos[order[i]-CHAROFFSET]=f;
		ori[order[i]-CHAROFFSET]=mor%val[i];
	}

	//solve the cube
	// four phase1s
	for( ; phase1<8; phase1+=2){
		// try each depth till solved
		for( j=0; !searchphase(j,0,9); j++);
		//output result of this phase1
		for( i=0; i<j; i++)
			cout<<"FBRLUD"[moves[i]]<<moveamount[i];
		cout<<" ";


	}

	getch();
   return 0;



}
