//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

#define minradius  25
#define maxradius  65
#include <iostream>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <vector>
#include <string>
#include <cmath>
#include <math.h>
#include "stdio.h"
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
using namespace std;
using namespace cv;

extern int rs232_init(char * devName, int baudrate); //inits the rs232-interface returns filedescriptor

extern void rs232_close(int fd); //closes the rs232_interface

extern int rs232_writeByte(int fd, unsigned char byte);

extern int rs232_writeBuffer(int fd, unsigned char * buffer, int size);

extern int rs232_writeShort(int fd, short * buffer, int size); 

//global vars

struct termios rs232_oldtio; //structure for old port settings



//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
ros::Publisher coordpub;

std_msgs::Int32MultiArray coord;


unsigned char startstring[5]={'>','*','>','d','i'};
short ctrlinput[6];
int apSerialFd;

int rs232_init(char * devName, int baudrate) //returns filedescriptor

{

  int fd; //file descriptor
   //reset values

   fd=-1;

   //open port

   fd = open(devName, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK );

   if ( fd == -1)

      {

       /*
        * Could not open the port.
        */

                //printf("rs232_init: Fehler: Geraet %s konnte nicht geoeffnet werden\n",devName);

                return -1;

      }

   else

      {

                struct termios newtio; //structure for port settings

                tcgetattr(fd,&rs232_oldtio); /* save current port settings */

        //set data connection to 8N1, no flow control

                bzero(&newtio, sizeof(newtio));

                newtio.c_cflag = baudrate | CS8 | CREAD | CLOCAL;

                newtio.c_iflag = IGNPAR;

                newtio.c_oflag = 0;

       

                /* set input mode (non-canonical, no echo,...) */

                newtio.c_lflag = 0;

        

                newtio.c_cc[VTIME]    = 1;   /* inter-character timer used */

                newtio.c_cc[VMIN]     = 0;   /* blocking read */

       

                tcflush(fd, TCIFLUSH);

                tcsetattr(fd,TCSANOW,&newtio);

                return fd;
      }
}
 
void rs232_close(int fd) //closes the rs232_interface

{

if (fd!=-1)

      {
       close(fd);
      }

   /* restore old port settings */

   tcsetattr(fd,TCSANOW,&rs232_oldtio);

}

int rs232_writeBuffer(int fd, unsigned char * buffer, int size)

{
                int res;

    res=write(fd,buffer,size);

                return res;
}

int rs232_writeShort(int fd, short int * buffer, int size)

{
                int res;

    res=write(fd,buffer,size);

                return res;
}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{

	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
		cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
		return;
	}
        CvMemStorage* storage = cvCreateMemStorage(0);
        CvSize size=cvSize(cv_ptr->image.cols,cv_ptr->image.rows);
        IplImage* gray = cvCreateImage(size, 8, 1 );

/*
	//Invert Image
	//Go through all the rows
	for(int i=0; i<cv_ptr->image.rows; i++)
	{
		//Go through all the columns
		for(int j=0; j<cv_ptr->image.cols; j++)
		{
			//Go through all the channels (b, g, r)
			for(int k=0; k<cv_ptr->image.channels(); k++)
			{
				//Invert the image by subtracting image data from 255				
				cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k] = 255-cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k];
			}
		}
	}
*/	



// image binarization 

for (int i=0;i<320*240*3;i=i+3)
{

// pick up colors of center

//if (i==centery*3*640+centerx*3)
//{
//RGB[0]=cv_ptr->imageData[i];
//RGB[1]=cv_ptr->imageData[i+1];
//RGB[2]=cv_ptr->imageData[i+2];
//cout<<"R="<<RGB[0]<<"   G="<<RGB[1]<<"   B="<<RGB[2]<<endl;
//}


if ((cv_ptr->image.data[i+1]>60)&&((cv_ptr->image.data[i]+15)<cv_ptr->image.data[i+1])&&((cv_ptr->image.data[i+2]+15)<cv_ptr->image.data[i+1]))
{
///  fprintf(fp, "%d*", cv_ptr->imageData[i]); 
       cv_ptr->image.data[i]=255;    
///  fprintf(fp, "%d*", cv_ptr->imageData[i+1]); 
	cv_ptr->image.data[i+1]=255;  
///  fprintf(fp, "%d   ", cv_ptr->imageData[i+2]); 
	cv_ptr->image.data[i+2]=255;
}
else
{
	cv_ptr->image.data[i]=0;
	cv_ptr->image.data[i+1]=0;
	cv_ptr->image.data[i+2]=0;
}
}

//Find the center of target

	// Convert image from sensor_msg::Image to IplImage
//	IplImage* image = bridge.imgMsgToCv( cv_ptr );

        IplImage image =cv_ptr->image;

        cvCvtColor( &image, gray, CV_RGB2GRAY );
        cvSmooth( gray, gray, CV_GAUSSIAN, 9, 9 ); // smooth it, otherwise a lot of false circles may be detected
        CvSeq* circles = cvHoughCircles( gray, storage, CV_HOUGH_GRADIENT, 2, gray->height/4, 200, 100 );
//        ROS_INFO("Looking for Circles, now got %d", circles->total);
        for(int j = 0; j < circles->total; j++ )
        {
             float* p = (float*)cvGetSeqElem( circles, j );
             cv::circle( cv_ptr->image, cvPoint(cvRound(p[0]),cvRound(p[1])), 3, CV_RGB(0,255,0), -1, 8, 0 );
             cv::circle( cv_ptr->image, cvPoint(cvRound(p[0]),cvRound(p[1])), cvRound(p[2]), CV_RGB(255,0,0), 3, 8, 0 );
             
             ROS_INFO("X = %d, Y= %d, R= %d", cvRound(p[0]), cvRound(p[1]), cvRound(p[2]));

// Publish the coordinates
//            coord[0]= cvRound(p[0]);
//            coord[1]= cvRound(p[1]);
//            coord[2]= cvRound(p[2]);
coord.data.clear();
for (int i = 0; i < 3; i++)
{
//assign array a random number between 0 and 255.
coord.data.push_back(cvRound(p[i]));
}

//            coordpub.publish(coord);
//            ROS_INFO("Publishing Coordinates");


    ros::Rate r(30); // 10 hz
    while (ros::ok())
    {
      // Publish control commands
//      ROS_INFO("Publishing Control Commands");
      rs232_writeBuffer(apSerialFd,&startstring[0],5);
      rs232_writeShort(apSerialFd,&ctrlinput[0],12);
      ros::spinOnce();
      r.sleep();
    }   

// 判断是否有误差， 给定最小和最大阈值
//             if ((cvRound(p[2])>minradius) && (cvRound(p[2])<maxradius) )
//           {  int centerx=cvRound(p[0]);
//              int centery=cvRound(p[1]);
//             int radius=cvRound(p[2]);

//              cout<<"圆心坐标x= "<<centerx<<endl<<"圆心坐标y= "<<centery<<endl;
//              cout<<"半径="<<radius<<endl;
//           }


//             else 
//              cout<<"Missed!"<<endl;
        }




	//Display the image using OpenCV
	cv::imshow(WINDOW, cv_ptr->image);
	//Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
	cv::waitKey(3);
	/**
	* The publish() function is how you send messages. The parameter
	* is the message object. The type of this object must agree with the type
	* given as a template parameter to the advertise<>() call, as was done
	* in the constructor in main().
	*/
	//Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
        pub.publish(cv_ptr->toImageMsg());
}

/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{

apSerialFd=rs232_init("/dev/ttyUSB0",B57600); //change /dev/ttyUSB0 to your serial port
ctrlinput[0]=0;
ctrlinput[1]=0;
ctrlinput[2]=20;
ctrlinput[3]=700;
ctrlinput[4]=0x0F;
ctrlinput[5]=ctrlinput[0]+ctrlinput[1]+ctrlinput[2]+ctrlinput[3]+ctrlinput[4]+0xAAAA;


	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line. For programmatic
	* remappings you can use a different version of init() which takes remappings
	* directly, but for most command-line programs, passing argc and argv is the easiest
	* way to do it.  The third argument to init() is the name of the node. Node names must be unique in a running system.
	* The name used here must be a base name, ie. it cannot have a / in it.
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
        ros::init(argc, argv, "image_processor");
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
        ros::NodeHandle nh;
	//Create an ImageTransport instance, initializing it with our NodeHandle.
        image_transport::ImageTransport it(nh);



	//OpenCV HighGUI call to create a display window on start-up.

	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	/**
	* Subscribe to the "camera/image_raw" base topic. The actual ROS topic subscribed to depends on which transport is used. 
	* In the default case, "raw" transport, the topic is in fact "camera/image_raw" with type sensor_msgs/Image. ROS will call 
	* the "imageCallback" function whenever a new image arrives. The 2nd argument is the queue size.
	* subscribe() returns an image_transport::Subscriber object, that you must hold on to until you want to unsubscribe. 
	* When the Subscriber object is destructed, it will automatically unsubscribe from the "camera/image_raw" base topic.
	*/
        image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
 

	//OpenCV HighGUI call to destroy a display window on shut-down.
	cv::destroyWindow(WINDOW);
	/**
	* The advertise() function is how you tell ROS that you want to
	* publish on a given topic name. This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing. After this advertise() call is made, the master
	* node will notify anyone who is trying to subscribe to this topic name,
	* and they will in turn negotiate a peer-to-peer connection with this
	* node.  advertise() returns a Publisher object which allows you to
	* publish messages on that topic through a call to publish().  Once
	* all copies of the returned Publisher object are destroyed, the topic
	* will be automatically unadvertised.
	*
	* The second parameter to advertise() is the size of the message queue
	* used for publishing messages.  If messages are published more quickly
	* than we can send them, the number here specifies how many messages to
	* buffer up before throwing some away.
	*/
        pub = it.advertise("camera/image_processed", 1);
        //****************  Register publisher for coordinates  ****************
//        coordpub = nh.advertise<std_msgs::Int32MultiArray>("Coord", 10);
	/**
	* In this application all user callbacks will be called from within the ros::spin() call. 
	* ros::spin() will not return until the node has been shutdown, either through a call 
	* to ros::shutdown() or a Ctrl-C.
	*/
        ros::spin();
tcflush(apSerialFd,TCIOFLUSH);
rs232_close(apSerialFd);
	//ROS_INFO is the replacement for printf/cout.
	ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");

}
