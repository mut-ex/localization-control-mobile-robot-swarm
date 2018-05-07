#include <string>
#include <iomanip>
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/highgui/highgui.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <math.h>
using namespace cv;
using namespace aruco;

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "std_msgs/Float64MultiArray.h"

//#include <sstream>

# define M_PI           3.14159265358979323846  /* pi */
const float p_off = M_PI;
const float r_off = M_PI/2;
const float y_off = M_PI/2;

#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#include <ctime>
#endif


uint64 GetTimeMs64()
{
 /* Linux */
 struct timeval tv;

 gettimeofday(&tv, NULL);

 uint64 ret = tv.tv_usec;
 /* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
 ret /= 1000;

 /* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
 ret += (tv.tv_sec * 1000);

 return ret;
}

void DrawRotatedRectangle(cv::Mat& image, cv::Point centerPoint, cv::Size rectangleSize, double rotationDegrees)
{
    Scalar color = Scalar(255.0, 255.0, 255.0) ;// white

    // Create the rotated rectangle
    RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);

    // We take the edges that OpenCV calculated for us
    Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);

    // Convert them so we can use them in a fillConvexPoly
    Point vertices[4];    
    for(int i = 0; i < 4; ++i){
        vertices[i] = vertices2f[i];
    }

    // Now we can fill the rotated rectangle with our specified color
    fillConvexPoly(image,
                       vertices,
                       4,
                       color);
}

int main(int argc, char **argv)
{
    int j;
    uint64 begin_t, end_t, temp;
    float delta;
    try
    {
        ros::init(argc, argv, "talker");
        ros::NodeHandle n;
        ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("chatter", 1);

        vector<double> vec;
        VideoCapture stream1(3);

        aruco::CameraParameters CamParam;

        cv::Mat InImage;

        if (!stream1.isOpened())   //check if video device has been initialised
        {
            cout << "cannot open camera";
        }

        stream1.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
        stream1.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

        double position[3], orientation[4], normalized, rotX, rotY, rotZ;

        stream1.read(InImage);
        int m;
        // while (1)
        // {
        // 	for(m = 0; m < 3; m++)
        // 		stream1.read(InImage);

        // 	stream1.read(InImage);
        // 	cv::imshow("in",InImage);
        // 	waitKey(0);
        //     //if (waitKey(30) >= 0)
        //         //break;
        // }
        // read camera parameters if specifed
        CamParam.readFromXMLFile("/home/jacob/microsoft_lifecam_1080.yml");
        CamParam.resize(InImage.size());

        // read marker size if specified (default value -1)
        float MarkerSize = 0.032;

        MarkerDetector MDetector;
        MarkerDetector::Params x;
        x = MDetector.getParams();
        x._thresParam1 = 14;
        x._thresParam2 = 14 ;
        MDetector.setParams(x);

        vector< Marker >  Markers;

        while (ros::ok())
        {
            vec.clear();
            //read the input image
            // stream1.read(InImage);
        	for(m = 0; m < 3; m++)
        		stream1.read(InImage);

        	stream1.read(InImage);
        	//imwrite( "backdrop.jpg", InImage );
        	//cv::imshow("in",InImage);
        	//waitKey(0);
            //if (waitKey(30) >= 0)
                //break;

            
            
            MDetector.detect(InImage, Markers, CamParam, MarkerSize);
            //for each marker, draw info and its boundaries in the image
            for (unsigned int i=0; i<Markers.size(); i++)
            {
                Markers[i].OgreGetPoseParameters(position, orientation);

                double t3 = +2.0 * (orientation[0] * orientation[3] + orientation[1] * orientation[2]);
                double t4 = +1.0 - 2.0 * (orientation[2]*orientation[2] + orientation[3] * orientation[3]); 
                rotZ = atan2(t3, t4)*180/M_PI;
                temp = GetTimeMs64();
                cout<<"id = "<<setw(4)<<Markers[i].id<<"  x = "<<setw(11)<<position[0]*1000<<" mm  y = "<<setw(11)<<position[1]*1000<<" mm  Theta(Z) = "<<setw(11)<<rotZ<<endl;//<<" TS = "<<temp<<endl;
                Markers[i].draw(InImage,Scalar(0,0,255),2);

                vec.push_back(Markers[i].id);
                vec.push_back(position[0]*1000);
                vec.push_back(position[1]*1000);
                vec.push_back(rotZ);
                vec.push_back(temp);

                //<<Markers[i]<<endl;
            }

            if (CamParam.isValid() && MarkerSize != -1)
            {
                for (unsigned int i = 0; i < Markers.size(); i++)
                {
                    CvDrawingUtils::draw3dCube(InImage, Markers[i], CamParam);
                    CvDrawingUtils::draw3dAxis(InImage, Markers[i], CamParam);
                }
            }

            if (vec.size() != 0)
            {
                std_msgs::Float64MultiArray msg;

                // set up dimensions
                msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
                msg.layout.dim[0].size = vec.size();
                msg.layout.dim[0].stride = 1;
                msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

                msg.data.clear();
                 //push data into data blob
                vector<double>::const_iterator itr, end(vec.end());
                for(itr = vec.begin(); itr!= end; ++itr) {
                    //cout<<*itr<<endl;
                    msg.data.push_back(*itr);
                }/**/

                //ROS_INFO("%s", msg.data.c_str());
                chatter_pub.publish(msg);
            }
            
            end_t = GetTimeMs64();
            delta = (float)(end_t-begin_t);
            cout<<"FPS = "<<std::fixed<<(1/(delta))*1000<<" Vector Size: "<<vec.size()<<"\n"<<endl;
            begin_t = GetTimeMs64();
			cv::imshow("in",InImage);

            if (waitKey(30) >= 0)
                break;
        }
    }
    catch (std::exception &ex)
    {
        cout<<"Exception :"<<ex.what()<<endl;
    }
}

