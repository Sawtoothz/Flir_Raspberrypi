#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
//#include <sensor_msgs/image_encodings.h>
#include "Palettes.h"
#include "SPI.h"
#include <iostream>
#include <QThread>
#include <QtCore>
#include <QPixmap>
#include <QImage>

#define PACKET_SIZE 164
#define PACKET_SIZE_UINT16 (PACKET_SIZE/2)
#define PACKETS_PER_FRAME 60
#define FRAME_SIZE_UINT16 (PACKET_SIZE_UINT16*PACKETS_PER_FRAME)
#define FPS 27;

using namespace std;

QImage myImage;
uint8_t result[PACKET_SIZE*PACKETS_PER_FRAME];
uint16_t *frameBuffer;

cv::Mat qimage_to_mat_ref(QImage &img, int format)
{
    return cv::Mat(img.height(), img.width(), 
            format, img.bits(), img.bytesPerLine());
}

cv::Mat qimage_to_mat_cpy(QImage const &img, int format)
{    
    return cv::Mat(img.height(), img.width(), format, 
                   const_cast<uchar*>(img.bits()), 
                   img.bytesPerLine()).clone();
}

int main(int argc,char **argv)
{
	myImage=QImage(80,60,QImage::Format_RGB888);
	ros::init(argc,argv,"flir_dev");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
  	image_transport::Publisher pub = it.advertise("temp/image", 1);
	ros::Rate loop_rate(27);
	SpiOpenPort(0);
	while(nh.ok())
	{
		int resets = 0;
		for(int j=0;j<PACKETS_PER_FRAME;j++) {
			//if it's a drop packet, reset j to 0, set to -1 so he'll be at 0 again loop
			read(spi_cs0_fd, result+sizeof(uint8_t)*PACKET_SIZE*j, sizeof(uint8_t)*PACKET_SIZE);
			int packetNumber = result[j*PACKET_SIZE+1];
			if(packetNumber != j) {
				j = -1;
				resets += 1;
				usleep(1000);
				//Note: we've selected 750 resets as an arbitrary limit, since there should never be 750 "null" packets between two valid transmissions at the current poll rate
				//By polling faster, developers may easily exceed this count, and the down period between frames may then be flagged as a loss of sync
				if(resets == 750) {
					SpiClosePort(0);
					usleep(750000);
					SpiOpenPort(0);
				}
			}
		}
		if(resets >= 30) {
			//ROS_INFO("Reset\r\n");
		}

		frameBuffer = (uint16_t *)result;
		int row, column;
		uint16_t value;
		uint16_t minValue = 65535;
		uint16_t maxValue = 0;
		for(int i=0;i<FRAME_SIZE_UINT16;i++) {
			//skip the first 2 uint16_t's of every packet, they're 4 header bytes
			if(i % PACKET_SIZE_UINT16 < 2) {
				continue;
			}
			
			//flip the MSB and LSB at the last second
			int temp = result[i*2];
			result[i*2] = result[i*2+1];
			result[i*2+1] = temp;
			
			value = frameBuffer[i];
			if(value > maxValue) {
				maxValue = value;
			}
			if(value < minValue) {
				minValue = value;
			}
			column = i % PACKET_SIZE_UINT16 - 2;
			row = i / PACKET_SIZE_UINT16 ;
		}
		float diff = maxValue - minValue;
		float scale = 255/diff;
		std::cout<<"Max : "<<maxValue<<" Min : "<<minValue<<std::endl;
		QRgb color;
		for(int i=0;i<FRAME_SIZE_UINT16;i++) {
			if(i % PACKET_SIZE_UINT16 < 2) {
				continue;
			}
			value = (frameBuffer[i] - minValue) * scale;
			const int *colormap = colormap_ironblack;
			color = qRgb(colormap[3*value+2], colormap[3*value+1], colormap[3*value]);
			column = (i % PACKET_SIZE_UINT16 ) - 2;
			row = i / PACKET_SIZE_UINT16;
			myImage.setPixel(column, row, color);
			
		}
		cv::Mat test_img = qimage_to_mat_ref (myImage,CV_8UC3); 
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", test_img).toImageMsg();
		pub.publish(msg);
		//cv::imshow("Test",test_img);
		//cv::waitKey(3);
		ros::spinOnce();
    		loop_rate.sleep();
	}
	SpiClosePort(0);
	return 0;
}
