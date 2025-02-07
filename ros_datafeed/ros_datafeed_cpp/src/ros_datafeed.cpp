/**
 *
 *  \file
 *  \brief      Main entry point for the socket server node.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2013, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <rosPointCloud2.h>

#include <topic_tools/shape_shifter.h>
#include <std_msgs/Byte.h>

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/time.h>

using namespace std;

#define M_PI 3.14159265358979323846

#define PORT 8080
#define LIDAR_RANGE_RESOLUTION 1.2
#define LIDAR_MAX_RANGE 5000
#define LIDAR_WIDTH 64

int socket_desc;
vector<int> client_socks;
bool bConnect;

struct ShapeShifterData
{
	float x;
	float y;
	float z;
	float dummy1;
	float t;
	uint16_t reflectivity;
	uint16_t intensity;
	uint8_t ring;
	uint8_t dummy2, dummy3, dummy4;
	float dummy5;
};

struct CloudPoint
{
	float x;
	float y;
	float z;
	uint8_t ring;
};

void createTcpServer()
{
	struct sockaddr_in server;
	int opt = 1;	
	
	//Create socket
	socket_desc = socket(AF_INET , SOCK_STREAM , 0);
	if (socket_desc == -1)
	{
		printf("Could not create socket \n");
	}
	printf("Socket created \n");
	
	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons( 8888 );
	
	//Bind
	if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
	{
		//print the error message
		printf("bind failed. Error \n");
		return;
	}
	printf("bind done \n");
	
	//Listen
	listen(socket_desc , 3);
	client_socks.clear();
	printf("connected client count: %d \n", client_socks.size());

}
void *acceptClient(void* arg)
{
	int c;
	struct sockaddr_in client;
	//Accept and incoming connection
	printf("Waiting for incoming connections... \n");
	c = sizeof(struct sockaddr_in);
	
	//accept connection from an incoming client
	while(true)
	{
		int client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c);
		if (client_sock < 0)
		{
			printf("accept failed \n");
			continue;
		}
		client_socks.push_back(client_sock);
		printf("new Connection accepted \n");
		printf("connected client count: %d \n", client_socks.size());
	}

	
}

datafeed_sensor_msgs::PointCloud2 pointCloud2;
int frameNum = 0;

timeval startT;
timeval endT;
void sendPointData(std::vector<uint8_t>& buffer)
{
	int send_size;
	uint32_t buffersize = buffer.size();

	unsigned char* inBuffer = new unsigned char[buffersize];

	std::copy(buffer.begin(), buffer.end(), inBuffer);

	pointCloud2.deserialize(inBuffer);

	int nCloudPtCnt = pointCloud2.height * pointCloud2.width;
	ShapeShifterData* points = (ShapeShifterData*)malloc( nCloudPtCnt * sizeof( ShapeShifterData ) );
	memcpy( &points[0], pointCloud2.data, nCloudPtCnt * sizeof( ShapeShifterData ) );

	int sendBufferSize = nCloudPtCnt * 2;

	char* client_message = new char[sendBufferSize + 8];
	memset(client_message, 0, sendBufferSize + 8);
	for(int i = 0; i < 4; i++)
	{
		*(client_message + i) = (sendBufferSize >> (8*i)) & 0xFF;
	}

	for(int i = 0; i < 4; i++)
	{
		*(client_message + i + 4) = (frameNum >> (8*i)) & 0xFF;
	}


	for(int i = 0; i < nCloudPtCnt; i++ ) 
	{
		float x = points[i].x;
		float y = points[i].y;
		float z = points[i].z;
		uint8_t ring = points[i].ring;

		float length = sqrt(x*x + y*y + z*z);

		if(length < 0.1)
			continue;

		if(length > LIDAR_MAX_RANGE * LIDAR_RANGE_RESOLUTION / 100)
			continue;

		uint16_t serialDepth = floor(length * 100 / LIDAR_RANGE_RESOLUTION);

		float angle_x = atan2f(y, x)*180/M_PI + 180;
 
		int rowNum = ring;
		int colNum = angle_x * 1024 /360;

		if(colNum >= 1024)
			colNum -= 1024;

		int index = 64 * colNum + rowNum;

		if(index < nCloudPtCnt)
		{
		
			*(client_message + 2*index + 8) = (serialDepth) & 0xFF;
			*(client_message + 2*index + 1 + 8) = (serialDepth >> 8) & 0xFF;
		}
	}
	

	vector<int> removeindex;

	for(int  i =0; i < client_socks.size(); i++ )
	{
		int client_sock = client_socks.at(i);
		send_size = write(client_sock , client_message , sendBufferSize + 8);

		if(send_size == 0)
		{
			printf("Client disconnected \n");
			fflush(stdout);
			removeindex.push_back(i);
		}
		else if(send_size == -1)
		{
			printf("recv failed \n");
			removeindex.push_back(i);
		}
		frameNum++;
		if(frameNum % 100 == 0)
		{
			//printf("frameNum : %d \n", frameNum);
			gettimeofday(&endT, 0);
			int duration = (endT.tv_sec - startT.tv_sec)*1000 + (endT.tv_usec - startT.tv_usec)/1000;
			if(duration > 0)
			{
				printf("Lidar FPS: %d \n", 100 * 1000 / duration);
				//printf("duration: %d \n", duration);
			}
			gettimeofday(&startT, 0);
		}		
	}	
	
	for(int i = removeindex.size() - 1; i >=0; i--)
	{
		client_socks.erase(client_socks.begin() + removeindex.at(i));
		printf(" %dth Connection removed \n", removeindex.at(i) + 1);
		printf("connected client count:%d \n", client_socks.size());
	}
	delete [] client_message;	
	delete [] inBuffer;
	delete [] points;
	return;
}


void messageCallback(const boost::shared_ptr<topic_tools::ShapeShifter const>& msg)
{
	size_t length = ros::serialization::serializationLength(*msg);
	std::vector<uint8_t> buffer(length);

	ros::serialization::OStream ostream(&buffer[0], length);
	ros::serialization::Serializer<topic_tools::ShapeShifter>::write(ostream, *msg);

	//printf("length=%d, buffer[0]=%02X,buffer[length-1]=%02X\n", buffer.size(), buffer[0],buffer[length-1]);

	sendPointData(buffer);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "ros_datafeed");

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<topic_tools::ShapeShifter>("/os1_node/points", 1, &messageCallback);

	createTcpServer();
	
	pthread_t acceptThread;
	pthread_create(&acceptThread, NULL, acceptClient, NULL);

	while(true)
	{
		ros::spinOnce();
		ros::Duration(0.05).sleep();
	}

	return 0;
}
