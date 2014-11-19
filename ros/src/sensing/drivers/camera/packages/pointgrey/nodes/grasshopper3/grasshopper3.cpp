/*
This program requires ROS and Flycapture SDK installed
Author: Abraham Monrroy (amonrroy@ertl.jp)
Initial version 2014-11-14
*/
#include "FlyCapture2.h"
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <sys/time.h>
#include <stdio.h>

#define FPS 30//CURRENT SETTING 30FPS (initializeCamera)

using namespace FlyCapture2;

void getNumCameras(BusManager* pbusMgr, unsigned int *pNumCameras);
void initializeCameras(BusManager* pbusMgr, unsigned int numCameras, Camera*** pCams);
void captureImage(Camera** ppCameras, unsigned int frame, unsigned int numCameras, Image images[]);
void startCapture(unsigned int numCameras, Camera** ppCameras);
void PrintError( Error error );
void PrintCameraInfo( CameraInfo* pCamInfo );

int main(int argc, char **argv)
{
	BusManager busMgr;
    Error error;

    unsigned int numCameras;

    getNumCameras(&busMgr, &numCameras);
    Camera** ppCameras = new Camera*[numCameras];
    initializeCameras(&busMgr, numCameras, &ppCameras);

    Image images[numCameras];

    //ROS STUFF

    ros::init(argc, argv, "grasshopper3");
    ros::NodeHandle n;

    ros::Publisher pub[numCameras];

    ros::Rate loop_rate(FPS); // Hz

    for (unsigned int i=0; i< numCameras; i++)
    {
    	char topic_name[512];
    	sprintf( topic_name, "image_raw%d",
    					i);
    	pub[i]= n.advertise<sensor_msgs::Image>(topic_name, 100);//publish as many cameras as we have
    }

    startCapture(numCameras, ppCameras);

    unsigned int count=0;
    while(ros::ok())
    {
    	sensor_msgs::Image imagemsg[numCameras];
    	for (unsigned int i=0; i<numCameras;i++)//for each camera capture and publish
    	{
    		struct timeval te;

    		gettimeofday(&te, NULL); // get current time

    		captureImage(ppCameras, count, numCameras, images);//Get image from camera

    		//fill ROS Message structure
    		imagemsg[i].header.seq=count;
    		imagemsg[i].header.frame_id=count;
    		imagemsg[i].header.stamp.sec=te.tv_sec;
    		imagemsg[i].header.stamp.nsec=te.tv_usec*1000;
    		imagemsg[i].height= images[i].GetRows();
    		imagemsg[i].width= images[i].GetCols();
    		imagemsg[i].encoding = "rgb8";
    		imagemsg[i].step = images[i].GetStride();
    		imagemsg[i].data.resize(images[i].GetDataSize());
    		memcpy(imagemsg[i].data.data(),images[i].GetData(), images[i].GetDataSize());

    		pub[i].publish(imagemsg[i]);//publish
    	}
    	ros::spinOnce();
    	loop_rate.sleep();
    	count++;
    }

    //close cameras
	for ( unsigned int i = 0; i < numCameras; i++ )
	{
		ppCameras[i]->StopCapture();
		ppCameras[i]->Disconnect();
		delete ppCameras[i];
	}

	delete [] ppCameras;//clean memory

	printf( "Done!\n" );

	return 0;

}

void PrintError( Error error )
{
    error.PrintErrorTrace();
}

void getNumCameras(BusManager* pbusMgr, unsigned int *pNumCameras)
{
	Error error;
	error = (*pbusMgr).GetNumOfCameras(pNumCameras);
	if (error != PGRERROR_OK)
	{
		PrintError( error );
		exit(-1);
	}

	printf( "Number of cameras detected: %u\n", *pNumCameras );

	if ( *pNumCameras < 1 )
	{
		printf( "This program requires at least 1 camera... press Enter to exit.\n");
		getchar();
		exit(-1);
	}
}

void initializeCameras(BusManager* pbusMgr, unsigned int numCameras, Camera*** pCams)
{
	Error error;
	Camera** ppCameras = *pCams;//*unwrap pointers

	// Connect to all detected cameras and attempt to set them to
	// a common video mode and frame rate
	for ( unsigned int i = 0; i < numCameras; i++)
	{
		ppCameras[i] = new Camera();

		PGRGuid guid;
		error = (*pbusMgr).GetCameraFromIndex( i, &guid );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			exit(-1);
		}

		// Connect to a camera
		error = ppCameras[i]->Connect( &guid );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			exit(-1);
		}

		EmbeddedImageInfo TestEmbeddedInfo;
		error = ppCameras[i]->GetEmbeddedImageInfo(&TestEmbeddedInfo);
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			exit(-1);
		}

		TestEmbeddedInfo.timestamp.onOff = true;
		error = ppCameras[i]->SetEmbeddedImageInfo(&TestEmbeddedInfo);
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			exit(-1);
		}
		// Set all cameras to a specific mode and frame rate so they
		// can be synchronized
		error = ppCameras[i]->SetVideoModeAndFrameRate(
				VIDEOMODE_1600x1200RGB,
				FRAMERATE_30 );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			printf(
					"Error starting cameras. \n"
					"This example requires cameras to be able to set to 1600x1200 RGB at 30fps. \n"
					"If your camera does not support this mode, please edit the source code and recompile the application. \n"
					"Press Enter to exit. \n");
			getchar();
			exit(-1);
		}

		// Get the camera information
		CameraInfo camInfo;
		error = ppCameras[i]->GetCameraInfo( &camInfo );
		if (error != PGRERROR_OK)
		{
			PrintError( error );
			exit(-1);
		}

		PrintCameraInfo(&camInfo);

	}
}

void startCapture(unsigned int numCameras, Camera** ppCameras)
{
	Error error;

	//initialize cameras
	for (unsigned int i=0; i<numCameras; i++)
	{
		error=ppCameras[i]->StartCapture();

		if (error != PGRERROR_OK)
		{
			PrintError( error );
			return;
		}
	}
	return;
}

void captureImage(Camera** ppCameras, unsigned int frame, unsigned int numCameras, Image images[])
{
	// Display the time stamps for all cameras to show that the image
	// capture is synchronized for each image

	JPEGOption opt;
	opt.progressive=false;
	opt.quality=100;


	Error error[numCameras];

	//retrieve images continuously
	for (unsigned int i=0; i<numCameras;i++)
	{
		error[i] = ppCameras[i]->RetrieveBuffer( &(images[i]) );
	}
	//check for Errors
	for (unsigned int i=0; i<numCameras;i++)
	{
		if (error[i] != PGRERROR_OK)
		{
			PrintError( error[i] );
			exit(-1);
		}
	}
}

void PrintCameraInfo( CameraInfo* pCamInfo )
{
    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\t"
        "Camera model - %s\t"
        "Camera vendor - %s\t"
        "Sensor - %s\n"
        "Resolution - %s\t"
        "Firmware version - %s\t"
        "Firmware build time - %s\n\n",
        pCamInfo->serialNumber,
        pCamInfo->modelName,
        pCamInfo->vendorName,
        pCamInfo->sensorInfo,
        pCamInfo->sensorResolution,
        pCamInfo->firmwareVersion,
        pCamInfo->firmwareBuildTime );
}
