#include <bgapidef.hpp>
#include "bgapi.hpp"
#include "bgapi_init.h"

int init_systems(int * system_count, std::vector<BGAPI::System*> * externppSystem)
{
	BGAPI_RESULT res = BGAPI_RESULT_FAIL;
	int i = 0;

	res = BGAPI::countSystems( system_count );

	if( res != BGAPI_RESULT_OK )
	{
		ROS_INFO( "BGAPI_CountSystems Errorcode: %d system_count %d\n", res, *system_count );
		return res;
	}

	for( i = 0; i < *system_count; i++ )
	{
		BGAPI::System * pSystem = NULL;
		res = BGAPI::createSystem( i, &pSystem );
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO("BGAPI::CreateSystem Errorcode: %d Systemnumber %d SysPointer 0x%p\n", res, i, (void*)pSystem );
			externppSystem->clear();
			return res;
		}
		res = pSystem->open();		
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO( "BGAPI::System::open Systemnumber %d Errorcode: %d\n", i, res );
			externppSystem->clear();
			return res;
		}
		externppSystem->push_back( pSystem );
	}
	return res;
}

void stop_cameras(std::vector<BGAPI::Camera*> & cameraObjects)
{
	BGAPI_RESULT res = BGAPI_RESULT_FAIL;
	for (unsigned int i = 0; i > cameraObjects.size(); i++)
	{//stop the camera when you are done with the capture
		res = cameraObjects[i]->setStart( false );
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO("BGAPI::Camera::setStart Errorcode: %d\n", res);
		}
	}
}

bool start_cameras(std::vector<BGAPI::Camera*> & cameraObjects)
{
	BGAPI_RESULT res = BGAPI_RESULT_FAIL;

	for (unsigned int i = 0; i < cameraObjects.size(); i++)
	{

		res = cameraObjects[i]->setStart( true );
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO("BGAPI::Camera::setStart returned with errorcode %d\n", res );
			return false;
		}
	}
	return true;
}

bool setup_cameras(std::vector<BGAPI::Camera*> & cameraObjects, std::string camera_pixel_format_str)
{
	BGAPI_RESULT res = BGAPI_RESULT_FAIL;
	for (unsigned int i=0; i<cameraObjects.size(); i++)
	{
		res = cameraObjects[i]->setImagePolling( true );
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO( "Error %d while set Image polling.\n", res );
			return false;
		}
		ROS_INFO( "Image polling OK.\n" );
		res = cameraObjects[i]->setFlipType( BGAPI_FLIPTYPE_XY );
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO("BGAPI::Camera::fliptype Errorcode: %d", res);
			return false;
		}
		ROS_INFO( "Flip ok.\n" );

		BGAPI_FeatureState state;
		state.cbSize = sizeof( BGAPI_FeatureState );

		BGAPIX_CameraPixelFormat pformat ;
		pformat.cbSize = sizeof(BGAPIX_CameraPixelFormat) ;

		BGAPIX_TypeListINT pixellist ;
		pixellist.cbSize = sizeof(BGAPIX_TypeListINT) ;

		BGAPIX_TypeListINT formatlist ;
		formatlist.cbSize = sizeof(BGAPIX_TypeListINT);
		//formatlist.current = 0;
		res = cameraObjects[i] -> getImageFormat(&state ,&formatlist) ;
		if(res != BGAPI_RESULT_OK)
		{
			printf("BGAPI::Camera::getImageFormat error : %d\n" ,res) ;
			return -1;
		}

		res = cameraObjects[i] -> getPixelFormat(formatlist.current, &state ,&pixellist) ;
		if(res != BGAPI_RESULT_OK)
		{
			ROS_INFO("BGAPI::Camera::getPixelFormat1 error:%d\n" ,res) ;
			return false;
		}
		ROS_INFO( "pixel format  OK.\n" );

		int pixel_formart_id_to_select = -1;
		for(int p = 0 ; p < pixellist.length ; p++)
		{
			res = cameraObjects[i] -> getPixelFormatDescription(formatlist.current, pixellist.array[p], &pformat);
			if(res != BGAPI_RESULT_OK)
			{
				ROS_INFO("BGAPI::Camera::getPixelFormatDesc error:%d\n" ,res);
				return false;
			}
			//ROS_INFO( "pixel format descr OK.\n" );
			ROS_INFO("%c ID %x:%s | %d\n" , (p==pixellist.current) ? '*' : ' ', pformat.iPixelFormat, (char*)pformat.sName, pformat.iPixelBits);
			if(strcmp(camera_pixel_format_str.c_str(), (char*)pformat.sName) == 0)
			{
				// change pixel format if current setting does not match arg pixel format.
				if(p != pixellist.current)
				{
					pixel_formart_id_to_select = p;
				}
			}
		}

		if(pixel_formart_id_to_select != -1)
		{
			res = cameraObjects[i] -> setPixelFormat(pixellist.array[pixel_formart_id_to_select]);
			if(res != BGAPI_RESULT_OK)
			{
				ROS_INFO("BGAPI::Camera::setPixelFormat error : %d\n" ,res) ;
				return false;
			}
			//ROS_INFO( "set pixel format  OK.\n" );

			res = cameraObjects[i] -> getPixelFormatDescription(formatlist.current, pixellist.array[pixel_formart_id_to_select], &pformat);
			if(res != BGAPI_RESULT_OK)
			{
				ROS_INFO("BGAPI::Camera::getPixelFormatDesc error:%d\n" ,res);
				return false;
			}
			ROS_INFO("setPixelFormat => (ID %x:%s | %d)\n" , pformat.iPixelFormat, (char*)pformat.sName, pformat.iPixelBits);

			// check result
			res = cameraObjects[i]-> getPixelFormat(formatlist.current, &state ,&pixellist) ;
			if(res != BGAPI_RESULT_OK)
			{
				ROS_INFO("BGAPI::Camera::getPixelFormat2 error:%d\n" ,res) ;
				return false;
			}
			//ROS_INFO( "get pixel format 2 OK.\n" );

			if(pixel_formart_id_to_select != pixellist.current)
			{
				ROS_INFO("can\'t set PixelFormat to =>(%d, %s) (%d)\n", pixellist.array[pixel_formart_id_to_select], camera_pixel_format_str.c_str(), pixellist.current);
				return false;
			}
		}

		res = cameraObjects[i]->setHDREnable( true );
		if (res != BGAPI_RESULT_OK)
		{
			ROS_INFO("Camera->setHDREnable errorcode: %d\n", res);
		}
		else
		{
			ROS_INFO("HDR Mode enabled");
		}

		BGAPIX_TypeRangeINT rangedint;
		rangedint.cbSize = sizeof( BGAPIX_TypeRangeINT );
		res = cameraObjects[i]->getExposure( &state, &rangedint );
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO("BGAPI::Camera::getExposure Errorcode: %d", res);
		}
		else
		{
			ROS_INFO("Current Exposure: %d\n", rangedint.current);
			ROS_INFO("Possible value range: %d to %d\n", rangedint.minimum, rangedint.maximum);
		}

		int exposurevalue = VLG22_DEFAULT_EXPOSURE;
		res = cameraObjects[i]->setExposure( exposurevalue );
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO("BGAPI::Camera::setExposure errorcode %d\n", res);

		}

		BGAPIX_TypeRangeFLOAT gainrange;
		gainrange.cbSize = sizeof( BGAPIX_TypeRangeFLOAT );
		res = cameraObjects[i]->getGain( &state, &gainrange );
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO("BGAPI::Camera::getGain Errorcode: %d\n", res);
		}

		ROS_INFO("Current Gain: %g max:%g min:%g\n", gainrange.current, gainrange.maximum, gainrange.minimum);

		res = cameraObjects[i]->setGain( gainrange.maximum/4 );
		if( res != BGAPI_RESULT_OK )
		{
			printf("BGAPI::Camera::setGain Errorcode: %d\n", res);
		}

	}
	return true;
}

int init_cameras( int system_count, std::vector<BGAPI::System*> * externppSystem, int * pCurrSystem, int& camera_count, std::vector<BGAPI::Camera*>& cameraObjects)
{
	BGAPI_RESULT res = BGAPI_RESULT_FAIL;
	int cam = 0;
	camera_count = 0;
	std::vector<int> cameras;
	std::vector<int>::iterator camIter;
	BGAPI_FeatureState state;
	BGAPIX_CameraInfo cameradeviceinfo;
	std::vector<BGAPI::System*>::iterator systemIter;
	ROS_INFO( "START COUNTING\n" );
	
	for( systemIter = externppSystem->begin(); systemIter != externppSystem->end(); systemIter++ )
	{		
		int count = 0;
		//this is an example how to count available cameras for all available systems
		res = (*systemIter)->countCameras( &count );		
		if( res != BGAPI_RESULT_OK )
		{
			printf( "BGAPI::System::countCameras Systemnumber %d Errorcode: %d\n", (int)(systemIter - externppSystem->begin()), res );
			return res;
		}
		cameras.push_back( count );

		for( cam = 0; cam < count; cam++ )
		{
			camera_count++;
			BGAPI::Camera *pCamera = NULL;
			//this is an example how to create a camera
			res = (*systemIter)->createCamera( cam, &pCamera );
			if( res != BGAPI_RESULT_OK )
			{
				printf("\n");
				printf( "BGAPI::System::createCamera Systemnumber %d Errorcode: %d\n", (int)(systemIter - externppSystem->begin()), res );
				return res;
			}

			//this is an example how to get the device information for a camera
			state.cbSize = sizeof( BGAPI_FeatureState );
			cameradeviceinfo.cbSize = sizeof( BGAPIX_CameraInfo );
			res = pCamera->getDeviceInformation( &state, &cameradeviceinfo );
			if( res != BGAPI_RESULT_OK )
			{	
				printf("\n");
				printf( "BGAPI::Camera::getDeviceInformation Errorcode: %d\n", res );
				return res;
			}
			printf("%d select Camera %d of system %d - %s SN: %s\n", camera_count, cam, (int)(systemIter - externppSystem->begin()), cameradeviceinfo.modelName, cameradeviceinfo.serialNumber );
			(*systemIter)->releaseCamera( pCamera );
		}
	}
	ROS_INFO( "CAMERAS %d\n", camera_count );
	camera_count = 0;
	for( systemIter = externppSystem->begin(); systemIter != externppSystem->end(); systemIter++ )
	{		
		for( cam = 0; cam < cameras[systemIter - externppSystem->begin()]; cam++ )
		{
			camera_count++;
			//search in all baumer systems
			*pCurrSystem = (int)(systemIter - externppSystem->begin());
			BGAPI::Camera *pCamera = NULL;
			//create camera
			ROS_INFO( "CREATE CAMERA\n" );
			res = (*externppSystem)[*pCurrSystem]->createCamera( cam, &pCamera );
			if( res != BGAPI_RESULT_OK )
			{
				ROS_INFO("\n");
				ROS_INFO( "BGAPI::System::createCamera Systemnumber Errorcode: %d\n", res );
				return res;
			}
			//store the camera pointer
			ROS_INFO( "PUSHING CAMERA\n" );
			cameraObjects.push_back(pCamera);
			//Open the camera
			res = pCamera->open();
			if( res != BGAPI_RESULT_OK )
			{
				ROS_INFO("\n");
				ROS_INFO( "BGAPI::Camera::open Systemnumber Errorcode: %d\n", res );
				return res;
			}
			ROS_INFO("Camera %d, correctly initialized\n", cam);
		}
	}
	return res;
}

int release_systems( std::vector<BGAPI::System*> * externppSystem )
{
	BGAPI_RESULT res = BGAPI_RESULT_FAIL;
	std::vector<BGAPI::System*>::iterator systemIter;

	for( systemIter = externppSystem->begin(); systemIter != externppSystem->end(); systemIter++ )
	{
		res = (*systemIter)->release();
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO( "BGAPI::System::release Errorcode: %d\n", (int)res );
		}
	}
	externppSystem->clear();
	return res;
}
int release_images( std::vector<BGAPI::Image*> * ppImage )
{
	BGAPI_RESULT res = BGAPI_RESULT_FAIL;
	std::vector<BGAPI::Image*>::iterator imageIter;
	bool tmpExtern = false;
	unsigned char* tmpBuffer = NULL;
	
	for( imageIter = ppImage->begin(); imageIter != ppImage->end(); imageIter++ )
	{
		res = ((BGAPI::Image*)(*imageIter))->isExternBuffer( &tmpExtern );
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO( "BGAPI::Image::isExternBuffer Errorcode: %d\n", (int)res );
		}
		
		if( tmpExtern )
		{
			res = ((BGAPI::Image*)(*imageIter))->getBuffer( &tmpBuffer );
			if( res != BGAPI_RESULT_OK )
			{
				ROS_INFO( "BGAPI::Image::getBuffer Errorcode: %d\n", (int)res );
			}
			else
			{
				free( tmpBuffer );
			}
		}
		res = BGAPI::releaseImage( *imageIter );
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO( "BGAPI::releaseImage Errorcode: %d\n", (int)res );
		}
	}
	ppImage->clear();
	return res;
}
