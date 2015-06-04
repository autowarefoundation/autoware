#include "bgapi.hpp"
#include "bgapi_init.h"

using namespace std;

int init_systems(int * system_count, vector<BGAPI::System*> * externppSystem)
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

bool create_images_start(std::vector<BGAPI::Image*>& imagePointers,  std::vector<BGAPI::Camera*> & cameraObjects)
{
	BGAPI_RESULT res = BGAPI_RESULT_FAIL;
	for (unsigned int i = 0; i > cameraObjects.size(); i++)
	{
		BGAPI::Image* pImage = NULL;
		//create an image for each camera
		res = BGAPI::createImage( &pImage ); 
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO( "Error %d while creating an image.\n", res );
			return false;
		}
		//store the pointer
		imagePointers.push_back(pImage);

		//set the image to the camera 
		res = cameraObjects[i]->setImage( pImage );
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO( "Error %d while setting an image to the camera.\n", res );
			return false;
		}
		
		res = cameraObjects[i]->setStart( true );
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO("BGAPI::Camera::setStart returned with errorcode %d\n", res );
			return false;
		}
		return true;
	}
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

		res = cameraObjects[i]->setFlipType( BGAPI_FLIPTYPE_XY );
		if( res != BGAPI_RESULT_OK )
		{
			ROS_INFO("BGAPI::Camera::fliptype Errorcode: %d", res);
			return false;
		}


	//PIXEL FORMAT
	// --------

		BGAPI_FeatureState state;
		state.cbSize = sizeof( BGAPI_FeatureState );

		BGAPIX_CameraPixelFormat pformat ;
		pformat.cbSize = sizeof(BGAPIX_CameraPixelFormat) ;

		BGAPIX_TypeListINT pixellist ;
		pixellist.cbSize = sizeof(BGAPIX_TypeListINT) ;

		BGAPIX_TypeListINT formatlist ;
		formatlist.cbSize = sizeof(BGAPIX_TypeListINT);

		res = cameraObjects[i] -> getPixelFormat(formatlist.current, &state ,&pixellist) ;
		if(res != BGAPI_RESULT_OK)
		{
			ROS_INFO("BGAPI::Camera::getPixelFormat error:%d\n" ,res) ;
			return false;
		}

		int pixel_formart_id_to_select = -1;
		for(int p = 0 ; p < pixellist.length ; p++)
		{
			res = cameraObjects[i] -> getPixelFormatDescription(formatlist.current, pixellist.array[p], &pformat);
			if(res != BGAPI_RESULT_OK)
			{
				ROS_INFO("BGAPI::Camera::getPixelFormatDesc error:%d\n" ,res);
				return false;
			}

			//ROS_INFO("%c ID %x:%s | %d\n" , (p==pixellist.current) ? '*' : ' ', pformat.iPixelFormat, (char*)pformat.sName, pformat.iPixelBits);
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

			res = cameraObjects[i] -> getPixelFormatDescription(formatlist.current, pixellist.array[pixel_formart_id_to_select], &pformat);
			if(res != BGAPI_RESULT_OK)
			{
				ROS_INFO("BGAPI::Camera::getPixelFormatDesc error:%d\n" ,res);
				return false;
			}
			ROS_INFO("setPixelFormat => (ID %x:%s | %d)\n" , pformat.iPixelFormat, (char*)pformat.sName, pformat.iPixelBits);


			// check result
			res = cameraObjects[i] -> getPixelFormat(formatlist.current, &state ,&pixellist) ;
			if(res != BGAPI_RESULT_OK)
			{
				ROS_INFO("BGAPI::Camera::getPixelFormat error:%d\n" ,res) ;
				return false;
			}

			if(pixel_formart_id_to_select != pixellist.current)
			{
				ROS_INFO("can\'t set PixelFormat to =>(%d, %s) (%d)\n", pixellist.array[pixel_formart_id_to_select], camera_pixel_format_str.c_str(), pixellist.current);
				return false;
			}
		}
	//PIXEL FORMAT
	}
}

int init_cameras( int system_count, vector<BGAPI::System*> * externppSystem, int * pCurrSystem, int& numCameras, std::vector<BGAPI::Camera*>& cameraObjects)
{
	BGAPI_RESULT res = BGAPI_RESULT_FAIL;
	int cam = 0;
	int camera_count = 0;
	vector<int> cameras;
	vector<int>::iterator camIter;
	BGAPI_FeatureState state;
	BGAPIX_CameraInfo cameradeviceinfo;
	int inputVal = 0;
	vector<BGAPI::System*>::iterator systemIter;

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
			res = (*externppSystem)[*pCurrSystem]->createCamera( cam, &pCamera );
			if( res != BGAPI_RESULT_OK )
			{
				ROS_INFO("\n");
				ROS_INFO( "BGAPI::System::createCamera Systemnumber Errorcode: %d\n", res );
				return res;
			}
			//store the camera pointer
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

int release_systems( vector<BGAPI::System*> * externppSystem )
{
	BGAPI_RESULT res = BGAPI_RESULT_FAIL;
	vector<BGAPI::System*>::iterator systemIter;

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
int release_images( vector<BGAPI::Image*> * ppImage )
{
	BGAPI_RESULT res = BGAPI_RESULT_FAIL;
	vector<BGAPI::Image*>::iterator imageIter;
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
