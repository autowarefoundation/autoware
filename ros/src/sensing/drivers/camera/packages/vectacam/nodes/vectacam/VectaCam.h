#ifndef VECTACAM_H_
#define VECTACAM_H_

#include <unistd.h>     /* defines STDIN_FILENO, system calls,etc */
#include <sys/types.h>  /* system data type definitions */
#include <sys/socket.h> /* socket specific definitions */
#include <netinet/in.h> /* INET constants and stuff */
#include <arpa/inet.h>  /* IP address conversion stuff */
#include <netdb.h>      /* gethostbyname */

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>

#include "opencv2/core/core.hpp"

#define VECTACAM_CONFIG_PORT 		16385
#define VECTACAM_DATA_PORT 			16386
#define VECTACAM_NUM_CAMERAS		8

#define VECTACAM_FRAME_START 		(0x01000000)
#define VECTACAM_FRAME_END   		(0x02000000)

#define VECTACAM_DATA_OFFSET 		(16)

#define VECTACAM_MAX_BUFFER 			1024*1024

#define VECTACAM_IMG_WIDTH			640
#define VECTACAM_IMG_HEIGHT			480

#define VECTACAM_CAMERA_IP			"10.0.0.1"

struct VectaCamCommand
{
	unsigned char low_nibble_address;
	unsigned char high_nibble_address;
	unsigned char command_data;
};

class VectaCam
{
public:
	VectaCam(std::string in_camera_ip, unsigned int in_configuration_port, unsigned int in_data_port, std::string in_parameter_file);
	void 			GetImage(cv::Mat& out_image);
	virtual 		~VectaCam();
	void 			StartCamera();
	void 			StopCamera();
	long int		GetFrameNumber();
	std::string 	GetFps();

private:
	int 			data_port_;
	int 			configuration_port_;
	bool 			running_;
	cv::Mat			camera_image_;
	bool			camera_ready_;
	long 	 		frame_counter_;
	long 	 		frame_number_;
	double 			fps_;
	int 			payload_offset_;
	int				image_width_;
	int				image_height_;
	char			*image_buffer_;
	std::string		camera_ip_;
	void 			_udp_receive(int in_socket_descriptor);
	void			_initialize_camera(unsigned int in_configuration_port, unsigned int in_data_port, std::string in_parameter_file);
	void			_parse_parameter_file(std::string in_parameter_file, std::vector<VectaCamCommand>& out_commands);
	void			_send_commands_to_camera(unsigned int in_port, std::vector<VectaCamCommand> in_commands);
	void 			_enable_camera(unsigned int in_port, bool in_enable);
	void 			_form_image(int in_line_number, char* in_buffer, uint32_t in_packet_offset, uint32_t in_packet_length);
	std::string		parameter_file_;
	std::vector<VectaCamCommand> camera_commands_;
};

#endif /* VECTACAM_H_ */
