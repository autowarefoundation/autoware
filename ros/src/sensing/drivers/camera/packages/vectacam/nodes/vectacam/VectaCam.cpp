#include "VectaCam.h"

VectaCam::VectaCam(std::string in_camera_ip, unsigned int in_configuration_port, unsigned int in_data_port, std::string in_parameter_file)
{
	this->data_port_ 			= in_data_port;
	this->configuration_port_ 	= in_configuration_port;
	this->camera_ready_ 		= false;
	this->running_ 				= false;
	this->fps_ 					= 0.0;
	this->frame_number_			= 0;
	this->frame_counter_		= 0;
	this->payload_offset_ 		= 16;
	this->image_width_ 			= VECTACAM_IMG_WIDTH * VECTACAM_NUM_CAMERAS;
	this->image_height_ 		= VECTACAM_IMG_HEIGHT;
	this->image_buffer_ 		= new char[image_width_ * 3 * image_height_];
	this->parameter_file_ 		= in_parameter_file;
	this->camera_ip_			= in_camera_ip;
	_initialize_camera(configuration_port_, data_port_, parameter_file_);
}

void VectaCam::_parse_parameter_file(std::string in_parameter_file, std::vector<VectaCamCommand>& out_commands)
{
	std::string line;
	std::ifstream config_file (parameter_file_);
	std::string delimiter = ":";
	out_commands.clear();
	if (!config_file.is_open())
	{
		std::cout << "Unable to open file:" << parameter_file_;
		return;
	}

	while ( getline (config_file,line) )
	{
		std::vector<std::string> tokens;
		boost::split(tokens, line, boost::is_any_of(":"));
		if (tokens.size()==2)
		{
			VectaCamCommand current_command;
			int full_address = std::stoi(tokens[0], NULL, 16);
			current_command.low_nibble_address = (unsigned char)( (full_address>>0) & 0xFF);
			current_command.high_nibble_address = (unsigned char)((full_address>>8) & 0xFF);
			current_command.command_data = std::stoi(tokens[1], NULL, 16);
			out_commands.push_back(current_command);
		}
		else
			std::cout << "Unrecognized command in configuration file: " << parameter_file_ << std::endl;
	}
	config_file.close();
}

void VectaCam::_send_commands_to_camera(unsigned int in_port, std::vector<VectaCamCommand> in_commands)
{

	int socket_descriptor;
	struct sockaddr_in socket_address;
	if ((socket_descriptor=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
	{
		std::cout << "Problem creating socket\n";
		return;
	}
	memset((char *) &socket_address, 0, sizeof(socket_address));
	socket_address.sin_family = AF_INET;
	socket_address.sin_port = htons(in_port);
	if (inet_aton(camera_ip_.c_str(), &socket_address.sin_addr)==0)
	{
		std::cout << "Invalid IP address" << std::endl;
		return;
	}
	for (unsigned int i=0; i< in_commands.size(); i++)
	{
		VectaCamCommand current_command = in_commands[i];
		unsigned char command_buffer[12] = {0x05, 0x00, 0x36, 0x00, 0x00, 0x00,
									current_command.low_nibble_address, current_command.high_nibble_address, current_command.command_data,
									0x00, 0x00, 0x00};
		ssize_t sent_data = sendto(socket_descriptor, command_buffer, sizeof(command_buffer), 0,(struct sockaddr *) &socket_address, sizeof(socket_address));
		if (sent_data < 0)
		{
			std::cout << "Could not send the data to the camera socket" << std::endl;
		}
		else
			std::cout << (int)current_command.high_nibble_address <<
						(int)current_command.low_nibble_address << ":" <<
						(int)current_command.command_data << std::endl;
		usleep(50000);
	}
}

void VectaCam::_enable_camera(unsigned int in_port, bool in_enable)
{
	unsigned char camonoff = in_enable ? 0xff : 0x00;
	unsigned char enable_command[] =	{0x00, 0x00, // upldest = 0
				 0x00, // inspect target
				 0x02, // command(write)
				 0x00, 0x00, 0x00, 0x00, // data_output
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00,
				 0x00, 0x00, 0x00, (camonoff),
				 0x00, 0x00, 0x00, 0x00, // module_reset
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00, //
				 0x00, 0x00, 0x00, 0x00 };
	int socket_descriptor;
	struct sockaddr_in socket_address;
	if ((socket_descriptor=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
	{
		std::cout << "Problem creating socket\n";
		return;
	}
	memset((char *) &socket_address, 0, sizeof(socket_address));
	socket_address.sin_family = AF_INET;
	socket_address.sin_port = htons(in_port);
	if (inet_aton(camera_ip_.c_str(), &socket_address.sin_addr)==0)
	{
		std::cout << "Invalid IP address" << std::endl;
		return;
	}
	if (sendto(socket_descriptor, enable_command, sizeof(enable_command), 0, reinterpret_cast<sockaddr *>(&socket_address), sizeof(socket_address)) < 0)
	{
		std::cout << "Could not send the enable command to the camera socket" << std::endl;
	}
}

void VectaCam::_initialize_camera(unsigned int in_configuration_port, unsigned int in_data_port, std::string in_parameter_file)
{
	std::cout << "Reading configuration file...";
	_parse_parameter_file(parameter_file_, camera_commands_);
	std::cout << "DONE" << std::endl << "Configuring camera.";
	_send_commands_to_camera(in_configuration_port, camera_commands_);
	std::cout << "DONE" << std::endl << "Enabling camera.";
	_enable_camera(in_data_port, true);
	std::cout << "DONE" << std::endl;
}

VectaCam::~VectaCam()
{
	this->running_=false;
	delete[] this->image_buffer_;
}

void VectaCam::GetImage(cv::Mat& out_image)
{
	out_image = camera_image_.clone();
}

void VectaCam::StopCamera()
{
	this->running_ = false;
	_enable_camera(data_port_, false);
}

long int VectaCam::GetFrameNumber()
{
	return this->frame_number_;
}

void VectaCam::StartCamera()
{
	int 		socket_descriptor;
	struct 		sockaddr_in socket_address;
	socklen_t length;

	if ((socket_descriptor = socket( PF_INET, SOCK_DGRAM, 0)) < 0)
	{
		std::cout << "Problem creating socket\n";
		return;
	}
	std::cout << "Socket Created\n";

	socket_address.sin_family = AF_INET;
	socket_address.sin_addr.s_addr = htonl(INADDR_ANY);
	socket_address.sin_port = htons(this->data_port_);

	if (bind(socket_descriptor, reinterpret_cast<sockaddr *> (&socket_address), sizeof(socket_address)) < 0)
	{
		std::cout << "Problem binding" << std::endl;
		return;
	}
	std::cout << "Binded..." << std::endl;

	length = sizeof(socket_address);
	if (getsockname(socket_descriptor, reinterpret_cast<sockaddr *> (&socket_address), &length) < 0)
	{
		std::cout << "Error getsockname" << std::endl;
		return;
	}
	this->camera_ready_ = true;
	this->running_ = true;

	std::cout << "Listening camera in UDP port number " << ntohs(socket_address.sin_port) << std::endl;

	_udp_receive(socket_descriptor);
}

void VectaCam::_form_image(int in_line_number, char* in_buffer, uint32_t in_packet_offset, uint32_t in_packet_length)
{
	int p = in_packet_offset * 3;
	int k = payload_offset_ + 4 * in_packet_length;
	int total = (in_packet_length);
	if (in_line_number < image_height_ && total < 1500 && p < image_width_ * 3
			&& total > 10)
	{
		for (int i = 0; i < total; ++i)
		{
			image_buffer_[in_line_number * image_width_ * 3 + p + 2] = (((((int) in_buffer[k + 0]) << 4) & 0x3f0) + ((((int) in_buffer[k + 1]) >> 4) & 0xf)) >> 2; // R
			image_buffer_[in_line_number * image_width_ * 3 + p + 1] = (((((int) in_buffer[k + 1]) << 6) & 0x3c0) + ((((int) in_buffer[k + 2]) >> 2) & 0x3f)) >>2; // G
			image_buffer_[in_line_number * image_width_ * 3 + p + 0] = (((((int) in_buffer[k + 2]) << 8) & 0x300)	+ ((((int) in_buffer[k + 3]) >> 0) & 0xff)) >> 2; // B

			p += 3;
			k -= 4;
		}
	}
}

void VectaCam::_udp_receive(int in_socket_descriptor)
{
	int 			n;
	size_t			length;
	char 			buffer_in[VECTACAM_MAX_BUFFER];
	struct 			sockaddr_in remote;
	length = sizeof(remote);

	int line_number = 0;

	while (this->running_)
	{
		n = recvfrom(in_socket_descriptor, buffer_in, VECTACAM_MAX_BUFFER, 0, reinterpret_cast<sockaddr *> (&remote), (unsigned int*) &length);
		if (n < 0)
		{
			std::cout << "Error receiving data" << std::endl;
		}
		else
		{
			//std::cout << "Received data " << n << std::endl;
			uint32_t packet_offset = ntohl(*(uint32_t*)(buffer_in + 4));
			uint32_t packet_length = ntohl(*(uint32_t*)(buffer_in + 8));
			uint32_t header = ntohl(*(uint32_t*)(buffer_in + 12));

			if (header == VECTACAM_FRAME_START) {
				// frame start
			}
			else if (header == VECTACAM_FRAME_END)
			{
				// frame end
				this->camera_image_ = cv::Mat(image_height_, image_width_, CV_8UC3, image_buffer_);
				frame_number_++;
			}
			else
			{
				line_number = header - 1;
				_form_image(line_number, buffer_in, packet_offset, packet_length);
			}
		}
	}
}

