/*
 *  Copyright (c) 2016, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <utility>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


class CamServer
{
	int soc;
	static const uint16_t setup_port = 0x4002;
	static const uint16_t config_port = 0x4001;
	static const int r_coeff = 0x0100;
	static const int g_coeff = 0x0100;
	static const int b_coeff = 0x0100;
	static const int mode = 0x01;
	static const int Read = 0;
	static const int Write = 1;
	struct seq{
		int addr;
		int value;
		int dir;
	};
	sockaddr_in sa;
	unsigned curseq;
	std::istream &m_is;
public:
	CamServer(const char *addr, std::istream &m_is);
	int SendPacket(int s,uint16_t  port, void *data, int &size);
	int RecvPacket(int s, void *data, int &size);	
	void ConfigCamera();
	int ConfigSequence(seq &a);
	void EnableCamera(int en);
};

int DefaultStream(std::stringstream &ss);

class HexaCam{
	static const int frame_start = 0x01000000;
	static const int frame_end = 0x02000000;
	static const int data_offset = 16;
	static const int x_size = 640;
	static const int y_size = 480;
	static const unsigned int magic_offset = 0xfefefefe;
	static const int port_base = 0x4002;

	uint32_t old_line;
	int start_flag ;
	int start_flag_old ;
	int port;
	int ld;
	std::vector<int> frame_flag;
	std::vector<ros::Publisher > pub;

	int id;
	std::vector<sensor_msgs::Image > msgs[2];
	std::vector<sensor_msgs::Image > *curmsg;
	std::vector<sensor_msgs::Image > *nextmsg;

	int count;
	const int numcam;
public:
	HexaCam(ros::NodeHandle &n, int cam);
	~HexaCam();
	void parse_packet();	
	
};
CamServer::CamServer(const char *remote,std::istream &is):curseq(0),m_is(is)
{
	sa.sin_family = AF_INET;
	sa.sin_addr.s_addr = inet_addr(remote);

}

int CamServer::SendPacket(int s,uint16_t  port, void *data, int &size)
{
	sockaddr_in mysa;
	mysa = sa;
	mysa.sin_port = htons(port);
	size = sendto(s, data, size, 0, reinterpret_cast<sockaddr *>(&mysa), sizeof(mysa));
	return 0;
}
int CamServer::RecvPacket(int s,void *data, int &size)
{
	sockaddr_in mysa;
	socklen_t addrlen = sizeof(mysa);
	size = recvfrom(s, data, size, 0, reinterpret_cast<sockaddr *>(&mysa),
			&addrlen);

	return size;

}
void CamServer::EnableCamera(int en)
 {
	int sock;
	sockaddr_in local;
	char buf[1500];
	memset(buf,0, sizeof(buf));
	unsigned char flag = (en)? 0xff: 0x00;
	int len;

	buf[0x03] = 2;
	buf[0x3c] =((r_coeff >> 8) & 0xFF);
	buf[0x3d] = ((r_coeff >> 0) & 0xFF);
	buf[0x3e] = ((g_coeff >> 8) & 0xFF);
	buf[0x3f] = ((g_coeff >> 0) & 0xFF);
	buf[0x40] = ((b_coeff >> 8) & 0xFF); 
	buf[0x41] = ((b_coeff >> 0) & 0xFF);
	buf[0x42] = (mode & 0xFF);
	buf[0x43] = (flag & 0xFF);

	if((sock = socket(PF_INET, SOCK_DGRAM, 0))<0){
		throw("START CAMERA SOCKET FAILED\n");
	}

	local.sin_family = AF_INET;
	local.sin_addr.s_addr = htonl(INADDR_ANY);
	local.sin_port = htons(setup_port);
	bind(sock, reinterpret_cast<sockaddr *>(&local), sizeof(local));
	len = 0x54;
	SendPacket(sock, setup_port, buf, len);
	len = sizeof(buf);
	RecvPacket(sock, buf, len);
	close(sock);
}

int DefaultStream(std::stringstream &ss)
{
  std::string file(
		   "[18:57:19:486019] 0103:01 ()\n"
		   "[18:57:19:486472] 0100:00 ()\n"
		   "[18:57:19:573903] 0100:00 ()\n"
		   "[18:57:19:574774] 0103:01 ()\n"
		   "[18:57:19:575622] 3034:1a ()\n"
		   "[18:57:19:576519] 3035:21 ()\n"
		   "[18:57:19:577425] 3036:69 (SC_CMMN_PLL_MULTIPLIER)\n"
		   "[18:57:19:578284] 303c:11 (SC_CMMN_PLL_MULTIPLIER)\n"
		   "[18:57:19:579208] 3106:f5 ()\n"
		   "[18:57:19:580068] 3821:00 (TIMING_TC_REG21)\n"
		   "[18:57:19:580971] 3820:00 (TIMING_TC_REG20)\n"
		   "[18:57:19:581848] 3827:ec ()\n"
		   "[18:57:19:582740] 370c:03 (ANALOG_CONGROL)\n"
		   "[18:57:19:583633] 3612:5b (ANALOG_CONGROL)\n"
		   "[18:57:19:584530] 3618:04 (ANALOG_CONGROL)\n"
		   "[18:57:19:585449] 5000:06 ()\n"
		   "#[18:57:19:586306] 5002:40 ()\n"
		   "[18:57:19:586306] 5002:41 (AWB gain)\n"
		   "[18:57:19:587233] 5003:08 ()\n"
		   "[18:57:19:588086] 5a00:08 ()\n"
		   "[18:57:19:588980] 3000:00 ()\n"
		   "[18:57:19:589872] 3001:00 ()\n"
		   "[18:57:19:590772] 3002:00 ()\n"
		   "[18:57:19:591653] 3016:08 ()\n"
		   "[18:57:19:592546] 3017:e0 ()\n"
		   "[18:57:19:593471] 3018:44 ()\n"
		   "[18:57:19:594331] 301c:f8 ()\n"
		   "[18:57:19:595256] 301d:f0 ()\n"
		   "[18:57:19:596113] 3a18:00 ()\n"
		   "[18:57:19:597002] 3a19:f8 ()\n"
		   "[18:57:19:597894] 3c01:80 ()\n"
		   "[18:57:19:598786] 3b07:0c ()\n"
		   "[18:57:19:599676] 380c:0b (TIMING_HTS)\n"
		   "[18:57:19:600584] 380d:1c (TIMING_HTS)\n"
		   "[18:57:19:601494] 380e:07 (TIMING_VTS)\n"
		   "[18:57:19:602348] 380f:b0 (TIMING_VTS)\n"
		   "[18:57:19:603278] 3814:11 (TIMING_X_INC)\n"
		   "[18:57:19:604135] 3815:11 (TIMING_Y_INC)\n"
		   "[18:57:19:605041] 3708:64 (ANALOG_CONGROL)\n"
		   "[18:57:19:605921] 3709:12 (ANALOG_CONGROL)\n"
		   "[18:57:19:606807] 3808:0a (TIMING_X_OUTPUT_SIZE)\n"
		   "[18:57:19:607707] 3809:20 (TIMING_X_OUTPUT_SIZE)\n"
		   "[18:57:19:608589] 380a:07 (TIMING_Y_OUTPUT_SIZE)\n"
		   "[18:57:19:609514] 380b:98 (TIMING_Y_OUTPUT_SIZE)\n"
		   "[18:57:19:610379] 3800:00 (TIMING_X_ADDR_START)\n"
		   "[18:57:19:611297] 3801:00 (TIMING_X_ADDR_START)\n"
		   "[18:57:19:612155] 3802:00 (TIMING_Y_ADDR_START)\n"
		   "[18:57:19:613048] 3803:00 (TIMING_Y_ADDR_START)\n"
		   "[18:57:19:613960] 3804:0a (TIMING_X_ADDR_END)\n"
		   "[18:57:19:614830] 3805:3f (TIMING_X_ADDR_END)\n"
		   "[18:57:19:615737] 3806:07 (TIMING_Y_ADDR_END)\n"
		   "[18:57:19:616649] 3807:a3 (TIMING_Y_ADDR_END)\n"
		   "[18:57:19:617538] 3811:10 (TIMING_ISP_X_WIN)\n"
		   "[18:57:19:618396] 3813:06 (TIMING_ISP_Y_WIN)\n"
		   "[18:57:19:619319] 3630:2e (ANALOG_CONGROL)\n"
		   "[18:57:19:620178] 3632:e2 (ANALOG_CONGROL)\n"
		   "[18:57:19:621071] 3633:23 (ANALOG_CONGROL)\n"
		   "[18:57:19:621969] 3634:44 (ANALOG_CONGROL)\n"
		   "[18:57:19:622864] 3636:06 (ANALOG_CONGROL)\n"
		   "[18:57:19:623791] 3620:64 (ANALOG_CONGROL)\n"
		   "[18:57:19:624636] 3621:e0 (ANALOG_CONGROL)\n"
		   "[18:57:19:625558] 3600:37 (ANALOG_CONGROL)\n"
		   "[18:57:19:626419] 3704:a0 (ANALOG_CONGROL)\n"
		   "[18:57:19:627352] 3703:5a (ANALOG_CONGROL)\n"
		   "[18:57:19:628200] 3715:78 (ANALOG_CONGROL)\n"
		   "[18:57:19:629089] 3717:01 (ANALOG_CONGROL)\n"
		   "[18:57:19:629984] 3731:02 (ANALOG_CONGROL)\n"
		   "[18:57:19:630875] 370b:60 (ANALOG_CONGROL)\n"
		   "[18:57:19:631767] 3705:1a (ANALOG_CONGROL)\n"
		   "[18:57:19:632681] 3f05:02 ()\n"
		   "[18:57:19:633587] 3f06:10 ()\n"
		   "[18:57:19:634440] 3f01:0a ()\n"
		   "[18:57:19:635368] 3a08:01 ()\n"
		   "[18:57:19:636222] 3a09:28 ()\n"
		   "[18:57:19:637128] 3a0a:00 ()\n"
		   "[18:57:19:638010] 3a0b:f6 ()\n"
		   "[18:57:19:638908] 3a0d:08 ()\n"
		   "[18:57:19:639796] 3a0e:06 ()\n"
		   "[18:57:19:640697] 3a0f:58 ()\n"
		   "[18:57:19:641603] 3a10:50 ()\n"
		   "[18:57:19:642459] 3a1b:58 ()\n"
		   "[18:57:19:643354] 3a1e:50 ()\n"
		   "[18:57:19:644244] 3a11:60 ()\n"
		   "[18:57:19:645138] 3a1f:28 ()\n"
		   "[18:57:19:646030] 4001:02 ()\n"
		   "[18:57:19:646918] 4004:04 ()\n"
		   "[18:57:19:647811] 4000:09 (BLC ENABLE)\n"
		   "[**:**:**:******] 4005:19 (BLC always update)\n"
		   "[18:57:19:648710] 4837:16 ()\n"
		   "#[18:57:19:649594] 4800:24 ()\n"
		   "#[18:57:19:649594] 4800:34 (short frame enable)\n"
		   "[18:57:19:649594] 4800:14 (short frame enable, clock free running)\n"
		   "#[18:57:19:650484] 3503:03 (AEC MANUAL CTRL)\n"
"[18:57:19:650484] 3503:00 (AEC AUTO CTRL)\n"
		   "[18:57:19:652378] 3820:00 (TIMING_TC_REG20)\n"
		   "[18:57:19:652770] 3821:02 (TIMING_TC_REG21)\n"
		   "#[18:57:19:652378] 3820:01 (TIMING_TC_REG20, subsampling vertical)\n"
		   "#[18:57:19:652770] 3821:03 (TIMING_TC_REG21, subsampling horizontal)\n"
		   "[18:57:19:653165] 350a:00 (AGC)\n"
		   "[18:57:19:653572] 350b:10 (AGC)\n"
"[18:57:19:653952] 3212:00 ()\n"
		   "[18:57:19:654430] 380e:08 (TIMING_VTS)\n"
		   "[**:**:**:******] 380f:00 (TIMING_VTS)\n"
		   "[18:57:19:654819] 3500:00 (EXPOSURE)\n"
		   "[18:57:19:655210] 3501:13 (EXPOSURE)\n"
		   "[18:57:19:655597] 3502:30 (EXPOSURE)\n"
		   "[18:57:19:655997] 3212:10 ()\n"
		   "[18:57:19:656380] 3212:a0 ()\n"
		   "[18:57:19:656787] 350a:00 (AGC)\n"
		   "[18:57:19:657198] 350b:10 (AGC)\n"
		   "[18:57:19:662673] 350a:00 (AGC)\n"
		   "[18:57:19:662988] 350b:10 (AGC)\n"
		   "[18:57:19:663376] 3212:00 ()\n"
		   "[18:57:19:663800] 3500:00 (EXPOSURE)\n"
		   "[18:57:19:664155] 3501:13 (EXPOSURE)\n"
		   "[18:57:19:664549] 3502:30 (EXPOSURE)\n"
		   "[18:57:19:664934] 3212:10 ()\n"
		   "[18:57:19:665324] 3212:a0 ()\n"
		   "[18:57:19:666905] 0100:01 ()\n"
		   "[18:57:19:676979] 350a:00 (AGC)\n"
		   "[18:57:19:677330] 350b:10 (AGC)\n"
		   "[18:57:19:677744] 3212:00 ()\n"
		   "[18:57:19:678157] 3500:00 (EXPOSURE)\n"
		   "[18:57:19:678495] 3501:13 (EXPOSURE)\n"
		   "[18:57:19:678883] 3502:30 (EXPOSURE)\n"
		   "[18:57:19:679274] 3212:10 ()\n"
		   "[18:57:19:679665] 3212:a0 ()\n"
		   "[18:57:19:876683] 350a:00 (AGC)\n"
		   "[18:57:19:877027] 350b:13 (AGC)\n"
		   "[18:57:19:877418] 3212:00 ()\n"
		   "[18:57:19:877837] 3500:00 (EXPOSURE)\n"
		   "[18:57:19:878193] 3501:3f (EXPOSURE)\n"
		   "[18:57:19:878581] 3502:60 (EXPOSURE)\n"
		   "[18:57:19:878970] 3212:10 ()\n"
		   "[18:57:19:879363] 3212:a0 ()\n"
		   "[18:57:19:943305] 350a:00 (AGC)\n"
		   "[18:57:19:943594] 350b:13 (AGC)\n"
		   "[18:57:19:943991] 3212:00 ()\n"
		   "[18:57:19:944387] 3500:00 (EXPOSURE)\n"
		   "[18:57:19:944843] 3501:3f (EXPOSURE)\n"
		   "[18:57:19:945164] 3502:60 (EXPOSURE)\n"
		   "[18:57:19:945541] 3212:10 ()\n"
		   "[18:57:19:945932] 3212:a0 ()\n"
		   "[18:57:20:076406] 350a:00 (AGC)\n"
		   "[18:57:20:076735] 350b:1a (AGC)\n"
		   "[18:57:20:077147] 3212:00 ()\n"
		   "[18:57:20:077527] 3500:00 (EXPOSURE)\n"
		   "[18:57:20:077932] 3501:4c (EXPOSURE)\n"
		   "[18:57:20:078295] 3502:d0 (EXPOSURE)\n"
		   "[18:57:20:078678] 3212:10 ()\n"
		   "[18:57:20:079068] 3212:a0 ()\n"
		   "[18:57:20:209528] 350a:00 (AGC)\n"
		   "[18:57:20:209952] 350b:22 (AGC)\n"
		   "[18:57:20:210272] 3212:00 ()\n"
		   "[18:57:20:210689] 3500:00 (EXPOSURE)\n"
		   "[18:57:20:211066] 3501:4c (EXPOSURE)\n"
		   "[18:57:20:211471] 3502:d0 (EXPOSURE)\n"
		   "[18:57:20:211822] 3212:10 ()\n"
		   "[18:57:20:212215] 3212:a0 ()\n"
		   "[18:57:20:342687] 350a:00 (AGC)\n"
		   "[18:57:20:343008] 350b:25 (AGC)\n"
		   "[18:57:20:343433] 3212:00 ()\n"
		   "[18:57:20:343789] 3500:00 (EXPOSURE)\n"
		   "[18:57:20:344220] 3501:4c (EXPOSURE)\n"
		   "[18:57:20:344567] 3502:d0 (EXPOSURE)\n"
		   "[18:57:20:344983] 3212:10 ()\n"
		   "[18:57:20:345337] 3212:a0 ()\n"
		   "[18:57:20:542358] 350a:00 (AGC)\n"
		   "[18:57:20:542713] 350b:26 (AGC)\n"
		   "[18:57:20:543087] 3212:00 ()\n"
		   "[18:57:20:543480] 3500:00 (EXPOSURE)\n"
		   "[18:57:20:543871] 3501:4c (EXPOSURE)\n"
		   "[18:57:20:544259] 3502:d0 (EXPOSURE)\n"
		   "[18:57:20:544649] 3212:10 ()\n"
		   "[18:57:20:545074] 3212:a0 ()\n"
		   "[18:57:20:608949] 350a:00 (AGC)\n"
		   "[18:57:20:609271] 350b:27 (AGC)\n"
		   "[18:57:20:609664] 3212:00 ()\n"
		   "[18:57:20:610053] 3500:00 (EXPOSURE)\n"
		   "[18:57:20:610475] 3501:4c (EXPOSURE)\n"
		   "[18:57:20:610842] 3502:d0 (EXPOSURE)\n"
		   "[18:57:20:611223] 3212:10 ()\n"
		   "[18:57:20:611613] 3212:a0 ()\n"
		   "[18:57:20:742107] 350a:00 (AGC)\n"
		   "[18:57:20:742411] 350b:28 (AGC)\n"
		   "[18:57:20:742794] 3212:00 ()\n"
		   "[18:57:20:743185] 3500:00 (EXPOSURE)\n"
		   "[18:57:20:743573] 3501:4c (EXPOSURE)\n"
		   "[18:57:20:743998] 3502:e0 (EXPOSURE)\n"
		   "[18:57:20:744351] 3212:10 ()\n"
		   "[18:57:20:744743] 3212:a0 ()\n"
		   "[18:57:21:274526] 350a:00 (AGC)\n"
		   "[18:57:21:274848] 350b:27 (AGC)\n"
		   "[18:57:21:275248] 3212:00 ()\n"
		   "[18:57:21:275675] 3500:00 (EXPOSURE)\n"
		   "[18:57:21:276014] 3501:4c (EXPOSURE)\n"
		   "[18:57:21:276412] 3502:d0 (EXPOSURE)\n"
		   "[18:57:21:276793] 3212:10 ()\n"
		   "[18:57:21:277216] 3212:a0 ()\n"
		   );
  ss.str(file);
  return 0;
}

int CamServer::ConfigSequence( seq &a)
{
	std::string str;
	std::stringstream line;
again:
	if(std::getline(m_is,str)==0)
		return 0;
	line.str(str);
	a.dir = Write;

	if(str[0]=='#')
	  goto again;
	line.ignore(20, ']');
	line>>std::hex>>a.addr;
	line.ignore(1,':');
	line>>std::hex>>a.value;
	return 1;
}
void CamServer::ConfigCamera()
{
	int sock;
	sockaddr_in mysa;
	seq myseq;
	std::stringstream ss;
	
	if((sock = socket(PF_INET, SOCK_DGRAM, 0))<0){
		throw("START CAMERA SOCKET FAILED\n");
	}

	mysa.sin_family = AF_INET;
	mysa.sin_addr.s_addr = htonl(INADDR_ANY);
	mysa.sin_port = htons(setup_port);
	bind(sock, reinterpret_cast<sockaddr *>(&mysa), sizeof(mysa));

	while(ConfigSequence(myseq)){
		uint8_t d[1500] ;
		int size;
		size = 12;
		if(myseq.dir == Write){

			d[0] = 0x05; d[1] = 0x00; d[2] = 0x36; d[3] = 0x00;
		}else{
			d[0] = 0x04; d[1] = 0x00; d[2] = 0x36; d[3] = 0x00;
		}
		d[4] = 0x00; d[5] = 0x00; 
		d[6] = (myseq.addr >> 0) & 0xFF; 
		d[7] = (myseq.addr >> 8) & 0xFF;
		d[8] = (myseq.value & 0xFF);
		d[9] = 0x00; d[10] = 0x00; d[11] = 0x00;
		SendPacket(sock, config_port, d, size);
		size = sizeof(d);
		RecvPacket(sock, d, size);
	}
	close(sock);
}

HexaCam::HexaCam(ros::NodeHandle &n, int cam):numcam(cam)
{
	start_flag = start_flag_old = 0;
	old_line = magic_offset;
	frame_flag.resize(y_size);
	struct sockaddr_in skaddr;
	port = port_base;

	pub.resize(numcam);
	msgs[0].resize(numcam);
	msgs[1].resize(numcam);
	for(int i=0; i < numcam; i++){
		std::string topic(std::string("image_raw"));
		topic = "camera"+ std::to_string(i) + "/"+ topic;
		pub[i] = n.advertise<sensor_msgs::Image>(topic, 100);
		ROS_INFO("Publishing.. %s", topic.c_str());
		
		msgs[0][i].header.frame_id = "camera";
		msgs[0][i].height = y_size;
		msgs[0][i].width  = x_size;
		msgs[0][i].encoding = "rgb8";
		msgs[0][i].step = x_size*3;
		msgs[0][i].data.resize(x_size*y_size*3);

		msgs[1][i].header.frame_id = "camera";
		msgs[1][i].height = y_size;
		msgs[1][i].width  = x_size;
		msgs[1][i].encoding = "rgb8";
		msgs[1][i].step = x_size*3;
		msgs[1][i].data.resize(x_size*y_size*3);
		
	}
	curmsg = &msgs[0];
	nextmsg = &msgs[1];
	if ((ld = socket( PF_INET, SOCK_DGRAM, 0 )) < 0) {
		throw("Problem creating socket\n");
	}

	skaddr.sin_family = AF_INET;
	skaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	skaddr.sin_port = htons(port);
	
	if (bind(ld, (struct sockaddr *) &skaddr, sizeof(skaddr))<0) {
		throw("Problem binding");
	}


	count = 0;
	
}
HexaCam::~HexaCam()
{

}

void HexaCam::parse_packet()
{
	int x, y;
	uint8_t packet[4096] __attribute__((may_alias));
	unsigned len, addrlen;
	struct sockaddr_in remote;


	for(int i =0; i < 20000;i++){
	len = recvfrom(ld,packet,sizeof(packet),0,
		       reinterpret_cast<sockaddr *>(&remote), &addrlen);
	uint32_t packet_id   = ntohl(*(uint32_t*)(packet+0));
	uint32_t packet_offset = ntohl(*(uint32_t*)(packet+4));
	uint32_t packet_length = ntohl(*(uint32_t*)(packet+8));
	uint32_t header = ntohl(*(uint32_t*)(packet+12));  // line number

	if( packet_offset == 0) {
		if( header != old_line+1 && 
		    header !=frame_start && 
		    old_line != frame_start && 
		    old_line != magic_offset &&
		    header != frame_end &&
		    old_line != frame_end ) {
		}
		old_line = header;
	}
	//  start_flag = 1;
	if( header == frame_start ) {
		// frame start
		for(int i=0; i < numcam;i++)
			(*curmsg)[i].data.resize(x_size*y_size*3);
		frame_flag.resize(y_size);
		start_flag = 1;
	} else if( header == frame_end) {
		// frame end
	} else if( header > 0 && header <= y_size ) {
		y = y_size - header+1;
		x = packet_offset;
		int camid = x/x_size;
		x %= x_size;			
		for(unsigned int i=0 ; i<packet_length ; i++){
			if(camid>=numcam){
				continue;
			}
			uint8_t *frame = (*curmsg)[camid].data.data();

			// 00rr-rrrr  rrrr-gggg gggg-ggbb bbbb-bbbb

			//      printf("%d ", x);

			// printf("%d, %d\n", packet_offset, packet_length);
			
			frame[x*3+y*x_size*3+0+i*3] = 
				(((((uint32_t)packet[data_offset + i*4+0]))<<2) & 0xfc)
				+ ((((uint32_t)
				     (packet[data_offset + i*4+1]))>>6) & 0x3); // r
			frame[x*3+y*x_size*3+1+i*3] = 
			  ((((uint32_t)
				   (packet[data_offset + i*4+1]))<<4) & 0xf0) 
				+ ((((uint32_t)
				     (packet[data_offset + i*4+2]))>>4) & 0xf); // g
			frame[x*3+y*x_size*3+2+i*3] =
				((((uint32_t)(packet[data_offset + i*4+2]))<<6) & 0xc0)
				+ ((((uint32_t)(packet[data_offset + i*4+3]))>>2) & 0x3f); // b
		}


		frame_flag[y]++;
		
		if( start_flag == 1 && start_flag_old == 0 ) {
			ROS_INFO("line=%d\n", y);
		}
		start_flag_old = start_flag;
		
	} else {
		ROS_INFO("header error. value=%08x\n", header);
	}

	if( header == y_size && (packet_offset+packet_length == x_size*8) && start_flag ){
		
//	  ROS_INFO("last line(line=%d, offset=%d, length=%d)\n", header, packet_offset, packet_length);

	  for( int i=0; i < numcam; i++){
		  unsigned char *frame = (*curmsg)[i].data.data();
		  (*curmsg)[i].header.stamp.sec = ros::Time::now().sec;
		  (*curmsg)[i].header.stamp.nsec = ros::Time::now().nsec;
		  (*curmsg)[i].header.seq = count;
		  (*curmsg)[i].header.frame_id = "camera";
		  (*curmsg)[i].height = y_size;
		  (*curmsg)[i].width  = x_size;
		  (*curmsg)[i].encoding = "rgb8";
		  (*curmsg)[i].step = x_size*3;

		  pub[i].publish((*curmsg)[i]);
		  
		  start_flag = 0;
	  }
	  count++;
	  std::swap(curmsg,nextmsg);
	  return;

	}
	}
}

int main(int argc, char **argv)
{
	////ROS STUFF////
	ros::init(argc, argv, "hexacam");
	ros::NodeHandle n;
	ros::NodeHandle private_nh("~");
	HexaCam *cameras;
	CamServer *cs;
	std::stringstream ss;
	std::streambuf *sb;


	DefaultStream(ss);
	sb = ss.rdbuf();
	
	//signal(SIGTERM, signalHandler);//detect closing
	std::string configfile;
	std::filebuf fb;

	double fps;
	if (private_nh.getParam("fps", fps))
	{
		ROS_INFO("fps set to %.2f", fps);
	} else {
		fps = 7.5;
		ROS_INFO("No param received, defaulting to %.2f", fps);
	}
	if (private_nh.getParam("configfile", configfile)&& configfile!="")
	{
		if(fb.open(configfile, std::ios::in)!=NULL){
			ROS_INFO("OPEN SUCCESS");
			sb = &fb;
		}
		ROS_INFO("file %s", configfile.c_str());
	}else{
		ROS_INFO("configfile param not found ");
	}
	std::istream is(sb);
	cs = new CamServer("10.0.0.1", is);
	
	cameras = new HexaCam(n, 8);

	ros::Publisher camera_info_pub;
	camera_info_pub = n.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1, true);

	ROS_INFO("DISABLECAMERA");
	cs->EnableCamera(0);
	ROS_INFO("CONFIGCAMERA");
	cs->ConfigCamera();

	ROS_INFO("ENABLECAMERA");
	cs->EnableCamera(1);
	
	int count = 0;
	ros::Rate loop_rate(fps); // Hz
	ROS_INFO("Starting loop");
	int i=0;
	while (ros::ok())
	{
		i++;
		cameras->parse_packet();
#if 0
		ros::spinOnce();
		loop_rate.sleep();
		if(i==1000){
		  ROS_INFO("HOGE");
		  i=0;
		}
#endif
	}
	ROS_INFO("EXIT");
	//close cameras
	delete cameras;

	cs->EnableCamera(0);
	delete cs;

	ROS_INFO("Camera node closed correctly");
	return 0;
}

