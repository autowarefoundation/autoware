UTIL_DIR	= ../utils
UTIL_OBJ	= $(UTIL_DIR)/timer.o
UTIL_INC 	= -I$(UTIL_DIR)
SHM_DIR		= ../common
SHM_INC 	= -I$(SHM_DIR)
SHM_OBJ 	= shm_client.o 
CC		= gcc
CXX		= g++
CFLAGS		= -g		
CXXFLAGS	= -g

all:	 state_comm_shm.o shm_client.o base_client base_server vel_cmd_client vel_cmd_server shm hev_reset_odom cmd_test

state_comm_shm.o: state_comm_shm.cc state_comm_shm.h
	$(CXX) -c $(CXXFLAGS) state_comm_shm.cc $(SHM_INC)

base_server.o: base_server.cc  $(SHM_OBJ)
	$(CXX) -c $(CXXFLAGS) base_server.cc $(SHM_INC)

base_server: base_server.o state_comm_shm.o $(SHM_OBJ)
	$(CXX) $(CXXFLAGS) -o base_server base_server.o state_comm_shm.o $(SHM_OBJ) -pthread $(UTIL_OBJ)	


base_client.o: base_client.cc  $(SHM_OBJ)
	$(CXX) -c $(CXXFLAGS) base_client.cc $(SHM_INC)

base_client: base_client.o state_comm_shm.o $(SHM_OBJ)
	$(CXX) $(CXXFLAGS) -o base_client base_client.o state_comm_shm.o $(SHM_OBJ) -pthread $(UTIL_OBJ)


vel_cmd_server.o: vel_cmd_server.cc  $(SHM_OBJ)
	$(CXX) -c $(CXXFLAGS) vel_cmd_server.cc $(SHM_INC)

vel_cmd_server: vel_cmd_server.o state_comm_shm.o $(SHM_OBJ)
	$(CXX) $(CXXFLAGS) -o vel_cmd_server vel_cmd_server.o state_comm_shm.o $(SHM_OBJ) -pthread $(UTIL_OBJ)	
vel_cmd_client.o: vel_cmd_client.cc  $(SHM_OBJ)
	$(CXX) -c $(CXXFLAGS) vel_cmd_client.cc $(SHM_INC)

vel_cmd_client: vel_cmd_client.o state_comm_shm.o $(SHM_OBJ)
	$(CXX) $(CXXFLAGS) -o vel_cmd_client vel_cmd_client.o state_comm_shm.o $(SHM_OBJ) -pthread $(UTIL_OBJ)	

shms: shm.cc
	$(CXX) $(CXXFLAGS) -o shm shm.cc

shm_client.o:	shm_client.cc shm_client.h
	$(CXX) -c $(CXXFLAGS) shm_client.cc $(UTIL_INC)

hev_reset_odom: hev_reset_odom.cc $(SHM_OBJ)  $(UTIL_OBJ)
	$(CXX) $(CXXFLAGS) -o hev_reset_odom hev_reset_odom.cc -pthread $(SHM_OBJ) $(UTIL_OBJ) 

cmd_test: cmd_test.cc $(SHM_OBJ)  $(UTIL_OBJ)
	$(CXX) $(CXXFLAGS) -o cmd_test cmd_test.cc -pthread $(SHM_OBJ) $(UTIL_OBJ) 
clean:		
	rm -f *.o shm base_client base_server vel_cmd_client vel_cmd_server hev_reset_odom cmd_test
