#define ERROR_INFO_TYPE		3
#define CAN_INFO_TYPE		4

struct error_request {
	int32_t type;
	int32_t error;

	error_request(const ui_socket::error_info& msg)
	: type(ERROR_INFO_TYPE), error(msg.error) {
	}
};

struct can_request {
	int32_t type;
	int32_t application;
	int32_t timestamp;
	int32_t startup;
	double can_str;
	double can_vel;
	double can_gas;
	double can_brake;
	double can_seatbelt;
	double can_water_temperature;
	double acc_x;
	double acc_y;
	double acc_z;
	double lat;
	double lon;
	double roll;
	double pitch;
	double yaw;
	double cal_orientation_x;
	double cal_orientation_y;
	double cal_orientation_z;
	double gravity_x;
	double gravity_y;
	double gravity_z;
	double gyroscope_x;
	double gyroscope_y;
	double gyroscope_z;
	double light;
	double linear_acc_x;
	double linear_acc_y;
	double linear_acc_z;
	double magnetic_field_x;
	double magnetic_field_y;
	double magnetic_field_z;
	double orientation_x;
	double orientation_y;
	double orientation_z;
	double pressure;
	double proximity;
	double rotation_vector_x;
	double rotation_vector_y;
	double rotation_vector_z;
	double rotation_vector_c;
	double temperature;
	double ambient_temperature;
	double relative_humidity;
	double bearing;
	double altitude;
	double accuracy;
	double speed;
	double env_battery;
	double env_temperature;
	char terminal[32];
	char img[256];

	can_request(const ui_socket::can_info& msg) {
		type = CAN_INFO_TYPE;
		application = msg.application;
		timestamp = msg.timestamp;
		startup = msg.startup;
		can_str = msg.can_str;
		can_vel = msg.can_vel;
		can_gas = msg.can_gas;
		can_brake = msg.can_brake;
		can_seatbelt = msg.can_seatbelt;
		can_water_temperature = msg.can_water_temperature;
		acc_x = msg.acc_x;
		acc_y = msg.acc_y;
		acc_z = msg.acc_z;
		lat = msg.lat;
		lon = msg.lon;
		roll = msg.roll;
		pitch = msg.pitch;
		yaw = msg.yaw;
		cal_orientation_x = msg.cal_orientation_x;
		cal_orientation_y = msg.cal_orientation_y;
		cal_orientation_z = msg.cal_orientation_z;
		gravity_x = msg.gravity_x;
		gravity_y = msg.gravity_y;
		gravity_z = msg.gravity_z;
		gyroscope_x = msg.gyroscope_x;
		gyroscope_y = msg.gyroscope_y;
		gyroscope_z = msg.gyroscope_z;
		light = msg.light;
		linear_acc_x = msg.linear_acc_x;
		linear_acc_y = msg.linear_acc_y;
		linear_acc_z = msg.linear_acc_z;
		magnetic_field_x = msg.magnetic_field_x;
		magnetic_field_y = msg.magnetic_field_y;
		magnetic_field_z = msg.magnetic_field_z;
		orientation_x = msg.orientation_x;
		orientation_y = msg.orientation_y;
		orientation_z = msg.orientation_z;
		pressure = msg.pressure;
		proximity = msg.proximity;
		rotation_vector_x = msg.rotation_vector_x;
		rotation_vector_y = msg.rotation_vector_y;
		rotation_vector_z = msg.rotation_vector_z;
		rotation_vector_c = msg.rotation_vector_c;
		temperature = msg.temperature;
		ambient_temperature = msg.ambient_temperature;
		relative_humidity = msg.relative_humidity;
		bearing = msg.bearing;
		altitude = msg.altitude;
		accuracy = msg.accuracy;
		speed = msg.speed;
		env_battery = msg.env_battery;
		env_temperature = msg.env_temperature;
		msg.terminal.copy(terminal, 32, 0);
		msg.img.copy(img, 256, 0);
	}
};
