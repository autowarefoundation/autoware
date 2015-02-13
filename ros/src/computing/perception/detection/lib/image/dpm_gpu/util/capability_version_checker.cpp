#include <iostream>
#include <vector>
#include <algorithm>

#include <cuda.h>

int main(void)
{
	int deviceCount;
	CUresult error_id = cuInit(0);

	error_id = cuDeviceGetCount(&deviceCount);

	if (error_id != CUDA_SUCCESS) {
		const char* error_string;
		cuGetErrorString(error_id, &error_string);
		std::cerr << "Failed: cuDeviceGetCount("
			  << static_cast<int>(error_id)
			  << ")"
			  << " -> "
			  << error_string
			  << std::endl;
		return -1;
	}

	if (deviceCount == 0) {
		std::cerr << "No CUDA GPU" << std::endl;
		return -1;
	}

	std::vector<int> capability_versions;
	for (int device = 0; device < deviceCount; ++device) {
		CUdevice devHandle;

		cuDeviceGet(&devHandle, device);

		int major = 0, minor = 0;
		cuDeviceComputeCapability(&major, &minor, devHandle);

		int capability_version = (10 * major) + minor;
		capability_versions.push_back(capability_version);
	}

	int min_version = *std::min_element(capability_versions.begin(),
					    capability_versions.end());

	std::cout << min_version << std::endl;
	return 0;
}
