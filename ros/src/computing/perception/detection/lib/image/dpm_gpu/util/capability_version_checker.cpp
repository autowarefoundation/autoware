#include <iostream>
#include <vector>
#include <algorithm>

#include <cuda.h>
#include <cuda_runtime.h>

int main(void)
{
	int deviceCount;
	cudaError_t error_id = cudaGetDeviceCount(&deviceCount);

	if (error_id != cudaSuccess) {
		std::cerr << "Failed: cudaGetDeviceCount(%d)"
			  << static_cast<int>(error_id)
			  << " -> "
			  << cudaGetErrorString(error_id)
			  << std::endl;
		return -1;
	}

	if (deviceCount == 0) {
		std::cerr << "No CUDA GPU" << std::endl;
		return -1;
	}

	std::vector<int> capability_versions;
	for (int device = 0; device < deviceCount; ++device) {
		cudaDeviceProp deviceProp;

		cudaSetDevice(device);
		cudaGetDeviceProperties(&deviceProp, device);

		int capability_version = (10 * deviceProp.major) + deviceProp.minor;
		capability_versions.push_back(capability_version);
	}

	int min_version = *std::min_element(capability_versions.begin(),
					    capability_versions.end());

	std::cout << min_version << std::endl;
	return 0;
}
