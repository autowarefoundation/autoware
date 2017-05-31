#!/usr/bin/env python
import h5py

def create_h5file(filename, keys, data):


if __name__ == "__main__":
    filename = "output.h5"
    input_file = "hogehoge.bag"

    h5file = h5py.File(output_file,'w')
    for i in np.arange(3):
        dir = 'frequency_'+str(np.int(sampling_frequency[i]))
        h5file.create_group(dir)
        h5file.create_dataset(dir+'/random_number',data= sample[i])
        h5file.create_dataset(dir+'/spectrum',data= sample_fft[i])
