import numpy as np
cimport numpy as np


def bitget(int byteval, int idx):
    return ((byteval & (1 << idx)) != 0)


def labelcolormap(int N=256):
    """Color map (RGB) considering label numbers N.

    Args:
        N (``int``): Number of labels.
    """
    cdef np.ndarray[np.float32_t, ndim=2] cmap = np.zeros((N, 3),
                                                          dtype=np.float32)
    cdef int i, j
    cdef int id
    cdef int r, g, b
    for i in range(0, N):
        id = i
        r, g, b = 0, 0, 0
        for j in range(0, 8):
            r = np.bitwise_or(r, (bitget(id, 0) << 7-j))
            g = np.bitwise_or(g, (bitget(id, 1) << 7-j))
            b = np.bitwise_or(b, (bitget(id, 2) << 7-j))
            id = (id >> 3)
        cmap[i, 0] = r / 255.
        cmap[i, 1] = g / 255.
        cmap[i, 2] = b / 255.
    return cmap
