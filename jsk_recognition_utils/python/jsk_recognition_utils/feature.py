#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from sklearn.cluster import MiniBatchKMeans
from sklearn.neighbors import NearestNeighbors


class BagOfFeatures(object):
    def __init__(self, hist_size=500):
        self.nn = None
        self.hist_size = hist_size

    def fit(self, X):
        """Fit features and extract bag of features"""
        k = self.hist_size
        km = MiniBatchKMeans(n_clusters=k, init_size=3*k, max_iter=300)
        km.fit(X)
        nn = NearestNeighbors(n_neighbors=1)
        nn.fit(km.cluster_centers_)
        self.nn = nn

    def transform(self, X):
        return np.vstack([self.make_hist(xi.reshape((-1, 128))) for xi in X])

    def make_hist(self, descriptors):
        """Make histogram for one image"""
        nn = self.nn
        if nn is None:
            raise ValueError('must fit features before making histogram')
        indices = nn.kneighbors(descriptors, return_distance=False)
        histogram = np.zeros(self.hist_size)
        for idx in np.unique(indices):
            mask = indices == idx
            histogram[idx] = mask.sum()  # count the idx
            indices = indices[mask == False]
        return histogram

