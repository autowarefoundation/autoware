import chainer
from chainer import cuda
import chainer.functions as F
import chainer.links as L
from chainer import Variable
from distutils.version import LooseVersion


class VGG_CNN_M_1024(chainer.Chain):

    def __init__(self, n_class=21, bg_label=-1):
        super(VGG_CNN_M_1024, self).__init__(
            conv1=L.Convolution2D(3, 96, ksize=7, stride=2),
            conv2=L.Convolution2D(96, 256, ksize=5, stride=2, pad=1),
            conv3=L.Convolution2D(256, 512, ksize=3, stride=1, pad=1),
            conv4=L.Convolution2D(512, 512, ksize=3, stride=1, pad=1),
            conv5=L.Convolution2D(512, 512, ksize=3, stride=1, pad=1),
            fc6=L.Linear(18432, 4096),
            fc7=L.Linear(4096, 1024),
            cls_score=L.Linear(1024, n_class),
            bbox_pred=L.Linear(1024, 4 * n_class)
        )
        self.n_class = n_class
        self.bg_label = bg_label

    def __call__(self, x, rois, t=None):
        h = self.conv1(x)
        h = F.relu(h)
        h = F.local_response_normalization(h, n=5, k=2, alpha=5e-4, beta=.75)
        h = F.max_pooling_2d(h, ksize=3, stride=2)

        h = self.conv2(h)
        h = F.relu(h)
        h = F.local_response_normalization(h, n=5, k=2, alpha=5e-4, beta=.75)
        h = F.max_pooling_2d(h, ksize=3, stride=2)

        h = self.conv3(h)
        h = F.relu(h)

        h = self.conv4(h)
        h = F.relu(h)

        h = self.conv5(h)
        h = F.relu(h)

        h = F.roi_pooling_2d(h, rois, 6, 6, spatial_scale=0.0625)

        h = self.fc6(h)
        h = F.relu(h)
        h = F.dropout(h, ratio=.5)

        h = self.fc7(h)
        h = F.relu(h)
        h = F.dropout(h, ratio=.5)

        h_cls_score = self.cls_score(h)
        cls_score = F.softmax(h_cls_score)
        bbox_pred = self.bbox_pred(h)

        if t is None:
            assert not chainer.config.train
            return cls_score, bbox_pred

        t_cls, t_bbox = t
        self.cls_loss = F.softmax_cross_entropy(h_cls_score, t_cls)
        self.bbox_loss = F.smooth_l1_loss(bbox_pred, t_bbox)

        xp = cuda.get_array_module(x.data)
        lambda_ = (0.5 * (t_cls.data != self.bg_label)).astype(xp.float32)
        L = self.cls_loss + F.sum(lambda_ * self.bbox_loss)
        return L
