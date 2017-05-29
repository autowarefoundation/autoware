#!/usr/bin/env python
# -*- coding: utf-8 -*-

from enum import Enum

import rospy


__all__ = ('ConnectionBasedTransport',)


ConnectionStatus = Enum('ConnectionStatus', 'SUBSCRIBED NOT_SUBSCRIBED')
SUBSCRIBED = ConnectionStatus.SUBSCRIBED
NOT_SUBSCRIBED = ConnectionStatus.NOT_SUBSCRIBED


# define a new metaclass which overrides the '__call__' function
# See: http://martyalchin.com/2008/jan/10/simple-plugin-framework/
class MetaConnectionBasedTransport(type):
    def __call__(cls, *args, **kwargs):
        """Called when you call ConnectionBasedTransport()"""
        obj = type.__call__(cls, *args, **kwargs)
        obj._post_init()


class ConnectionBasedTransport(rospy.SubscribeListener):

    __metaclass__ = MetaConnectionBasedTransport

    def __init__(self):
        super(ConnectionBasedTransport, self).__init__()
        self._publishers = []
        self._ever_subscribed = False
        self._connection_status = NOT_SUBSCRIBED
        rospy.Timer(rospy.Duration(5),
                    self._warn_never_subscribed_cb, oneshot=True)

    def _post_init(self):
        if rospy.get_param('~always_subscribe', False):
            self.subscribe()
            self._connection_status = SUBSCRIBED
            self._ever_subscribed = True

    def _warn_never_subscribed_cb(self, timer_event):
        if not self._ever_subscribed:
            rospy.logwarn('[{name}] subscribes topics only with'
                " child subscribers. Set '~always_subscribe' as True"
                ' to have it subscribe always.'.format(name=rospy.get_name()))

    def subscribe(self):
        raise NotImplementedError('Please overwrite this method')

    def unsubscribe(self):
        raise NotImplementedError('Please overwrite this method')

    def peer_subscribe(self, *args, **kwargs):
        rospy.logdebug('[{topic}] is subscribed'.format(topic=args[0]))
        if self._connection_status == NOT_SUBSCRIBED:
            self.subscribe()
            self._connection_status = SUBSCRIBED
            if not self._ever_subscribed:
                self._ever_subscribed = True

    def peer_unsubscribe(self, *args, **kwargs):
        rospy.logdebug('[{topic}] is unsubscribed'.format(topic=args[0]))
        if rospy.get_param('~always_subscribe', False):
            return  # do not unsubscribe
        if self._connection_status == NOT_SUBSCRIBED:
            return  # no need to unsubscribe
        for pub in self._publishers:
            if pub.get_num_connections() > 0:
                break
        else:
            self.unsubscribe()
            self._connection_status = NOT_SUBSCRIBED

    def advertise(self, *args, **kwargs):
        # subscriber_listener should be 'self'
        # to detect connection and disconnection of the publishing topics
        assert len(args) < 3 or args[2] is None
        assert kwargs.get('subscriber_listener') is None
        kwargs['subscriber_listener'] = self

        pub = rospy.Publisher(*args, **kwargs)
        self._publishers.append(pub)
        return pub
