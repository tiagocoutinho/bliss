# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

from bliss.common.event import dispatcher
from bliss.data.node import _get_or_create_node
import numpy


class AcquisitionChannelList(list):
    def update(self, values_dict):
        """Update all channels and emit the new_data event

        Input:

           values_dict - { channel_name: value, ... }
        """
        for channel in self:
            if channel.name in values_dict:
                channel.emit(values_dict[channel.name])

    def update_from_iterable(self, iterable):
        for i, channel in enumerate(self):
            channel.emit(iterable[i])

    def update_from_array(self, array):
        for i, channel in enumerate(self):
            channel.emit(array[:, i])


class AcquisitionChannel(object):

    def __init__(self, name, dtype, shape,
                 description=None, reference=False, data_node_type="channel"):
        self.__name = name
        self.__dtype = dtype
        self.__shape = shape
        self.__reference = reference
        self.__description = {'reference': reference}
        self.__data_node_type = data_node_type

        if isinstance(description, dict):
            self.__description.update(description)

    @property
    def name(self):
        return self.__name

    @property
    def description(self):
        return self.__description

    @property
    def reference(self):
        return self.__reference

    @property
    def dtype(self):
        return self.__dtype

    @dtype.setter
    def dtype(self, value):
        self.__dtype = value

    @property
    def shape(self):
        return self.__shape

    @shape.setter
    def shape(self, value):
        self.__shape = value

    def emit(self, data):
        if not self.reference:
            data = self._check_and_reshape(data)
            if data.size == 0:
                return
        self.__description['dtype'] = self.dtype
        self.__description['shape'] = self.shape
        data_dct = {"name": self.name,
                    "description": self.__description,
                    "data": data,
                    "channel": self}
        dispatcher.send("new_data", self, data_dct)

    def data_node(self, parent_node):
        return _get_or_create_node(
            self.name, self.__data_node_type, parent_node,
            shape=self.shape, dtype=self.dtype)

    def _check_and_reshape(self, data):
        ndim = len(self.shape)
        data = numpy.array(data)

        # Empty data
        if data.size == 0:
            return numpy.empty((0,) + self.shape)

        # Invalid dimensions
        if data.ndim not in (ndim, ndim+1):
            raise ValueError(
                "Data should either be of {} or {} dimensions"
                .format(ndim, ndim+1))

        # Single point case
        if data.ndim == ndim and data.shape != self.shape:
            raise ValueError(
                "Single point of shape {} does not match expected shape {}"
                .format(data.shape, self.shape))

        # Block case
        if data.ndim == ndim+1 and data.shape[1:] != self.shape:
            raise ValueError(
                "Multiple points of shape {} does not match expected shape {}"
                .format(data.shape[1:], self.shape))

        # Permissive reshaping
        data.shape = (-1, ) + self.shape
        return data
