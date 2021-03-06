# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

'''Exception classes for communication'''


class CommunicationError(RuntimeError):
    '''Base communication error'''


class CommunicationTimeout(CommunicationError):
    '''Communication timeout error'''

