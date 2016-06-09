# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

from bliss.controllers.motor import CalcController


class simpliest(CalcController):

    def __init__(self, *args, **kwargs):
        CalcController.__init__(self, *args, **kwargs)

    def calc_from_real(self, positions_dict):
        return {"m1": positions_dict["m0"] * 2}

    def calc_to_real(self, positions_dict):
        return {"m0": positions_dict["m1"] / 2}
