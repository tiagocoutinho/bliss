# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

"""Repository of motor controllers

.. autosummary::
    :toctree:

    calc_motor_mockup
    energy_wl
    FlexDC
    GalilDMC213
    IcePAP
    IcePAPTraj
    id16beam
    ID31DiffractLinTilt
    ID31Diffract
    kb
    libicepap
    MD2
    mockup
    NF8753
    PI_C663
    PI_E517
    PI_E518
    PI_E51X
    PI_E712
    PI_E753
    PI_E871
    pi_gcs
    PM600
    PMD206
    setpoint
    simpliest2
    simpliest
    Slitbox
    slits
    spectro_eh1_test_V1
    spectro_eh1_test_V2
    tab3
    tabsup
    TacoMaxe
    TangoEMot
    template
    trans4
    Undulator
    VSCANNER
"""
__all__ = []
def _init_module() :
    import os
    for root,dirs,files in os.walk(__path__[0],followlinks=True) :
        for file_name in files :
            if file_name.startswith('__') : continue
            base,ext = os.path.splitext(file_name)
            if ext == '.py' :
                subdir = root[len(__path__[0]) + 1:]
                if subdir:
                    base = '%s.%s' % (subdir,base)
                __all__.append(base)
_init_module()
