# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

import pytest
from bliss.common.axis import AxisState

def test_empty_state():
    s = AxisState()
    assert s == 'UNKNOWN'

def test_mutually_exclusive():
    s = AxisState()

    s.set("MOVING")
    assert s == "MOVING"
    assert not s.READY

    s.set("READY")
    assert s.READY
    assert not s.MOVING

def test_custom_state():
    s = AxisState()
    s.set("READY")
    s.create_state("PARKED", "here I am")
    s.set("PARKED")
    assert s.READY
    assert s == "PARKED"

def test_state_print():
    s = AxisState()

    s.set("READY")
    assert isinstance(s.current_states(), str)

def test_bad_name():
    s = AxisState()
    with pytest.raises(ValueError):
        s.create_state("A bad state")

def test_desc():
    s = AxisState(("KAPUT", "auf"), "LIMNEG", "READY")
    assert s.READY
    assert s._state_desc["KAPUT"] == "auf"
    assert s._state_desc["LIMNEG"] == "Hardware low limit active"

def test_from_current_states_str():
    s = AxisState(("KAPUT", "auf"), "LIMNEG", "READY")
    states_str = s.current_states()
    t = AxisState(states_str)
    assert t.READY
    assert t._state_desc["KAPUT"] == "auf"
    assert t._state_desc["LIMNEG"] == "Hardware low limit active"
    assert s.current_states() == t.current_states()
    u = AxisState()
    v = AxisState(u.current_states())
    assert u.current_states() == v.current_states()

def test_state_from_state():
    s = AxisState("READY")
    t = AxisState(s)
    assert s.current_states() == t.current_states()
      
def test_clear_state():
    s = AxisState("READY")
    s.clear()
    assert s == "UNKNOWN"

    s.set("MOVING")
    assert s == "MOVING"

