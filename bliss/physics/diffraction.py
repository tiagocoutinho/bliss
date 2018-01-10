# -*- coding: utf-8 -*-
#
# This file is part of the bliss project
#
# Copyright (c) 2016 Beamline Control Unit, ESRF
# Distributed under the GNU LGPLv3. See LICENSE for more info.

"""X-Ray crystal diffraction physics (Bragg, Laue)

Disclaimer
----------

The following module uses the SI system of units.
All arguments which represent physical quantities must be given 
in the corresponding SI value. Failure to comply will result
in unexpected values for the user.

Theory
------

Bragg's law: nλ = 2dsin(θ)
  n: integer plane number [1..]
  λ: wavelength incident angle
  θ: scattering angle
  d: interplanar distance between lattice planes

Cubic crystal diffraction: d = a / √(h²+k²+l²)
  d: interplanar distance between lattice planes
  a: lattice spacing of the cubic crystal

  E = nhc / 2dsin(θ)

De Broglie: λ = h/p
  λ: wavelength
  h: Planck constant
  p: momomentum

Examples
--------

How to find the bragg angle (in rad) for a silicon crystal and you want to
know the bragg angle for the 110 plane when the energy is 12.5 keV::

    >>> from bliss.physics.diffraction import Si
    >>> Si110 = Si('110')
    >>> energy_keV = 12.5    # energy in keV 
    >>> energy_J = energy_keV  * 1.60218e-16
    >>> angle_rad = Si110.bragg_angle(energy_keV / J)
"""

from collections import namedtuple

from numpy import sqrt, sin, arcsin
from scipy.constants import h, c

hc = h * c


#: A crystal Plane in hkl coordinates
HKL = namedtuple('HKL', 'h k l')


def string_to_hkl(splane):
    """
    Convert a string representing hkl plane to a HKL object.

    Args:
        plane (str): string with three integers separated by single space
                     (ex: '1 1 0', '10 10 0'). If all planes are one digit
                     long, it also accepts a compact format without spaces
                     (ex: '111', '110').
    Returns:
        HKL: the crystal plane for the given hkl coordinates
    Raises:
        ValueError: if argument is not in the correct format
    """
    try:
        if len(splane) == 3:
            return HKL(*map(int, splane))
        elif ' ' in splane:
            return HKL(*map(int, splane.split()))
    except Exception as err:
        raise ValueError('Invalid crystal plane {0!r}: {1}'.format(splane, err))
    raise ValueError('Invalid crystal plane {0!r}'.format(splane))


def hkl_to_string(hkl):
    join = '' if all(map(lambda i: i<10, hkl)) else ' '
    return join.join(map(str, hkl))


HKL.fromstring = staticmethod(string_to_hkl)
HKL.tostring = hkl_to_string


def distance_cubic_lattice_diffraction_plane(h, k, l, a):
    """
    Calculates the interplanar distance between lattice planes for a specific
    diffraction plane (given by h, k, l) and a specific lattice with lattice
    parameter *a*.

    d = a / √(h²+k²+l²)

    Args:
        h (float): a diffraction plane *h*
        k (float): a diffraction plane *k*
        l (float): a diffraction plane *l*
        a (float): crystal lattic parameter *a*
    Returns:
        float: the distance (d) between lattice planes
    """
    return a / sqrt(h**2 + k**2 + l**2)


def bragg_wavelength(theta, d, n=1):
    """
    Return a bragg wavelength (m) for the given theta and distance between
    lattice planes.

    Args:
        theta (float): scattering angle (rad)
        d (float): interplanar distance between lattice planes (m)
        n (int): non zero positive integer (default: 1)
    Returns:
        float: bragg wavelength (m) for the given theta and lattice distance
    """
    return 2 * d * sin(theta) / n


def bragg_energy(theta, d, n=1):
    """
    Return a bragg energy for the given theta and distance between lattice
    planes.

    Args:
        theta (float): scattering angle (rad)
        d (float): interplanar distance between lattice planes (m)
        n (int): non zero positive integer (default: 1)
    Returns:
        float: bragg energy (J) for the given theta and lattice distance
    """
    return hc / bragg_wavelength(theta, d, n=n)


def bragg_angle(energy, d, n=1):
    """
    Return a bragg angle (rad) for the given theta and distance between
    lattice planes.

    Args:
        energy (float): energy (J)
        d (float): interplanar distance between lattice planes (m)
        n (int): non zero positive integer (default: 1)
    Returns:
        float: bragg angle (rad) for the given theta and lattice distance
    """
    return arcsin( n * hc / (2 * d * energy))



class CrystalPlane(object):
    """
    Cubic crystal plane.

    Example::

        >>> Si = Crystal('Si', 5.4307e-10)
        >>> Si111 = CrystalPlane(Si, HKL(1, 1, 0))
        >>> e_at_3deg = Si111.bragg_energy(numpy.deg2rad(3))
    """

    def __init__(self, crystal, plane):
        self.crystal = crystal
        self.plane = plane

    @property
    def d(self):
        # may optimize in future: self.d = ... in the constructor
        h, k, l = self.plane
        a = self.crystal.lattice_constant
        return distance_cubic_lattice_diffraction_plane(h, k, l, a)

    def bragg_wavelength(self, theta, n=1):
        """
        Returns a bragg wavelength (m) for the given theta on this crystal plane
        
        Args:
            theta (float): scattering angle (rad)
            n (int): non zero positive integer (default: 1)
        Returns:
            float: bragg wavelength (m) for the given theta and lattice distance
        """
        return bragg_wavelength(theta, self.d, n=n)
    
    def bragg_energy(self, theta, n=1):
        """
        Returns a bragg energy (J) for the given theta on this crystal plane
        
        Args:
            theta (float): scattering angle (rad)
            n (int): non zero positive integer (default: 1)
        Returns:
            float: bragg energy (J) for the given theta and lattice distance
        """
        return bragg_energy(theta, self.d, n=n)

    def bragg_angle(self, energy, n=1):
        """
        Returns a bragg angle (rad) for the given energy on this crystal plane
        
        Args:
            energy (float): energy (J)
            n (int): non zero positive integer (default: 1)
        Returns:
            float: bragg angle (rad) for the given theta and lattice distance
        """
        return bragg_angle(energy, self.d, n=n)

    def __repr__(self):
        return '{0}({1})'.format(self.crystal, self.plane.tostring())


class Crystal(object):
    """
    Cubic crystal.

    Example::

        >>> Si = Crystal()
        >>> Si111 = Si('111')
    """

    def __init__(self, element):
        if isinstance(element, (list, tuple)):
            name, a = element
        else:
            name, a = element.symbol, element.lattice_constant * 1e-10
        self.name = name
        self.lattice_constant = a
        #: diffraction planes cache dict<hkl(str): planes(CrystalPlane)>
        self._planes = {}

    def __call__(self, plane):
        """Helper to get a crystal plane from a string (ex: '110')"""
        if isinstance(plane, CrystalPlane):
            self._planes[plane.tostring()] = plane
            return plane
        try:
            return self._planes[plane]
        except KeyError:
            pass
        result = CrystalPlane(self, HKL.fromstring(plane))
        self._planes[plane] = result
        return result

    def bragg_wavelength(self, theta, plane, n=1):
        """
        Returns a bragg wavelength (m) for the given theta on the given plane
        
        Args:
            theta (float): scattering angle (rad)
            plane (str or CrystalPlane): crystal plane
            n (int): non zero positive integer (default: 1)
        Returns:
            float: bragg wavelength (m) for the given theta and lattice distance
        """
        return self(plane).bragg_wavelength(theta, n=n)

    def bragg_energy(self, theta, plane, n=1):
        """
        Returns a bragg energy (J) for the given theta on the given plane
        
        Args:
            theta (float): scattering angle (rad)
            plane (str or CrystalPlane): crystal plane
            n (int): non zero positive integer (default: 1)
        Returns:
            float: bragg energy (J) for the given theta and lattice distance
        """
        return self(plane).bragg_energy(theta, n=n)
        
    def bragg_angle(self, energy, plane, n=1):
        """
        Returns a bragg angle (read) for the given energy on the given plane
        
        Args:
            energy (float): energy (J)
            plane (str or CrystalPlane): crystal plane
            n (int): non zero positive integer (default: 1)
        Returns:
            float: bragg energy (J) for the given theta and lattice distance
        """
        return self(plane).bragg_angle(energy, n=n)

    def __repr__(self):
        return self.name


import mendeleev.elements

def get_all_cubic_crystals():
    result = []
    for elem_symbol in mendeleev.elements.__all__:
        elem = getattr(mendeleev.elements, elem_symbol)
        if elem.lattice_constant is not None:
            result.append(Crystal(elem))
    return result

globals().update({c.name: c for c in get_all_cubic_crystals()})

