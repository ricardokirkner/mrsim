# -*- coding: utf-8 -*-
#------------------------------------------------------------------------------
# MrSim - MTRAN Robot Simulator
# Written by Ricardo Martín Kirkner <ricardo@kirkner.com.ar>
# Copyright (C) 2007 Ricardo Martín Kirkner, Departamento de Computación, FCEyN, UBA.
#
# This file is part of MrSim.
#
# MrSim is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# 
# MrSim is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#------------------------------------------------------------------------------

from cgkit.cgtypes import mat4
from cgkit.scene import getScene
from constants import IDLE, BUSY, TIMEOUT
import logging

logger = logging.getLogger('mrsim.util')

def getPivotedRotation(pivot, rot):
    m = mat4(1.0)
    # translate to origin
    m.translate(pivot)
    # rotate at origin
    rot4 = mat4(rot[0][0], rot[0][1], rot[0][2], 0,
                rot[1][0], rot[1][1], rot[1][2], 0,
                rot[2][0], rot[2][1], rot[2][2], 0,
                        0,         0,         0, 1)
    m *= rot4
    # translate back
    m.translate(-pivot)
    return m

def isIdle(item):
    return item == IDLE

def allIdle(items):
    if len(items) > 1:
        return isIdle(items[0]) and allIdle(items[1:])
    elif len(items) == 1:
        return isIdle(items[0])
    else:
        return True

def getStatus(items):
    status = IDLE
    for item in items:
        if item == BUSY:
            return BUSY
        if item == TIMEOUT:
            status = TIMEOUT
    return status

def removeObject(obj, world):
    scene = getScene()
    try:
        scene.items.remove(obj)
    except:
        pass
    if world.body_dict.has_key(obj):
        body = world.body_dict.pop(obj)
        world.bodies.remove(body)

class curry:
    def __init__(self, fun, *args, **kwargs):
        self.fun = fun
        self.pending = args[:]
        self.kwargs = kwargs.copy()

    def __call__(self, *args, **kwargs):
        if kwargs and self.kwargs:
            kw = self.kwargs.copy()
            kw.update(kwargs)
        else:
            kw = kwargs or self.kwargs
        
        return self.fun(*(self.pending + args), **kw)
