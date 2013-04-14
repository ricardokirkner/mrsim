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

import protocols
from cgkit.Interfaces import *
from cgkit.component import Component
from cgkit.cgtypes import vec3
from cgkit.slots import *
from constants import LIGHT_SENSOR, DIRECTION_SENSOR, ROTATION_SENSOR, OBSTACLE_SENSOR
import logging
from math import degrees, cos, sin

logger = logging.getLogger('mrsim.sensor')

class Sensor(Component):

    protocols.advise(instancesProvide=[ISceneItem])

    exec slotPropertyCode("pos")

    def __init__(self, 
                 name="Sensor",
                 receiver=None,
                 emitter=None, 
                 relative_pos=vec3()):

        Component.__init__(self, name)
        self.receiver = receiver
        self.emitter = emitter
        self.type = None
        self.relative_pos = relative_pos
        self.pos_slot = ProceduralVec3Slot(self._computeSensorPos)
        self.addSlot("pos", self.pos_slot)

    def _computeSensorValue(self):
        raise NotImplementedError

    def _computeSensorPos(self):
        basePos = self.receiver.pos
        baseRot = self.receiver.rot
        pos = basePos + baseRot * self.relative_pos
        return pos


class LightSensor(Sensor):

    exec slotPropertyCode("value")

    def __init__(self, 
                 name="LightSensor", 
                 receiver=None,
                 emitter=None,
                 relative_pos=vec3(),
                 effective_range=10,
                 buckets=10):

        Sensor.__init__(self, name=name, receiver=receiver, emitter=emitter, 
                        relative_pos=relative_pos)
        self.type = LIGHT_SENSOR
        self.value_slot = ProceduralIntSlot(self._computeSensorValue)
        self.addSlot("value", self.value_slot)
        self.receiver.pos_slot.addDependent(self.pos_slot)
        self.pos_slot.addDependent(self.value_slot)

        self.effective_range = effective_range
        self.buckets = buckets
        
    def _computeSensorValue(self):
        distance = abs(self.pos - self.emitter.pos)
        value = self._discretize(distance)
        return value

    def _discretize(self, distance):
        if distance >= self.effective_range:
            value = 0
        else:
            value = int((1.0 - distance / float(self.effective_range)) * self.buckets)
        return value
        

class DirectionSensor(Sensor):

    exec slotPropertyCode("value")

    def __init__(self, 
                 name="DirectionSensor",
                 receiver=None,
                 emitter=None):

        Sensor.__init__(self, name, receiver=receiver, emitter=emitter)
        self.type = DIRECTION_SENSOR
        self.value_slot = ProceduralIntSlot(self._computeSensorValue)
        self.addSlot("value", self.value_slot)
        self.receiver.active.pos_slot.addDependent(self.value_slot)
        self.receiver.passive.pos_slot.addDependent(self.value_slot)
        self.axis = self._getAxis()

    def _computeSensorValue(self):
        activeBlockPos = self.receiver.active.pos
        passiveBlockPos = self.receiver.passive.pos
        lightPos = self.emitter.pos
        activeBlockDirection = self._getDirection(activeBlockPos, lightPos)
        passiveBlockDirection = self._getDirection(passiveBlockPos, lightPos)
        direction = activeBlockDirection + passiveBlockDirection
        #value = self._normalize(direction * self.axis)
        axis = self._getAxis()
        value = self._normalize(direction * axis)
        return value

    def _getDirection(self, a, b):
        direction = lambda x: x > 0 and -1 or x < 0 and 1 or 0
        res = map(direction, a-b)
        return vec3(res)

    def _normalize(self, value):
        normalize = lambda x: x > 1 and 1 or x < -1 and -1 or 0
        res = normalize(value)
        return res

    def _getAxis(self):
        direction = (self.receiver.passive.pos - self.receiver.active.pos).normalize()
        #rotation = self.receiver.rot
        #axis = direction * rotation
        axis = direction
        return axis


class RotationSensor(Sensor):

    exec slotPropertyCode("value")

    def __init__(self, 
                 name="RotationSensor",
                 receiver=None):

        Sensor.__init__(self, name, receiver=receiver)
        self.type = ROTATION_SENSOR
        self.value_slot = ProceduralIntSlot(self._computeSensorValue)
        self.addSlot("value", self.value_slot)
        self.receiver.active.pos_slot.addDependent(self.value_slot)
        self.receiver.passive.pos_slot.addDependent(self.value_slot)
        self._initial_angles = vec3(self.receiver.rot.toEulerXYZ())

    def _computeSensorValue(self):
        activeBlockState = self.receiver.active.state
        passiveBlockState = self.receiver.passive.state
        rotation = self._getRotation(activeBlockState, passiveBlockState)
        return rotation

    def _getRotation(self, a, b):
        x = a - b
        relative_position = x > 0 and -1 or x < 0 and 1 or 0
        current_angles = vec3(self.receiver.rot.toEulerXYZ())
        angles = current_angles - self._initial_angles
        rotY = cos(angles[1])
        rotZ = 1 - sin(angles[2])
        rotation = relative_position * rotY * rotZ
        return int(round(rotation))


class ObstacleSensor(Sensor):

    exec slotPropertyCode("value")

    def __init__(self, 
                 name="ObstacleSensor", 
                 receiver=None,
                 obstacles=[],
                 relative_pos=vec3(),
                 effective_range=2.0):

        Sensor.__init__(self, name=name, receiver=receiver, relative_pos=relative_pos)
        self.type = OBSTACLE_SENSOR
        self.value_slot = ProceduralIntSlot(self._computeSensorValue)
        self.pos_slot = ProceduralVec3Slot(self._computePos)
        self.addSlot("value", self.value_slot)
        self.receiver.active.pos_slot.addDependent(self.pos_slot)
        self.receiver.passive.pos_slot.addDependent(self.pos_slot)
        self.pos_slot.addDependent(self.value_slot)

        self.obstacles = obstacles
        self.effective_range = effective_range * self.receiver.active.lx
        
    def _computeSensorValue(self):
        value = 0
        for obstacle in self.obstacles:
            vector = map(abs, self.pos - obstacle.pos)
            lengths = vec3(obstacle.lx, obstacle.ly, obstacle.lz)
            max_value = max(vector)
            idx = vector.index(max_value)
            #if vector[idx] < 0.75 * lengths[idx]:
            if vector[idx] < 0.5 * lengths[idx] + self.effective_range:
                value = 1
            logger.debug("obstacle '%s' lengths: %s" % (obstacle.name, lengths))
            logger.debug("obstacle '%s' pos: %s" % (obstacle.name, obstacle.pos))
            #logger.debug("vector[idx]: %s 0.75*lengths[idx]: %s" % (vector[idx], 0.75*lengths[idx]))
            logger.debug("vector[idx]: %s 0.5*lengths[idx] + self.effective_range: %s" % (vector[idx], 0.5*lengths[idx]+self.effective_range))
        return value

    def _computePos(self):
        activeBlockPos = self.receiver.active.pos
        passiveBlockPos = self.receiver.passive.pos
        modulePos = max(activeBlockPos, passiveBlockPos)
        return modulePos
