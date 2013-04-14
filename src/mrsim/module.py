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
from cgkit.slots import *
from cgkit.cgtypes import vec3, mat3
from math import radians
from block import Block
from link import Link
from util import getPivotedRotation
from sensor import DirectionSensor, RotationSensor, ObstacleSensor
from constants import ACTIVE, PASSIVE, LINK, BOTTOM, IDLE, BUSY, TIMEOUT
from util import allIdle, getStatus, removeObject, curry
import logging

logger = logging.getLogger('mrsim.module')

class Module(Component):

    protocols.advise(instancesProvide=[ISceneItem])

    #
    #=== Public Methods ======================================================
    #
    #
    #--- Creation ------------------------------------------------------------
    #
    # REQ: cfg = {'block': {'lengths': double, 
    #                       'active': dict,
    #                       'passive': dict},
    #             'link': dict}. <>
    def __init__(self, 
                 world,
                 name = "Module", 
                 pos = vec3(),
                 rot = mat3(),
                 robot = None,
                 cfg = None,
                 data = None):

        Component.__init__(self, name=name)

        self.world = world
        self.pos = pos
        self.rot = rot
        self.robot = robot
        self.cfg = cfg
        self.data = data
        self.sensor = {}

        self._createBlocks()
        self._createLink()
        self._connectObjects()
        self._positionObjects()
        self._configureSensors()

    #
    #--- Docking --------------------------------------------------------------
    #
    def dock(self, activeSide, module):
        passiveSide = self._findMatchingDockingSide(activeSide, module.passive)
        if passiveSide is not None:
            self.active.dock(activeSide, module.passive, passiveSide)
        else:
            logger.warn("No matching docking side found for side '%s'." % 
                        activeSide)

    def undock(self, side):
        self.active.undock(side)

    #
    #--- State information ----------------------------------------------------
    #
    def _getState(self):
        activeBlockAngle = self.link.motor['active'].angle
        passiveBlockAngle = self.link.motor['passive'].angle
        activeBlockLightValue = self.active.state
        passiveBlockLightValue = self.passive.state
        moduleLightValue = 0.5 * (activeBlockLightValue + passiveBlockLightValue)
        # get module direction
        moduleDirection = self.sensor['direction'].value
        # get module rotation
        moduleRotation = self.sensor['rotation'].value
        value = [moduleLightValue, moduleRotation, moduleDirection, activeBlockAngle, passiveBlockAngle]
        return value
    state = property(_getState)

    def _getStatus(self):
        motorStatus = [motor.status for motor in self.link.motor.values()]
        logger.debug("motorStatus: %s" % str(motorStatus) )
        moduleStatus = allIdle(motorStatus) and IDLE or BUSY
        logger.debug("moduleStatus: %s" % moduleStatus)
        return moduleStatus
    status = property(_getStatus)
    
    #
    #=== Private Methods ======================================================
    #
    #
    #--- Creation -------------------------------------------------------------
    #
    def _createBlocks(self):
        self.active = self._createBlock(ACTIVE)
        self.passive = self._createBlock(PASSIVE)

    def _createBlock(self, type):
        cfg = self._getBlockConfig(type)
        pos, rot = self._getInitialValues(type)
        name = '%s.%s' % (self.name, type)
        block = Block(self.world, name=name, type=type, pos=pos, rot=rot, 
                      cfg=cfg, module=self) 
        self.world.add(block)
        logger.info("using block pos: %s" % pos)
        return block

    def _getBlockConfig(self, type):
        cfg = self._getBaseBlockConfig()
        cfg.update(self.cfg['block'][type])
        return cfg

    def _getBaseBlockConfig(self):
        cfg = dict(self.cfg['block'])
        del cfg['active']
        del cfg['passive']
        return cfg

    def _getInitialValues(self, type):
        lx = self.cfg['link']['lengths'][0]
        axis = vec3(0, 1, 0)
        #distance = 0.75*lx
        distance = 0.5*lx

        if type == ACTIVE:
            # base position and rotation
            pos = self.pos + vec3(-distance, 0, 0)
            rot = mat3().rotation(radians(90), axis)
        elif type == PASSIVE:
            # base position
            pos = self.pos + vec3(distance, 0, 0)
            rot = mat3().rotation(radians(-90), axis)
        elif type == LINK:
            # base position
            pos = self.pos
            rot = mat3(1.0)
        else:
            raise ValueError("Invalid object type (%s)." % type)
        return (pos, rot)

    def _createLink(self):
        pos, rot = self._getInitialValues(LINK)
        name = '%s.link' % self.name
        self.link = Link(self.world, name=name, pos=pos, rot=rot, 
                         cfg=self.cfg['link'], module=self, data=self.data)
        self.world.add(self.link)
        logger.info("using link pos: %s" % pos)

    def _connectObjects(self):
        self.link.connect(self.active)
        self.link.connect(self.passive)

    def _positionObjects(self):
        self._positionObject(ACTIVE, self.active)
        self._positionObject(PASSIVE, self.passive)
        self._positionObject(LINK, self.link)

    def _positionObject(self, type, obj):
        pos, rot = self._getFinalValues(type, obj.pos, obj.rot)
        self._updateValues(obj, pos, rot)

    def _getFinalValues(self, type, pos, rot):
        def compensateMotorAngle(rot, angle):
            axis = vec3(0, 1, 0)
            rot = rot.rotate(radians(-angle), axis)
            return rot

        def compensateModuleRotation(pos, rot):
            m = getPivotedRotation(self.pos, self.rot)
            # update position and rotation according to module's rotation
            pos = m * pos
            rot = m.getMat3() * rot
            return (pos, rot)

        # compensate motor angle
        if type in (ACTIVE, PASSIVE):
            angle = self.cfg['link']['motor']['angle'][type]
            rot = compensateMotorAngle(rot, angle)
        # compensate module rotation
        pos, rot = compensateModuleRotation(pos, rot)
        return (pos, rot)

    def _updateValues(self, obj, pos, rot):
        obj.pos = pos
        obj.rot = rot
        # reset initial position
        odebody = self.world.body_dict[obj]
        odebody.initial_pos = pos
        odebody.initial_rot = rot
        odebody.reset()

    def _configureSensors(self):
        self._configureBlockSensors()
        self._configureDirectionSensor()
        self._configureRotationSensor()

    def _configureBlockSensors(self):
        self.active.configureSensors()
        self.passive.configureSensors()

    def _configureDirectionSensor(self):
        emitter = self.cfg['sensor']['direction']['emitter']
        self.sensor['direction'] = DirectionSensor(receiver=self, emitter=emitter)

    def _configureRotationSensor(self):
        self.sensor['rotation'] = RotationSensor(receiver=self)

    def configureAdditionalSensors(self):
        self._configureObstacleSensor()        

    def _configureObstacleSensor(self):
        obstacles = self.cfg['sensor']['obstacle']['obstacles']
        self.sensor['obstacle'] = ObstacleSensor(receiver=self, obstacles=obstacles) 

    #
    #--- Docking -------------------------------------------------------------
    #
    def _findMatchingDockingSide(self, activeSide, passiveBlock):
        # FIXME: remove commented code. <>
        #from cgkit import cmds
        activeDockingPoints = self.active.getDockingPoints(activeSide)
        #for p in activeDockingPoints:
        #    cmds.drawMarker(p, size=0.005, color=vec3(0,1,1))
        passiveSide = None
        points = iter(activeDockingPoints)
        while points and passiveSide is None:
            passiveSide = passiveBlock.getDockingSide(points.next())
        return passiveSide


class Module1(Module):

    def _getStatus(self):
        motorStatus = [motor.status for motor in self.link.motor.values()]
        logger.debug("motorStatus: %s" % str(motorStatus) )
        moduleStatus = getStatus(motorStatus)
        logger.debug("moduleStatus: %s" % moduleStatus)
        return moduleStatus
    status = property(_getStatus)

    def resetStatus(self, status):
        for motor in self.link.motor.values():
            motor.status = status

class ModuleCombiner(Module):

    def __init__(self, 
                 world,
                 name = "Module", 
                 pos = vec3(),
                 rot = mat3(),
                 robot = None,
                 cfg = None,
                 data = None):
        self._initialized = False
        Module.__init__(self, world, name=name, pos=pos, rot=rot, robot=robot, cfg=cfg, data=data)
        self._modules = data['class']['module']
        self._index = 0
        self.data = data
        moduleClassName = self._modules[0]
        moduleClass = getattr(__import__('mrsim.module', None, None, ['']), moduleClassName)
        self._current = moduleClass
        self._initialized = True

    def adapt(self, index):
        self._index = index
        # create module
        moduleClassName = self._modules[index]
        moduleClass = getattr(__import__('mrsim.module', None, None, ['']), moduleClassName)
        # adapt motors
        for motor in self.link.motor.values():
            motor.adapt(index)
        # update active module
        self._current = moduleClass

    def getObstacleSensorValue(self):
        value = self.sensor['obstacle'].value
        return value

    def __getattribute__(self, name):
        initialized = Module.__getattribute__(self, '_initialized')
        if not initialized:
            return Module.__getattribute__(self, name)
        if name in Module.__getattribute__(self, '__dict__'):
            return Module.__getattribute__(self, name)
        elif name in ['adapt', 'getObstacleSensorValue']:
            return Module.__getattribute__(self, name)
        else:
            method = getattr(self._current, name)
            if hasattr(method, 'im_func'):
                value = curry(method.im_func, self)
            elif hasattr(method, 'fget'):
                value = curry(method.fget, self)()
            else:
                value = method
            return value
