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

from cgkit.box import Box
from cgkit.cmds import load
from cgkit.cgtypes import vec3, vec4, mat3
from cgkit.odedynamics import ODESliderJoint
from cgkit.glmaterial import GLMaterial, GLTexture
from constants import ACTIVE, PASSIVE, BOTTOM, FRONT, REAR
from sensor import LightSensor
from util import getPivotedRotation
from math import radians
import logging

logger = logging.getLogger('mrsim.block')

class Block(Box):

    #
    #=== Public Methods =======================================================
    #
    #
    #--- Creation -------------------------------------------------------------
    #
    # REQ: cfg = {'lengths': double, 'mass': double, 
    #             'sensor': {'emitter': WorldObject,
    #                        'effective_range': double}} <>
    def __init__(self,
                 world,
                 name = "Block",
                 type = None,
                 module = None,
                 cfg = None,
                 **params):

        Box.__init__(self, name=name, **params)

        if type not in (ACTIVE, PASSIVE):
            raise ValueError("Invalid type (%s)." % type)

        self.world = world
        self.type = type
        self.module = module
        self.cfg = cfg
        self.neighbor = {}
        self.sensor = {}

        self._configureBody()
        self._configureGeom()

    #
    #--- Docking --------------------------------------------------------------
    #
    def dock(self, activeSide, passiveBlock, passiveSide):
        if self._aligned(activeSide, passiveBlock, passiveSide):
            self._match(activeSide, passiveBlock, passiveSide)
            self._attach(activeSide, passiveBlock, passiveSide)
        else:
            logger.warn("Blocks '%s' and '%s' not aligned. Docking not possible."
                        % (self, passiveBlock))

    def getDockingPoints(self, side):
        if side == BOTTOM:
            points = self._getBottomDockingPoints()
        elif side in (FRONT, REAR):
            points = self._getLateralDockingPoints(side)
        else:
            raise ValueError("Invalid side (%s)." % side)

        contactPoints = [self._compensateRotation(p + self.pos) for p in points]
        return contactPoints

    def getDockingSide(self, dockingPoint):
        sideDockingPoints = {}
        for side in (BOTTOM, FRONT, REAR):
            sideDockingPoints[side] = self.getDockingPoints(side)

        for side in (BOTTOM, FRONT, REAR):
            for point in sideDockingPoints[side]:
                distance = abs(point - dockingPoint)
                if distance < self.cfg['nearThreshold']:
                    logger.info("Side '%s' found suitable for docking." % side)
                    return side
        return None

    def getDockingSideCenter(self, dockingSide):
        dockingPoints = self.getDockingPoints(dockingSide)
        center = 0.5 * (dockingPoints[0] + dockingPoints[1]) + \
                 0.5 * (dockingPoints[2] + dockingPoints[3])
        return center

    def undock(self, activeSide):
        self._detach(activeSide)

    #
    #--- State information ----------------------------------------------------
    #
    def _getState(self):
        lightValue = self.sensor['light'].value
        return lightValue
    state = property(_getState)
    
    #
    #=== Private Methods =====================================================
    #
    #
    #--- Creation ------------------------------------------------------------
    #
    def _configureBody(self):
        self.lx, self.ly, self.lz = self.cfg['lengths']
        #self.lz *= 0.5
        self.mass = self.cfg['mass']
        logger.info("using block lengths: (%s, %s, %s)" % (self.lx, self.ly, self.lz))
        logger.info("using block mass: %s" % self.mass)

    def _configureGeom(self):
        self._loadGeom()
        color = self.cfg['color']
        light = vec4(color[0] / 255.0, color[1] / 255.0, color[2] / 255.0, 1.0)
        material = GLMaterial(ambient=light, diffuse=light)
        self.child('Mesh').setMaterial(material, 0)
        self.setMaterial(material, 0)

    def _loadGeom(self):
        load('mdl/block.obj', parent=self)
        self.child('Mesh').dynamics = False
        #self.child('Mesh').scale = vec3(self.lx, self.ly, 2.0 * self.lz)
        self.child('Mesh').scale = vec3(self.lx, self.ly, self.lz)
        #self.child('Mesh').pos = vec3(0, 0, 0.5*self.lz)
        self.visible = False

    def configureSensors(self):
        self._configureLightSensor()
    #
    #--- Docking --------------------------------------------------------------
    #
    def _aligned(self, activeSide, passiveBlock, passiveSide):
        activeDockingPoints = self.getDockingPoints(activeSide)
        passiveDockingPoints = passiveBlock.getDockingPoints(passiveSide)
        return self._dockingPointsAligned(activeDockingPoints, 
                                          passiveDockingPoints)

    def _getBottomDockingPoints(self):
        #lx, ly, lz = self.cfg['lengths']
        lx, ly, lz = self.lx, self.ly, self.lz
        points = [vec3(0.25*lx, 0.25*ly, -0.5*lz),
                  vec3(0.25*lx, -0.25*ly, -0.5*lz),
                  vec3(-0.25*lx, -0.25*ly, -0.5*lz),
                  vec3(-0.25*lx, 0.25*ly, -0.5*lz)]
        return points

    def _getLateralDockingPoints(self, side):
        if side == FRONT:
            sgn = 1
        elif side == REAR:
            sgn = -1
        else:
            raise ValueError("Invalid side (%s)." % side)

        lx, ly, lz = self.cfg['lengths']
        points = [vec3(0.25*lx, sgn*0.5*ly, 0.25*lz),
                  vec3(-0.25*lx, sgn*0.5*ly,  0.25*lz),
                  vec3(-0.25*lx, sgn*0.5*ly, -0.25*lz),
                  vec3(0.25*lx, sgn*0.5*ly, -0.25*lz)]
        return points

    def _compensateRotation(self, pos):
        #cmds.drawMarker(pos, size=0.003, color=(0, 0, 0))
        pos = getPivotedRotation(self.pos, self.rot.inverse()) * pos
        #cmds.drawMarker(pos, size=0.003, color=(1, 1, 1))
        return pos
        
    def _dockingPointsAligned(self, activePoints, passivePoints):
        distance = [0, 0, 0, 0]
        for i in range(len(activePoints)):
            # get minimum distance to any point
            distance[i] = min([abs(activePoints[i] - p) for p in 
                              passivePoints])
        return max(distance) < self.cfg['nearThreshold']
        
    def _match(self, activeSide, passiveBlock, passiveSide):
        pass

    def _attach(self, activeSide, passiveBlock, passiveSide):
        neighborData = self.neighbor.get(activeSide)
        if neighborData is not None:
            logger.warn("Block is already docked on side '%s'." % activeSide)
            return

        # create joint
        sj = ODESliderJoint(body1=self, body2=passiveBlock)
        # make it fixed
        sj.lostop = 0
        sj.histop = 0
        sj.motorfmax = 0
        # active and persist
        self.world.add(sj)
        self.neighbor[activeSide] = {'block': passiveBlock, 'joint': sj}

    def _detach(self, activeSide):
        neighborData = self.neighbor.get(activeSide)
        if neighborData is None:
            logger.warn("Block is not docked on side '%s'." % activeSide)
        else:
            # remove joint from world
            sj = neighborData['joint']
            self.world.joints.remove(sj)
            # delete underlying ODE joint
            del sj.odejoint
            # and remove neighbor data
            #self.neighbor[activeSide] = None
            del self.neighbor[activeSide]

    def _configureLightSensor(self):
        emitter = self.cfg['sensor']['emitter']
        motorPos = self.module.link.motor[self.type].pos
        blockPos = self.pos
        #relative_pos = self.cfg['sensor'].get('relative_pos', vec3(0, 0, 0))
        relative_pos = motorPos - blockPos
        effective_range = self.cfg['sensor']['effective_range']
        buckets = self.cfg['sensor']['buckets']
        self.sensor['light'] = LightSensor(receiver=self, 
                                  emitter=emitter,
                                  relative_pos=relative_pos,
                                  effective_range=effective_range,
                                  buckets=buckets)

