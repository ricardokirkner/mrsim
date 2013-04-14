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
from cgkit.glmaterial import GLMaterial
from cgkit.cgtypes import vec3, vec4
from motor import Motor
import logging

logger = logging.getLogger('mrsim.link')

class Link(Box):

    #
    #--- Creation -------------------------------------------------------------
    #
    # REQ: cfg = {'lengths': vec3, 'mass': double, 'motor': dict,
    #             'color': vec3}. <>
    def __init__(self,
                 world,
                 name = "Link",
                 module = None,
                 cfg = None,
                 data = None,
                 **params):

        Box.__init__(self, name=name, **params)

        self.world = world
        self.module = module
        self.cfg = cfg
        self.data = data
        self.motor = {}

        self._configureBody()
        self._configureGeom()

    #
    #--- State information ----------------------------------------------------
    #
    def _getState(self):
        values = [self.motor['active'].angle, self.motor['passive'].angle]
        return values
    state = property(_getState)

    #
    #--- Docking --------------------------------------------------------------
    #
    def connect(self, block):
        if self.motor.has_key(block.name):
            raise ValueError("Block '%s' is already connected." % block.name)
        angle = self.cfg['motor']['angle'][block.type]
        motorClass = getattr(__import__('mrsim.motor', None, None, ['']), self.cfg['motor']['class'])
        self.motor[block.type] = motorClass(self.world, block=block, link=self, 
                                       angle=angle, cfg=self.cfg['motor'], data=self.data)

    #
    #=== Private methods ======================================================
    #
    #
    #--- Creation -------------------------------------------------------------
    #
    def _configureBody(self):
        self.lx, self.ly, self.lz = self.cfg['lengths']
        #self.lz = self.lx
        self.mass = self.cfg['mass']
        logger.info("using link lengths: (%s, %s, %s)" % (self.lx, self.ly, self.lz))
        logger.info("using link mass: %s" % self.mass)

    def _configureGeom(self):
        self._loadGeom()
        color = self.cfg['color']
        light = vec4(color[0] / 255.0, color[1] / 255.0, color[2] / 255.0, 1.0)
        material = GLMaterial(ambient=light, diffuse=light)
        self.child('Box').setMaterial(material, 0)
        self.setMaterial(material, 0)

    def _loadGeom(self):
        box = Box(lx=self.lx, ly=self.ly, lz=self.ly, 
                  dynamics=False, auto_insert=False)
        self.addChild(box)
        self.visible = False

