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
from cgkit.quadrics import Sphere
from cgkit.cgtypes import vec3, vec4, mat3
from cgkit.cmds import getScene
from cgkit.odedynamics import ODEDynamics, ODEContactProperties
from cgkit.glmaterial import GLMaterial
from robot import Robot
from module import Module
from block import Block
from link import Link
from sensor import LightSensor
from constants import LIGHT_SENSOR
from math import radians
import logging

logger = logging.getLogger('mrsim.builder')

class Builder(object):

    def __init__(self, dataFileName, cfgFileName):
        object.__init__(self)
        self.world = None
        self.data = eval(open(dataFileName).read())
        self.cfg = eval(open(cfgFileName).read())
        self._scaleUnits()

    def _scaleUnits(self):
        self._loadUnitConversionData()
        # scale cfg units
        self._scaleDistanceUnits(self._findDistanceObjectsInCfg())
        self._scaleMassUnits(self._findMassObjectsInCfg())
        # transform data units
        #self._transformDistanceObjects(self._findDistanceObjectsInData())
        #self._transformMassObjects(self._findMassObjectsInData())
        self._scaleDistanceUnits(self._findDistanceObjectsInData())
        self._scaleMassUnits(self._findMassObjectsInData())

    def _loadUnitConversionData(self):
        self.distanceUnit = float(self.cfg['units']['distance'])
        self.massUnit = float(self.cfg['units']['mass'])
        self.timeUnit = float(self.cfg['units']['time'])

    def _findDistanceObjectsInCfg(self):
        return self.cfg['robot']['module'].items()

    def _scaleDistanceUnits(self, distanceObjects):
        distanceUnit = self.distanceUnit
        for objectName, objectData in distanceObjects:
            for itemName in objectData:
                item = objectData[itemName]
                if itemName in ('position', 'lengths'):
                    logger.debug("[_scaleDistanceUnits] position|lengths (before): %s" % str(objectData[itemName]))
                    objectData[itemName] = vec3(item) * distanceUnit
                    logger.debug("[_scaleDistanceUnits] position|lengths (after): %s" % str(objectData[itemName]))
                elif itemName in ('radius', 'nearThreshold'):
                    logger.debug("[_scaleDistanceUnits] radius|nearThreshold (before): %s" % str(objectData[itemName]))
                    objectData[itemName] = item * distanceUnit
                    logger.debug("[_scaleDistanceUnits] radius|nearThreshold (after): %s" % str(objectData[itemName]))
                elif itemName in ('motor',):
                    motorObject = objectData[itemName]
                    if motorObject.has_key('motorfmax'):
                        logger.debug("[_scaleDistanceUnits] motorfmax (before): %s" % str(motorObject['motorfmax']))
                        motorObject['motorfmax'] *= distanceUnit
                        logger.debug("[_scaleDistanceUnits] motorfmax (after): %s" % str(motorObject['motorfmax']))
                elif itemName in ('sensor',):
                    sensorObject = objectData[itemName]
                    if sensorObject.has_key('effective_range'):
                        logger.debug("[_scaleDistanceUnits] effective_range (before): %s" % str(sensorObject['effective_range']))
                        sensorObject['effective_range'] *= distanceUnit
                        logger.debug("[_scaleDistanceUnits] effective_range (after): %s" % str(sensorObject['effective_range']))

    def _findMassObjectsInCfg(self):
        return self.cfg['robot']['module'].items()

    def _scaleMassUnits(self, massObjects):
        massUnit = self.massUnit
        for objectName, objectData in massObjects:
            for itemName in objectData:
                if itemName == 'mass':
                    logger.debug("[_scaleMassUnits] mass (before): %s" % str(objectData['mass']))
                    objectData['mass'] *= massUnit
                    logger.debug("[_scaleMassUnits] mass (after): %s" % str(objectData['mass']))
                elif itemName in ('active', 'passive'):
                    blockObject = objectData[itemName]
                    if blockObject.has_key('mass'):
                        logger.debug("[_scaleMassUnits] mass (before): %s" % str(blockObject['mass']))
                        blockObject['mass'] *= massUnit
                        logger.debug("[_scaleMassUnits] mass (after): %s" % str(blockObject['mass']))
                elif itemName in ('motor',):
                    motorObject = objectData[itemName]
                    if motorObject.has_key('motorfmax'):
                        logger.debug("[_scaleMassUnits] motorfmax (before): %s" % str(motorObject['motorfmax']))
                        motorObject['motorfmax'] *= massUnit
                        logger.debug("[_scaleMassUnits] motorfmax (after): %s" % str(motorObject['motorfmax']))

    def _findDistanceObjectsInData(self):
        return self.data['object'].items() + self.data['module'].items() 

#    def _transformDistanceObjects(self, distanceObjects):
#        distanceUnitScale = self.distanceUnitScale
#        distanceUnit = self.distanceUnit
#        for objectName, objectData in distanceObjects:
#            for itemName in objectData:
#                item = objectData[itemName]
#                if itemName in ('position', 'lengths'):
#                    logger.debug("[_transformDistanceObjects] position|lengths (before): %s" % str(objectData[itemName]))
#                    objectData[itemName] = vec3(item) * distanceUnitScale * distanceUnit
#                    logger.debug("[_transformDistanceObjects] position|lengths (after): %s" % str(objectData[itemName]))
#                elif itemName in ('radius', 'nearThreshold'):
#                    logger.debug("[_transformDistanceObjects] radius|nearThreshold (before): %s" % str(objectData[itemName]))
#                    objectData[itemName] = item * distanceUnitScale * distanceUnit
#                    logger.debug("[_transformDistanceObjects] radius|nearThreshold (after): %s" % str(objectData[itemName]))
#                elif itemName in ('motor',):
#                    motorObject = objectData[itemName]
#                    if motorObject.has_key('motorfmax'):
#                        logger.debug("[_transformDistanceObjects] motorfmax (before): %s" % str(motorObject['motorfmax']))
#                        motorObject['motorfmax'] *= distanceUnitScale * distanceUnit
#                        logger.debug("[_transformDistanceObjects] motorfmax (after): %s" % str(motorObject['motorfmax']))
#                elif itemName in ('sensor',):
#                    sensorObject = objectData[itemName]
#                    if sensorObject.has_key('effective_range'):
#                        logger.debug("[_transformDistanceObjects] effective_range (before): %s" % str(sensorObject['effective_range']))
#                        sensorObject['effective_range'] *= distanceUnitScale * distanceUnit
#                        logger.debug("[_transformDistanceObjects] effective_range (after): %s" % str(sensorObject['effective_range']))
#
    def _findMassObjectsInData(self):
        return []
#
#    def _transformMassObjects(self, massObjects):
#        massUnitScale = self.massUnitScale
#        massUnit = self.massUnit
#        for objectName, objectData in massObjects:
#            for itemName in objectData:
#                item = objectData[itemName]
#                if itemName in ('mass',):
#                    logger.debug("mass (before): %s" % str(objectData[itemName]))
#                    objectData[itemName] = item * massUnitScale * massUnit
#                    logger.debug("mass (after): %s" % str(objectData[itemName]))
#                if itemName in ('motor',):
#                    motorObject = objectData[itemName]
#                    if motorObject.has_key('motorfmax'):
#                        logger.debug("motorfmax (before): %s" % str(motorObject['motorfmax']))
#                        motorObject['motorfmax'] *= massUnitScale * massUnit
#                        logger.debug("motorfmax (after): %s" % str(motorObject['motorfmax']))

    def createScene(self):
        self._createWorld()
        self._createEnvironment()
        self._createRobots()

    def _createWorld(self):
        gravity = self.cfg['world']['gravity']
        logger.debug("gravity (before): %s" % str(gravity))
        gravity = gravity * self.distanceUnit * (self.timeUnit ** 2)
        logger.debug("gravity (after): %s" % str(gravity))
        logger.info("using gravity: %s" % gravity)
        substeps = self.cfg['simulator'].get('substeps')
        erp = self.cfg['simulator'].get('erp')
        cfm = self.cfg['simulator'].get('cfm')
        kwargs = {'gravity': gravity}
        if substeps is not None:
            kwargs['substeps'] = substeps
        if erp is not None:
            kwargs['erp'] = erp
        if cfm is not None:
            kwargs['cfm'] = cfm
        contactProperties = ODEContactProperties(mu=20, bounce=0.0)
        kwargs['defaultcontactproperties'] = contactProperties
        self.world = ODEDynamics(name='world', **kwargs)
        def nearCallback(args, geom1, geom2):
            try:
                obj1 = geom1.worldobj
                obj2 = geom2.worldobj

                if isinstance(obj1, (Block, Link)) and \
                   isinstance(obj2, (Block, Link)) and \
                   obj1.module == obj2.module:
                    return
                ODEDynamics.nearCallback(self.world, args, geom1, geom2)
            except:
                ODEDynamics.nearCallback(self.world, args, geom1, geom2)

        self.world.nearCallback = nearCallback

    def _createEnvironment(self):
        for objName in self.data['environment']['objects']:
            self._createObject(objName)

    def _createObject(self, objName):
        objData = self.data['object'][objName]
        if objData['type'] == 'wall':
            obj = self._createWall(objName, objData)
        elif objData['type'] == 'light':
            obj = self._createLight(objName, objData)
        self.world.add(obj)

    def _createWall(self, objName, objData):
        lx, ly, lz = vec3(objData['lengths'])
        pos = vec3(objData['position'])
        x, y, z= objData['rotation']
        rot = mat3().fromEulerXYZ(radians(x), radians(y), radians(z))
        obj = Box(name=objName, lx=lx, ly=ly, lz=lz, pos=pos, rot=rot,
                  static=True)
        logger.info("using box lengths: (%s, %s, %s)" % (lx, ly, lz))
        logger.info("using box position: %s" % pos)
        return obj

    def _createLight(self, objName, objData):
        radius = objData['radius']
        pos = vec3(objData['position'])
        color = objData.get('color')
        if color is not None:
            light = vec4(color[0] / 255.0, color[1] / 255.0, color[2] / 255.0, 1.0)
            material = GLMaterial(ambient=light, diffuse=light)
        else:
            material = None
        obj = Sphere(name=objName, radius=radius, pos=pos, dynamics=False, material=material)
        logger.info("using sphere radius: %s" % radius)
        logger.info("using sphere pos: %s" % pos)
        return obj

    def _createRobots(self):
        for robotName in self.data['robot']:
            self._createRobot(robotName, self.cfg['robot'])

    def _createRobot(self, name, cfg):
        robotData = self.data['robot'][name]
        robotClass = getattr(__import__('mrsim.robot', None, None, ['']), self.data['class']['robot'])
        robot = robotClass(self.world, name=name, cfg=cfg, data=robotData)
        #robot = Robot(self.world, name=name, cfg=cfg)
        # create modules
        for moduleName in robotData['modules']:
            module = self._createModule(moduleName, cfg['module'], data=robotData)
            module.configureAdditionalSensors()
            robot.addChild(module)
        # connect modules together
        for moduleName in robotData['modules']:
            self._dockModule(robot, moduleName)
        # activate brains
        robot.initBrain(robotData['brain']['policy_mode'], robotData['brain']['class'], robotData['brain']['goal'], robotData['brain']['action_value_function_filename'], 
robotData['program'])

    def _createModule(self, name, cfg, data=None):
        moduleData = self.data['module'][name]
        pos = vec3(moduleData['position'])
        dX, dY, dZ = moduleData['rotation']
        rot = mat3().fromEulerXYZ(radians(dX), radians(dY), radians(dZ))
        lightSource = self._getLightSources()[0]
        obstacles = self._getObstacles()
        cfg['sensor'] = cfg.setdefault('sensor', {'direction':{}, 'obstacle':{}})
        cfg['sensor']['direction']['emitter'] = lightSource
        cfg['sensor']['obstacle']['obstacles'] = obstacles
        cfg['block']['sensor'] = cfg['block'].setdefault('sensor', {})
        cfg['block']['sensor']['emitter'] = lightSource
        cfg['link']['motor']['angle'] = {'active': moduleData['angleActive'],
                                         'passive': moduleData['anglePassive']}
        cfg['link']['motor']['class'] = self.data['class']['motor']
        moduleClass = getattr(__import__('mrsim.module', None, None, ['']), self.data['class']['module'])
        module = moduleClass(self.world, name=name, pos=pos, rot=rot, cfg=cfg, data=data)
        logger.info("using module pos: %s" % pos)
        return module
        
    def _dockModule(self, robot, moduleName):
        jointData = self.data['module'][moduleName]['joint']
        for jointName in jointData:
            otherModule = robot.findChildByName(jointData[jointName])
            if otherModule is not None:
                module = robot.findChildByName(moduleName)
                module.dock(jointName, otherModule)

    def _getLightSources(self):
        lightNames = [objName for objName in self.data['object'] \
                      if self.data['object'][objName]['type'] == 'light' and \
                      objName in self.data['environment']['objects']]
        lights = [getScene().worldObject(objName) for objName in lightNames]
        return lights

    def _getObstacles(self):
        obstacleNames = [objName for objName in self.data['object'] \
                         if self.data['object'][objName].get('obstacle', False) and objName in self.data['environment']['objects']]
        obstacles = [getScene().worldObject(objName) for objName in obstacleNames]
        return obstacles
