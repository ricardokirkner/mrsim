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

from cgkit.cmds import getScene
from cgkit.component import Component
from cgkit.cgtypes import vec3, mat3
from cgkit.odedynamics import ODEHingeJoint
from cgkit.pidcontroller import PIDController
from cgkit.expression import Expression
from cgkit.eventmanager import eventManager
from cgkit.events import STEP_FRAME
from cgkit.slots import *
from cgkit import _core
from constants import IDLE, BUSY, TIMEOUT, ACTIVE, PASSIVE
from util import removeObject, curry
from math import radians, degrees
import logging

logger = logging.getLogger('mrsim.motor')

class Motor(Component):

    exec slotPropertyCode("angle")
    exec slotPropertyCode("status")
    exec slotPropertyCode("pos")

    #
    #=== Public Methods =======================================================
    #
    #
    #--- Creation -------------------------------------------------------------
    #
    # REQ: cfg = {'velocityThreshold': double}. <>
    def __init__(self, 
                 world,
                 name = "Motor",
                 block = None, 
                 link = None,
                 angle = 0.0,
                 cfg = None,
                 **params):

        Component.__init__(self, name)
        # define motor slots
        self.initialize_slots = True
        self.status_slot = StrSlot(IDLE)
        self.angle_slot = DoubleSlot(angle)
        self.pos_slot = Vec3Slot()
        self._lastAngle = angle
        self.addSlot("status", self.status_slot)
        self.addSlot("angle", self.angle_slot)
        self.addSlot("pos", self.pos_slot)
        self._angle_forwader = NotificationForwarder(self._onAngleChanged)
        self.angle_slot.addDependent(self._angle_forwader)
        self.initialize_slots = False

        self.world = world
        self.cfg = cfg
        self.block = block
        self.link = link
        self._remainingTime = self.cfg['timeout']

        self._createComponents()
        self._configureComponents()
        self._connectComponents()

        # add joint to world
        self.world.add(self._joint)

        # listen to events
        eventManager().connect(STEP_FRAME, self._onStepFrame)

    #
    #=== Private Methods ======================================================
    #
    #
    #--- Creation -------------------------------------------------------------
    #
    def _createComponents(self):
        # create the joint this motor should control
        self._joint = ODEHingeJoint(body1=self.block, body2=self.link, 
                                    auto_insert=False)

        # create a controller for this joint
        self._controller = PIDController(auto_insert=False)

        # create an expression component that converts angles 
        # between radians and degrees
        self._radiansConverter = Expression("degrees(radians)", radians=0)

    def _connectComponents(self):
        ## convert radians to degrees
        self._joint.angle_slot.connect(self._radiansConverter.radians_slot)
        self._joint.pos_slot.connect(self.pos_slot)
        # connect joint with controller
        self._radiansConverter.output_slot.connect(self._controller.input_slot)
        # connect controller with joint
        self._controller.output_slot.connect(self._joint.motorvel_slot)
        # connect motor with controller
        self.angle_slot.connect(self._controller.setpoint_slot)

    def _configureComponents(self):
        self._configureJoint()
        self._configureController()

    def _configureJoint(self):
        # set joint position and orientation
        block = self._joint.body1
        if block.type == ACTIVE:
                sgn = 1
        elif block.type == PASSIVE:
                sgn = -1
        #jointPosition = block.pos + sgn * vec3(0.25 * block.lx, 0, 0)
        jointPosition = block.pos 
        jointRotation = block.rot.rotate(radians(90), vec3(0, 0, 1))
        self._joint.pos = jointPosition
        self._joint.rot = jointRotation

        # set joint parameters
        if self.cfg is not None:
            motorfmax = self.cfg.get('motorfmax')
            lostop = self.cfg.get('lostop')
            histop = self.cfg.get('histop')
            if motorfmax is not None:
                self._joint.motorfmax = motorfmax
            if lostop is not None:
                self._joint.lostop = lostop
            if histop is not None:
                self._joint.histop = histop
            logger.info("using motor fmax: %s" % motorfmax)

    def _configureController(self):
        if self.cfg is None or not self.cfg.has_key('controller'):
            Kp = 0.1
            Ki = None
            Kd = None
        else:
            Kp = self.cfg['controller'].get('Kp')
            Ki = self.cfg['controller'].get('Ki')
            Kd = self.cfg['controller'].get('Kd')

        if Kp is not None:
            self._controller.Kp = Kp
        if Ki is not None:
            self._controller.Ki = Ki
        if Kd is not None:
            self._controller.Kd = Kd

    #
    #--- Simulation -----------------------------------------------------------
    #
    def _onStepFrame(self):
        self._updateStatus()

    def _updateStatus(self):
        if self._checkMotorStopped() or self._checkTimeout():
            self._stopMotor()

    def _checkMotorStopped(self):
        vel = degrees(self._joint.motorvel)
        logger.debug("vel: %s" % vel)
        return abs(vel) < self.cfg['velocityThreshold']

    def _checkTimeout(self):
        elapsedTime = getScene().timer().timestep
        self._remainingTime -= elapsedTime
        return self._remainingTime < 0

    def _stopMotor(self):
            self._joint.motorvel = 0
            self._remainingTime = self.cfg['timeout']
            self.status = IDLE

    def _onAngleChanged(self):
        if self.initialize_slots:
            return

        self.status = BUSY
        self._remainingTime = self.cfg['timeout']

        histop = self._joint.histop
        lostop = self._joint.lostop
        if self.angle > histop:
            self.angle = histop
        elif self.angle < lostop:
            self.angle = lostop


class Motor1(Motor):

    def _updateStatus(self):
        #print 'angle: ', self.angle
        #print 'motor vel: ', self._joint.motorvel
        #print 'motor angle: ', degrees(self._joint.angle)
        #print 'remaining time: ', self._remainingTime
        #print '-----------------------------------------'
        timedOut = self._checkTimeout()
        if self._checkMotorStopped():
            self._stopMotor()
        elif timedOut:
            self._stopMotor(timedOut)
        #if self._checkMotorStopped() or timedOut:
        #    self._stopMotor(timedOut)

    def _stopMotor(self, timedOut=False):
            self._joint.motorvel = 0
            if timedOut:
                self.status = TIMEOUT
            else:
                self.status = IDLE
            self._remainingTime = self.cfg['timeout']

class MotorCombiner(Motor):

    def __init__(self, 
                 world,
                 name = "Motor",
                 block = None, 
                 link = None,
                 angle = 0.0,
                 cfg = None,
                 data = None,
                 **params):
        self._initialized = False
        Motor.__init__(self, world, name=name, block=block, link=link, angle=angle, cfg=cfg, **params)
        self._motors = data['class']['motor']
        self._index = 0
        self.data = data
        motorClassName = self._motors[0]
        motorClass = getattr(__import__('mrsim.motor', None, None, ['']), motorClassName)
        self._current = motorClass
        self._initialized = True

    def adapt(self, index):
        self._index = index
        # create motor
        motorClassName = self._motors[index]
        motorClass = getattr(__import__('mrsim.motor', None, None, ['']), motorClassName)
        # update active motor
        self._current = motorClass

    def __getattribute__(self, name):
        initialized = Motor.__getattribute__(self, '_initialized')
        if not initialized:
            return Motor.__getattribute__(self, name)
        if name in Motor.__getattribute__(self, '__dict__'):
            return Motor.__getattribute__(self, name)
        elif name in ['adapt']:
            return Motor.__getattribute__(self, name)
        else:
            method = getattr(self._current, name)
            if hasattr(method, 'im_func'):
                value = curry(method.im_func, self)
            elif hasattr(method, 'fget'):
                value = curry(method.fget, self)()
            else:
                value = method
            return value

