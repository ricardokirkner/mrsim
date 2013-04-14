#!/usr/bin/python /usr/bin/viewer.py
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

from mrsim import Builder, BOTTOM, FRONT, REAR, EXPLORE, EXPLOIT, EPSILON_GREEDY
from cgkit.cmds import getScene
from cgkit.odedynamics import ODESliderJoint
from cgkit.keydefs import KEY_LEFT, KEY_RIGHT, KEY_UP, KEY_DOWN
import logging
import logging.config

logging.config.fileConfig('logconf.ini')
logger = logging.getLogger('mrsim')

def setup():
    buildScene()
    setupEventHandlers()
    scene = getScene()
    return scene.item('world')

def buildScene():
    builder = Builder('demo.scn', 'mrsim.cfg')
    builder.createScene()

def setupEventHandlers():
    def key(evt):
        global block
        robot = getScene().item('robot1')
        light0 = getScene().worldObject('light0')
        if evt.key == "1":
            if block is not None and hasattr(block, 'module'):
                block.module.link.motor[block.type].angle = -45
        #    module = robot.modules[0]
        #    module.link.motor['active'].angle = -90
        elif evt.key == "2":
            if block is not None and hasattr(block, 'module'):
                block.module.link.motor[block.type].angle = 0
        #    module = robot.modules[0]
        #    module.link.motor['active'].angle = 0
        elif evt.key == "3":
            if block is not None and hasattr(block, 'module'):
                block.module.link.motor[block.type].angle = 45
        #    module = robot.modules[0]
        #    module.link.motor['active'].angle = 90
        #elif evt.key == "4":
        #    module = robot.modules[0]
        #    module.link.motor['passive'].angle = -90
        #elif evt.key == "5":
        #    module = robot.modules[0]
        #    module.link.motor['passive'].angle = 0
        #elif evt.key == "6":
        #    module = robot.modules[0]
        #    module.link.motor['passive'].angle = 90
        #elif evt.key == "7":
        #    module = robot.modules[1]
        #    module.link.motor['active'].angle = -90
        #elif evt.key == "8":
        #    module = robot.modules[1]
        #    module.link.motor['active'].angle = 0
        #elif evt.key == "9":
        #    module = robot.modules[1]
        #    module.link.motor['active'].angle = 90
        #elif evt.key == "0":
        #    module = robot.modules[1]
        #    module.link.motor['passive'].angle = -90
        #elif evt.key == "-":
        #    module = robot.modules[1]
        #    module.link.motor['passive'].angle = 0
        #elif evt.key == "=":
        #    module = robot.modules[1]
        #    module.link.motor['passive'].angle = 90
        if evt.key == 'i':
            #print 'action-value function: ', robot.brain._Q 
            #print 'actions: ', robot.brain._Q.getActions(robot.state)
            print 'action: ', robot.brain.getAction(robot.state)
            print 'state: ', robot.state
            print 'state hash: ', robot.brain._Q.getStateHash(robot.state)
            print 'goal: ', robot.brain.goal
            try:
                obstacleSensors = [module.getObstacleSensorValue() for module in robot.modules]
                print 'obstacle sensors: ', obstacleSensors
            except:
                pass
        elif evt.key == 'f':
            module = robot.modules[1]
            fixate(module.active)
        elif evt.key == 'd':
            mod0 = robot.modules[1]
            mod1 = robot.modules[0]
            mod0.dock(BOTTOM, mod1)
            mod0.dock(FRONT, mod1)
            mod0.dock(REAR, mod1)
        elif evt.key == 'u':
            module = block.module
            module.undock(BOTTOM)
            module.undock(FRONT)
            module.undock(REAR)
        elif evt.key == 'l':
            robot.interactive = not robot.interactive
            logger.info('interactive mode is %s' % str(robot.interactive).upper())
        elif evt.key == 'D':
            robot.brain.dump('brain.dump')
        elif evt.key == 'L':
            robot.brain.restore('brain.dump')
        elif evt.key == 'r':
            world.reset()
        elif evt.key == 'v':
            for module in robot.modules:
                module.active.child('Mesh').visible = not module.active.child('Mesh').visible
                module.active.visible = not module.active.visible
                module.passive.child('Mesh').visible = not module.passive.child('Mesh').visible
                module.passive.visible = not module.passive.visible
                module.link.child('Box').visible = not module.link.child('Box').visible
                module.link.visible = not module.link.visible
        elif evt.key == 'x':
            robot.brain.setPolicyMode(EXPLORE)
            logger.info("Using EXPLORE policy")
        elif evt.key == 't':
            robot.brain.setPolicyMode(EXPLOIT)
            logger.info("Using EXPLOIT policy")
        elif evt.key == 'g':
            robot.brain.setPolicyMode(EPSILON_GREEDY)
            logger.info("Using EPSILON_GREEDY policy")
        elif evt.keycode == KEY_RIGHT:
            if block is not None and hasattr(block, 'module'):
                block.module.link.motor[block.type].angle += 90
                logger.info('new angle: %s' % block.module.link.motor[block.type].angle)
        #    light0.pos += vec3(-0.06,0,0)
        elif evt.keycode == KEY_LEFT:
            if block is not None and hasattr(block, 'module'):
                block.module.link.motor[block.type].angle += -90
                logger.info('new angle: %s' % block.module.link.motor[block.type].angle)
        #    light0.pos += vec3(0.06,0,0)
        #elif evt.keycode == KEY_UP:
        #    light0.pos += vec3(0,-0.06,0)
        #elif evt.keycode == KEY_DOWN:
        #    light0.pos += vec3(0,0.06,0)
        elif evt.key == 'b':
            logger.info(light0.pos)
        elif evt.key == 'n':
            idx = robot._index
            new_idx = (idx + 1) % 4
            robot.adapt(new_idx)

    def mouse(evt):
        renderer = getApp().renderer
        hits = renderer.pick(evt.x, evt.y, 2, 2)
        if hits:
            global block
            block = hits[0][2]
            

    eventmanager.connect(KEY_PRESS, key)
    eventmanager.connect(LEFT_DOWN, mouse)

def fixate(obj):
    global world
    sj = ODESliderJoint(body1=obj)
    sj.lostop = 0
    sj.histop = 0
    world.add(sj)

#==============================================================================

global world
global block
world = setup()
block = None
#listWorld()

#print "bodies: ", [body.obj.name for body in world.bodies]
robot = getScene().item('robot1')
# pause movemente at start
robot.interactive = True

