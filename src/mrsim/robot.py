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
import sys
from cgkit.Interfaces import *
from cgkit.cmds import getScene
from cgkit.component import Component
from cgkit.eventmanager import eventManager
from cgkit.events import STEP_FRAME
from math import degrees
from brain import RLBrain, ProgramBrain, PolicyCombiner
from module import Module
from constants import IDLE, BUSY, TIMEOUT, EPSILON_GREEDY, EXPLORE, EXPLOIT, ACTIVE, PASSIVE
from util import allIdle, getStatus, removeObject, curry
import logging

logger = logging.getLogger('mrsim.robot')

class Robot(Component):

    protocols.advise(instancesProvide=[ISceneItem, ISceneItemContainer])
    
    #
    #=== Public Methods =======================================================
    #
    #
    #--- Creation -------------------------------------------------------------
    #
    def __init__(self,
                 world,
                 name="Robot",
                 cfg = None,
                 data = None):

        Component.__init__(self, name)

        self.world = world
        self.modules = []
        self.cfg = cfg
        self.brain = None
        self.interactive = True
        self._preState = None
        self._action = None
        self._actionDone = True
        self._steps = 0
        self._initialPos = 0
        self._totalWalked = 0
        self._timer = getScene().timer()
        self._beforeExecution = self._timer.time

        eventManager().connect(STEP_FRAME, self._onStepFrame)

    #
    #--- State information ----------------------------------------------------
    #
    def _getState(self):
        value = [module.state for module in self.modules]
        return value
    state = property(_getState)

    def _getStatus(self):
        moduleStatus = [module.status for module in self.modules]
        robotStatus = allIdle(moduleStatus) and IDLE or BUSY
        return robotStatus
    status = property(_getStatus)

    def _getPos(self):
        states = [0.5 * (module.active.state + module.passive.state) for module in self.modules]
        return min(states)
    pos = property(_getPos)

    #
    #--- Learning -------------------------------------------------------------
    #
    def initBrain(self, policy_mode, brainClassName='RLBrain', goal=None, action_value_function_filename=None, program=[]):
        # create a brain
        args = {}
        brainClass = getattr(__import__('mrsim.brain', None, None, ['']), brainClassName)

        if issubclass(brainClass, ProgramBrain):
            args['program'] = program
        if issubclass(brainClass, RLBrain):
            alpha = self.cfg['brain']['alpha']
            gamma = self.cfg['brain']['gamma']
            epsilon = self.cfg['brain']['epsilon']
            args['alpha'] = alpha
            args['gamma'] = gamma
            args['epsilon'] = epsilon
            args['action_value_function_filename'] = action_value_function_filename
            args['goal'] = goal

        self.brain = brainClass(self, **args)
        self.brain.setPolicyMode(policy_mode)
        self.interactive = policy_mode not in (EPSILON_GREEDY, EXPLORE, EXPLOIT)

    def getAvailableActions(self):
        def powerset(l):
            r = []
            if l:
                head, tail = l[0], l[1:]
                ps = powerset(tail)
                for e in ps:
                    r.append(e)
                    ep = list(e)
                    ep.append(head)
                    r.append(ep)
            else:
                r.append([])
            return r

        moduleCount = len(self.modules)
        angles = ['90']
        B = ['%s|%s' % (module, block) for module in range(moduleCount) for block in ('A', 'P')]
        A = ['%s%s' % (sign, angle) for sign in ('+', '-') for angle in angles]
        P = powerset(B)
        P.remove([])
        actions = []

        for i in P:
            var_use = [chr(ord('a') + a) for a in range(len(i))]
            var_def = ('for %s in A ' * len(var_use)) % tuple(var_use)
            template = ','.join(map(lambda x: '%s|%s' % x, zip(i, ['%s'] * len(i))))
            expr = '[template % (' + ','.join(var_use) + ') ' + var_def + ']'
            actions.extend(eval(expr))

        return actions

    #
    #--- ISceneItemContainer interface ----------------------------------------
    #
    def lenChilds(self):
        return len(self.modules)

    def iterChilds(self):
        return iter(self.modules)

    def addChild(self, child):
        if not isinstance(child, Module):
            raise TypeError("A Robot object can only have Module objects as children.", child)
        if self.findChildByName(child.name) is not None:
            raise KeyError("Object '%s' already has a children called '%s'." % (self.name, child.name))
        child.robot = self
        self.modules.append(child)

    def removeChild(self, child):
        if not isinstance(child, Module):
            raise TypeError("A Robot object can only have Module objects as children.", child)
        self.modules.remove(child)

    def findChildByName(self, name):
        result = None
        for child in self.modules:
            if child.name == name:
                result = child
                break
        return result

    #
    #=== Private Methods ======================================================
    #
    #
    #--- Learning -------------------------------------------------------------
    #
    def _onStepFrame(self):
        if not self.interactive:
            if self._actionDone:
                logger.debug("state: %s" % str(self.state))
                # goal reached? simulation should stop?
                self._testGoalReached()
                # get next action
                action = self._getNextAction()
                # execute action
                self._beforeExecution = self._timer.time
                self._executeAction(action)
            self._testActionDone()

    def _testGoalReached(self):
        if self.brain.goalReached():
        #if self.pos > 0.85 * self.cfg['module']['block']['sensor']['buckets'] or self._steps > 200:
            # goal reached
            self._showResults()
        #elif hasattr(self.brain, 'shouldAbort') and self.brain.shouldAbort():
        #    self.world.reset()
        else:
            if self._initialPos is None:
                self._initialPos = self.pos
            self._steps += 1
            logger.info('steps: %s' % self._steps)

    def _getNextAction(self):
        # get next action and store preState and current action
        # for later retrieval
        self._preState = self.state
        self._action = self.brain.getAction(self._preState)
        return self._action

    def _executeAction(self, action):
        self._actionDone = False
        motions = action.split(',')
        for motion in motions:
            logger.debug('motion: %s' % motion)
            parts = motion.split('|')
            if parts[0] in ('dock', 'undock'):
                action, moduleAIdx, moduleBIdx, side = parts 
                moduleA = self.modules[int(moduleAIdx)]
                if action == 'dock':
                    moduleB = self.modules[int(moduleBIdx)]
                    moduleA.dock(side, moduleB)
                else:
                    moduleA.undock(side)
            else:
                moduleIdx, blockId, angleDelta = parts
                module = self.modules[int(moduleIdx)]
                if blockId == 'A':
                    block = module.active
                elif blockId == 'P':
                    block = module.passive
                else:
                    raise ValueError("Invalid block id (%s)." % blockId)
                motor = module.link.motor[block.type]
                motor.angle += int(angleDelta)

    def _testActionDone(self):
        if self.status == IDLE:
            logger.debug("ACTION DONE")
            self._evaluatePerformance()
            self._actionDone = True

    def _evaluatePerformance(self):
        now = self._timer.time
        elapsed = now - self._beforeExecution
        logger.info('elapsed time: %s' % elapsed)
        logger.debug("state: %s" % str(self._preState))
        logger.debug("action: %s" % str(self._action))
        if self.brain.getPolicyMode() == EPSILON_GREEDY:
            self.brain.update(self._preState, self._action)
        self._updateStats()

    def _showResults(self):
        policy_mode = self.brain.getPolicyMode()
        if policy_mode == EPSILON_GREEDY:
            self.brain.dump()
        elif policy_mode in (EXPLORE, EXPLOIT):
            avanzada = self.pos - self._initialPos
            recorrida = self._totalWalked
            acciones = self._steps
            buckets = self.cfg['module']['block']['sensor']['buckets']
            initialDistance = buckets - self._initialPos
            minimumActions = initialDistance 
            try:
                performance = (float(avanzada) / recorrida) * (minimumActions / float(acciones))
            except:
                performance = -999999
            #print "RESULT:Distancia avanzada:%s" % avanzada
            #print "RESULT:Distancia recorrida:%s" % recorrida
            #print "RESULT:Acciones realizadas:%s" % acciones
            #print "RESULT:Performance:%s" % performance
            print "%s" % avanzada
            print "%s" % recorrida
            print "%s" % acciones
            print "%s" % performance
        #self.interactive = True
        print "Acciones realizadas: ", self._steps
        sys.exit(0)

    def _updateStats(self):
        prePos = min([state[0] for state in self._preState])
        walked = abs(self.pos - prePos)
        self._totalWalked += walked


####################################################
# Custom Robots
####################################################

class Robot1(Robot):

    def _getState(self):
        angles = [[module.link.motor['active'].angle, module.link.motor['passive'].angle] for module in self.modules]
        docks = []
        for moduleAIdx in range(len(self.modules)):
            neighbors = self.modules[moduleAIdx].active.neighbor.items()
            for side, neighbor in neighbors:
                try:
                    moduleBIdx = self.modules.index(neighbor['block'].module)
                    docks.append((moduleAIdx, moduleBIdx, side))
                except:
                    print "Loose module detected. Should abort."
                    docks.append((moduleAIdx, None, None))
        state = [angles, docks]
        return state
    state = property(_getState)

    def _updateStats(self):
        pass


class Robot2(Robot):

    def _getPos(self):
        states = [max(module.active.state, module.passive.state) for module in self.modules]
        return max(states)
    pos = property(_getPos)


class Robot3(Robot2):

    def _executeAction(self, action):
        if not self.legalAction(action):
            self._actionDone = True
            return

        self._actionDone = False
        motions = action.split(',')
        for motion in motions:
            logger.debug('motion: %s' % motion)
            parts = motion.split('|')
            if parts[0] in ('dock', 'undock'):
                action, moduleAIdx, moduleBIdx, side = parts 
                moduleA = self.modules[int(moduleAIdx)]
                if action == 'dock':
                    moduleB = self.modules[int(moduleBIdx)]
                    moduleA.dock(side, moduleB)
                else:
                    moduleA.undock(side)
            else:
                moduleIdx, blockId, angleDelta = parts
                module = self.modules[int(moduleIdx)]
                if blockId == 'A':
                    block = module.active
                elif blockId == 'P':
                    block = module.passive
                else:
                    raise ValueError("Invalid block id (%s)." % blockId)
                motor = module.link.motor[block.type]
                motor.angle += int(angleDelta)

    def legalAction(self, action):
        internal_sum = 180 * (len(self.modules) - 2)
        # get current internal angles
        motorAngles = []
        for module in self.modules:
            #activeAngle = round(degrees(module.link.motor[ACTIVE]._joint.angle), 0)
            #passiveAngle = round(degrees(module.link.motor[PASSIVE]._joint.angle), 0)
            activeAngle = module.link.motor[ACTIVE].angle
            passiveAngle = module.link.motor[PASSIVE].angle
            motorAngles.append(activeAngle)
            motorAngles.append(passiveAngle)

        if not self.legalMotorAngles(motorAngles):
            return False

        angles = []
        for i in range(len(self.modules)):
            angle = 180 + motorAngles[2*i] \
                        - motorAngles[2*i + 1]
            angles.append(angle)

        #if not self.legalAngles(angles):
        #    return False

        angles_before = sum(angles)
        if abs(internal_sum - angles_before) < 5:
            angles_before = internal_sum
        #print angles
        #print angles_before
        #print action

        # get next internal angles
        motions = action.split(',')
        for motion in motions:
            parts = motion.split('|')
            if parts[0] not in ('dock', 'undock'):
                moduleIdx, blockId, angleDelta = parts
                angleIdx = 2*int(moduleIdx)
                if blockId == 'P':
                    angleIdx += 1
                motorAngles[angleIdx] += int(angleDelta)
        
        if not self.legalMotorAngles(motorAngles):
            return False

        angles = []
        for i in range(len(self.modules)):
            angle = 180 + motorAngles[2*i] \
                        - motorAngles[2*i + 1]
            angles.append(angle)

        #if not self.legalAngles(angles):
        #    return False

        angles_after = sum(angles)
        if abs(internal_sum - angles_after) < 5:
            angles_after = internal_sum
        #print angles
        #print angles_after
        #print
        legalAction = angles_before == angles_after == internal_sum
        logger.info('action: %s is legal? %s' % (action, legalAction))
        logger.info('angles_before: %s, angles_after: %s' % (angles_before, angles_after))

        # assert sum remains equal
        return legalAction

    def _getStatus(self):
        moduleStatus = [module.status for module in self.modules]
        robotStatus = getStatus(moduleStatus)
        return robotStatus
    status = property(_getStatus)

    def _testActionDone(self):
        if self.status == IDLE:
            logger.debug("ACTION DONE")
            self._evaluatePerformance()
            self._actionDone = True
        elif self.status == TIMEOUT:
            logger.debug("ACTION TIMED OUT. RESETTING.")
            self._evaluatePerformance()
            #self._resetAction()
            self._actionDone = True
            self.resetStatus(IDLE)

    def resetStatus(self, status):
        for module in self.modules:
            module.resetStatus(status)

    def _resetAction(self):
        last_action = self._action
        motions = last_action.split(',')
        counter_motions = []
        for motion in motions:
            parts = motion.split('|')
            if parts[0] not in ('dock', 'undock'):
                module, block, angle = parts
                sign = angle[0]
                if sign == '+':
                    angle = '-' + angle[1:]
                else:
                    angle = '+' + angle[1:]
                counter_motion = '|'.join([module, block, angle])
                counter_motions.append(counter_motion)
        counter_action = ','.join(counter_motions)
        self._executeAction(counter_action)

    def legalMotorAngles(self, motorAngles):
        legalAngles = [-90, 0, 90]
        #legalAngles = [-90, -45, -10, 0, 10, 45, 90]
        legalMotorAngles = reduce(lambda x,y: x and y, map(lambda x: x in legalAngles, motorAngles))
        legalMotorAngleSum = sum(motorAngles) == 0
        return legalMotorAngles and legalMotorAngleSum

    def legalAngles(self, angles):
        #legalAngles = reduce(lambda x,y: x and y, map(lambda x: abs(x) > 0, angles))
        angleCount = {}
        for angle in angles:
            absAngle = abs(angle)
            count = angleCount.setdefault(absAngle, 0)
            angleCount[absAngle] = count + 1
        legalAngles = angleCount.get(0, 0) < 2
        return legalAngles

class Robot4(Robot3):

    def legalAction(self, action):
        # get current internal angles
        motorAngles = []
        for module in self.modules:
            activeAngle = module.link.motor[ACTIVE].angle
            passiveAngle = module.link.motor[PASSIVE].angle
            motorAngles.append(activeAngle)
            motorAngles.append(passiveAngle)

        legalState = self.legalMotorAngles(motorAngles)
        if not legalState:
            logger.info("legalState FALSE. motorAngles: %s" % str(motorAngles))
            return legalState

        # get next internal angles
        motions = action.split(',')
        for motion in motions:
            parts = motion.split('|')
            if parts[0] not in ('dock', 'undock'):
                moduleIdx, blockId, angleDelta = parts
                angleIdx = 2*int(moduleIdx)
                if blockId == 'P':
                    angleIdx += 1
                motorAngles[angleIdx] += int(angleDelta)
        
        legalState = self.legalMotorAngles(motorAngles)
        if not legalState:
            logger.info("legalState FALSE. motorAngles: %s" % str(motorAngles))
        return legalState

    def legalMotorAngles(self, motorAngles):
        legalAngles = [-90, 0, 90]
        legalMotorAngles = reduce(lambda x,y: x and y, map(lambda x: x in legalAngles, motorAngles))
        return legalMotorAngles

class Robot5(Robot4):

    def legalMotorAngles(self, motorAngles):
        legalAngles = [-45, 0, 45]
        legalMotorAngles = reduce(lambda x,y: x and y, map(lambda x: x in legalAngles, motorAngles))
        #legalMotorAngleSum = sum(motorAngles) == 0
        #return legalMotorAngles and legalMotorAngleSum
        return legalMotorAngles

class Robot6(Robot3):

    def legalMotorAngles(self, motorAngles):
        legalAngles = [-90, 0, 90]
        legalMotorAngles = reduce(lambda x,y: x and y, map(lambda x: x in legalAngles, motorAngles))
        return legalMotorAngles

class Robot7(Robot2):

    def _getState(self):
        value = Robot1._getState.im_func(self)
        return value
    state = property(_getState)

    def _updateStats(self):
        pass        

class RobotCombiner(Robot):

    def __init__(self,
                 world,
                 name="RobotCombiner",
                 cfg = None,
                 data = None):
        self._initialized = False
        Robot.__init__(self, world, name=name, cfg=cfg)
        self._robots = data['class']['robot']
        self._index = 0
        self.data = data
        robotClassName = self._robots[self._index]
        robotClass = getattr(__import__('mrsim.robot', None, None, ['']), robotClassName)
        self._current = robotClass
        self.policies = {0: {'[0, 0]': 0,
                             '[0, 1]': 0,
                             '[1, 0]': 1,
                             '[1, 1]': 1,
                             'next': 0},
                         1: {'[0, 0]': 1,
                             '[0, 1]': 1,
                             '[1, 0]': 1,
                             '[1, 1]': 1,
                             'next': 2},
                         2: {'[0, 0]': 2,
                             '[0, 1]': 2,
                             '[1, 0]': 2,
                             '[1, 1]': 2,
                             'next': 3},
                         3: {'[0, 0]': 4,
                             '[0, 1]': 3,
                             '[1, 0]': 3,
                             '[1, 1]': 3,
                             'next': 3},
                         4: {'[0, 0]': 4,
                             '[0, 1]': 4,
                             '[1, 0]': 4,
                             '[1, 1]': 4,
                             'next': 5},
                         5: {'[0, 0]': 5,
                             '[0, 1]': 5,
                             '[1, 0]': 5,
                             '[1, 1]': 5,
                             'next': 0},
                         6: {'[0, 0]': 6,
                             '[0, 1]': 6,
                             '[1, 0]': 6,
                             '[1, 1]': 6,
                             'next': 0}}
        self._initialized = True

    def initBrain(self, policy_mode, brainClassName='RLBrain', goal=None, action_value_function_filename=None, program=[]):
        self._brains = brainClassName
        self._files = action_value_function_filename
        self.policy_mode = policy_mode
        self._goal = goal
        self.program = program
        initBrainFunc = self._current.initBrain.im_func
        initBrainFunc(self, policy_mode, self._brains[self._index], goal=self._goal[self._index], action_value_function_filename=self._files[self._index], program=program)

    def adapt(self, index):
        self._index = index
        # create robot
        robotClassName = self._robots[index]
        robotClass = getattr(__import__('mrsim.robot', None, None, ['']), robotClassName)
        # adapt modules
        for module in self.modules:
            module.adapt(index)
        # update active robot
        self._current = robotClass
        # initialize brain
        initBrainFunc = self._current.initBrain.im_func
        initBrainFunc(self, self.policy_mode, self._brains[index], self._goal[index], self._files[index], self.program)
        # refresh preState
        self._preState = self.state
        logger.info('robot class: %s' % self._current)
        logger.info('module class: %s' %  self.modules[0]._current)
        logger.info('motor class: %s' % self.modules[0].link.motor['active']._current)

    def _testGoalReached(self):
        
        states = [max(module.active.state, module.passive.state) for module in self.modules]
        max_state = max(states)
        min_state = min(states)
        idx_max_state = states.index(max_state)
        idx_min_state = states.index(min_state)
        obstacleSensorValues = [self.modules[idx_max_state].getObstacleSensorValue(), self.modules[idx_min_state].getObstacleSensorValue()]
        logger.debug('obstacleSensorValues: %s' % obstacleSensorValues)
        
        new_idx = self.policies[self._index][str(obstacleSensorValues)]
        if new_idx != self._index:
            self.adapt(new_idx)
        else:
            if self.brain.goalReached():
                # goal reached
                new_idx = self.policies[self._index]['next']
                if new_idx == -1:
                    # last brain, so end simulation
                    self._showResults()
                else:
                    # next brain
                    self.adapt(new_idx)
            else:
                self._initialPos = self.pos
                self._steps += 1
                logger.info('steps: %s' % self._steps)

    # proxy robot methods
    def __getattribute__(self, name):
        initialized = Robot.__getattribute__(self, '_initialized')
        if not initialized:
            return Robot.__getattribute__(self, name)
        if name in Robot.__getattribute__(self, '__dict__'):
            return Robot.__getattribute__(self, name)
        elif name in ['adapt', 'initBrain', '_testGoalReached']:
            return Robot.__getattribute__(self, name)
        else:
            method = getattr(self._current, name)
            if hasattr(method, 'im_func'):
                value = curry(method.im_func, self)
            elif hasattr(method, 'fget'):
                value = curry(method.fget, self)()
            else:
                value = method
            return value

