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

import random
import time
import cPickle
import logging
import pprint
from constants import EXPLORE, EXPLOIT, EPSILON_GREEDY, BOTTOM, FRONT, REAR, TIMEOUT

logger = logging.getLogger('mrsim.brain')

class Brain(object):

    def __init__(self):
        object.__init__(self)

    def getAction(self, state):
        raise NotImplementedError

    def update(self, state, action):
        raise NotImplementedError

    def setPolicyMode(self, policy_mode):
        raise NotImplementedError

    def getPolicyMode(self):
        raise NotImplementedError

    def goalReached(self):
        raise NotImplementedError


class RandomBrain(Brain):

    def __init__(self, body):
        Brain.__init__(self)
        self.body = body
        self.actions = self.body.getAvailableActions()

    def getAction(self, state):
        return random.choice(self.actions)

    def update(self, state, action):
        pass

    def setPolicyMode(self, policy_mode):
        pass

    def getPolicyMode(self):
        return 'random'

    def goalReached(self):
        pass


class RLBrain(Brain):

    def __init__(self, body, alpha=0.2, gamma=0.5, epsilon=0.5, action_value_function_filename=None, goal=None, action_value_function_class='ActionValueFunction'):
        Brain.__init__(self)
        self.body = body
        self.alpha = alpha
        self.gamma = gamma
        self.action_value_function_filename = action_value_function_filename
        #self.actions = self.body.getAvailableActions()
        self.actions = self.getAvailableActions()
        actionValueFunctionClass = eval(action_value_function_class)
        self._Q = actionValueFunctionClass(self.actions, data_filename=action_value_function_filename, default=0)
        self._P = Policy(self._Q, epsilon=epsilon)

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

        moduleCount = len(self.body.modules)
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

    def getAction(self, state):
        action = self._P.getAction(state)
        stateHash = self._Q.getStateHash(state)
        logger.debug("state: %s" % stateHash)
        logger.debug("action: %s" % action)
        return action

    def update(self, state, action):
        rewards = self.getReward(state, action) + self.gamma * self.getMaximalTemporalDifference(state, action)
        self._Q[state, action] = self._Q[state, action] + self.alpha * rewards
        self._P.actionValueFunction = self._Q

    def getReward(self, state, action):
        raise NotImplementedError

    def getMaximalTemporalDifference(self, state, action):
        maxValue = None
        nextState = self.body.state
        for nextAction in self.actions:
            if maxValue is None:
                maxValue = self._Q[nextState, nextAction] - self._Q[state, action]
            else:
                value = self._Q[nextState, nextAction] - self._Q[state, action]
                maxValue = max(maxValue, value)
        if maxValue is None:
            maxValue = 0
        return maxValue

    def dump(self, fileName=None):
        if fileName is None:
            if self.action_value_function_filename is None:
                timestamp = time.time()
                fileName = 'brain_a%s_e%s_g%s_%s.dump' % (self.alpha, self._P.epsilon, self.gamma, timestamp)
            else:
                fileName = self.action_value_function_filename
        self._Q.dump(fileName)

    def restore(self, fileName=None):
        if fileName is None:
            if self.action_value_function_filename is None:
                fileName = 'brain_a%s_e%s_g%s.dump' % (self.alpha, self._P.epsilon, self.gamma)
            else:
                fileName = self.action_value_function_filename
        self._Q.restore(fileName)

    def setPolicyMode(self, mode):
        self._P.mode = mode

    def getPolicyMode(self):
        return self._P.mode


class ActionValueFunction(object):

    def __init__(self, actions, data_filename=None, default=0):
        object.__init__(self)
        self.default = default
        self.actions = actions
        if data_filename is not None:
            try:
                self.restore(data_filename)
            except:
                print "Error restoring from %s. Using default data (empty)" % data_filename
                self._data = {}
        else:
            self._data = {}
        self._defaultActions = dict.fromkeys(self.actions, self.default)
        #self._refreshOptimalActions()

    def __getitem__(self, stateAction):
        state, action = stateAction
        stateHash = self.getStateHash(state)
        actionHash = self.getActionHash(action)
        actions = self._data.get(stateHash)
        if actions is not None:
            value = actions.get(actionHash, self.default)
        else:
            value = self.default
        return value

    def __setitem__(self, stateAction, value):
        state, action = stateAction
        stateHash = self.getStateHash(state)
        actionHash = self.getActionHash(action)
        actions = self._data.get(stateHash)
        if value != self.default:
            if actions is not None:
                actions[actionHash] = value
            else:
                self._data[stateHash] = {actionHash: value}
        elif actions is not None and actions.has_key(actionHash):
            del actions[actionHash]

        #self._updateValues(stateHash, actionHash, value)

    def getActions(self, state):
        stateHash = self.getStateHash(state)
        stateActions = self._data.get(stateHash, {})
        actions = self._defaultActions.copy()
        actions.update(stateActions)
        return actions

#    def getActions(self, state):
#        #logger.info("actions: %s" % str(self.actions))
#        return self.actions

#    def getOptimalActions(self, state):
#        stateHash = self.getStateHash(state)
#        optimalActions = self._optimalActions.get(stateHash, self.actions)
#        return list(optimalActions)

#    def _updateValues(self, state, action, value):
#        optimalValue = self._optimalValue.get(state, self.default)
#        if value > optimalValue:
#            self._optimalValue[state] = value
#            self._optimalActions[state] = set([action])
#        elif value == optimalValue:
#            optimalActions = self._optimalActions.get(state, set())
#            optimalActions.add(action)
#            self._optimalActions[state] = optimalActions

#    def _refreshOptimalActions(self):
#        self._optimalActions = {}
#        self._optimalValue = {}
#        for state in self._data.keys():
#            actions = self._defaultActions.copy()
#            actions.update(self._data[state])
#            optimalValue = None
#            optimalActions = set()
#            for action in actions:
#                value = actions[action]
#                if optimalValue is None:
#                    optimalValue = value
#                    optimalActions = set([action])
#                else:
#                    if value > optimalValue:
#                        optimalValue = value
#                        optimalActions = set([action])
#                    elif value == optimalValue:
#                        optimalActions.add(action)
#            self._optimalActions[state] = optimalActions
#            self._optimalValue[state] = optimalValue

    def getStates(self):
        return self._data.keys()

    def getStateHash(self, state):
        stateHash = []
        for moduleState in state:
            moduleLightValue, moduleRotationValue, moduleDirection, activeBlockAngle, passiveBlockAngle = moduleState
            hash = "%s::%s::%s::%s" % (moduleRotationValue, moduleDirection, activeBlockAngle, passiveBlockAngle)
            #hash = "%s::%s" % (activeBlockAngle, passiveBlockAngle)
            stateHash.append(hash)
        return str(stateHash)

    def getActionHash(self, action):
        return str(action)

    def _getBlockStateHash(self, blockState):
        pos = "%d,%d,%d" % blockState['position'].tuple
        rot = "%d,%d,%d,%d,%d,%d,%d,%d,%d" % blockState['rotation'].tuple
        ang = "%d" % blockState['angle']
        des = "%d" % blockState['desiredAngle']
        nei = []
        for neighbor in blockState['neighbors']:
            side = neighbor['side']
            id = neighbor['block'].id
            nei.append("%s,%s" % (side[0].upper(), id[0].upper()))
        #state = "%s|%s|%s|%s|%s" % (pos, rot, ang, des, ';'.join(nei))
        stateHash = "%d" % blockState['position'].x
        return stateHash

    def _getModuleStateHash(self, moduleState):
        state = []
        for blockId in moduleState['block']:
            blockStateHash = self._getBlockStateHash(moduleState['block'][blockId])
            state.append("%c|%s" % (blockId[0].upper(), blockStateHash))
        stateHash = '<=>'.join(state)
        return stateHash

    def dump(self, fileName):
        fileObj = open(fileName, 'w')
        #backupFileObj = open(fileName + '.%s' % time.time(), 'w')
        cPickle.dump(self, fileObj)
        #cPickle.dump(self, backupFileObj)

    def restore(self, fileName):
        fileObj = open(fileName, 'r')
        cfg = cPickle.load(fileObj)
        self.default = cfg.default
        self.actions = cfg.actions
        self._data = cfg._data

    def __str__(self):
        str = pprint.pformat(self._data)
        return str


class Policy(object):

    def __init__(self, actionValueFunction, seed=None, epsilon=0.5, iterations=200):
        object.__init__(self)
        if seed is None:
            #self.seed = 42
            seed = time.time()
            logger.info('Seed: %s' % seed)
        random.seed(seed)
        self.actionValueFunction = actionValueFunction
        self.epsilon = epsilon
        self.iterations = iterations
        self.epsilon_discount = float(self.epsilon) / self.iterations
        self.mode = EPSILON_GREEDY

    def getAction(self, state):
        actions = self.actionValueFunction.getActions(state)
        logger.debug("actions: %s" % str(actions))
        if self.mode == EXPLORE:
            optimalActions = actions.keys()
        else:
            optimalValue = None
            optimalActions = []
            for action in actions:
                try:
                    actionValue = self.actionValueFunction[state, action]
                    if optimalValue < actionValue:
                        optimalValue = actionValue
                        optimalActions = [action]
                    elif optimalValue == actionValue:
                        optimalActions.append(action)
                except:
                    optimalValue = self.actionValueFunction[state, action]
                    optimalActions = [action]
        
            if self.mode == EXPLOIT:
                pass
            elif self.mode == EPSILON_GREEDY:
                if random.random() < self.epsilon:
                    logger.info("RANDOM %s" %  self.epsilon)
                    optimalActions = actions.keys()
                if self.epsilon > self.epsilon_discount:
                    self.epsilon -= self.epsilon_discount
                    self.iterations -= 1
            else:
                raise ValueError("invalid policy type (%s)" % self.mode)
        optimalAction = random.choice(optimalActions)
        # alter epsilon by each iteration
        return optimalAction

#    def getAction(self, state):
#        if self.mode == EXPLORE:
#            optimalActions = self.actionValueFunction.getActions(state)
#        else:
#            optimalActions = self.actionValueFunction.getOptimalActions(state)
#        
#            if self.mode == EXPLOIT:
#                pass
#            elif self.mode == EPSILON_GREEDY:
#                if random.random() < self.epsilon:
#                    logger.debug("RANDOM %s" %  self.epsilon)
#                    optimalActions = self.actionValueFunction.getActions(state)
#                if self.epsilon > self.epsilon_discount:
#                    self.epsilon -= self.epsilon_discount
#                    self.iterations -= 1
#            else:
#                raise ValueError("invalid policy type (%s)" % self.mode)
#        optimalAction = random.choice(optimalActions)
#        logger.debug("optimalActions: %s" % str(optimalActions))
#        logger.debug("optimalValue: %s" % self.actionValueFunction[state, optimalAction])
#        return optimalAction


class ProgramBrain(Brain):

    def __init__(self, body, program=[]):
        Brain.__init__(self)
        self._programActions = iter(program)

    def getAction(self, state):
        return self._programActions.next()

    def update(self, state, action):
        pass

    def setPolicyMode(self, policy_mode):
        pass

    def getPolicyMode(self):
        return 'program'

    def goalReached(self):
        return len(self._programActions) == 0


class PolicyLearner(RLBrain, ProgramBrain):

    def __init__(self, body, alpha=0.2, gamma=0.5, epsilon=0.5, action_value_function_filename=None, goal=None, action_value_function_class='ActionValueFunction1', program=[]):
        RLBrain.__init__(self, body, alpha=alpha, gamma=gamma, epsilon=epsilon, action_value_function_filename=action_value_function_filename, goal=goal, action_value_function_class=action_value_function_class)
        ProgramBrain.__init__(self, body, program=program)

    def getAction(self, state):
        if self.getPolicyMode() == EXPLOIT:
            return RLBrain.getAction(self, state)
        else:
            return ProgramBrain.getAction(self, state)

    def update(self, state, action):
        RLBrain.update(self, state, action)

    def getReward(self, state, action):
        return 1

    def goalReached(self):
        if self.getPolicyMode() == EXPLOIT:
            state = self.body.state[0][0]
            if state > 0.85 * self.body.cfg['module']['block']['sensor']['buckets']:
                print "Goal Reached. Stopping learning."
                return True
            elif self.body._steps > 200:
                print "Exceeded maximum steps allowed. Aborting."
                return True
            else:
                print "Current Sensor State: ", state
                return False
        else:
            return ProgramBrain.goalReached(self)

class PolicyCombiner(Brain):

    def __init__(self, body, policy_data=[], epsilon=0.5):
        Brain.__init__(self)
        self._brain = []
        self._robot = []
        for data in policy_data:
            filename, brainClassName, robotClassName, moduleClassName, motorClassName = data
            brainClass = eval(brainClassName)
            brain = brainClass(self.body, action_value_function_filename=filename)
            brain.setPolicyMode(EXPLOIT)
            self._brain.append(brain)
            self._robot.append(robot)
        self._current = 0

    def getAction(self, state):
        currentPolicy = self.getCurrentPolicy()
        action = currentPolicy.getAction(state)
        return action

    def getCurrentPolicy(self):
        return self._brain[self._current]

    def update(self, state, action):
        pass

    def setPolicyMode(self, policy_mode):
        pass

    def getPolicyMode(self):
        return 'combiner'

    def goalReached(self):
        robot = self._robot[self._current]
        if robot.body.pos > 0.85 * robot.body.cfg['module']['block']['sensor']['buckets']:
            print "Goal Reached. Stopping learning."
            return True
        elif robot.body._steps > 200:
            print "Exceeded maximum steps allowed. Aborting."
            return True
        else:
            return False


###############################
# Custom ActionValueFunctions
###############################

class ActionValueFunction1(ActionValueFunction):

    def getStateHash(self, state):
        stateHash = []
        for moduleState in state:
            moduleLightValue, moduleRotationValue, moduleDirection, activeBlockAngle, passiveBlockAngle = moduleState
            #hash = "%s::%s::%s::%s" % (moduleRotationValue, moduleDirection, activeBlockAngle, passiveBlockAngle)
            hash = "%s::%s" % (activeBlockAngle, passiveBlockAngle)
            stateHash.append(hash)
        return str(stateHash)


class ActionValueFunction2(ActionValueFunction):

    def getStateHash(self, state):
        angles, docks = state
        anglesHash = []
        for angle in angles:
            anglesHash.append('%s::%s' % (angle[0], angle[1]))
        stateHash = (anglesHash, docks)
        return str(stateHash)

        
###############################
# Custom Brains
###############################

class LightSeeker(RLBrain):
    
    def __init__(self, body, alpha=0.5, gamma=0.5, epsilon=0.5, action_value_function_filename=None, goal=None):
        RLBrain.__init__(self, body, alpha=alpha, gamma=gamma, epsilon=epsilon, action_value_function_filename=action_value_function_filename)
        self.actionCount = 0

    def goalReached(self):
        if self.body.pos > 0.85 * self.body.cfg['module']['block']['sensor']['buckets']:
            print "Goal Reached. Stopping learning."
            return True
        elif self.body._steps > 200:
            print "Exceeded maximum steps allowed. Aborting."
            return True
        else:
            logger.debug("pos: %s, goal: %s" % (self.body.pos, 0.85 * self.body.cfg['module']['block']['sensor']['buckets']))
            return False

    def maxSensorValue(self, state):
        sensorValues = []
        for module in state:
            sensorValues.append(module[0])
        return max(sensorValues)

    def getReward(self, state, action):
        newState = self.body.state
        oldState = state

        if newState == oldState:
            logger.info("state unchanged... penalize")
            reward = -5
        else:
            maxNewStateValue = self.maxSensorValue(newState)
            maxOldStateValue = self.maxSensorValue(oldState)
            advance = maxNewStateValue - maxOldStateValue
            reward = int(advance)
        #logger.info("new state: %s" % newState)
        #logger.info("reward: %s" % reward)
        #logger.info("-----------------------------------------")
        self.actionCount += 1
        logger.info("%s | %s -> %s" % (self.actionCount, action, reward))
        return reward


class LightSeeker1(LightSeeker):

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

        moduleCount = len(self.body.modules)
        #angles = ['90']
        angles = ['10','45','90']
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


class LightSeeker2(LightSeeker):

    def getReward(self, state, action):
        newState = self.body.state
        oldState = state

        if not self.body.legalAction(action):
            reward = -1
            logger.info('ILLEGAL ACTION DETECTED')
        elif self.body.status == TIMEOUT:
            reward = -1
            logger.info('ACTION TIMED OUT')
        else:
            maxNewStateValue = self.maxSensorValue(newState)
            maxOldStateValue = self.maxSensorValue(oldState)
            advance = int(maxNewStateValue - maxOldStateValue)
            if advance > 0:
                reward = 1
            elif advance < 0:
                reward = -1
            else:
                reward = 0
        self.actionCount += 1
        logger.info("%s | %s -> %s" % (self.actionCount, action, reward))
        return reward

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

        moduleCount = len(self.body.modules)
        #angles = ['10', '45', '90']
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

class LightSeeker3(LightSeeker2):

    def getAvailableActions(self):
        actions = ['%s|%s|%s90' % (module, block, sign) \
                for module in range(len(self.body.modules)) \
                for block in ('A','P') \
                for sign in ('+','-')]
        return actions

class LightSeeker4(LightSeeker):

    def getReward(self, state, action):
        newState = self.body.state
        oldState = state

        #if not self.body.legalAction(action):
        #    reward = -1
        #    actionState = 'ILLEGAL'
        #    logger.info('ILLEGAL ACTION DETECTED')
        #elif self.body.status == TIMEOUT:
        if newState == oldState:
            reward = -1
            actionState = 'ILLEGAL'
        elif self.body.status == TIMEOUT: 
            reward = -1
            actionState = 'TIMEOUT'
            logger.info('ACTION TIMED OUT')
        else:
            actionState = 'OK'
            maxNewStateValue = self.maxSensorValue(newState)
            maxOldStateValue = self.maxSensorValue(oldState)
            advance = maxNewStateValue - maxOldStateValue
            if abs(advance) > 0.25:
                reward = advance
            else:
                reward = 0
        self.actionCount += 1
        logger.info("%s | %s -> %s" % (self.actionCount, action, reward))
        logger.info("action: %s, reward: %s" % (actionState, reward))
        return reward

class LightSeeker5(LightSeeker4):

    def getAvailableActions(self):
        actions = ['%s|%s|%s90' % (module, block, sign) \
                for module in range(len(self.body.modules)) \
                for block in ('A','P') \
                for sign in ('+','-')]
        return actions

class LightSeeker6(LightSeeker4):

    def getAvailableActions(self):
        actions = ['0|A|+90', '0|A|-90', '4|P|+90', '4|P|-90',
                   '0|A|+90,4|P|+90', '0|A|+90,4|P|-90',
                   '0|A|-90,4|P|+90', '0|A|-90,4|P|-90']
        return actions


class Reconfiguration(RLBrain):

    def __init__(self, body, alpha=0.5, gamma=0.5, epsilon=0.5, action_value_function_filename=None, goal=None, brainFileName=None, action_value_function_class='ActionValueFunction1'):
        RLBrain.__init__(self, body, alpha=alpha, gamma=gamma, epsilon=epsilon, action_value_function_filename=action_value_function_filename, action_value_function_class=action_value_function_class)
        self.goal = goal
        self.prevSubGoalReward = 0
        self.actionCount = 0

    def getAvailableActions(self):
        actions = ['%s|%s|%s90' % (module, block, sign) \
                for module in range(len(self.body.modules)) \
                for block in ('A','P') \
                for sign in ('+','-')]
        return actions

    def goalReached(self):
        assert(self.goal is not None)
        goal = eval(self.goal)
        state = eval(self._Q.getStateHash(self.body.state))
        return goal == state

    def subGoalReward(self):
        import operator
        goalStates = eval(self.goal)
        currentStates = eval(self._Q.getStateHash(self.body.state))
        subgoalsReached = map(lambda (x, y): x == y and 1 or 0, zip(goalStates, currentStates))
        reward = reduce(operator.add, subgoalsReached)
        print subgoalsReached
        return reward

    def getReward(self, state, action):
        if self.goalReached():
            reward = 1
            print "Goal Reached. Stopping learning."
            print "%s | %s -> %s" % (self.actionCount, action, reward)
            #self.body.interactive = True
            self.dump()
        else:
        #    subGoalReward = self.subGoalReward()
        #    reward = subGoalReward - self.prevSubGoalReward
        #    self.prevSubGoalReward = subGoalReward
            reward = 0
        self.actionCount += 1
        logger.info("%s | %s -> %s" % (self.actionCount, action, reward))
        return reward

class Reconfiguration1(Reconfiguration):

    def getAvailableActions(self):
        return RLBrain.getAvailableActions(self)

class Reconfiguration2(Reconfiguration):

    def __init__(self, body, alpha=0.5, gamma=0.5, epsilon=0.5, action_value_function_filename=None, goal=None, brainFileName=None, action_value_function_class='ActionValueFunction2'):
        Reconfiguration.__init__(self, body, alpha=alpha, gamma=gamma, epsilon=epsilon, action_value_function_filename=action_value_function_filename, action_value_function_class=action_value_function_class, goal=goal, brainFileName=brainFileName)

    def getAvailableActions(self):
        actions = RLBrain.getAvailableActions(self)
        actions += ['dock|%s|%s|%s' % (moduleA, moduleB, side) \
                for moduleA in range(len(self.body.modules)) \
                for moduleB in range(len(self.body.modules)) \
                for side in (BOTTOM, FRONT, REAR)]
        actions += ['undock|%s||%s' % (moduleA, side) \
                for moduleA in range(len(self.body.modules)) \
                for side in (BOTTOM, FRONT, REAR)]
        return actions

    def shouldAbort(self):
        angles, docks = self.body.state
        abort = False
        for dock in docks:
            moduleAIdx, moduleBIdx, side = dock
            if moduleBIdx is None or side is None:
                abort = True
                break
        return abort

    def goalReached(self):
        assert(self.goal is not None)
        goal = eval(self.goal)
        state = eval(self._Q.getStateHash(self.body.state))
        return goal == state or self.shouldAbort()

    def getReward(self, state, action):
        if self.goalReached():
            reward = 1
            print "Goal Reached. Stopping learning."
            print "%s | %s -> %s" % (self.actionCount, action, reward)
            self.dump()
        elif self.shouldAbort():
            reward = -1
            print "Robot disconnected. Aborting."
            print "%s | %s -> %s" % (self.actionCount, action, reward)
            self.dump()
        else:
            reward = 0
        self.actionCount += 1
        logger.info("%s | %s -> %s" % (self.actionCount, action, reward))
        return reward

class Reconfiguration3(Reconfiguration):

    def __init__(self, body, alpha=0.5, gamma=0.5, epsilon=0.5, action_value_function_filename=None, goal=None, brainFileName=None, action_value_function_class='ActionValueFunction2'):
        Reconfiguration.__init__(self, body, alpha=alpha, gamma=gamma, epsilon=epsilon, action_value_function_filename=action_value_function_filename, action_value_function_class=action_value_function_class, goal=goal, brainFileName=brainFileName)

    def getAvailableActions(self):
        actions = Reconfiguration.getAvailableActions(self)
        actions += ['dock|%s|%s|%s' % (moduleA, moduleB, side) \
                for moduleA in range(len(self.body.modules)) \
                for moduleB in range(len(self.body.modules)) \
                for side in (BOTTOM, FRONT, REAR)]
        return actions

    def shouldAbort(self):
        angles, docks = self.body.state
        abort = len(docks) == len(self.body.modules)
        return abort

    def goalReached(self):
        assert(self.goal is not None)
        goal = eval(self.goal)
        state = eval(self._Q.getStateHash(self.body.state))
        return goal == state or self.shouldAbort()

    def getPartialReward(self, state, action):
        angles, docks = self.body.state
        goal = eval(self.goal)
        reward = 0
        if angles == goal[0]:
            reward += 1
        if docks == goal[1]:
            reward += 1
        return reward

    def getReward(self, state, action):
        if self.goalReached():
            reward = 2
            print "Goal Reached. Stopping learning."
            print "%s | %s -> %s" % (self.actionCount, action, reward)
            self.dump()
        elif self.shouldAbort():
            reward = -1
            print "Goal not reached. Aborting."
            print "%s | %s -> %s" % (self.actionCount, action, reward)
            self.dump()
        else:
            reward = self.getPartialReward(state, action)
        self.actionCount += 1
        logger.info("%s | %s -> %s" % (self.actionCount, action, reward))
        return reward

class Reconfiguration4(Reconfiguration):

    def __init__(self, body, alpha=0.5, gamma=0.5, epsilon=0.5, action_value_function_filename=None, goal=None, brainFileName=None, action_value_function_class='ActionValueFunction2'):
        Reconfiguration.__init__(self, body, alpha=alpha, gamma=gamma, epsilon=epsilon, action_value_function_filename=action_value_function_filename, action_value_function_class=action_value_function_class, goal=goal, brainFileName=brainFileName)

    def getAvailableActions(self):
        actions = RLBrain.getAvailableActions(self)
        actions += ['dock|%s|%s|%s' % (moduleA, moduleB, side) \
                for moduleA in range(len(self.body.modules)) \
                for moduleB in range(len(self.body.modules)) \
                for side in (BOTTOM, FRONT, REAR)]
        actions += ['undock|%s||%s' % (moduleA, side) \
                for moduleA in range(len(self.body.modules)) \
                for side in (BOTTOM, FRONT, REAR)]
        return actions

    def shouldAbort(self):
        angles, docks = self.body.state
        abort = len(docks) < len(self.body.modules) - 1
        return abort

    def goalReached(self):
        assert(self.goal is not None)
        goal = eval(self.goal)
        state = eval(self._Q.getStateHash(self.body.state))
        return goal == state or self.shouldAbort()

    def getReward(self, state, action):
        if self.shouldAbort():
            reward = -1
            print "Robot disconnected. Aborting."
            print "%s | %s -> %s" % (self.actionCount, action, reward)
            self.dump()
        elif self.goalReached():
            reward = 1
            print "Goal Reached. Stopping learning."
            print "%s | %s -> %s" % (self.actionCount, action, reward)
            self.dump()
        else:
            reward = 0
        self.actionCount += 1
        logger.info("%s | %s -> %s" % (self.actionCount, action, reward))
        return reward

class Reconfiguration5(Reconfiguration4):

    def getAvailableActions(self):
        actions = Reconfiguration.getAvailableActions(self)
        actions += ['dock|%s|%s|%s' % (moduleA, moduleB, side) \
                for moduleA in range(len(self.body.modules)) \
                for moduleB in range(len(self.body.modules)) if moduleB != moduleA \
                for side in (BOTTOM, FRONT, REAR)]
        actions += ['undock|%s||%s' % (moduleA, side) \
                for moduleA in range(len(self.body.modules)) \
                for side in (BOTTOM, FRONT, REAR)]
        return actions

class Reconfiguration6(Reconfiguration4):

    def getAvailableActions(self):
        actions = Reconfiguration.getAvailableActions(self)
        actions += ['undock|%s||%s' % (moduleA, side) \
                for moduleA in range(len(self.body.modules)) \
                for side in (BOTTOM, FRONT, REAR)]
        return actions

    def shouldAbort(self):
        angles, docks = self.body.state
        dockCount = len(docks)
        moduleCount = len(self.body.modules)
        goal = eval(self.goal)
        abort = dockCount < moduleCount - 1 or \
                (dockCount == moduleCount - 1 and \
                set(docks) != set(goal[1]))
        return abort

    def goalReached(self):
        assert(self.goal is not None)
        goal = eval(self.goal)
        state = (self.body.state[0], self.body.state[1])
        return goal == state or self.shouldAbort()

    def getPartialReward(self, state, action):
        angles, docks = self.body.state
        goal = eval(self.goal)
        reward = 0
        if state != self.body.state and \
           set(docks) == set(goal[1]):
            for i in range(len(angles)):
                if angles[i] == goal[0][i]:
                    reward += 1
        logger.info("[getPartialReward] reward: %s" % reward)
        return reward

    def getReward(self, state, action):
        if self.shouldAbort():
            reward = -1
            print "Robot disconnected. Aborting."
            print "%s | %s -> %s" % (self.actionCount, action, reward)
            self.dump()
        else:
            reward = self.getPartialReward(state, action)
            if self.goalReached():
                print "Goal Reached. Stopping learning."
                print "%s | %s -> %s" % (self.actionCount, action, reward)
                self.dump()
        self.actionCount += 1
        logger.info("%s | %s -> %s" % (self.actionCount, action, reward))
        return reward

class Reconfiguration7(Reconfiguration6):

    def getPartialReward(self, state, action):
        angles, docks = self.body.state
        goal = eval(self.goal)
        reward = 0
        if state != self.body.state and \
           set(docks) == set(goal[1]):
            for i in range(len(angles)):
                for j in [0,1]:
                    if angles[i][j] == goal[0][i][j]:
                        reward += 1
        logger.info("[getPartialReward] reward: %s" % reward)
        return reward

class Reconfiguration8(Reconfiguration6):

    def getPartialReward(self, state, action):
        currentAngles, currentDocks = self.body.state
        lastAngles, lastDocks = state
        goalAngles, goalDocks = eval(self.goal)
        reward = 0
        if state != self.body.state and set(currentDocks) == set(goalDocks):
            if currentDocks != lastDocks:
                # primer desacoplamiento
                reward = 1
            else:
                for i in range(len(currentAngles)):
                    for j in [0, 1]:
                        if currentAngles[i][j] == goalAngles[i][j]:
                            if lastAngles[i][j] != goalAngles[i][j]:
                                reward += 1
                        else:
                            if lastAngles[i][j] == goalAngles[i][j]:
                                reward -= 1
        if reward == 0:
            reward = -1
        logger.info("[getPartialReward] reward: %s" % reward)
        return reward
#    def getPartialReward(self, state, action):
#        angles, docks = self.body.state
#        goal = eval(self.goal)
#        reward = -1
#        if state != self.body.state and set(docks) == set(goal[1]):
#            if docks != self.body.state[1]:
#                # primer desacoplamiento
#                reward = 1
#            else:
#                currentAnglesOK = self.getAnglesOK(self.body.state)
#                lastAnglesOK = self.getAnglesOK(state)
#                reward = currentAnglesOK - lastAnglesOK
#                if reward == 0:
#                    reward = -1
#        logger.info("[getPartialReward] reward: %s" % reward)
#        return reward

    def getAnglesOK(self, state):
        angles, docks = state
        goal = eval(self.goal)
        anglesOK = 0
        for i in range(len(angles)):
            for j in [0, 1]:
                if angles[i][j] == goal[0][i][j]:
                    anglesOK +=1
        logger.info("[getAnglesOK] anglesOK == %s" % anglesOK)
        return anglesOK

class Reconfiguration9(Reconfiguration8): 

    def getAvailableActions(self):
        actions = Reconfiguration.getAvailableActions(self)
        actions += ['dock|%s|%s|%s' % (moduleA, moduleB, side) \
                for moduleA in range(len(self.body.modules)) \
                for moduleB in range(len(self.body.modules)) if moduleB != moduleA \
                for side in (BOTTOM, FRONT, REAR)]
        actions += ['undock|%s||%s' % (moduleA, side) \
                for moduleA in range(len(self.body.modules)) \
                for side in (BOTTOM, FRONT, REAR)]
        return actions

    def shouldAbort(self):
        angles, docks = self.body.state
        dockCount = len(docks)
        moduleCount = len(self.body.modules)
        abort = dockCount < moduleCount - 1 
        return abort

    def goalReached(self):
        assert(self.goal is not None)
        goal = eval(self.goal)
        state = (self.body.state[0], self.body.state[1])
        return goal == state or self.shouldAbort()

    def getPartialReward(self, state, action):
        currentAngles, currentDocks = self.body.state
        lastAngles, lastDocks = state
        goalAngles, goalDocks = eval(self.goal)
        reward = 0
        for i in range(len(currentAngles)):
            for j in [0, 1]:
                if currentAngles[i][j] == goalAngles[i][j]:
                    if lastAngles[i][j] != goalAngles[i][j]:
                        reward += 1
                else:
                    if lastAngles[i][j] == goalAngles[i][j]:
                        reward -= 1
        if set(currentDocks) == set(goalDocks):
            reward += 1
        if reward == 0:
            reward = -1
        logger.info("[getPartialReward] reward: %s" % reward)
        return reward

    def getReward(self, state, action):
        if self.shouldAbort():
            reward = -1
            print "Robot disconnected. Aborting."
            print "%s | %s -> %s" % (self.actionCount, action, reward)
            self.dump()
        else:
            reward = self.getPartialReward(state, action)
            if self.goalReached():
                print "Goal Reached. Stopping learning."
                print "%s | %s -> %s" % (self.actionCount, action, reward)
                self.dump()
        self.actionCount += 1
        logger.info("%s | %s -> %s" % (self.actionCount, action, reward))
        return reward

class Reconfiguration10(Reconfiguration):

    def getAvailableActions(self):
        actions = ['0|A|+90', '0|A|-90', '4|P|+90', '4|P|-90',
                   '0|A|+90,4|P|+90', '0|A|+90,4|P|-90',
                   '0|A|-90,4|P|+90', '0|A|-90,4|P|-90']
        return actions

    def goalReached(self):
        assert(self.goal is not None)
        goal = eval(self.goal)
        state = eval(self._Q.getStateHash(self.body.state))
        return goal == state

class Reconfiguration11(Reconfiguration8):
    
    def getAvailableActions(self):
        actions = Reconfiguration.getAvailableActions(self)
        #actions += ['undock|%s||%s' % (moduleA, side) \
        #        for moduleA in range(len(self.body.modules)) \
        #        for side in (BOTTOM, FRONT, REAR)]
        return actions

    def goalReached(self):
        assert(self.goal is not None)
        goal = eval(self.goal)
        state = eval(self._Q.getStateHash(self.body.state))
        goalReached = goal == state
        return goalReached or self.body._steps > 200
