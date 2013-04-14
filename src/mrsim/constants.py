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

# block type
ACTIVE = 'active'
PASSIVE = 'passive'
LINK = 'link'

# block side
BOTTOM = 'bottom'
FRONT = 'front'
REAR = 'rear'

# sensor type
LIGHT_SENSOR = 'light'
DIRECTION_SENSOR = 'direction'
ROTATION_SENSOR = 'rotation'
OBSTACLE_SENSOR = 'obstacle'

# status
IDLE = 'idle'
BUSY = 'busy'
TIMEOUT = 'timeout'

# policy type
EXPLOIT = 'exploit'
EXPLORE = 'explore'
EPSILON_GREEDY = 'epsilon_greedy'
