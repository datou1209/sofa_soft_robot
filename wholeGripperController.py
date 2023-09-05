#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa.Core
from Sofa.constants import *
import math

num = 3

# class WholeGripperController(Sofa.Core.Controller):

#     def __init__(self, *a, **kw):

#         Sofa.Core.Controller.__init__(self, *a, **kw)
#         self.node = kw["node"]
#         self.constraints = []
#         self.dofs = []
#         for i in range(1,num+1):
#             self.dofs.append(self.node.getChild('finger' + str(i)).tetras)
#             self.constraints.append(self.node.getChild('finger'+str(i)).cavity.SurfacePressureConstraint)

#         self.centerPosY = 70
#         self.centerPosZ = 0
#         self.rotAngle = 0

#         return

#     def onKeypressedEvent(self,e):

#         increment = 0.01  #0.01

#         if e["key"] == Sofa.constants.Key.plus:
#             for i in range(num):
#                 pressureValue = self.constraints[i].value.value[0] + increment
#                 if pressureValue > 1.5:
#                     pressureValue = 1.5
#                 self.constraints[i].value = [pressureValue]

#         if e["key"] == Sofa.constants.Key.minus:
#             for i in range(num):
#                 pressureValue = self.constraints[i].value.value[0] - increment
#                 if pressureValue < 0:
#                     pressureValue = 0
#                 self.constraints[i].value = [pressureValue]

class WholeGripperController(Sofa.Core.Controller):

    def __init__(self, *a, **kw):

        Sofa.Core.Controller.__init__(self, *a, **kw)
        self.node = kw["node"]
        self.constraints = []
        self.dofs = []
        for i in range(1,num+1):
            #self.dofs.append(self.node.getChild('finger1').getChild('cavity' + str(i - 1)).cavity)
            #self.dofs.append(self.node.getChild('cavity' + str(i)).tetras)
            self.constraints.append(self.node.finger1.getChild('cavity' + str(i)).SurfacePressureConstraint)
        self.centerPosY = 70
        self.centerPosZ = 0
        self.rotAngle = 0

        return

    def onKeypressedEvent(self,e):

        increment = 0.001  #0.01

        if e["key"] == Sofa.constants.Key.KP_4:
            
            pressureValue = self.constraints[0].value.value[0] + increment
            if pressureValue > 3.0:
                pressureValue = 3.0
            self.constraints[0].value = [pressureValue]


        if e["key"] == Sofa.constants.Key.KP_5:
        
            pressureValue = self.constraints[1].value.value[0] + increment
            if pressureValue > 3.0:
                pressureValue = 3.0
            self.constraints[1].value = [pressureValue]

        if e["key"] == Sofa.constants.Key.KP_6:
        
            pressureValue = self.constraints[2].value.value[0] + increment
            if pressureValue > 3.0:
                pressureValue = 3.0
            self.constraints[2].value = [pressureValue]

        if e["key"] == Sofa.constants.Key.KP_1:
        
            pressureValue = self.constraints[0].value.value[0] - increment
            if pressureValue < 0:
                pressureValue = 0
            self.constraints[0].value = [pressureValue]

        if e["key"] == Sofa.constants.Key.KP_2:
        
            pressureValue = self.constraints[1].value.value[0] - increment
            if pressureValue < 0:
                pressureValue = 0
            self.constraints[1].value = [pressureValue]

        if e["key"] == Sofa.constants.Key.KP_3:
        
            pressureValue = self.constraints[2].value.value[0] - increment
            if pressureValue < 0:
                pressureValue = 0
            self.constraints[2].value = [pressureValue]

