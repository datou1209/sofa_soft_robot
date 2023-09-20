# -*- coding: utf-8 -*-
import os
import imp
import platform
from sys import argv

#   STLIB IMPORT
try:
	from stlib3.scene.wrapper import Wrapper
except:
    raise ImportError("ModelOrderReduction plugin depend on SPLIB"\
                     +"Please install it : https://github.com/SofaDefrost/STLIB")

# MOR IMPORT
from mor.utility import sceneCreation as u

slash = '/'
if "Windows" in platform.platform():
    slash = "\\"

# Our Original Scene IMPORT
originalScene = r'/home/eintelligence/Projectcode/sofa_soft_snake/test_add_friction.py'
originalScene = os.path.normpath(originalScene)
originalScene = imp.load_source(originalScene.split(slash)[-1], originalScene)

paramWrapper = ('/modelNode', {'paramForcefield': {'prepareECSW': True, 'modesPath': '/home/eintelligence/Projectcode/sofa_soft_snake/data/modes.txt', 'periodSaveGIE': 6, 'nbTrainingSet': 8}, 'paramMORMapping': {'input': '@../MechanicalObject', 'modesPath': '/home/eintelligence/Projectcode/sofa_soft_snake/data/modes.txt'}, 'paramMappedMatrixMapping': {'nodeToParse': '@./modelNode', 'template': 'Vec1d,Vec1d', 'object1': '@./MechanicalObject', 'object2': '@./MechanicalObject', 'timeInvariantMapping1': True, 'timeInvariantMapping2': True, 'performECSW': False}})

def createScene(rootNode):

    if (len(argv) > 1):
        stateFileName = str(argv[1])
    else:	
        stateFileName="stateFile.state"
    originalScene.createScene(rootNode)

    path , param = paramWrapper
    pathToNode = path[1:]

    u.createDebug(rootNode,pathToNode,stateFileName)
