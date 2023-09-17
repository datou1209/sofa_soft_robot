import Sofa
import math

from wholeGripperController import WholeGripperController

youngModulusFingers = 600
youngModulusStiffLayerFingers = 1500

rotation1 = [0, 180, 0]
rotation2 = [0, 0, 0]
rotation3 = [240, 0, 0]
rotations = [rotation1, rotation2, rotation3]

translateFinger1 = [0, 0, 0]
translations = [translateFinger1]

angles = [0]

# def getPosition(y):

#     stringRadius = 17
#     stringPosition = []
#     for i in range(37):
#         stringPosition.append([stringRadius*math.sin(2 * math.pi *i /36),stringRadius*math.cos(2 * math.pi *i /36),y])
#     return stringPosition

def createScene(rootNode):
    rootNode.addObject('RequiredPlugin',
                       pluginName='SoftRobots SofaPython3 SofaLoader SofaSimpleFem SofaEngine SofaDeformable SofaImplicitOdeSolver SofaConstraint SofaSparseSolver SofaMeshCollision SofaRigid SofaOpenglVisual')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop') # Needed to use components [FreeMotionAnimationLoop]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm') # Needed to use components [BVHNarrowPhase,BruteForceBroadPhase,CollisionPipeline]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Intersection') # Needed to use components [LocalMinDistance]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry') # Needed to use components [LineCollisionModel,PointCollisionModel,TriangleCollisionModel]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact') # Needed to use components [DefaultContactManager]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction') # Needed to use components [LinearSolverConstraintCorrection]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver') # Needed to use components [GenericConstraintSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select') # Needed to use components [BoxROI]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh') # Needed to use components [MeshOBJLoader,MeshSTLLoader,MeshVTKLoader]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct') # Needed to use components [SparseLDLSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear') # Needed to use components [BarycentricMapping]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mass') # Needed to use components [UniformMass]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward') # Needed to use components [EulerImplicitSolver]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Setting') # Needed to use components [BackgroundSetting]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic') # Needed to use components [TetrahedronFEMForceField]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring') # Needed to use components [RestShapeSpringsForceField]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer') # Needed to use components [MechanicalObject]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual') # Needed to use components [VisualStyle]  
    rootNode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglModel,OglSceneFrame] 
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels hideCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe showBehavior')
    rootNode.gravity.value = [9810, 0, 0]
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-12, maxIterations=10000)
    rootNode.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.01, rayleighMass=0.1)
    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response='FrictionContactConstraint', responseParams='mu=0.6')
    rootNode.addObject('LocalMinDistance', name='Proximity', alarmDistance=5, contactDistance=1, angleCone=0.0)

    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1.])
    rootNode.addObject('OglSceneFrame', style='Arrows', alignment='TopRight')
###
    planeNode = rootNode.addChild('Plane')
    planeNode.addObject('MeshObjLoader', name='loader', filename='data/mesh/floorFlat.obj', triangulate=True,
                        rotation=[0, 0, 270], scale=10, translation=[-122, 0, 0])
    planeNode.addObject('MeshTopology', src='@loader')
    planeNode.addObject('MechanicalObject', src='@loader')
    planeNode.addObject('TriangleCollisionModel', simulated=False, moving=False)
    planeNode.addObject('LineCollisionModel', simulated=False, moving=False)
    planeNode.addObject('PointCollisionModel', simulated=False, moving=False)
    planeNode.addObject('OglModel', name='Visual', src='@loader', color=[0.5, 0.5, 1, 1])
###


    for i in range(1):
        ##########################################
        # Finger Model	 						 #
        ##########################################
        finger = rootNode.addChild('finger' + str(i + 1))
        finger.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        finger.addObject('SparseLDLSolver', name='preconditioner')

        # finger.addObject('MeshVTKLoader', name='loader', filename='data/mesh/out.vtk',
        #                  rotation=[0, 0, 0], translation=[0, 0, 0])
        finger.addObject('MeshVTKLoader', name='loader', filename='data1/out.vtk',
                         rotation=[0, 0, 0], translation=[0, 0, 0])
        finger.addObject('MeshTopology', src='@loader', name='container')
        
        finger.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=True, showObjectScale=1, showIndices=True, showIndicesScale=4e-5)
        finger.addObject('UniformMass', totalMass=0.01)
        finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                                       youngModulus=600)
        finger.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.3,
                         youngModulus=800)
        
        finger.addObject('BoxROI', name='boxROI', box=[-5, -20, -20, 5, 20, 20], strict=False, drawBoxes=True)
        #finger.addObject('BoxROI', name='boxROI', box=[-20, -20, 92.5, 20, 20, 97.5], strict=False, drawBoxes=True)


        if i == 0:
            finger.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12,
                              angularStiffness=1e12)
        else:
            finger.addObject('RestShapeSpringsForceField', points='@../finger1/boxROI.indices', stiffness=1e12,
                             angularStiffness=1e12)
            
        finger.addObject('LinearSolverConstraintCorrection', solverName='preconditioner')

    ##########################################
    # Cable                                  #
    ##########################################
        # for j in range(20):
        #     cable = finger.addChild('cable'+ str(j + 1))
        #     cable.addObject('MechanicalObject', name='mo',
        #                     position= getPosition(j*5),
        #                     showObject=True)

        #     # Add a CableConstraint object with a name.
        #     # the indices are referring to the MechanicalObject's positions.
        #     # The last index is where the pullPoint is connected.
        #     # By default, the Cable is controlled by displacement, rather than force.
        #     cable.addObject('CableConstraint', name="aCable",
        #                     indices=list(range(37)),
        #                     pullPoint=[0, 0, 50],
        #                     maxPositiveDisp= 0.5,
        #                     maxNegativeDisp= 0.5,
        #                     maxForce = 1,
        #                     minForce = 0,)

        #     # This adds a BarycentricMapping. A BarycentricMapping is a key element as it will add a bidirectional link
        #     # between the cable's DoFs and the finger's ones so that movements of the cable's DoFs will be mapped
        #     # to the finger and vice-versa;
        #     cable.addObject('BarycentricMapping')
        # #     #cable.addObject(WholeGripperController(node=cable))

        # hitching = finger.addChild("hitching")
        # #nbDOFs = 10
        # loopnum=16
        # Ks=1e7
        # Kd=0
        # L=6.33

        # #Hitching

        # hitching.addObject("MechanicalObject", template="Vec3", name="DOF",
        #                 position=[[-17, 0, (i)*6.33] for i in range(loopnum)],
        #                 showObject=True, showObjectScale=1)
        # hitching.addObject('MeshTopology', name='lines', lines=[[i, i + 1] for i in range(loopnum-1)])
        # hitching.addObject("StiffSpringForceField", template="Vec3d", name="springs", showArrowSize=0.5, drawMode=1,
        #             stiffness=Ks,damping=Kd, indices1=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28],
        #             indices2=[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29], lengths=L)
        # hitching.addObject('BarycentricMapping', name='mapping')
        # #hitching.addObject('UniformMass', totalMass=0.8)

        # #Mapping 

        # finger.addObject('MechanicalMatrixMapper', template="Vec3,Vec3",
        #                 nodeToParse=hitching.getLinkPath(),  # where to find the forces to map
        #                 object1=finger.tetras.getLinkPath(),  # where to map the forces
        #                 object2=finger.tetras.getLinkPath())  # in case of multi-mapping, here you can give the second parent
        # #hitching.addObject("FixedConstraint", name="FixedConstraint", indices=[0])

    ##########################################
    # Fiber  纤维                                #
    ##########################################

        # loopnum=26
        # int=3.8
        # loops, spring_set = looptest(int, loopnum)
        
        # for k in range(0,loopnum):
        #     fiber = finger.addChild("fiber"+str(k))
        #     fiber.addObject("MechanicalObject", template="Vec3", name="DOF",
        #                     position=loops[k],
        #                     showObject=True, showObjectScale=1)
        #     fiber.addObject('MeshTopology', name='lines', lines=[[i, i + 1] for i in range(loopnum-1)])
            
        #     #fiber.addObject("FixedConstraint", name="FixedConstraint", indices=[0])
        #     fiber.addObject("StiffSpringForceField", template="Vec3d", name="springs", showArrowSize=0.5, drawMode=1,
        #                     spring=spring_set[k])

        #     # Mapping 

        #     fiber.addObject('BarycentricMapping', name='mapping')
        #     finger.addObject('MechanicalMatrixMapper', template="Vec3,Vec3",
        #                     nodeToParse=fiber.getLinkPath(),  # where to find the forces to map
        #                     object1=finger.tetras.getLinkPath()  # where to map the forces
        #                     )  # in case of multi-mapping, here you can give the second parent

        ##########################################
        # Constraint							 #
        ##########################################
        #cavity = []
    for j in range(2):
        cavity = finger.addChild('cavity' + str(j+1))
        # cavity.addObject('MeshVTKLoader', name='loader', filename='data/mesh/newbodyin.vtk',
        #                 translation=[0, 0, 5], rotation=[rotations[j]], scale=1.0)
        cavity.addObject('MeshVTKLoader', name='loader', filename='data1/in.vtk',
                        translation=[0, 0, 0], rotation=[rotations[j]], scale=1.0)
        cavity.addObject('MeshTopology', src='@loader', name='topo')
        cavity.addObject('MechanicalObject', name='cavity')
        cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3', value=0.0001,
                        triangles='@topo.triangles', valueType='pressure')
        cavity.addObject('BarycentricMapping', name='mapping')


    #########################################
    #Collision							  #
    #########################################

    collisionFinger = finger.addChild('collisionFinger')
    collisionFinger.addObject('MeshSTLLoader', name='loader', filename='data1/out.stl',
                                translation=translations[0], rotation=[360 - angles[0] * 180 / math.pi, 0, 0])
    collisionFinger.addObject('MeshTopology', src='@loader', name='topo')
    collisionFinger.addObject('MechanicalObject', name='collisMech')
    collisionFinger.addObject('TriangleCollisionModel', selfCollision=False)
    collisionFinger.addObject('LineCollisionModel', selfCollision=False)
    collisionFinger.addObject('PointCollisionModel', selfCollision=False)
    collisionFinger.addObject('BarycentricMapping')

    # #########################################
    # #Visualization						  #
    # #########################################
    # modelVisu = finger.addChild('visu')
    # modelVisu.addObject('MeshSTLLoader', name='loader', filename='data/mesh/snake_out.stl')
    # modelVisu.addObject('OglModel', src='@loader', color=[0.7, 0.7, 0.7, 0.6], translation=translations[0],
    #                     rotation=[360 - angles[0] * 180 / math.pi, 0, 0])
    # modelVisu.addObject('BarycentricMapping')

    rootNode.addObject(WholeGripperController(node=rootNode))

    return rootNode