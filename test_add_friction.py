# from ast import mod
# from audioop import mul
# from pyexpat import model
# from turtle import mode
import Sofa
import math
from stlib3.scene import MainHeader, ContactHeader
from wholeGripperController import WholeGripperController


youngModulusFingers = 600
youngModulusStiffLayerFingers = 1500

rotation1 = [0, 0, 0]
rotation2 = [0, 0, 180]
rotation3 = [180, 0, 0]
rotation4 = [0, 0, 0]
rotation5 = [180, 0, 0]
rotation6 = [0, 0, 0]
rotations = [rotation1, rotation2, rotation3, rotation4, rotation5, rotation6]

translateFinger1 = [0, 0, 0]
translations = [translateFinger1]
translationCavity1 = [0, 0, 0]
translationCavity2 = [0, 0, 0]
translationCavity3 = [106, 0, 0]
translationCavity4 = [106, 0, 0]
translationCavity5 = [212, 0, 0]
translationCavity6 = [212, 0, 0]
translationsCavity = [translationCavity1, translationCavity2, translationCavity3, translationCavity4, translationCavity5, translationCavity6]

angles = [0]

def Floor(parentNode, color=[0.5, 0.5, 0.5, 1.], rotation=[0, 0, 0], translation=[0, -30, 0]):
    floor = parentNode.addChild('Floor')
    floor.addObject('MeshOBJLoader', name='loader', filename='data/mesh/floorFlat.obj', triangulate=True, scale=20, rotation=rotation,
                    translation=translation)
    floor.addObject('OglModel', src='@loader', color=color)
    floor.addObject('MeshTopology', src='@loader', name='topo')
    floor.addObject('MechanicalObject', src='@loader')
    floor.addObject('TriangleCollisionModel')
    floor.addObject('LineCollisionModel')
    floor.addObject('PointCollisionModel')
    floor.addObject('UncoupledConstraintCorrection')
    return floor


class SnakeRobot:


    def __init__(self, parentNode, youngModulus=600, poissonRation=0.3):

        for i in range(1):
            self.node = parentNode.addChild('finger' + str(i + 1))
            self.node.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
            self.node.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d" )
            self.node.addObject("GenericConstraintCorrection", solverName="SparseLDLSolver")
            self.node.addObject('MeshVTKLoader', name='loader', filename='data/mesh_fiber/fiber_out.vtk',
                                rotation=[0, 0, 0], translation=[0, 0, 0])
            self.node.addObject('MeshTopology', src='@loader', name='container')
            self.node.addObject('MechanicalObject', name='tetras', template='Vec3d', showObject=True, showObjectScale=1, showIndices=True, showIndicesScale=4e-5)
            self.node.addObject('UniformMass', totalMass=10)
            self.node.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=poissonRation,
                                youngModulus=youngModulus)
            
            self.node.addObject('LinearSolverConstraintCorrection', solverName='SparseLDLSolver')
            

            self.__addCavity()
            self.__addVisualModel()
            self.__addFiber()

    def __addCavity(self):
        
        for j in range(2):
            cavity = self.node.addChild('cavity' + str(j+1))
            cavity.addObject('MeshVTKLoader', name='loader', filename='data/mesh_fiber/fiber_in.vtk',
                        translation=[translationsCavity[j]], rotation=[rotations[j]], scale=1.0)                                                                                                                                                                              
            cavity.addObject('MeshTopology', src='@loader', name='topo')
            cavity.addObject('MechanicalObject', name='cavity')
            #应该是加了这个的原因会跳动
            #cavity.addObject('UncoupledConstraintCorrection')
            cavity.addObject('SurfacePressureConstraint', name='SurfacePressureConstraint', template='Vec3d', value=0.0,
                        triangles='@topo.triangles', valueType='pressure')
            cavity.addObject('BarycentricMapping', name='mapping')

    def __addFiber(self):

        for k in range(19):
            fiber = self.node.addChild('fiber' + str(k+1))
            fiber.addObject('MechanicalObject', name='fiber', template='Rigid3d')
            fiber.addObject('UniformMass', totalMass=0.001)
            fiber_collision = fiber.addChild("collision")
            fiber_collision.addObject("MeshGmshLoader", name="loader", filename="data/mesh_fiber/fiber.msh",
                                    translation=[0, 0, 4+5*k],rotation=[0, 0, 0], scale=1.0)
            fiber_collision.addObject("MeshTopology", src="@loader")
            fiber_collision.addObject("MechanicalObject", template="Vec3")
            fiber_collision.addObject("RigidMapping")
            fiber_collision.addObject("TriangleCollisionModel", name="TriangleModel")
            fiber_collision.addObject("LineCollisionModel", name="LineModel")
            fiber_collision.addObject("PointCollisionModel", name="PointModel")
            fiber_visu = fiber_collision.addChild('visu')
            fiber_visu.addObject("OglModel", name="visu", src="@../loader")
            fiber_visu.addObject("BarycentricMapping")
            

    def addCollisionModel(self):
        modelContact = self.node.addChild('CollisionModel')
        modelContact.addObject('MeshGmshLoader', name='loader', filename='data/mesh_fiber/fiber_out.msh',
                                translation=translations[0], rotation=[0, 0, 0])
        modelContact.addObject('MeshTopology', src='@loader', name='topo')
        modelContact.addObject('MechanicalObject', name='collisMech', showObject=False)
        modelContact.addObject('TriangleCollisionModel', contactStiffness=10.0, group=0, contactFriction=1.2, selfCollision=False)
        modelContact.addObject('LineCollisionModel', contactStiffness=10.0, group=0, contactFriction=1.2, selfCollision=False)
        modelContact.addObject('PointCollisionModel', contactStiffness=10.0, group=0, contactFriction=1.2, selfCollision=False)
        modelContact.addObject('BarycentricMapping')

    def __addVisualModel(self):
        modelVisu = self.node.addChild('visu')
        modelVisu.addObject('MeshSTLLoader', name='loader', filename='data/mesh_fiber/fiber_out.STL')
        modelVisu.addObject('OglModel', src='@loader', color=[0.7, 0.7, 0.7, 0.6], translation=translations[0],
                            rotation=[0, 0, 0])
        modelVisu.addObject('BarycentricMapping')



def createScene(rootNode):

    INVERSE = True

    # rootNode.addObject('VisualStyle',
    #                    displayFlags='showVisualModels hideBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showForceFields showInteractionForceFields hideWireframe')
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
    
    MainHeader(rootNode, dt=0.001, gravity=[0.0, -9810.0, 0.0], plugins=['SoftRobots', 'SofaPython3',
                                                     "Sofa.Component.AnimationLoop",
                                                
                                                     # Needed to use components FreeMotionAnimationLoop
                                                     "Sofa.Component.Collision.Detection.Algorithm",
                                                     "Sofa.Component.Mapping.NonLinear",
                                                     # Needed to use components BVHNarrowPhase, BruteForceBroadPhase, DefaultPipeline
                                                     "Sofa.Component.Collision.Detection.Intersection",
                                                     # Needed to use components LocalMinDistance
                                                     "Sofa.Component.Collision.Geometry",
                                                     # Needed to use components LineCollisionModel, PointCollisionModel, TriangleCollisionModel
                                                     "Sofa.Component.Collision.Response.Contact",
                                                     # Needed to use components DefaultContactManager
                                                     "Sofa.Component.Constraint.Lagrangian.Correction",
                                                     # Needed to use components GenericConstraintCorrection, UncoupledConstraintCorrection
                                                     "Sofa.Component.Constraint.Lagrangian.Solver",
                                                     # Needed to use components GenericConstraintSolver
                                                     "Sofa.Component.IO.Mesh",
                                                     # Needed to use components MeshOBJLoader, MeshSTLLoader, MeshVTKLoader
                                                     "Sofa.Component.LinearSolver.Direct",
                                                     # Needed to use components SparseLDLSolver
                                                     "Sofa.Component.LinearSolver.Iterative",
                                                     # Needed to use components CGLinearSolver
                                                     "Sofa.Component.Mass",  # Needed to use components UniformMass
                                                     "Sofa.Component.ODESolver.Backward",
                                                     # Needed to use components EulerImplicitSolver
                                                     "Sofa.Component.SolidMechanics.FEM.Elastic",
                                                     # Needed to use components TetrahedronFEMForceField
                                                     "Sofa.Component.Topology.Container.Constant",
                                                     # Needed to use components MeshTopology
                                                     "Sofa.Component.Topology.Container.Dynamic",
                                                     # Needed to use components TetrahedronSetTopologyContainer, TetrahedronSetTopologyModifier
                                                     "Sofa.Component.Visual",  # Needed to use VisualStyle
                                                     "Sofa.GL.Component.Rendering3D",
                                                     # Needed to use OglModel
                                                     "Sofa.GUI.Component",  # Needed to use AttachBodyButtonSetting
                                                     "Sofa.Component.Mapping.Linear", # Needed to use components [BarycentricMapping]
                                                     "Sofa.Component.StateContainer", # Needed to use components [MechanicalObject]
                                                     ])

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-12, maxIterations=10000)
    rootNode.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.01, rayleighMass=0.1)

    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('AttachBodyButtonSetting', stiffness=10)

    # if INVERSE:
    #     rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    #     rootNode.addObject('LCPConstraintSolver', maxIt="2500", tolerance="1e-7",
    #                        mu="1.2")
    # else:
    #     rootNode.addObject('GenericConstraintSolver', maxIterations=500, tolerance=1e-5)

    
    # Contact detection methods
    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response="FrictionContactConstraint", responseParams="mu=1.2")
    # rootNode.addObject('DefaultContactManager', response='FrictionContact', responseParams='mu=1.2')
    # rootNode.addObject('LocalMinDistance', alarmDistance=5, contactDistance=1, angleCone=0.0)
    ContactHeader(rootNode, alarmDistance=5, contactDistance=1, frictionCoef=1.2)
    rootNode.addObject('OglSceneFrame', style='Arrows', alignment='TopRight')
    # rootNode.addObject('CudaCollisionDetection')

    Floor(rootNode)

    snakerobot = SnakeRobot(rootNode)

    snakerobot.addCollisionModel()


    rootNode.addObject(WholeGripperController(node=rootNode))

    return rootNode