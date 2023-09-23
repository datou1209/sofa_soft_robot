import Sofa

def createScene(rootNode):

    rootNode.addObject( 'RequiredPlugin' , pluginName= 'CGALPlugin')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh') # Needed to use components [VTKExporter]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology] 
    node = rootNode.addChild('node')
    node.addObject('Mesh',name='mesh',filename='/home/eintelligence/Downloads/whole.obj')
    node.addObject('MeshGenerationFromPolyhedron',name='gen',inputPoints='@mesh.position', inputTriangles='@mesh.triangles', drawTetras='1', facetSize="4",    
                                facetApproximation="0.15", 
                                cellRatio="2", cellSize="1.5")
    node.addObject('Mesh', position='@gen.outputPoints', tetrahedra='@gen.outputTetras')
    node.addObject('VTKExporter', filename='whole', edges='0', tetras='1', exportAtBegin='1')

    return rootNode
