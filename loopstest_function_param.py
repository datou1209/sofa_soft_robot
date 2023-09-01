import math

def looptest(int, loopnum):
    
    vertex_loop = []
    loops = []
    spring_set=[]
    spring = []
    Ks = 1e5
    Kd = 0

    for loop_id in range(0, 26):
        for v_id in range(0, 37):
            if v_id==0:
                vertex_loop.append([17*math.cos(v_id/18*math.pi), 17*math.sin(v_id/18*math.pi), int*(loop_id)])
            if 0<v_id<36:
                vertex_loop.append([17*math.cos(v_id/18*math.pi), 17*math.sin(v_id/18*math.pi), int*(loop_id)])
                L=((math.dist(vertex_loop[v_id],vertex_loop[v_id-1])))
                spring.append([v_id-1,v_id,Ks,Kd,L])
            # elif 25<=v_id<28:
            #     vertex_loop.append([int*(1+loop_id), 5.5*math.cos(math.pi), 5.5*math.sin(math.pi)-v_id+25])
            #     L=((math.dist(vertex_loop[v_id],vertex_loop[v_id-1])))
            #     spring.append([v_id-1,v_id,Ks,Kd,L])
            # elif v_id>=28 and v_id<39:
            #     vertex_loop.append([int*(1+loop_id), 5.5*math.cos(math.pi)+1+v_id-28, -2])
            #     L=((math.dist(vertex_loop[v_id],vertex_loop[v_id-1])))
            #     spring.append([v_id-1,v_id,Ks,Kd,L])
            # #elif v_id>=39:
            #     #vertex_loop.append([int*(1+loop_id), -5.5+11,-2])
            #     #L=((math.dist(vertex_loop[v_id],vertex_loop[v_id-1])))
            #     #spring.append([v_id-1,v_id,Ks,Kd,L])
            if v_id==35:
                L=((math.dist(vertex_loop[v_id],vertex_loop[0])))
                spring.append([v_id,0,Ks,Kd,L])
        spring_set.append(spring)
        loops.append(vertex_loop)
        vertex_loop = []
        spring = []
    return loops, spring_set


    #fiber.addObject('MeshTopology', src='@loader', name='container')
    #fiber.addObject('MechanicalObject', name='lines', template='Vec3', showObject=True, showObjectScale=1)


