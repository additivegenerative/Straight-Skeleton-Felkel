import numpy as np
import math
from queue import PriorityQueue

class Skeleton:

    def __init__(self, contour):

        # Set contour
        self.contour = contour

        # Dicts holding edge and vertice data
        self.vertices = dict()                  # Key is hash with x, y and direction
        self.edges = dict()                     # Key is hash is x, y, and direction for two vertices
        self.vertToEdges = dict()               # Maps hash for vertice to hash to edges containing the vertice
        self.events = dict()                    # Hash for collapse event with x, y and time
        self.removedFromQueue = dict()          # Keeps track of which events have been removed from the queue

        # Priority queue for sequential handling of events
        self.q = PriorityQueue()

        # Set time at start
        self.currentTime = 0

        # Skeleton
        self.skeleton = np.array([], dtype=object)

    def generate(self):
        self.parseContour(self.contour)
        self.checkAllIntersections()
        self.runWavefrontSimulation()

        return self.skeleton

    # Process each edge in a contour
    def parseContour(self, contour):

        motorcycles = dict()                # Holds all motorcycles

        print('Parsing contour')

        contourLength = len(contour)

        for i in range(contourLength):

            # Current vertice
            v1 = [contour[i][0], contour[i][1]]
            
            # Next vertice
            if(i != contourLength-1):
                v2 = [contour[i+1][0], contour[i+1][1]]
            else:
                v2 = [contour[0][0], contour[0][1]]

            # Previous vertice
            if(i == 0):
                v0 = [contour[contourLength-1][0], contour[contourLength-1][1]]
            else:
                v0 = [contour[i-1][0], contour[i-1][1]]

            # Check if vert is reflex
            angle = self.getAngle(v0, v1, v2)

            # Add line segment to edges dict
            key = hash((v1[0], v1[1], v2[0], v2[1]))
            edgeAngle = self.getAngleOfLineBetweenTwoPoints([v1[0], v1[1]], [v2[0], v2[1]])
            v1_key = hash((v1[0], v1[1]))
            v2_key = hash((v2[0], v2[1]))
            value = [v1_key, v2_key, edgeAngle]
            self.edges[key] = value

            # If reflex, add motorcycle at vertice
            if(angle < 180):
                keyM = hash((v1[0], v1[1], angle))
                direction = self.getDirection(v0, v1, v2) - 180
                valueM = [v1, direction, 0, 1]
                self.vertices[v1_key] = valueM
                motorcycles[v1_key] = valueM
            else:
                keyM = hash((v1[0], v1[1], angle))
                direction = self.getDirection(v0, v1, v2)
                valueM = [v1, direction, 0, 0]
                self.vertices[v1_key] = valueM

        self.parseMotorcycles(motorcycles)
        self.mapVertsToEdges()
    
    # Parse all motorcycles to check for intersections and edge splitting
    def parseMotorcycles(self, motorcycles):

        print('Generating motorcycle graph')

        movingPoints = dict()           # Holds all motorcycles
        motorcycle_edges = dict()       # Holds all motorcycle edges
        splitedges = dict()             # Holds all new edges from edges split by motorcycles 
        queue = dict()                  # Queue for motorcycle events

        # To be added: Check intersecting motorcycles here

        for m in motorcycles:
            v = motorcycles[m]
            movingPoints[m] = v
            d = self.point_on_circle(v[1])
            for e in self.edges:
                edge = self.edges[e]
                intersection = self.lineRayIntersectionPoint(v[0], d, self.vertices[edge[0]][0], self.vertices[edge[1]][0])
                if(len(intersection) > 0 and intersection[0] > 0):
                    if m in queue:
                        if(queue[m][0] > intersection[0]):
                            queue[m] = [intersection[0]]
                            revDirection = v[1] + 180 if v[1] < 180 else v[1] - 180
                            keyM = hash((self.truncate(intersection[1][0][0]), self.truncate(intersection[1][0][1]), revDirection))
                            valueM = [[intersection[1][0][0], intersection[1][0][1]], revDirection, 0, 2]
                            self.vertices[keyM] = valueM
                            splitedges[e] = valueM

                            new_e_key = hash((v[0][0], v[0][1], intersection[1][0][0], intersection[1][0][1]))
                            motorcycle_edges[new_e_key] = [m, keyM]

                    else:
                        queue[m] = [intersection[0]]
                        revDirection = v[1] + 180 if v[1] < 180 else v[1] - 180
                        keyM = hash((self.truncate(intersection[1][0][0]), self.truncate(intersection[1][0][1]), revDirection))
                        valueM = [[self.truncate(intersection[1][0][0]), self.truncate(intersection[1][0][1])], revDirection, 0, 2]
                        self.vertices[keyM] = valueM
                        splitedges[e] = valueM

                        new_e_key = hash((v[0][0], v[0][1], intersection[1][0][0], intersection[1][0][1]))
                        motorcycle_edges[new_e_key] = [m, keyM]

        for me in motorcycle_edges:
            self.edges[me] = motorcycle_edges[me]
            v1_key = motorcycle_edges[me][0]
            v2_key = motorcycle_edges[me][1]

        for e in splitedges:
            splitVertice = splitedges[e]
            splitVertice_key = hash((splitVertice[0][0], splitVertice[0][1], splitVertice[1]))
            self.vertices[splitVertice_key] = splitVertice

            v1_key = self.edges[e][0]
            v2_key = self.edges[e][1]
            v1 = self.vertices[v1_key]
            v2 = self.vertices[v2_key]

            e1_key = hash((v1[0][0], v1[0][1], splitVertice[0][0], splitVertice[0][1]))
            e2_key = hash((splitVertice[0][0], splitVertice[0][1], v2[0][0], v2[0][1]))
            self.edges[e1_key] = [v1_key, splitVertice_key]
            self.edges[e2_key] = [splitVertice_key, v2_key]
            del self.edges[e]

    # Map edges to vertices for reverse lookup
    def mapVertsToEdges(self):

        print('Mapping vertices to edges')

        for e in self.edges:
            v1_key = self.edges[e][0]
            v2_key = self.edges[e][1]

            if v1_key in self.vertToEdges:
                self.vertToEdges[v1_key][e] = True
            else:
                self.vertToEdges[v1_key] = dict()
                self.vertToEdges[v1_key][e] = True
            if v2_key in self.vertToEdges:
                self.vertToEdges[v2_key][e] = True
            else:
                self.vertToEdges[v2_key] = dict()
                self.vertToEdges[v2_key][e] = True

    # Calculate all vertice intersections and thereby edge collapses
    def checkAllIntersections(self):

        print('Check all intersections')

        for e in self.edges:
            v1_key = self.edges[e][0]
            v2_key = self.edges[e][1]
            v1 = self.vertices[v1_key]
            v2 = self.vertices[v2_key]
            d1 = self.point_on_circle(v1[1])
            d2 = self.point_on_circle(v2[1])

            intersection = self.rayRayIntersectionPoint(v1[0], d1, v1[1], v1[2], v2[0], d2, v2[1], v2[2])
            edgeAngle = self.getAngleOfLineBetweenTwoPoints([v1[0][0], v1[0][1]], [v2[0][0], v2[0][1]])
            aNorm = (edgeAngle - 90) % 360
            if(len(intersection) > 0):
                t = self.truncate(intersection[0])
                self.edges[e] = [v1_key, v2_key, edgeAngle, aNorm]
                temp = [t, v1_key, v2_key, self.truncate(intersection[1][0]), self.truncate(intersection[1][1]), e, self.currentTime, edgeAngle]
                self.addToCollapseDict(t, intersection[1][0], intersection[1][1], temp)
                self.q.put([t, temp])
            else:
                self.edges[e] = [v1_key, v2_key, edgeAngle, aNorm]

        print(self.edges)

    # Simulate the propagating wavefront by iterating through the events in the priority queue
    def runWavefrontSimulation(self):

        print('Simulating wavefront propagation')

        while not self.q.empty():

            f = self.q.get()
            e = f[1]

            print('*** Process event ***')

            # First check if event is already removed
            if e[5] in self.removedFromQueue:
                print('Event removed from queue')
                continue

            eventKey = hash((e[0], e[3], e[4]))

            # Check if event involves one or multiple edges
            if eventKey in self.events:
                    if len(self.events[eventKey][1]) > 1:
                        print('- Multi edge event')
                        self.handleMultiEdgeEvent(e, self.events[eventKey][1])
                    else:
                        print('- Single edge event')
                        self.handleSingleEdgeEvent(e)
            else:
                print('- Event is not in the event list')


    def handleSingleEdgeEvent(self, initEdge):

        initEdge_key = initEdge[5]

        # Affected vertices
        v1 = self.vertices[initEdge[1]]
        v2 = self.vertices[initEdge[2]]
        # Affected edges
        e1 = self.vertToEdges[initEdge[1]]
        e2 = self.vertToEdges[initEdge[2]]
        # Intersect
        intersect = [initEdge[3], initEdge[4]]
        t = initEdge[0]
        a1 = (v1[1] - 180) % 360
        a2 = (v2[1] - 180) % 360

        self.currentTime = t

        if(v1[3] == 0 and v2[3] == 0):

            print('--> Classic edge event')

            # direction = self.truncate(((a1 + a2) / 2)) - 180 if abs(a1 - a2) < 180 else self.truncate(((a1 + a2) / 2))
            # direction = direction % 360

            leftAngle = 0
            rightAngle = 0

            print(e1)
            print(e2)
            
            for edge in e1:
                if edge == initEdge[5] or edge in self.removedFromQueue:
                    continue
                verts = self.edges[edge]
                laN = verts[3]
                print(laN)

            for edge in e2:
                print(self.edges[edge][3])
                if edge == initEdge[5] or edge in self.removedFromQueue:
                    continue
                verts = self.edges[edge]
                raN = verts[3]
                print(raN)

            print(initEdge)
            print(laN)
            print(raN)
            direction = self.truncate(((laN + raN) / 2)) if abs(laN - raN) > 180 else self.truncate(((laN + raN) / 2)) - 180
            direction = direction % 360
            print(direction)

            new_v_key = hash((intersect[0], direction))
            self.vertices[new_v_key] = [[initEdge[3], initEdge[4]], direction, t, 0]

            print('new vert')
            print(self.vertices[new_v_key])

            for edge in e1.copy():
                if edge == initEdge[5] or edge in self.removedFromQueue:
                    continue
                verts = self.edges[edge]
                v = verts[0]
                if verts[0] == initEdge[1]:
                    v = verts[1]
                self.processEdge([v, new_v_key], self.currentTime, verts[2])
                self.removedFromQueue[edge] = True

            for edge in e2.copy():
                if edge == initEdge[5] or edge in self.removedFromQueue:
                    continue
                verts = self.edges[edge]
                v = verts[0]
                if verts[0] == initEdge[2]:
                    v = verts[1]
                self.processEdge([v, new_v_key], self.currentTime, verts[2])
                self.removedFromQueue[edge] = True

            temp1 = np.array([v1[0][0], v1[0][1], initEdge[3], initEdge[4]], dtype=object)
            temp2 = np.array([v2[0][0], v2[0][1], initEdge[3], initEdge[4]], dtype=object)
            self.skeleton = np.append(self.skeleton, temp1, axis=0)
            self.skeleton = np.append(self.skeleton, temp2, axis=0)

        elif(v1[3] == 1 and v2[3] == 2) or (v1[3] == 2 and v2[3] == 1):

            print('--> Split event')

            vReflex_key = initEdge[1]
            vSteiner_key = initEdge[2]
            vReflex = v1
            vSteiner = v2
            vReflex_edges = self.vertToEdges[vReflex_key]
            vSteiner_edges = self.vertToEdges[vSteiner_key]

            angSteiner_right = 0
            angSteiner_left = 0
            angReflex_right = 0
            angReflex_left = 0

            for edge in vReflex_edges:
                if edge == initEdge[5] or edge in self.removedFromQueue:
                    continue

                if self.edges[edge][1] == vReflex_key:
                    # Left edge
                    v = self.edges[edge][0]
                    angReflex_left = self.edges[edge][3]
                else:
                    # Right edge
                    v = self.edges[edge][1]
                    angReflex_right = self.edges[edge][3]

            for edge in vSteiner_edges:
                if edge == initEdge[5] or edge in self.removedFromQueue:
                    continue

                if self.edges[edge][0] == vSteiner_key:
                    # Left edge
                    v = self.edges[edge][1]
                    angSteiner_left = self.edges[edge][3]
                else:
                    # Right edge
                    v = self.edges[edge][0]
                    angSteiner_right = self.edges[edge][3]

            # angSteiner_right = (vSteiner[1] + 90) % 360
            # angSteiner_left = (vSteiner[1] - 90) % 360
            # angReflex_right = (vReflex[1] + 90) % 360
            # angReflex_left = (vReflex[1] - 90) % 360

            print(angReflex_left)
            print(angSteiner_right)
            print(angReflex_right)
            print(angSteiner_left)

            angNew_left = (angReflex_left + angSteiner_right) / 2
            angNew_right = (angReflex_right + angSteiner_left) / 2 - 180

            for edge in vSteiner_edges.copy():
                if edge == initEdge[5] or edge in self.removedFromQueue:
                    continue

                if self.edges[edge][0] == vSteiner_key:
                    # Left edge
                    v = self.edges[edge][1]
                    ang = angNew_left
                    edgeAng = self.edges[edge][2]
                else:
                    # Right edge
                    v = self.edges[edge][0]
                    ang = angNew_right
                    edgeAng = self.edges[edge][2]

                new_v_key = hash((intersect[0], ang))
                self.vertices[new_v_key] = [[initEdge[3], initEdge[4]], ang, t, 0]

                print(self.vertices[new_v_key])
                print(self.vertices[v])

                print(edgeAng)
                
                self.processEdge([new_v_key, v], self.currentTime, edgeAng)

            for edge in vReflex_edges.copy():
                if edge == initEdge[5] or edge in self.removedFromQueue:
                    continue

                if self.edges[edge][1] == vReflex_key:
                    # Left edge
                    v = self.edges[edge][0]
                    ang = angNew_left
                    edgeAng = self.edges[edge][2]
                else:
                    # Right edge
                    v = self.edges[edge][1]
                    ang = angNew_right
                    edgeAng = self.edges[edge][2]

                new_v_key = hash((intersect[0], ang))
                self.vertices[new_v_key] = [[initEdge[3], initEdge[4]], ang, t, 0]

                print(self.vertices[new_v_key])
                print(self.vertices[v])
                
                self.processEdge([new_v_key, v], self.currentTime, edgeAng)
            
            self.removedFromQueue[edge] = True

            temp1 = np.array([v1[0][0], v1[0][1], initEdge[3], initEdge[4]], dtype=object)
            self.skeleton = np.append(self.skeleton, temp1, axis=0)

        elif(v1[3] == 0 and v2[3] == 1) or (v1[3] == 1 and v2[3] == 0):
            print('--> Switch event (convex + reflex)')
        elif(v1[3] == 0 and v2[3] == 2) or (v1[3] == 2 and v2[3] == 0):
            print('--> Switch event (convex + moving Steiner)')
        else:
            print('--> Unknown event')

        # Set affected edges as removed
        # for edge in e1:
        #         self.removedFromQueue[edge] = True
        # for edge in e2:
        #        self.removedFromQueue[edge] = True

    def handleMultiEdgeEvent(self, initEdge, allEvents):

        print(initEdge)

        t = initEdge[0]
        self.currentTime = t

        # Get vertices of edge initiating the multi edge event
        initEdge_key = initEdge[5]
        vLeft = initEdge[1]
        vRight = initEdge[2]
        allEventKeys = dict()
        for e in allEvents:
            allEventKeys[e[5]] = True

        # Check which edges in allEvents are connected to initEdge
        leftEdges = []
        rightEdges = []
        removedVertices = dict()
        v1 = initEdge[1]
        v2 = initEdge[2]
        removedVertices[v1] = True
        removedVertices[v2] = True
        connectCheckLeft = True
        connectCheckRight = True
        eLeft = list(self.vertToEdges[vLeft].keys())                # Get all edges connected to the vertices
        eRight = list(self.vertToEdges[vRight].keys())              # Get all edges connected to the vertices
        while connectCheckLeft:
            for e in eLeft:
                if e in allEventKeys and e != initEdge_key:
                    v1 = self.edges[e][0]
                    v2 = self.edges[e][1]
                    removedVertices[v1] = True
                    removedVertices[v2] = True
                    vLeft = v1 if v2 == vLeft else v2
                    eLeft = list(self.vertToEdges[vLeft].keys())
                    for eL in eLeft:
                        if eL != e:
                            leftEdges.append(eL)
                else:
                    connectCheckLeft = False
        while connectCheckRight:
            for e in eRight:
                if e in allEventKeys and e != initEdge_key:
                    v1 = self.edges[e][0]
                    v2 = self.edges[e][1]
                    removedVertices[v1] = True
                    removedVertices[v2] = True
                    vRight = v1 if v2 == vRight else v2
                    eRight = list(self.vertToEdges[vRight].keys())
                    for eR in eRight:
                        if eR != e:
                            rightEdges.append(eR)
                else:
                    connectCheckRight = False

        leftEdges.reverse()
        chainedEdges = leftEdges + [initEdge_key] + rightEdges

        # Create new vertice based on vLeft and vRight
        v1 = self.vertices[vLeft]
        v2 = self.vertices[vRight]
        a1 = v1[1]
        a2 = v2[1]
        intersect = [initEdge[3], initEdge[4]]
        t = initEdge[0]
        direction = self.truncate(((a1 + a2) / 2)) if abs(a1 - a2) < 180 else self.truncate(((a1 + a2) / 2)) - 180
        direction = direction % 360
        new_v_key = hash((intersect[0], intersect[1], direction))
        self.vertices[new_v_key] = [intersect, direction, t, 0]

        print(self.vertices[new_v_key])
        print(removedVertices)

        # Edges to update
        e1 = self.vertToEdges[vLeft]
        e2 = self.vertToEdges[vRight]

        for edge in e1.copy():
            if edge == initEdge_key or edge in self.removedFromQueue:
                continue
            verts = self.edges[edge]
            v = verts[0]
            if verts[0] == vLeft:
                v = verts[1]
            self.processEdge([v, new_v_key], self.currentTime, verts[2])

        for edge in e2.copy():
            if edge == initEdge_key or edge in self.removedFromQueue:
                continue
            verts = self.edges[edge]
            v = verts[0]
            if verts[0] == vRight:
                v = verts[1]
            self.processEdge([v, new_v_key], self.currentTime, verts[2])

        # Remove edges from the priority queue
        for e in chainedEdges:
            self.removedFromQueue[e] = True

        for v in removedVertices:
            vSkel = self.vertices[v]
            temp = np.array([vSkel[0][0], vSkel[0][1], intersect[0], intersect[1]], dtype=object)
            self.skeleton = np.append(self.skeleton, temp, axis=0)

    # Truncate to avoid float arithmetic inaccuracies
    def truncate(self, num):
        return round(num * 10000000) / 10000000

    # Get local angle between points a, b, c to check if reflex or convex
    def getAngle(self, a, b, c):
        a0 = math.atan2(c[1]-b[1], c[0]-b[0])
        a1 = math.atan2(a[1]-b[1], a[0]-b[0])
        # I suggest you normalize the angle from 0 to 2*pi, to do that
        # if (a0-a1) is less than 0, add 2*pi to (a0-a1), then convert it into degrees. Now you have only positive values.
        # remember to follow the convention of 'b' as current vertex, 'a' as previous vertex (vector ba is vector 1) and 'c' as next vertex (vector bc is vector 2). 
        # we are finding difference of angle (always measured in counterclockwise), so angle of vector 2 minus angle of vector 1. 
        # vector ba when rotated counterclockwise by (a0-a1) degrees, it will align with vector bc
        # ang_diff = a0-a1
        # if ang_diff < 0:
            #ang_diff +=(2*pi)
        # ang_rad = ang_diff
        # ang = math.degrees(ang_rad)
        
        ang = math.degrees(a1 - a0)        
        return self.truncate(ang) % 360 

    # Get bisection angle between points a, b, c relative to global coord system
    def getDirection(self, a, b, c):
        xDiff_line1 = a[0] - b[0]
        yDiff_line1 = a[1] - b[1]
        ang_line1 = math.degrees(math.atan2(yDiff_line1, xDiff_line1)) % 360
        xDiff_line2 = c[0] - b[0]
        yDiff_line2 = c[1] - b[1]
        ang_line2 = math.degrees(math.atan2(yDiff_line2, xDiff_line2)) % 360
        ang = (ang_line1 + ang_line2) / 2 if abs(ang_line1 - ang_line2) < 180 else (ang_line1 + ang_line2) / 2 + 180
        return self.truncate(ang) % 360

    def point_on_circle(self, angle):
        center = [0,0]
        ang = math.radians(angle)
        radius = 1
        x = center[0] + (radius * math.cos(ang))
        y = center[1] + (radius * math.sin(ang))

        return [self.truncate(x), self.truncate(y)]

    def getAngleOfLineBetweenTwoPoints(self, p1, p2):
        xDiff = p2[0] - p1[0]
        yDiff = p2[1] - p1[1]
        ang = math.degrees(math.atan2(yDiff, xDiff))
        return self.truncate(ang) % 360

    def magnitude(self, vector):
        return np.sqrt(np.dot(np.array(vector),np.array(vector)))

    def norm(self, vector):
        return np.array(vector)/self.magnitude(np.array(vector))

    def addToCollapseDict(self, t, i1, i2, temp):

        key = hash((t, i1, i2))

        if key in self.events:
            self.events[key][1].append(temp)
        else:
            self.events[key] = [t, [temp]]

    def lineRayIntersectionPoint(self, rayOrigin, rayDirection, point1, point2):
        # Convert to numpy arrays
        rayOrigin = np.array(rayOrigin, dtype=np.float)
        rayDirection = np.array(self.norm(rayDirection), dtype=np.float)
        point1 = np.array(point1, dtype=np.float)
        point2 = np.array(point2, dtype=np.float)
        
        # Ray-Line Segment Intersection Test in 2D
        # http://bit.ly/1CoxdrG
        v1 = rayOrigin - point1
        v2 = point2 - point1
        v3 = np.array([-rayDirection[1], rayDirection[0]])
        t1 = np.cross(v2, v1) / np.dot(v2, v3)
        t2 = np.dot(v1, v3) / np.dot(v2, v3)
        if t1 >= 0.0 and t2 >= 0.0 and t2 <= 1.0:
            return [t1, [rayOrigin + t1 * rayDirection]]
        return []

    def rayRayIntersectionPoint(self, rO1, rD1, th1, t01, rO2, rD2, th2, t02):

        xdiff = (rD1[0], rD2[0])
        ydiff = (rD1[1], rD2[1])
        line1 = ((rO1[0], rO1[1]), (rO1[0]+rD1[0], rO1[1]+rD1[1]))
        line2 = ((rO2[0], rO2[1]), (rO2[0]+rD2[0], rO2[1]+rD2[1]))

        edgeAngle = self.getAngleOfLineBetweenTwoPoints([rO1[0], rO1[1]], [rO2[0], rO2[1]]) % 360
        # print(edgeAngle)
        # print(th1)
        angle = abs(edgeAngle - th1) if abs(edgeAngle - th1) < 90 else 180 - abs(edgeAngle - th1)

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
            if -0.000001 < abs(th1 - th2) - 180 < 0.000001:
                if rO1[0] - rO2[0] == 0.0:
                    x = rO1[0]
                else:
                    x = (rO1[0]+rO2[0])/2
                y = (rO1[1]+rO2[1])/2
            elif abs(th1 - th2) < 0.000001 or abs(th1 - th2) > 359.999999:
                print('Lines are parallel')
                if rD1[0] != 0.0 and rD2[0] != 0.0:
                    m1 = rD1[1]/rD1[0]
                    c1 = rO1[1] - m1 * rO1[0]
                    m2 = rD2[1]/rD2[0]
                    c2 = rO2[1] - m2 * rO2[0]
                    if -0.000001 < abs(c1 - c2) < 0.000001:
                        tci1 = (rO2[0] - rO1[0]) / rD1[0]
                        tci2 = (rO1[0] - rO2[0]) / rD2[0]
                        y = rO1[1] if tci2 > tci1 else rO2[1]
                        x = rO1[0] if tci2 > tci1 else rO2[0]
                        print('Lines coincide')
                    else:
                        return []
                else:
                    if -0.000001 < abs(rO1[0] - rO2[0]) < 0.000001:
                        print('Lines coincide')
                        x = rO1[0]
                        tci1 = (rO2[1]-rO1[1]) / rD1[1]
                        tci2 = (rO1[1]-rO2[1]) / rD2[1]
                        y = rO1[1] if tci2 > tci1 else rO2[1]
                    else:
                        return []
            else:
                print('Lines do not intersect')
                return []
        else:
            d = (det(*line1), det(*line2))
            x = -det(d, xdiff) / div
            y = -det(d, ydiff) / div

        x = self.truncate(x)
        y = self.truncate(y)

        # print(x)
        # print(rO1[0])
        # print(rD1[0])
        if rD1[0] != 0.0:
            t1 = (x - rO1[0]) / rD1[0]
        else:
            t1 = (y - rO1[1]) / rD1[1]

        if rD2[0] != 0.0:
            t2 = (x - rO2[0]) / rD2[0]
        else:
            t2 = (y - rO2[1]) / rD2[1]

        angle = self.getAngle([x, y], [rO1[0], rO1[1]], [rO2[0], rO2[1]])
        angle = 360 - angle if angle > 180 else angle
        # print(angle)
        
        t = t01 if t02 <= t01 else t02
        if -0.000001 < abs(th1 - th2) - 180 < 0.000001:
            time = t1
        else:
            time = math.sin(math.radians(angle)) * t1

        """
        print('Intersection')
        print((x, y))
        print((rO1, rO2))
        print((th1, th2))
        print(angle)
        print(t1)
        print(t2)
        print(time)
        """

        if self.truncate(time) >= 0.0 and (t1 >= 0.0 or t2 >= 0.0):
            # print(time + t)
            if self.truncate(time + t) >= self.currentTime:
                return [self.truncate(time + t), [self.truncate(x), self.truncate(y)]]
            else:
                return []
        return []

    def processEdge(self, edge, addedTime, angle):

        v1_key = edge[0]
        v2_key = edge[1]
        v1 = self.vertices[v1_key]
        v2 = self.vertices[v2_key]
        d1 = self.point_on_circle(v1[1])
        d2 = self.point_on_circle(v2[1])

        # Add new edge to dict
        key = hash((v1[0][0], v1[0][1], v2[0][0], v2[0][1]))
        aNorm = (angle - 90) % 360
        value = [v1_key, v2_key, angle, aNorm]
        self.edges[key] = value
        if v1_key in self.vertToEdges:
            self.vertToEdges[v1_key][key] = True
        else:
            self.vertToEdges[v1_key] = dict()
            self.vertToEdges[v1_key][key] = True
        if v2_key in self.vertToEdges:
            self.vertToEdges[v2_key][key] = True
        else:
            self.vertToEdges[v2_key] = dict()
            self.vertToEdges[v2_key][key] = True

        intersection = self.rayRayIntersectionPoint(v1[0], d1, v1[1], v1[2], v2[0], d2, v2[1], v2[2])
        if(len(intersection) > 0):
            t = self.truncate(intersection[0])
            # self.edges[key] = [v1_key, v2_key, t, self.truncate(intersection[1][0]), self.truncate(intersection[1][1])]
            temp = [t, v1_key, v2_key, self.truncate(intersection[1][0]), self.truncate(intersection[1][1]), key, self.currentTime]
            self.addToCollapseDict(t, intersection[1][0], intersection[1][1], temp)
            temp = [t, v1_key, v2_key, self.truncate(intersection[1][0]), self.truncate(intersection[1][1]), key, addedTime]
            self.q.put([t, temp])
        else:
            self.edges[key] = [v1_key, v2_key, angle, aNorm]