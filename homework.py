import sys
import os
import collections
import Queue
import time
from Queue import PriorityQueue
class Elevation:
    def __init__(self):
        self.elevation={}
# to convert  output in desired format
    def formatOutput(self,result):
        temp = ''
        for result_a in result:
            if(result_a == 'FAIL'):
                temp += result_a+'\n'
            else:
                temp_a = ''
                for x in result_a[0]:
                    temp_a +=' '+str(x[0])+','+str(x[1]) 
                temp += temp_a.lstrip()+'\n'
        return temp.rstrip()
    # to check if the move is valid 
    def validMove(self,x_src, y_src, x_tgt, y_tgt, max_diff):        
        if(abs((int(self.elevation[x_tgt, y_tgt]))-(int(self.elevation[x_src, y_src]))) <= int(max_diff)):
            return True
        else:
            return False
    # to calculate cost in UCS algorithm 
    def calculateDistance_ucs(self,src_x, src_y, dst_x, dst_y):
        if(src_x == dst_x or src_y == dst_y):
            return 10
        else:
            return 14
    # to calculate cost along with heuristics in astar algorithm 
    def calculateDistance_aStar(self,src_x, src_y, dst_x, dst_y, target_ind):
        elevation_tgt = int(self.elevation[dst_x, dst_y])
        elevation_src = int(self.elevation[src_x, src_y])        
        diff = abs(elevation_tgt-elevation_src)
        if(src_x == dst_x or src_y == dst_y):
            return 10+diff
        else:
            return 14+diff
    # to calculate cost along with heuristics in astar algorithm 
    def heuristic_aStar(self,src_x, src_y, dst_x, dst_y, target_ind):
        target_x = target_ind[0]
        target_y = target_ind[1]
        # normalisation factor of 10 
        ed = (abs(target_x-dst_x)+abs(target_y-dst_y)) * 9.8
        return ed
    # to calculate path using UCS 
    def run_ucs(self,land, target, width, height, max_diff):
        result = []
        land_int = land.copy()
        for target_ind in target:
            result_ind = []
            land = land_int.copy()           
            queue = PriorityQueue()
            temp_cord=land.pop()
            queue.put((0, [temp_cord]))
            seen = {}
            seen[temp_cord] = 0
            while not queue.empty():
                distance, path = queue.get()
                # print("Distance --->", distance, "Seeing path -->", path)
                x, y = path[-1]
                if((x, y) == target_ind):
                    result_ind.append((path))
                    break
                for x_next, y_next in ((x+1, y), (x+1, y+1), (x+1, y-1), (x, y+1), (x, y-1), (x-1, y+1), (x-1, y), (x-1, y-1)):
                    distance_1 = distance
                    if(0 <= x_next < width and 0 <= y_next < height  and  self.validMove(x, y, x_next, y_next, max_diff) ):
                        distance_1+=10 if(x==x_next or y==y_next) else 14
                        if((x_next, y_next) in seen.keys()):
                            if(distance_1 < seen[x_next, y_next]):
                                seen[x_next, y_next] = distance_1
                                queue.put((distance_1, path + [(x_next, y_next)]))
                        else:
                            seen[x_next, y_next] = distance_1
                            queue.put((distance_1, path + [(x_next, y_next)]))
            if(len(result_ind) == 0):
                result.append(("FAIL"))
            else:
                result.append(result_ind)
        return result
    # to calculate path using A*
    def run_aStar(self,land, target, width, height, max_diff):
        result = []
        land_int = land.copy()
        for target_ind in target:
            result_ind = []
            land = land_int.copy()
            queue = PriorityQueue()
            temp_cord=land.pop()
            queue.put((0, 0, [temp_cord]))
            seen = {}
            seen[temp_cord] = 0
            while not queue.empty():
                c_distance,distance, path = queue.get()     
                # print("C_distance_1--->",c_distance,"Distance --->", distance, "Seeing path -->", path)
                x, y = path[-1]
                if((x, y) == target_ind):
                    result_ind.append((path))
                    break
                for x_next, y_next in ((x+1, y), (x+1, y+1), (x+1, y-1), (x, y+1), (x, y-1), (x-1, y+1), (x-1, y), (x-1, y-1)):
                    c_distance_1 = c_distance
                    distance_1 = distance
                    if(0 <= x_next < width and 0 <= y_next < height and self.validMove(x, y, x_next, y_next, max_diff)):
                        distance_1 += self.calculateDistance_aStar(x, y, x_next, y_next, target_ind)
                        c_distance_1 = distance_1 + self.heuristic_aStar(x, y, x_next, y_next, target_ind)                   
                        if((x_next, y_next) in seen.keys()):
                            if(distance_1 < seen[x_next, y_next]):
                                seen[x_next, y_next] = distance_1
                                queue.put((c_distance_1,distance_1, path + [(x_next, y_next)]))
                        else:
                            seen[x_next, y_next] = distance_1
                            queue.put((c_distance_1,distance_1, path + [(x_next, y_next)]))                        
            if(len(result_ind) == 0):
                result.append(("FAIL"))
            else:
                result.append(result_ind)
        return result
    # to calculate path using BFS
    def run_bfs(self,land, target, width, height, max_diff):
        result = []
        land_int = land.copy()
        for target_ind in target:
            result_ind = []
            land = land_int.copy()
            queue = collections.deque([[land.pop()]])
            seen = land.copy()
            while queue:
                path = queue.popleft()
                x, y = path[-1]
                if((x, y) == target_ind):
                    result_ind.append((path))
                for x_next, y_next in ((x+1, y), (x+1, y+1), (x+1, y-1), (x, y+1), (x, y-1), (x-1, y+1), (x-1, y), (x-1, y-1)):
                    if(0 <= x_next < width and 0 <= y_next < height and self.validMove(x, y, x_next, y_next, max_diff) and (x_next, y_next) not in seen):
                        queue.append(path + [(x_next, y_next)])
                        seen.add((x_next, y_next))
            if(len(result_ind) == 0):
                result.append(("FAIL"))
            else:
                result.append([min(result_ind)])
        return result
#   main function
filePath_in = 'input.txt'
filePath_out = 'output.txt'
# to get input from file 
O=Elevation()
with open(filePath_in, 'r') as fp:
    start = time.time()
    lineObj = fp.readlines()
    algorithm = lineObj[0].rstrip()
    width, height = lineObj[1].rstrip().split(" ")
    width = int(width)
    height = int(height)    
    land = set([(int(lineObj[2].rstrip().split(" ")[0]), int(lineObj[2].rstrip().split(" ")[1]))])
    max_diff = lineObj[3].rstrip()
    num_sites = int(lineObj[4].rstrip())
    target = []
    for x in range(5, 5+num_sites, 1):
        target_xy = (lineObj[x].rstrip().split(" "))
        target.append(((int(target_xy[0]), int(target_xy[1]))))
    for x in range(5+num_sites, 5+num_sites+int(height)):
        for key, value in enumerate((lineObj[x].rstrip().split(" "))):
            O.elevation[key, x-(5+num_sites)] = int(value)
    if(algorithm == 'BFS'):
        result = O.run_bfs(land, target, width, height, max_diff)
    if(algorithm == 'UCS'):
        result = O.run_ucs(land, target, width, height, max_diff)
    if(algorithm == 'A*'):
        result = O.run_aStar(land, target, width, height, max_diff)
    result = O.formatOutput(result)
with open(filePath_out, 'w+') as fp:
    fp.write(result)
end = time.time()
diff = end-start
print(diff)
