# priority queue for OPEN list
from pqdict import pqdict
import math
import numpy as np

#######################################################################################################

class AStarNode(object):
  def __init__(self, pqkey, coord, hval):
    self.pqkey = pqkey
    self.coord = coord
    self.g = math.inf
    self.h = hval
    self.parent_node = None
    self.parent_action = None
    self.closed = False

  def __lt__(self, other):
    return self.g < other.g

#######################################################################################################

class Environment:
  def __init__(self, boundary, blocks):
    self.boundary = boundary
    self.blocks = blocks

  def getHeuristic(self, coord, goal):
    return np.linalg.norm(np.array(coord) - np.array(goal))
  
  # def is_valid(self, coord):
  #   if any(coord < self.boundary[0, :3]) or any(coord > self.boundary[0, 3:6]):
  #     return False
    
  #   for block in self.blocks:
  #     if all(coord >= block[:3]) and all(coord <= block[3:6]):
  #       return False
      
  #   return True

  def check_line_aabb_collision(self, P1, P2, AABB_min, AABB_max):
    def check_axis_overlap(p1, p2, min_val, max_val):
      if p1 > p2:
        p1, p2 = p2, p1
      return max(p1, min_val) <= min(p2, max_val)
    
    for i in range(3):  # Check each axis (x, y, z)
      if not check_axis_overlap(P1[i], P2[i], AABB_min[i], AABB_max[i]):
        return False
    return True

  def is_valid(self, start, end):
    boundary_min = self.boundary[0, :3]
    boundary_max = self.boundary[0, 3:6]

    if np.any(end < boundary_min) or np.any(end > boundary_max):
      return False

    for k in range(self.blocks.shape[0]):
      if self.check_line_aabb_collision(start, end, self.blocks[k, :3], self.blocks[k, 3:6]):
        return False

    return True
  
  def get_neighbors(self, node):
    # numofdirs = 26
    # [dX, dY, dZ] = np.meshgrid([-1, 0, 1], [-1, 0, 1], [-1, 0, 1])
    # dR = np.vstack((dX.flatten(), dY.flatten(), dZ.flatten()))
    # dR = np.delete(dR, 13, axis=1)  # Remove zero vector
    # dR = dR / np.sqrt(np.sum(dR**2, axis=0)) / 2.0  # Normalize and scale by 0.5
    
    # neighbors = []
    # for k in range(numofdirs):
    #   next_coord = tuple(node.coord + dR[:, k])
    #   if self.is_valid(next_coord):
    #     neighbors.append(next_coord)
    res = 0.4
    directions = [
        np.array([res, 0, 0]), np.array([-res, 0, 0]),
        np.array([0, res, 0]), np.array([0, -res, 0]),
        np.array([0, 0, res]), np.array([0, 0, -res])
    ]
    
    neighbors = []
    for direction in directions:
      next_coord = tuple(node.coord + direction)
      if self.is_valid(node.coord, next_coord):
        neighbors.append(next_coord)
  
    return neighbors
  
#######################################################################################################

class AStar(object):
  @staticmethod
  def plan(start_coord, goal_coord, environment, epsilon = 1):
    # Initialize the graph and open list
    Graph = {}
    OPEN = pqdict()
    
    # current node
    start_node = AStarNode(tuple(start_coord), start_coord, environment.getHeuristic(start_coord, goal_coord))
    start_node.g = 0
    start_node.f = start_node.g + epsilon * start_node.h
    OPEN[start_node.pqkey] = start_node.f
    
    Graph[start_node.pqkey] = start_node

    total_open_nodes = 0
    total_closed_nodes = 0

    # TODO: Implement A* here
    while OPEN:
      # Remove node with smallest f value
      curr_key, curr_f = OPEN.popitem()
      curr_node = Graph[curr_key]
      total_closed_nodes += 1
      
      # If the goal is reached, reconstruct and return the path
      if np.allclose(curr_node.coord, goal_coord, atol=0.3):
        return AStar.reconstruct_path(curr_node, total_open_nodes, total_closed_nodes)
      
      curr_node.closed = True

      # Expand neighbors
      neighbors = environment.get_neighbors(curr_node)

      for neighbor_coord in neighbors:
        if neighbor_coord in Graph:
          neighbor_node = Graph[neighbor_coord]
        else:
          neighbor_node = AStarNode(neighbor_coord, neighbor_coord, environment.getHeuristic(neighbor_coord, goal_coord))
          Graph[neighbor_coord] = neighbor_node
        
        if neighbor_node.closed:
          continue
        
        tentative_g = curr_node.g + np.linalg.norm(np.array(neighbor_coord) - np.array(curr_node.coord))
                
        if tentative_g < neighbor_node.g:
          neighbor_node.g = tentative_g
          neighbor_node.f = neighbor_node.g + epsilon * neighbor_node.h
          neighbor_node.parent_node = curr_node

          if neighbor_node.pqkey in OPEN:
            OPEN.updateitem(neighbor_node.pqkey, neighbor_node.f)
          else:
            OPEN[neighbor_node.pqkey] = neighbor_node.f
            total_open_nodes += 1
    
    print("No Path Found")
    return None  # No path found
  
    # path = []
    # node = curr_node

    # while node:
    #   path.append(node.coord)
    #   node = node.parent_node
    # return path[::-1]
  
  @staticmethod
  def reconstruct_path(node, total_open_nodes, total_closed_nodes):
    path = []
    while node:
      path.append(node.coord)
      node = node.parent_node
    return path[::-1], total_open_nodes, total_closed_nodes
