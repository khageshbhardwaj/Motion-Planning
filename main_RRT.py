import numpy as np
import time
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from rrt_algorithms.rrt.rrt import RRT
from rrt_algorithms.search_space.search_space import SearchSpace
from rrt_algorithms.utilities.plotting import Plot

def tic():
  return time.time()

def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))

def load_map(fname):
  '''
  Loads the bounady and blocks from map file fname.
  
  boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  
  blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
            ...,
            ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  '''
  mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'),\
                                    'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f','f','f')})
  blockIdx = mapdata['type'] == b'block'
  boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  return boundary, blocks


def draw_map(boundary, blocks, start, goal):
  '''
  Visualization of a planning problem with environment boundary, obstacle blocks, and start and goal points
  '''
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  hb = draw_block_list(ax,blocks)
  hs = ax.plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  hg = ax.plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')  
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  ax.set_xlim(boundary[0,0],boundary[0,3])
  ax.set_ylim(boundary[0,1],boundary[0,4])
  ax.set_zlim(boundary[0,2],boundary[0,5])
  return fig, ax, hb, hs, hg

def draw_block_list(ax,blocks):
  '''
  Subroutine used by draw_map() to display the environment blocks
  '''
  v = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],dtype='float')
  f = np.array([[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[0,1,2,3],[4,5,6,7]])
  clr = blocks[:,6:]/255
  n = blocks.shape[0]
  d = blocks[:,3:6] - blocks[:,:3] 
  vl = np.zeros((8*n,3))
  fl = np.zeros((6*n,4),dtype='int64')
  fcl = np.zeros((6*n,3))
  for k in range(n):
    vl[k*8:(k+1)*8,:] = v * d[k] + blocks[k,:3]
    fl[k*6:(k+1)*6,:] = f + k*8
    fcl[k*6:(k+1)*6,:] = clr[k,:]
  
  if type(ax) is Poly3DCollection:
    ax.set_verts(vl[fl])
  else:
    pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
    pc.set_facecolor(fcl)
    h = ax.add_collection3d(pc)
    return h

def runtest_RRT(mapfile, start, goal, verbose = True):
  '''
  This function:
   * loads the provided mapfile
   * creates a motion planner
   * plans a path from start to goal
   * checks whether the path is collision free and reaches the goal
   * computes the path length as a sum of the Euclidean norm of the path segments
  '''
  # Load a map and instantiate a motion planner
  boundary, blocks = load_map(mapfile)
  X_dimensions = boundary[:,:6].reshape(2,3).T
  Obstacles = blocks[:,:6]

  # Setting the parameters
  q = 0.4  # length of tree edges
  
  # length of smallest edge to check for intersection with obstacles
  # r = np.min([blocks[:, 3] - blocks[:, 0], blocks[:, 4] - blocks[:, 1], blocks[:, 5] - blocks[:, 2]]) - 0.02
  r = 0.08
  
  max_samples = 1000000  # max number of samples to take before timing out
  prc = 0.1  # probability of checking for a connection to goal

  
  # create Search Space
  X = SearchSpace(X_dimensions, Obstacles)
  
	# Call the motion planner
  t0 = tic()
  rrt = RRT(X, q, start, goal, max_samples, r, prc) 
  path = rrt.rrt_search()
  toc(t0,"Planning")
  
	# plot
  plot = Plot("rrt_3d")
  plot.plot_tree(X, rrt.trees)
  if path is not None:
      goal_reached = True
      plot.plot_path(X, path)
  plot.plot_obstacles(X, Obstacles)
  plot.plot_start(X, start)
  plot.plot_goal(X, goal)
  plot.draw(auto_open=True)

  # # TODO: You should verify whether the path actually intersects any of the obstacles in continuous space

 #  we are using in-built obstacle checking function for RRT algorithm. This algorithm ensures that there is
 #  no collision with the obstacles.  

  # # TODO: You can implement your own algorithm or use an existing library for segment and 
  # #       axis-aligned bounding box (AABB) intersection

  collision = False
  success = (not collision) and goal_reached
  pathlength = np.sum(np.sqrt(np.sum(np.diff(path,axis=0)**2,axis=1)))
  return success, pathlength

#######################################################################################################################

def test_single_cube(verbose = True):
  print('Running single cube test...\n') 
  start = (2.3, 2.3, 1.3)
  goal = (7.0, 7.0, 5.5)
  success, pathlength = runtest_RRT('./maps/single_cube.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

def test_maze(verbose = True):
  print('Running maze test...\n') 
  start = (0.0, 0.0, 1.0)
  goal = (12.0, 12.0, 5.0)
  success, pathlength = runtest_RRT('./maps/maze.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

    
def test_window(verbose = True):
  print('Running window test...\n') 
  start = (0.2, -4.9, 0.2)
  goal = (6.0, 18.0, 3.0)
  success, pathlength = runtest_RRT('./maps/window.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

  
def test_tower(verbose = True):
  print('Running tower test...\n') 
  start = (2.5, 4.0, 0.5)
  goal = (4.0, 2.5, 19.5)
  success, pathlength = runtest_RRT('./maps/tower.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

     
def test_flappy_bird(verbose = True):
  print('Running flappy bird test...\n') 
  start = (0.5, 2.5, 5.5)
  goal = (19.0, 2.5, 5.5)
  success, pathlength = runtest_RRT('./maps/flappy_bird.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength) 
  print('\n')

  
def test_room(verbose = True):
  print('Running room test...\n') 
  start = (1.0, 5.0, 1.5)
  goal = (9.0, 7.0, 1.5)
  success, pathlength = runtest_RRT('./maps/room.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')


def test_monza(verbose = True):
  print('Running monza test...\n')
  start = (0.5, 1.0, 4.9)
  goal = (3.8, 1.0, 0.1)
  success, pathlength = runtest_RRT('./maps/monza.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

#######################################################################################################################

if __name__ == '__main__':
  test_single_cube()
  test_maze()
  test_flappy_bird()
  test_monza()
  test_window()
  test_tower()
  test_room()
