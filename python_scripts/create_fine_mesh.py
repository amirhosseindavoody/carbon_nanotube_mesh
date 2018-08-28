import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import matplotlib as mpl
from tqdm import tqdm
import argparse
from typing import List, Tuple

import plotly.offline as plo
import plotly.graph_objs as go

from tube import fiber
import util

# # customize matplotlib styles
# mpl.rc('lines', linewidth=4)  # default linewidth
# mpl.rc('lines', dash_capstyle='round')  # default dashed line capstyle
# mpl.rc('lines', solid_capstyle='round')  # default solid line capstyle
# mpl.rc('xtick', labelsize=20)  # default label size for xtick
# mpl.rc('ytick', labelsize=20)  # default label size for ytick
# mpl.rc('axes', titlesize=30)  # default title size
# mpl.rc('axes', labelsize=25)  # default label size for all axes
# mpl.rc('legend', fontsize=20)  # default fontsize for legends
# mpl.rc('figure', titlesize=35)  # default title size for figures with subplot

def load_fibers(directory: str):
  '''
  Load all the fiber mesh points in the a directory

  Parameters:
    directory (str): input directory

  Returns:
    list(fiber): list containing all the fiber objects
  '''
  fibers = []

  for i in range(1, 100):
    filename = os.path.join(directory, f'tube{i}.pos.dat')
    if (not os.path.isfile(filename)):
      continue
    print(f"reading file: {filename}")
    with open(filename) as file:
      for line in file:
        line = line.strip('\n; ')
        line = line.split(';')
        line = line[1:]
        r = [list(map(float,n.split(','))) for n in line]
        fibers.append(fiber(r))
  
  return fibers

def min_neighbor_distance(fibers: List[fiber], mode='fine', n=1000):
  '''
  Calculate the minimum distance between a list of fibers

  Parameters:
    fibers (list[fiber]): list of fibers
    mode (str): specify set of points that are going to be used for calculating the distances
    n (int): total number of fibers to take from the begining of the fibers list.

  Return:
    (min_dist_per_cnt, pair_min_dist):
      min_dist_per_cnt: np.ndarray of shape (n,1) that specifies nearest neighbor for each fiber
      min_dist: np.ndarray of shape (n,n) that specifies the distance between each two fiber in the list
  '''
  fibers = fibers[:n]

  n = len(fibers)

  pair_min_dist = np.full((n, n), 1.e4)

  for i, f1 in enumerate(tqdm(fibers)):
      for j in range(i+1, n):
          f2 = fibers[j]
          # temp = np.empty((f1.r(mode).shape[0], f2.r(mode).shape[0], 3))
          temp = np.empty((f2.r(mode).shape[0], f1.r(mode).shape[0], 3))
          for d in range(3):
              x, y = np.meshgrid(f1.r(mode)[:, d], f2.r(mode)[:, d])
              temp[:, :, d] = x-y
          temp = np.linalg.norm(temp, axis=2)
          pair_min_dist[i, j] = pair_min_dist[j, i] = temp.min()

  min_dist_per_cnt = pair_min_dist.min(axis=1)

  for i in range(pair_min_dist.shape[0]):
    pair_min_dist[i, i] = -1

  return min_dist_per_cnt, pair_min_dist

def create_single_CNTs(fib: fiber, coor: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
  '''
  Create coordinates of single CNTs inside a fiber

  Parameters:
    fib (fiber): the fiber object
    coor (np.ndarray): coordinates of CNT axis in the cross section plane of the fiber

  Returns:
    tuple of (pos, orient), where `pos` and `orient` are `np.ndarray` of shape `(n_cnt, n_coor_per_cnt, 3)` indicating position and orientation of points on all cnts.
  '''
  norm_vecs = fib.tangent_vec()
  a1, a2, a3 = util.cartesian_basis_vector(norm_vecs[0])

  r = fib.r('fine')

  pos = []
  orient = []

  for i, vec in enumerate(norm_vecs):
    axis = np.cross(a3, vec)
    alpha = np.arcsin(np.linalg.norm(axis))
    qt = util.get_quaternion(axis, alpha)
    Rot = util.rotation_from_quaternion(qt)

    a1 = np.matmul(Rot, a1.T).T
    a2 = np.matmul(Rot, a2.T).T
    a3 = vec

    pos.append(r[i] + np.matmul(coor, np.vstack((a1, a2))))
    orient.append([a3 for i in range(coor.shape[0])])
  
  pos = np.array(pos).swapaxes(0, 1)
  orient = np.array(orient).swapaxes(0, 1)

  return pos, orient

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--drop', help='Plot the mean height of fibers by their dropping order to see if any fiber have slipped through and fallen down', action='store_true')
  parser.add_argument('--drawRough', help='Plot the rough mesh created through BulletPhysics simulation', action='store_true')
  parser.add_argument('--check_interpolation', help='Check interpolation function to see if the fine mesh has proper shape', action='store_true')
  parser.add_argument('--nearest_neighbor', help='Calculate and plot the distance to the nearest neighbor for each fiber', action='store_true')
  parser.add_argument('--plot_cnts', help='Create and plot position of individual CNTs for a limited number of fibers', action='store_true')
  parser.add_argument('--create_cnts', help='Create the position of individual CNTs within the fibers', action='store_true')
  parser.add_argument('--check_cnt_nearest_neighbor', help='Calculate the nearest neighbors distance for each CNT', action='store_true')

  args = parser.parse_args()

  directory = os.path.expanduser("~/research/mesh/cnt_mesh_fiber")

  fibers = load_fibers(directory)
  
  print(f'Total number of fibers: {len(fibers)}')
  
  num_mesh = 0
  for f in fibers:
    num_mesh += f.num_nodes('rough')
  print(f'Total number of rough mesh points: {num_mesh}')

  avg_number_of_sections = np.mean([f.num_nodes('rough') for f in fibers])
  print(f'average number of sections per fiber: {avg_number_of_sections:.2f}')

  if args.drop:
    avg_y = np.array([f.avg_y() for f in fibers])

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(avg_y)
    ax.set_title("average position along y-axis in order of dropping")
    ax.set_xlabel("Drop order")
    ax.set_ylabel("Average y-coordinate [nm]")

    plt.show()

  if args.drawRough:
    fig = plt.figure()
    ax = fig.add_subplot('111', projection='3d')
    ax.set_aspect('equal')

    begin = 0
    n_fibers = len(fibers)

    mode = 'rough'

    for f in fibers[begin:begin+n_fibers]:
      ax.plot(f.z(mode), f.x(mode), f.y(mode))

    xlim = (np.min([f.x(mode).min() for f in fibers]), np.max([f.x(mode).max() for f in fibers]))
    ylim = (np.min([f.y(mode).min() for f in fibers]), np.max([f.y(mode).max() for f in fibers]))
    zlim = (np.min([f.z(mode).min() for f in fibers]), np.max([f.z(mode).max() for f in fibers]))

    # print(f'x limits: {xlim}')
    # print(f'y limits: {ylim}')
    # print(f'z limits: {zlim}')

    # Create cubic bounding box to simulate equal aspect ratio
    max_range = np.array([zlim[1]-zlim[0], xlim[1]-xlim[0], ylim[1]-ylim[0]]).max()
    Xb = 0.5*max_range*np.mgrid[-1:2:2, -1:2:2, -1:2:2][0].flatten() + 0.5*(zlim[0]+zlim[1])
    Yb = 0.5*max_range*np.mgrid[-1:2:2, -1:2:2, -1:2:2][1].flatten() + 0.5*(xlim[0]+xlim[1])
    Zb = 0.5*max_range*np.mgrid[-1:2:2, -1:2:2, -1:2:2][2].flatten() + 0.5*(ylim[0]+ylim[1])
    Zb = Zb - Zb.min() + ylim[0]

    # Comment or uncomment following both lines to test the fake bounding box:
    for xb, yb, zb in zip(Xb, Yb, Zb):
      ax.plot([xb], [yb], [zb], 'w')

    plt.grid()

    filename = os.path.join(directory, 'rough_fiber_mesh_3d.png')
    plt.savefig(filename, dpi=400)

    # plot a top view of the mesh
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')


    for f in fibers[begin:begin+n_fibers]:
      ax.plot(f.z(mode), f.x(mode))

    ax.set_xlim([-1500, 1500])
    ax.set_ylim([-1500, 1500])
    ax.set_xlabel("[nanometers]")
    ax.set_ylabel("[nanometers]")

    filename = os.path.join(directory, 'rough_fiber_mesh_top_view.png')
    plt.savefig(filename, dpi=400)

    plt.show()

  if args.check_interpolation:

    m = []
    std = []

    for f in fibers[:]:
      dr = np.diff(f.r(mode='fine'), axis=0)
      dr = np.linalg.norm(dr, axis=1)
      m.append(np.mean(dr))
      std.append(np.std(dr))

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(m, label='mean value per fiber')
    ax.plot(std, label='standard deviation per fiber')
    ax.legend()

    plt.show()

  if args.nearest_neighbor:
    min_dist_per_cnt, pair_min_dist = min_neighbor_distance(fibers, mode='fine', n=1000)

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(min_dist_per_cnt)
    ax.set_title("Minimum distance of a CNT fiber with a neighboring fiber")

    filename = os.path.join(directory, 'minimum_distance_per_fiber.png')
    plt.savefig(filename, dpi=400)


    hist_min_dist, bins = np.histogram(pair_min_dist, bins=1000, range=(0, 50), density=True)
    bins = (bins[:-1]+bins[1:])/2

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    _ = ax.plot(bins, hist_min_dist)
    ax.set_title("Probability distribution function of minimum distances between CNT fibers")
    ax.set_xlabel("distance [nm]")
    ax.set_ylabel("Probability distribution")

    filename = os.path.join(directory, 'distance_distribution.png')
    plt.savefig(filename, dpi=400)

    plt.show()

  if args.plot_cnts:
    fiber_diameter = 5
    cnt_diameter = 1.4

    coor = util.HCP_coordinates(fiber_diameter, cnt_diameter)
    print(f"number of cnts per fiber: {coor.shape}")

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.set_aspect('equal')
    ax.plot(coor[:, 0], coor[:, 1], 'o')

    theta = np.linspace(0, 2*np.pi, 100)
    x = fiber_diameter*np.cos(theta)
    y = fiber_diameter*np.sin(theta)
    ax.plot(x, y)
    
    cnt_pos = []
    cnt_orient = []

    for fib in tqdm(fibers[:10]):
      p, o = create_single_CNTs(fib, coor)
      cnt_pos.append(p)
      cnt_orient.append(o)

    cnt_pos = np.array(cnt_pos)
    cnt_orient = np.array(cnt_orient)

    # fig = plt.figure()
    # ax = fig.add_subplot(1, 1, 1, projection='3d')
    # ax.set_aspect('equal')
    # for f in range(1):
    #   for r in cnt_pos[f, :, :, :]:
    #     ax.plot(r[:, 2], r[:, 0], r[:, 1])

    # ax.view_init(elev=10., azim=np.pi/2)
    # plt.show()

    coor_per_cnt = cnt_pos.shape[2]
    cnt_pos = cnt_pos.reshape((-1, coor_per_cnt, 3))
    cnt_orient = cnt_orient.reshape((-1, coor_per_cnt, 3))

    # plot cnt's using plotly
    data = []
    for r in cnt_pos[:, :, :]:
      trace = go.Scatter3d(
        x=r[:, 2], y=r[:, 0], z=r[:, 1],
        marker={'size':1, 'colorscale':'Viridis'},
        line={'width':10}
      )
      data.append(trace)

    layout = {
        'autosize':True,
        'title':'Iris dataset',
        'scene':{
          'xaxis':{
            'gridcolor':'rgb(255, 255, 255)',
            'zerolinecolor':'rgb(255, 255, 255)',
            'showbackground':True,
            'backgroundcolor':'rgb(230, 230,230)'
          },
          'yaxis':{
            'gridcolor':'rgb(255, 255, 255)',
            'zerolinecolor':'rgb(255, 255, 255)',
            'showbackground':True,
            'backgroundcolor':'rgb(230, 230,230)'
          },
          'zaxis':{
            'gridcolor':'rgb(255, 255, 255)',
            'zerolinecolor':'rgb(255, 255, 255)',
            'showbackground':True,
            'backgroundcolor':'rgb(230, 230,230)'
          },
          'aspectratio':{'x':1, 'y':1, 'z':0.02}
        },
    }

    fig = go.Figure(data=data, layout=layout)
    plo.plot(fig, filename=os.path.join(directory, "temp-pyplot.html"))

  if args.create_cnts:
    fiber_diameter = 5
    cnt_diameter = 1.4

    coor = util.HCP_coordinates(fiber_diameter, cnt_diameter)
    print(f'number of cnts per fiber: {coor.shape[0]}')

    cnt_pos = []
    cnt_orient = []

    for fib in tqdm(fibers):
      p, o = create_single_CNTs(fib, coor)
      cnt_pos.append(p)
      cnt_orient.append(o)

    cnt_pos = np.array(cnt_pos)
    cnt_orient = np.array(cnt_orient)

    coor_per_cnt = cnt_pos.shape[2]
    cnt_pos = cnt_pos.reshape((-1, coor_per_cnt, 3))
    cnt_orient = cnt_orient.reshape((-1, coor_per_cnt, 3))

    
    header = 'ARMA_MAT_TXT_FN008\n'
    header += f'{cnt_pos.shape[0]} {cnt_pos.shape[1]}\n'
    header += f'fiber diameter: {fiber_diameter}, cnt diameter: {cnt_diameter}'
    fmt = "%+.4e"
    cmts = ""

    filename = os.path.join(directory, "single_cnt.pos.x.dat")
    np.savetxt(filename, cnt_pos[:, :, 0], header=header, fmt=fmt, comments=cmts)
    
    filename = os.path.join(directory, "single_cnt.pos.y.dat")
    np.savetxt(filename, cnt_pos[:, :, 1], header=header, fmt=fmt, comments=cmts)

    filename = os.path.join(directory, "single_cnt.pos.z.dat")
    np.savetxt(filename, cnt_pos[:, :, 2], header=header, fmt=fmt, comments=cmts)

    filename = os.path.join(directory, "single_cnt.pos.x")
    np.save(filename, cnt_pos[:, :, 0])

    filename = os.path.join(directory, "single_cnt.pos.y")
    np.save(filename, cnt_pos[:, :, 1])

    filename = os.path.join(directory, "single_cnt.pos.z")
    np.save(filename, cnt_pos[:, :, 2])

    filename = os.path.join(directory, "single_cnt.orient.x.dat")
    np.savetxt(filename, cnt_orient[:, :, 0], header=header, fmt=fmt, comments=cmts)

    filename = os.path.join(directory, "single_cnt.orient.y.dat")
    np.savetxt(filename, cnt_orient[:, :, 1], header=header, fmt=fmt, comments=cmts)

    filename = os.path.join(directory, "single_cnt.orient.z.dat")
    np.savetxt(filename, cnt_orient[:, :, 2], header=header, fmt=fmt, comments=cmts)

  if args.check_cnt_nearest_neighbor:

    filename = os.path.join(directory, "single_cnt.pos.x.npy")
    x = np.load(filename)
    filename = os.path.join(directory, "single_cnt.pos.y.npy")
    y = np.load(filename)
    filename = os.path.join(directory, "single_cnt.pos.z.npy")
    z = np.load(filename)

    for i in range(x.shape[1]):
      print(x[0,i], y[0,i], z[0,i])

    # print(x.shape)

    # print('***********')
    # for i in range(2):
    #   print(x[i])

    # print('***********')

    # x = x.flatten()
    # for i in range(200):
    #   print(x[i])

    # print(type(x[0]))

    # filename = os.path.join(directory, "single_cnt.pos.x.dat")
    # x = np.loadtxt(filename, skiprows=3)

    # filename = os.path.join(directory, "single_cnt.pos.y.dat")
    # y = np.loadtxt(filename, skiprows=3)

    # filename = os.path.join(directory, "single_cnt.pos.z.dat")
    # z = np.loadtxt(filename, skiprows=3)

    # cnt_pos = np.stack((x, y, z), axis=2)
    # del x, y, z

    # print(cnt_pos.shape)
    # # input('done with loading!!!')

    # # n = cnt_pos.shape[0]
    # # pair_min_dist = np.full((n,n),1.e14)

    # # input('done with preparation')

    # temp = np.zeros((cnt_pos.shape[1], cnt_pos.shape[1], 3))

    # min_dist_per_cnt = []
    # for i, c1 in enumerate(tqdm(cnt_pos)):
    #   pair_min_dist = []
    #   for j, c2 in enumerate(cnt_pos):
    #     if i == j:
    #       continue
    #     for d in range(3):
    #       x, y = np.meshgrid(c1[:, d], c2[:, d])
    #       temp[:, :, d] = x-y
    #     t = np.linalg.norm(temp, axis=2).min()
    #     pair_min_dist.append(t.min())
    #   min_dist_per_cnt.append(np.amin(pair_min_dist))

    # # min_dist_per_cnt = pair_min_dist.min(axis=1)

    # hist_min_dist, bins = np.histogram(pair_min_dist, bins=1000, range=(0, 50), density=True)
    # bins = (bins[:-1]+bins[1:])/2

    # fig = plt.figure()
    # ax = fig.add_subplot(1, 1, 1)
    # _ = ax.plot(bins, hist_min_dist)
    # ax.set_title("Probability distribution function of minimum distances between CNT fibers")
    # ax.set_xlabel("distance [nm]")
    # ax.set_ylabel("Probability distribution")

    # filename = os.path.join(directory, 'distance_distribution_cnt.png')
    # plt.savefig(filename, dpi=400)

    # plt.show()


if __name__ == '__main__':
  main()

