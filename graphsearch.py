from copy import copy
from math import sqrt, pow
from imageManipulator import PIXEL
from typing import List, Tuple, TypeVar
import heapq
T = TypeVar('T')

# next imports are used only to show a graph
'''
import imageManipulator as im
import matplotlib.pyplot as plt
import numpy as np
from problem import Problem
'''

# The Node class define a node in the graph, will be a pixel in the image
class Node:
  def __init__(self, state, path_cost, parent):
    self.state = state
    self.path_cost = path_cost
    self.parent = parent
  
  # Define hash and eq to use Node as a hash
  def __hash__(self):
    return hash(self.state)

  def __eq__(self, other):
    if isinstance(other, tuple):
      return self.state == other
    elif other != None:
      return self.state == other.state
    else:
      return False

  # def the custom < and > operators
  def __lt__(self, other):
    if isinstance(other, tuple):
      return 1
    elif other != None:
      return self.path_cost < other.path_cost
    else:
      return False

  def __gt__(self, other):
    if isinstance(other, tuple):
      return 1
    elif other != None:
      return self.path_cost > other.path_cost
    else:
      return False

# Priority queue used in A* https://www.redblobgames.com/pathfinding/a-star/implementation.py
class PriorityQueue:
    def __init__(self):
        self.elements: List[Tuple[float, T]] = []
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self) -> T:
        return heapq.heappop(self.elements)[1]

    # Return an iterable list of the elements in the queue
    def get_list(self) -> List[T]:
        return [item[1] for item in self.elements]

# Breadth First Search
class BFS:
  def __init__(self, problem):
    self.problem = problem
    self.problem.actions = self.actions
    self.problem.goal_test = self.goal_test
    self.problem.path_cost = self.solution

  def execute(self):
    self.node = Node(self.problem.initial_state[0], 1, None)
    if self.problem.goal_test(self.node.state):
      self.problem.path_cost(self.node)
    self.frontier = [self.node]
    self.explored = []
    while len(self.frontier) > 0:
      self.node = self.frontier.pop(0)
      self.explored.append(self.node)
      for action in self.problem.actions(self.node.state):
        self.child = Node(action, 1, self.node)
        if not self.is_in(self.child, self.explored) and not self.is_in(self.child, self.frontier):
          if self.problem.goal_test(self.child.state):
            return self.problem.path_cost(self.child)
          self.frontier.append(self.child)

  # Return an array of states
  def actions(self, state):
    return actions(state, self.problem.matrix)

  def goal_test(self, state):
    if state in self.problem.results:
      return True
    return False

  # Go through the references of parents from the goal node
  def solution(self, node):
    self.backward = node.parent
    while self.backward.parent != None:
      self.problem.path.append(copy(self.backward))
      self.backward = self.backward.parent
    return self.problem.path, self.explored, self.frontier

  # Determine if Node is on list based on state
  def is_in(self, node, list):
    for n in list:
      if n.state == node.state:
        return True
    return False

# Depth First Search
class DFS:
  def __init__(self, problem, image_url):
    self.problem = problem
    self.problem.actions = self.actions
    self.problem.goal_test = self.goal_test
    self.problem.result = self.solution
    self.frontier = []
    self.explored = []
    # Commented declarations used to show a graph
    '''
    self.img = im.read_image(image_url)
    self.img, self.startpoints, self.endpoints = im.reduce_image(self.img)
    '''

  def execute(self):
    self.node = Node(self.problem.initial_state[0], 1, None)
    self.recursive_dls(self.node)
    return self.problem.result(self.child)

  # Used to validate and recursive on every node (child node)
  def recursive_dls(self, node):
    if self.problem.goal_test(node.state):
      return True
    else:
      # If the node was already on frontier, use the parent of this to reduce the path
      if self.is_in(node, self.frontier):
        for i in range(len(self.frontier)):
          if self.frontier[i].state == node.state:
            # only use the parent if the parent of actual node was already discarded and if the parent of both nodes are the same
            if not ((not self.is_in(self.frontier[i].parent, self.problem.path)) and self.frontier[i].parent.state != node.parent.state):
              node.parent = self.frontier[i].parent
            break

      self.explored.append(node)
      # find and remove element in frontier
      if self.is_in(node, self.frontier):
        for i in range(len(self.frontier)):
          if self.frontier[i].state == node.state:
            self.frontier.pop(i)
            break

      # Calculate the path (result) is necessary to go discarding nodes
      if node.parent != None:
        self.problem.result(node)

      # Code commented show the plot of the path on every step
      '''
      if node.parent != None:
        path, explored, frontier = self.problem.result(node)
        self.img = im.replace_image(self.img, path, explored, frontier)
        # Show result with path
        plt.imshow(self.img)
        plt.show()
      '''

      # Add childs to frontier if not in explored or frontier
      for action in self.problem.actions(node.state):
        if not self.is_in(Node(action, 1, node), self.frontier) and not self.is_in(Node(action, 1, node), self.explored):
          self.frontier.append(Node(action, 1, node))

      # Do the recursive for every child
      self.result = False
      for action in self.problem.actions(node.state):
        self.child = Node(action, 1, node)
        if (not self.is_in(self.child, self.explored) ):
          self.result = self.recursive_dls(self.child)
          # If the child was the goal, return True, breaking every recursive for loop (of childs)
          if (self.result):
            break
      return self.result

  def is_in(self, node, list):
    for n in list:
      if n.state == node.state:
        return True
    return False

  def actions(self, state):
    return actions(state, self.problem.matrix)

  def goal_test(self, state):
    if state in self.problem.results:
      return True
    return False

  def solution(self, node):
    self.problem.path = []
    self.backward = node.parent
    while self.backward.parent != None:
      self.problem.path.append(copy(self.backward))
      self.backward = self.backward.parent
    return self.problem.path, self.explored, self.frontier

# A*
class a_star:
  def __init__(self, problem):
    self.problem = problem
    self.problem.actions = self.actions
    self.problem.goal_test = self.goal_test
    self.problem.path_cost = self.solution
    self.problem.step_cost = self.step_cost

  def execute(self):
    self.node = Node(self.problem.initial_state[0], 1, None)
    self.frontier = PriorityQueue()
    self.frontier.put(self.node, 0)
    self.came_from = {}
    self.cost_so_far = {}
    self.came_from[self.node] = None
    self.cost_so_far[self.node] = 0

    # reference https://www.redblobgames.com/pathfinding/a-star/implementation.html
    while not self.frontier.empty():
      self.current = self.frontier.get()

      if self.problem.goal_test(self.current):
        return self.problem.path_cost(self.current) # TODO: Do this or only return?

      # Do for every (priority) Node on frontier while goal is not found
      for next in self.problem.actions(self.current.state):
        self.child = Node(next, 1, self.current)
        # Step cost always be 1 for this matrix
        self.new_cost = self.cost_so_far[self.current] + self.problem.step_cost(self.current, self.child)
        # If child not in "frontier"
        # If the cost of the parent + step cost (1) is less than the cost of the child
        if self.child not in self.cost_so_far or self.new_cost < self.cost_so_far[self.child]:
          self.cost_so_far[self.child] = self.new_cost
          # Calculate priority based on the cost and heuristics (less is better priority)
          self.priority = self.new_cost + self.heuristic1(self.child) + self.heuristic2(self.child)
          self.frontier.put(self.child, self.priority)
          self.came_from[self.child] = self.current
    return self.problem.path_cost(self.current)

  # Return an array of states
  def actions(self, state):
    return actions(state, self.problem.matrix)

  # By the distance from current to goal
  def heuristic1(self, g):
    fs = []
    (x1, y1) = g.state
    for goal in self.problem.results:
      (x2, y2) = goal
      fs.append(sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)))
    return min(fs)

  # By the direction from start to current and start to goal
  def heuristic2(self, g):
    results = []
    start = self.problem.initial_state[0]
    current = g.state
    for goal in self.problem.results:
      # producto punto entre el vector de direccion y el vector de direccion al goal
      x = (current[0] - start[0], current[1] - start[1])
      y = (goal[0] - start[0], goal[1] - start[1])
      result = (x[0] * y[0]) + (x[1] * y[1])
      if result <= 0:
        results.append(abs(result))
      else:
        results.append(0)
    return results[0] + results[1]

  def goal_test(self, state):
    if state in self.problem.results:
      return True
    return False

  def solution(self, node):
    self.backward = node.parent
    while self.backward.parent != None:
      self.problem.path.append(copy(self.backward))
      self.backward = self.backward.parent
    return self.problem.path, self.came_from, self.frontier.get_list()

  # on the matrix, every cost from a current node to next node is 1
  def step_cost(self, current, next):
    return 1

# Used on the three algorithms. Return an array of states
def actions(state, matrix):
  available_nodes = []
  if (state[0] - 1 > 0) and matrix.getpixel((state[0] - 1, state[1])) != PIXEL['BLACK']:
    available_nodes.append((state[0] - 1, state[1]))

  if (state[1] - 1 > 0) and matrix.getpixel((state[0], state[1] - 1)) != PIXEL['BLACK']:
    available_nodes.append((state[0], state[1] - 1))
  if (state[1] + 1 < matrix.size[1]) and matrix.getpixel((state[0], state[1] + 1)) != PIXEL['BLACK']:
    available_nodes.append((state[0], state[1] + 1))

  if (state[0] + 1 < matrix.size[0]) and matrix.getpixel((state[0] + 1, state[1])) != PIXEL['BLACK']:
    available_nodes.append((state[0] + 1, state[1]))
  return available_nodes
