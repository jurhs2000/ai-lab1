from copy import copy
from math import sqrt, pow
from imageManipulator import PIXEL
from typing import List, Tuple, TypeVar
import heapq
T = TypeVar('T')

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

class PriorityQueue:
    def __init__(self):
        self.elements: List[Tuple[float, T]] = []
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self) -> T:
        return heapq.heappop(self.elements)[1]

    def getList(self) -> List[T]:
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

  def solution(self, node):
    self.backward = node.parent
    while self.backward.parent != None:
      self.problem.path.append(copy(self.backward))
      self.backward = self.backward.parent
    return self.problem.path, self.explored, self.frontier

  def is_in(self, node, list):
    for n in list:
      if n.state == node.state:
        return True
    return False

# Depth First Search
class DFS:
  def __init__(self, problem):
    self.problem = problem

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

      for next in self.problem.actions(self.current.state):
        self.child = Node(next, 1, self.current)
        self.new_cost = self.cost_so_far[self.current] + self.problem.step_cost(self.current, self.child)
        if self.child not in self.cost_so_far or self.new_cost < self.cost_so_far[self.child]:
          self.cost_so_far[self.child] = self.new_cost
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
    return self.problem.path, self.came_from, self.frontier.getList()

  # on the matrix, every cost from a current node to next node is 1
  def step_cost(self, current, next):
    return 1

# Return an array of states
def actions(state, matrix):
  available_nodes = []
  #if matrix.getpixel((state[0] - 1, state[1] - 1)) != PIXEL['BLACK']:
  #  available_nodes.append((state[0] - 1, state[1] - 1))
  if (state[0] - 1 > 0) and matrix.getpixel((state[0] - 1, state[1])) != PIXEL['BLACK']:
    available_nodes.append((state[0] - 1, state[1]))
  #if matrix.getpixel((state[0] - 1, state[1] + 1)) != PIXEL['BLACK']:
  #  available_nodes.append((state[0] - 1, state[1] + 1))

  if (state[1] - 1 > 0) and matrix.getpixel((state[0], state[1] - 1)) != PIXEL['BLACK']:
    available_nodes.append((state[0], state[1] - 1))
  if (state[1] + 1 < matrix.size[1]) and matrix.getpixel((state[0], state[1] + 1)) != PIXEL['BLACK']:
    available_nodes.append((state[0], state[1] + 1))

  #if matrix.getpixel((state[0] + 1, state[1] - 1)) != PIXEL['BLACK']:
  #  available_nodes.append((state[0] + 1, state[1] - 1))
  if (state[0] + 1 < matrix.size[0]) and matrix.getpixel((state[0] + 1, state[1])) != PIXEL['BLACK']:
    available_nodes.append((state[0] + 1, state[1]))
  #if matrix.getpixel((state[0] + 1, state[1] + 1)) != PIXEL['BLACK']:
  #  available_nodes.append((state[0] + 1, state[1] + 1))
  return available_nodes