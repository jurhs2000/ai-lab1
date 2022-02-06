from copy import copy
from imageManipulator import PIXEL

# Breadth First Search

class Node:
  def __init__(self, state, path_cost, parent):
    self.state = state
    self.path_cost = path_cost
    self.parent = parent

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
    available_nodes = []
    if self.problem.matrix.getpixel((state[0] - 1, state[1] - 1)) != PIXEL['BLACK']:
      available_nodes.append((state[0] - 1, state[1] - 1))
    if self.problem.matrix.getpixel((state[0] - 1, state[1])) != PIXEL['BLACK']:
      available_nodes.append((state[0] - 1, state[1]))
    if self.problem.matrix.getpixel((state[0] - 1, state[1] + 1)) != PIXEL['BLACK']:
      available_nodes.append((state[0] - 1, state[1] + 1))

    if self.problem.matrix.getpixel((state[0], state[1] - 1)) != PIXEL['BLACK']:
      available_nodes.append((state[0], state[1] - 1))
    if self.problem.matrix.getpixel((state[0], state[1] + 1)) != PIXEL['BLACK']:
      available_nodes.append((state[0], state[1] + 1))

    if self.problem.matrix.getpixel((state[0] + 1, state[1] - 1)) != PIXEL['BLACK']:
      available_nodes.append((state[0] + 1, state[1] - 1))
    if self.problem.matrix.getpixel((state[0] + 1, state[1])) != PIXEL['BLACK']:
      available_nodes.append((state[0] + 1, state[1]))
    if self.problem.matrix.getpixel((state[0] + 1, state[1] + 1)) != PIXEL['BLACK']:
      available_nodes.append((state[0] + 1, state[1] + 1))
    return available_nodes

  def goal_test(self, state):
    if state in self.problem.results:
      return True
    return False

  def solution(self, node):
    self.backward = node.parent
    while self.backward.parent != None:
      self.problem.path.append(copy(self.backward))
      self.backward = self.backward.parent
    return self.problem.path

  def is_in(self, node, list):
    for n in list:
      if n.state == node.state:
        return True
    return False

# Depth First Search
class DFS:
  def __init__(self, problem):
    self.problem = problem

# Heuristic Search 1
class H1:
  def __init__(self, problem):
    self.problem = problem

# Heuristic Search 2
class H2:
  def __init__(self, problem):
    self.problem = problem
