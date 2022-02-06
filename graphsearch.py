from imageManipulator import PIXEL

# Breadth First Search

class Node:
  def __init__(self, state, path_cost):
    self.state = state
    self.path_cost = path_cost

class BFS:
  def __init__(self, problem):
    self.problem = problem
    self.problem.actions = self.actions
    self.problem.goal_test = self.goal_test
    self.problem.path_cost = self.solution

  def execute(self):
    self.node = Node(self.problem.initial_state[0], 1)
    if self.problem.goal_test(self.node.state):
      self.problem.path_cost(self.node)
    self.frontier = [self.node]
    self.explored = []
    while len(self.frontier) > 0:
      self.node = self.frontier.pop(0)
      self.explored.append(self.node)
      for action in self.problem.actions(self.node.state):
        self.child = Node(action, 1)
        if self.child not in self.explored or self.child not in self.frontier:
          if self.problem.goal_test(self.child.state):
            return self.problem.path_cost(self.child)
          self.frontier.append(self.child)

  # Return an array of states
  def actions(self, state):
    available_nodes = []
    print(state)
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
    self.problem.path.append(node)
    return self.problem.path

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
