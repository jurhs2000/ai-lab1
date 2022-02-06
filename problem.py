# Class Problem to define the formal problem for path finding
class Problem:
  def __init__(self, matrix, initial_state, results):
    self.matrix = matrix
    self.initial_state = initial_state
    self.results = results
    self.path = []

  # Return a set of actions fot the given state
  def actions(self, state):
    pass

  # Return a state
  def result(self, state, action):
    pass

  # Return True or False
  def goal_test(self, state):
    pass

  # Return float value of the cost of the step (action)
  def step_cost(self, state, action):
    pass

  # Return float value of the cost of the path
  # Receive a list of steps
  def path_cost(self, path):
    pass
