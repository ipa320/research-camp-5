
## Simple script server actions
#
# Implements the python interface for the script server actions.
class BaseAction:

    # You must override this!
    action_name = None
    disabled = False

    def __init__(self, actions):
        self.actions = actions

    def __call__(self, *args, **kwargs):
        return self.execute(*args, **kwargs)

    def execute(self, *args, **kwargs):
        raise NotImplementedError("Execute not implemented")
