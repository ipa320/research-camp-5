
## Simple script server actions
#
# Implements the python interface for the script server actions.
class BaseAction:

    # You must override this!
    action_name = None
    disabled = False

    def __call__(self, *args, **kwargs):
        return self.execute(*args, **kwargs)

    def execute(self):
        raise NotImplementedError("Execute not implemented")
