
## Simple script server actions
#
# Implements the python interface for the script server actions.
class AbstractAction:

    # You must override this!
    action_name = None
    disabled = False

    def __call__(self, *args, **kwargs):
        ret = self.execute(*args, **kwargs)
        print ret
        return ret 

    def execute(self, *args, **kwargs):
        raise NotImplementedError("Execute not implemented")
