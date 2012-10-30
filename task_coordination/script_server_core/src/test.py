#!/usr/bin/python

import script_server; script_server.init(["script_server_youbot_util"])

if __name__ == "__main__":

    #for a in script_server.actions:
    #    print("Action: %s - %s" % (a, script_server.actions[a].action_name))

    script_server.test("foo")
