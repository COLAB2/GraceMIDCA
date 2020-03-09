# This file contains helpful functions for the nbeacons domain

import os
'''
 translate MIDCA init state to problem file in JSHOP

'''

def world_display(world):
    print(world)

def preferApprehend(goal1, goal2):
    if 'predicate' not in goal1 or 'predicate' not in goal2:
        return 0

    elif goal1['predicate'] == 'cleaned_vines' and goal2['predicate'] != 'cleaned_vines':
        return -1
    elif goal1['predicate'] != 'cleaned_vines' and goal2['predicate'] == 'cleaned_vines':
        return 1

    return 0

def jshop2_state_from_world(world, STATE_FILE, name = "state"):
    thisDir =  os.path.dirname(os.path.realpath(__file__))
#     MIDCA_ROOT = thisDir + "/../"
# #     STATE_FILE = MIDCA_ROOT + "jshop_domains/logistics/problem"
#     STATE_FILE = "C:/Users/Zohreh/git/MIDCA/modules/_plan/jShop/problem"
    f = open(STATE_FILE, 'w')
    f.write('\n')
    f.write("(defproblem problem fish-diving ((loc)\n")
    f.write("\n")

    for obj in world.objects.keys():


        if world.objects[obj].type.name == "AIRPORT":
            f.write("(AIRPORT " + obj + ")\n")



    for atom in world.atoms:

        if atom.predicate.name == "at_pooldepth":
            f.write("(at_pooldepth " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
        elif atom.predicate.name == "recorded_depth":
            f.write("(recorded_depth " + atom.args[0].name + " " +  atom.args[1].name + ")\n")
        elif atom.predicate.name == "enabled":
            f.write("(enabled " + atom.args[0].name + " )\n")

    f.write(")\n")
    f.close()

'''
translate MIDCA goal to JSHOP tasks
'''


def jshop2_tasks_from_goals(goals,pyhopState, STATE_FILE):
    thisDir =  os.path.dirname(os.path.realpath(__file__))
#     MIDCA_ROOT = thisDir + "/../"
# #     STATE_FILE = MIDCA_ROOT + "jshop_domains/logistics/problem"
#     STATE_FILE = "C:/Users/Zohreh/git/MIDCA/modules/_plan/jShop/problem"
    f = open(STATE_FILE, 'a')

    alltasks = []
    f.write(" ((achieve-goals (list\n")
    for goal in goals:
        #extract predicate
        if 'predicate' in goal.kwargs:
            predicate = str(goal.kwargs['predicate'])
        elif 'Predicate' in goal.kwargs:
            predicate = str(goal.kwargs['Predicate'])
        elif goal.args:
            predicate = str(goal.args[0])
        else:
            raise ValueError("Goal " + str(goal) + " does not translate to a valid pyhop task")
        args = [str(arg) for arg in goal.args]
        if args[0] == predicate:
            args.pop(0)
        if predicate == "communicated_depth":
            f.write("(communicated_depth " +  args[0] + " " +  args[1] + " " +  args[2] + ")\n")
        elif predicate == "cleaned_remora":
            f.write("(cleaned_remora " +  args[0] + ")\n")
        else:
            raise Exception("No task corresponds to predicate " + predicate)
    f.write(" ))))")
    f.close()
