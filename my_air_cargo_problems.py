from aimacode.logic import PropKB
from aimacode.planning import Action
from aimacode.search import (
    Node, Problem,
)
from aimacode.utils import expr
from lp_utils import (
    FluentState, encode_state, decode_state,
)
from my_planning_graph import PlanningGraph
from functools import lru_cache

class AirCargoProblem(Problem):
    def __init__(self, cargos, planes, airports, initial: FluentState, goal: list):
        """

        :param cargos: list of str
            cargos in the problem
        :param planes: list of str
            planes in the problem
        :param airports: list of str
            airports in the problem
        :param initial: FluentState object
            positive and negative literal fluents (as expr) describing initial state
        :param goal: list of expr
            literal fluents required for goal test
        """
        self.state_map = initial.pos + initial.neg
        self.initial_state_TF = encode_state(initial, self.state_map)
        Problem.__init__(self, self.initial_state_TF, goal=goal)
        self.cargos = cargos
        self.planes = planes
        self.airports = airports
        self.actions_list = self.get_actions()

    def get_actions(self):
        '''
        This method creates concrete actions (no variables) for all actions in the problem
        domain action schema and turns them into complete Action objects as defined in the
        aimacode.planning module. It is computationally expensive to call this method directly;
        however, it is called in the constructor and the results cached in the `actions_list` property.

        Returns:
        ----------
        list<Action>
            list of Action objects
        '''

        # TODO create concrete Action objects based on the domain action schema for: Load, Unload, and Fly
        # concrete actions definition: specific literal action that does not include variables as with the schema
        # for example, the action schema 'Load(c, p, a)' can represent the concrete actions 'Load(C1, P1, SFO)'
        # or 'Load(C2, P2, JFK)'.  The actions for the planning problem must be concrete because the problems in
        # forward search and Planning Graphs must use Propositional Logic
        
        cpa_combinations = [(c, p, a) for c in self.cargos
                            for p in self.planes for a in self.airports]
        def load_actions():
            '''Create all concrete Load actions and return a list

            :return: list of Action objects
            '''
            loads = []
            # TODO create all load ground actions from the domain Load action
            for c, p, a in cpa_combinations:
                format_dict = {'c': c, 'a': a, 'p': p}
                precond_pos = [expr('At({c}, {a})'.format(**format_dict)),
                               expr('At({p}, {a})'.format(**format_dict))]
                effect_add = [expr('In({c}, {p})'.format(**format_dict))]
                effect_rem = [expr('At({c}, {a})'.format(**format_dict))]
                loads.append(Action(expr('Load({c}, {p}, {a})'.format(**format_dict)),
                                    [precond_pos, []], [effect_add, effect_rem]))
            return loads

        def unload_actions():
            '''Create all concrete Unload actions and return a list

            :return: list of Action objects
            '''
            unloads = []
            # TODO create all Unload ground actions from the domain Unload action
            for c, p, a in cpa_combinations:
                format_dict = {'c': c, 'a': a, 'p': p}
                precond_pos = [expr('In({c}, {p})'.format(**format_dict)),
                               expr('At({p}, {a})'.format(**format_dict))]
                effect_add = [expr('At({c}, {a})'.format(**format_dict))]
                effect_rem = [expr('In({c}, {p})'.format(**format_dict))]
                unloads.append(
                    Action(expr('Unload({c}, {p}, {a})'.format(**format_dict)),
                           [precond_pos, []], [effect_add, effect_rem])
                )
            return unloads

        def fly_actions():
            '''Create all concrete Fly actions and return a list

            :return: list of Action objects
            '''
            flys = []
            for fr in self.airports:
                for to in self.airports:
                    if fr != to:
                        for p in self.planes:
                            precond_pos = [expr("At({}, {})".format(p, fr)),
                                           ]
                            precond_neg = []
                            effect_add = [expr("At({}, {})".format(p, to))]
                            effect_rem = [expr("At({}, {})".format(p, fr))]
                            fly = Action(expr("Fly({}, {}, {})".format(p, fr, to)),
                                         [precond_pos, precond_neg],
                                         [effect_add, effect_rem])
                            flys.append(fly)
            return flys

        return load_actions() + unload_actions() + fly_actions()

    def actions(self, state: str) -> list:
        """ Return the actions that can be executed in the given state.

        :param state: str
            state represented as T/F string of mapped fluents (state variables)
            e.g. 'FTTTFF'
        :return: list of Action objects
        """
        # TODO implement
        kb = PropKB()
        kb.tell(decode_state(state, self.state_map).pos_sentence())
        possible_actions = []
        for a in self.actions_list:
            if a.check_precond(kb, a.args):
                possible_actions.append(a)
        return possible_actions

    def result(self, state: str, action: Action):
        """ Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state).

        :param state: state entering node
        :param action: Action applied
        :return: resulting state after action
        """
        # TODO implement
        kb = PropKB()
        kb.tell(decode_state(state, self.state_map).pos_sentence()) 
        action.act(kb, action.args)
        new_state = FluentState(kb.clauses, [])
        return encode_state(new_state, self.state_map)

    def goal_test(self, state: str) -> bool:
        """ Test the state to see if goal is reached

        :param state: str representing state
        :return: bool
        """
        kb = PropKB()
        kb.tell(decode_state(state, self.state_map).pos_sentence())
        for clause in self.goal:
            if clause not in kb.clauses:
                return False
        return True

    def h_1(self, node: Node):
        # note that this is not a true heuristic
        h_const = 1
        return h_const
    
    @lru_cache(maxsize=8192)
    def h_pg_levelsum(self, node: Node):
        '''
        This heuristic uses a planning graph representation of the problem
        state space to estimate the sum of all actions that must be carried
        out from the current state in order to satisfy each individual goal
        condition.
        '''
        # requires implemented PlanningGraph class
        pg = PlanningGraph(self, node.state)
        pg_levelsum = pg.h_levelsum()
        return pg_levelsum
    
    @lru_cache(maxsize=8192)
    def h_ignore_preconditions(self, node: Node):
        '''
        This heuristic estimates the minimum number of actions that must be
        carried out from the current state in order to satisfy all of the goal
        conditions by ignoring the preconditions required for an action to be
        executed.
        '''
        # TODO implement (see Russell-Norvig Ed-3 10.2.3  or Russell-Norvig Ed-2 11.2)
        # Assume sub-goal independence
        unrealized_goals = []
        kb = PropKB()
        kb.tell(decode_state(node.state, self.state_map).pos_sentence())
        count = 0
        for clause in self.goal:
            if clause not in kb.clauses:
                count += 1
        return count

    
def air_cargo_p1() -> AirCargoProblem:
    cargos = ['C1', 'C2']
    planes = ['P1', 'P2']
    airports = ['JFK', 'SFO']
    pos = [expr('At(C1, SFO)'),
           expr('At(C2, JFK)'),
           expr('At(P1, SFO)'),
           expr('At(P2, JFK)'),
           ]
    neg = [expr('At(C2, SFO)'),
           expr('In(C2, P1)'),
           expr('In(C2, P2)'),
           expr('At(C1, JFK)'),
           expr('In(C1, P1)'),
           expr('In(C1, P2)'),
           expr('At(P1, JFK)'),
           expr('At(P2, SFO)'),
           ]
    init = FluentState(pos, neg)
    goal = [expr('At(C1, JFK)'),
            expr('At(C2, SFO)'),
            ]
    return AirCargoProblem(cargos, planes, airports, init, goal)


def air_cargo_p2() -> AirCargoProblem:
    cargos = ['C1', 'C2', 'C3']
    planes = ['P1', 'P2', 'P3']
    airports = ['JFK', 'SFO', 'ATL']
    pos = [expr('At(C1, SFO)'), expr('At(C2, JFK)'), expr('At(C3, ATL)'),
           expr('At(P1, SFO)'), expr('At(P2, JFK)'), expr('At(P3, ATL)')]
    neg = [expr('At(C1, JFK)'), expr('At(C1, ATL)'),
           expr('At(C2, SFO)'), expr('At(C2, ATL)'),
           expr('At(C3, SFO)'), expr('At(C3, JFK)'),
           expr('At(P1, JFK)'), expr('At(P1, ATL)'),
           expr('At(P2, SFO)'), expr('At(P2, ATL)'),
           expr('At(P3, SFO)'), expr('At(P3, JFK)'),
    ]
    neg += [expr('In({}, {})'.format(c, p)) for c in cargos for p in planes]
    init = FluentState(pos, neg)
    goal = [expr('At(C1, JFK)'), expr('At(C2, SFO)'), expr('At(C3, SFO)')]
    return AirCargoProblem(cargos, planes, airports, init, goal)


def air_cargo_p3() -> AirCargoProblem:
    cargos = ['C1', 'C2', 'C3', 'C4']
    planes = ['P1', 'P2']
    airports = ['JFK', 'SFO', 'ATL', 'ORD']
    pos = [
        expr('At(C1, SFO)'), expr('At(C2, JFK)'), expr('At(C3, ATL)'),
        expr('At(C4, ORD)'), expr('At(P1, SFO)'), expr('At(P2, JFK)')        
    ]
    at_dict = {'C1': 'SFO', 'C2': 'JFK', 'C3': 'ATL', 'C4': 'ORD',
               'P1': 'SFO', 'P2': 'JFK'}
    neg = []
    for item, place in at_dict.items():
        for a in airports:
            if a != place:
                neg.append(expr('At({}, {})'.format(item, a)))
    neg += [expr('In({}, {})'.format(c, p)) for c in cargos for p in planes]    
    init = FluentState(pos, neg)
    goal = [expr('At(C1, JFK)'), expr('At(C3, JFK)'), expr('At(C2, SFO)'),
            expr('At(C4, SFO)')]
    return AirCargoProblem(cargos, planes, airports, init, goal)
