# -*- coding: utf-8 -*-
"""
Created on Thu Aug 18 16:21:35 2016

@author: cd32
"""

AND = "and"
OR = "or"
NOT = "not"

def create_predicate(parameters, *args):
    res = []
    for a in args:
        for e in a:
            t = [str(e.name)]
            t.extend([parameters[str(p.key)] for p in e.typed_parameters])
            res.append("__".join(t))
    return res

def create_condition(operator, *args):
    cond = []
    for a in args:
        if a:
            cond.extend(a if isinstance(a,list) else [a])

    if len(cond) == 1:
        if operator != NOT:
            # Prevent to have something like (and cond) as 'and' and
            # 'or' need at least two arguments
            return cond[0]
    elif len(cond) == 0:
        return "" #No empty conditions
    elif len(cond) > 1 and operator == NOT:
        # not can only wrap around a single predicate
        return create_condition(AND, [create_condition(operator, c) for c in cond])
            

    return "("+operator+" "+" ".join(cond)+")"
