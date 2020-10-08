# Copyright (c) 2019 Robert Bosch GmbH
# All rights reserved.
#
# This source code is licensed under the BSD-3-Clause license found in the
# LICENSE file in the root directory of this source tree.

import enum
import math
import logging
import functools
import contextlib

import pycpa.model as model
import pycpa.analysis as analysis
from pycpa.junctions import ORJoin

# Global flag: Should chains be analyzed as a whole or piecewise?
disable_chain_analysis = False


@contextlib.contextmanager
def chainAnalysis(disabled):
    """Disables/Enables the chain analysis inside a context manager.
       This is the closest to dynamic scoping for the disable_chain_analysis variable we can get in python"""
    global disable_chain_analysis

    old = disable_chain_analysis
    disable_chain_analysis = disabled
    try:
        yield None
    finally:
        disable_chain_analysis = old


# Due to discrete time, the smallest time step increment is 1
epsilon = 1


@enum.unique
class CBType(enum.IntEnum):
    EVENT_SOURCE = 0
    TIMER = 1
    SUBSCRIBER = 2
    SERVICE = 3
    CLIENT = 4
    WAITABLE = 5

    def polling_based(self):
        return self.value >= self.__class__.SUBSCRIBER


def arrival_curve_steps(event_model, optimized=True):
    """Yields the steps of the arrival curve for the given event model.
       If optimized is true, may use optimized implementations for specific models.
       TODO: This should really be methods on the event models"""

    def PJd(event_model):
        "Computes the arrival curve steps for the PJd event model"
        J = event_model.J
        P = event_model.P
        d = event_model.dmin

        yield 0
        # First look at the burst part, if present
        if 0 < d < P:
            inflection_point = (J * d) / (P - d)
            for t in range(1, math.ceil(inflection_point)+1, d):
                yield t
            # compute in which period we currently are
            period_nr = math.ceil((inflection_point + J) / P)
            # compute the next period change
            t = ((P * period_nr) - J) + epsilon
        else:
            t = 1

        # Now continue with the asymptotic part
        while True:
            yield t
            t += P

    def bruteforce(event_model):
        "Computes the arrival curve steps by evaluating the arrival curve a lot"
        i = 0
        last = None
        while True:
            new = event_model.eta_plus(i)
            if last != new:
                yield i
            i += 1
            last = new

    if optimized:
        if isinstance(event_model, model.PJdEventModel):
            return PJd(event_model)
    return bruteforce(event_model)


class Callback(model.Task):
    def __init__(self, name, typ, scheduling_parameter, *args, **kwargs):
        """In addition to the model.Task parameters, typ contains the callback type (of class CBType).
           The scheduling_parameter is the priority among the same callback type (lower is higher priority)."""
        super().__init__(name, *args, **kwargs)
        self.typ = typ
        # Lexicographical comparison does the right thing here: Prioritize according to callback type, and use
        # scheduling_parameter as a fallback
        self.scheduling_parameter = (typ, scheduling_parameter)

    def polling_based(self):
        return self.typ.polling_based()

    def bind_resource(self, r):
        super().bind_resource(r)
        if self.typ == CBType.EVENT_SOURCE:
            assert len(r.tasks) == 1

    def request_bound(self, delta):
        "Returns the highest amount of workload required in an interval of length delta"
        return self.in_event_model.eta_plus(delta) * self.wcet

    def request_bound_steps(self):
        """Yields the steps of the request_bound function"""
        return arrival_curve_steps(self.in_event_model)

    def __repr__(self):
        return 'Callback({},{})'.format(super().__repr__(), repr(self.typ))

    def __str__(self):
        return self.name

    @property
    def prev_tasks(self):
        """Returns the predecessor as a single-element (or empty) list for consistency with junctions"""
        return [self.prev_task] if self.prev_task else []


class Chain:
    def __init__(self, start, end):
        self.chain = [end]
        cur = end
        while cur != start:
            # This implicitly raises an exception if we come across a junction
            cur = cur.prev_task
            self.chain.insert(0, cur)

    def __getitem__(self, key):
        return self.chain[key]

    def __len__(self):
        return len(self.chain)

    @property
    def wcet(self):
        return sum([t.wcet for t in self.chain])

    def request_bound(self, delta):
        """Computes the maximum execution time requested by chain in any timespan delta. rbf^{chain} in the paper."""
        eta_plus = self.chain[0].in_event_model.eta_plus
        return eta_plus(delta) * self.wcet

    def request_bound_steps(self):
        """Returns an iterable that gives the steps of the request bound function"""
        return self.chain[0].request_bound_steps()


def on_same_resource(a, b):
    return getattr(a, 'resource', None) == getattr(b, 'resource', None)


def longest_unique_subchain(task, cross_resources=False):
    """Returns the longest chain ending in task that does not cross a junction.
    cross_resources determines if the chain may cross resource boundaries"""
    start = task
    # Go back through the subchain until we hit something that has multiple predecessors or
    # crosses resource (if cross_resources=False)
    while (len(start.prev_tasks) == 1
           and (cross_resources
                or on_same_resource(start, start.prev_tasks[0]))):
        start = start.prev_task
    return Chain(start, task)


def find_path(start, end):
    """Finds the (unique) path between start and end."""
    if start == end:
        return [start]

    paths = [find_path(start, to) for to in end.prev_tasks]
    valid_paths = [p for p in paths if p is not None]
    if not valid_paths:
        return None
    if len(valid_paths) != 1:
        raise ValueError('Path between %s and %s is not unique', start, end)
    return valid_paths[0] + [end]


def e2e_latency(path, results):
    """Computes the end-to-end latency on the given path, based on the WCRT-values in results."""
    latency = 0
    idx = len(path) - 1
    while idx >= 0:
        node = path[idx]
        if len(node.prev_tasks) > 1:
            if isinstance(node.strategy, ORJoin):
                # Ignore join nodes
                idx -= 1
                continue
            else:
                raise ValueError('Unknown junction strategy', node.strategy)
        latency += results[node.name].wcrt
        global disable_chain_analysis
        if disable_chain_analysis:
            chain = [node]
        else:
            chain = longest_unique_subchain(node)
        idx -= len(chain)
    assert path[0].typ in {CBType.TIMER, CBType.EVENT_SOURCE}
    return latency


class CBSReservation (model.Resource):
    def __init__(self, name=None, scheduler=None, budget=1, period=1, **kwargs):
        super().__init__(name, scheduler, **kwargs)
        assert budget <= period
        assert int(budget) == budget and int(
            period) == period, "Period and/or budget are not integer"
        self.budget = int(budget)
        self.period = int(period)

    @property
    def max_delay(self):
        return 2 * (self.period - self.budget)

    def __repr__(self):
        return '{} (CBS {}/{})'.format(str(self.name), self.budget, self.period)

    def load(self, accuracy=10000):
        raw_load = super().load(accuracy)
        # raw_load refers to a budget of 100%. Artificially inflate our load to account for the reduced budget
        if self.budget == 0:
            return 0 if raw_load == 0 else float('inf')
        return (raw_load * self.period) / self.budget

    @functools.lru_cache()
    def chains(self):
        chains = set()
        # Chain ends are graph sinks or tasks whose successor are all either junctions or in another reservation
        ends = {t for t in self.tasks
                if ((not t.next_tasks)
                    or all([getattr(cb, 'resource', None) != self
                            for cb in t.next_tasks]))}
        while ends:
            e = ends.pop()
            assert e.resource == self

            newchain = longest_unique_subchain(e, cross_resources=False)
            ends.update([e for e in newchain[0].prev_tasks if getattr(
                e, 'resource', None) == self])
            chains.add(newchain)

        return chains

    def invalidate_chains_cache(self):
        "Invalidate the chains() cache. Does nothing, if chains() is not memoized"
        if hasattr(self.chains, 'cache_clear'):
            self.chains.cache_clear()

    def bind_task(self, t):
        self.invalidate_chains_cache()
        super().bind_task(t)

    def unmap_tasks(self):
        self.invalidate_chains_cache()
        super().unmap_tasks()

    # from Schedulability Analysis of Hierarchical
    # Real-Time Systems under Shared Resources
    # Alessandro Biondi, Giorgio C. Buttazzo, Fellow, IEEE, and Marko Bertogna, Senior Member, IEEE
    def supply_bound(self, t):
        "Returns the minimum supply provided by the reservation in time t"

        if t <= self.max_delay:
            return 0

        # t is in the k-th period, which starts at tA, runs out of budget at tB and ends at tC
        k = math.ceil((t - self.max_delay) / self.period)
        tA = self.max_delay + (k - 1) * self.period
        tB = tA + self.budget
        tC = tA + self.period

        if not (tA < t <= tC):
            raise analysis.NotSchedulableException("Numbers too large for python floats. System probably not schedulable")
        # Everything between tB and tC has the same amount of supply as tB
        t = min(t, tB)
        supply = t - self.max_delay - (k - 1) * (self.period - self.budget)

        assert 0 <= supply <= t
        return supply

    def supply_time(self, supply):
        """Returns the first point in time at which supply has certainly been provided.
           The inverse of supply_bound"""
        if self.budget == 0:
            return 0
        # First identfy the period in which this much supply is going to be provided
        period_start = self.max_delay + \
            (math.floor(supply / self.budget) * self.period)
        return period_start + (supply - self.supply_bound(period_start))


def partition(iterable, pivot, key):
    """Returns 3 lists: the lists of smaller, equal and larger elements, compared to pivot.
      Elements are compared by applying the key function and comparing the results"""
    pkey = key(pivot)
    less = []
    equal = []
    greater = []
    for x in iterable:
        k = key(x)
        if k < pkey:
            less.append(x)
        elif k == pkey:
            equal.append(x)
        else:
            greater.append(x)
    return less, equal, greater


def fixed_point(f, x0):
    """computes the fixed point of f^n(x0) = f(f(f(...f(x0)))).
       f should return a tuple of the value and some state information for debugging"""
    x = x0
    logging.debug('Starting FPI of %s with initial value %s', f, x0)
    while True:
        xnew, state = f(x)
        if xnew == x:
            return x
        logging.debug('FPI update: %s (state %s)', xnew, state)
        x = xnew

# TODO the name is quite ad-hoc. There has to be some common concept behind all the equally-looking
#      functions but I don't see it yet
def balance_rbf_sbf_updater(resource, compute_request):
    """Returns a function that balances supply and demand by returning the time T' required to fulfil
       the demand at time T. The function is suitable for use with fixed_point.
       resource is the resource providing the supply.
       compute_request is a function that takes a time and computes the demand during that time."""
    def updater(T):
        supplied = resource.supply_bound(T)
        requested = compute_request(T)
        assert supplied <= requested
        if supplied == requested:
            return T, None
        return resource.supply_time(requested), (supplied, requested)

    return updater


def latest_cb_response_at(task, A):
    """Computes the latest point in time at which task can complete, assuming it is released at A."""

    resource = task.resource

    if task.typ == CBType.EVENT_SOURCE:
        # We can compute the event source case directly without any FPI
        return resource.supply_time(task.request_bound(A + epsilon))

    if task.typ == CBType.TIMER:
        hp_tasks, _, lp_tasks = partition(resource.tasks,
                                          task,
                                          lambda t: t.scheduling_parameter)

        def compute_request(T):
            return (task.request_bound(A + epsilon)
                    + sum([t.request_bound(T - task.wcet) for t in hp_tasks])
                    + max([t.wcet for t in lp_tasks]))
    else:
        def compute_request(T):
            return (task.request_bound(A + epsilon)
                    + sum([t.request_bound(T - task.wcet) for t in resource.tasks
                           if t != task]))
    return fixed_point(balance_rbf_sbf_updater(resource, compute_request), A + task.wcet)

def latest_chain_response_at(chain, A):
    """Computes the latest point in time at which chain (a sequence of tasks) can complete, assuming the first task in
       the chain is released at A.
       Assumes the entire chain is in the same resource"""

    # The analysis below only works for chains of length > 1
    if len(chain) == 1:
        return latest_cb_response_at(chain[0], A)

    assert all([on_same_resource(t, chain[0]) for t in chain])
    resource = chain[0].resource

    # We only need analyze the *last* element of the chain. Cf 6.2 of the paper
    tail = chain[-1]

    def compute_request(T):
        return sum([gamma.request_bound(T - tail.wcet) for gamma in resource.chains()])

    return fixed_point(balance_rbf_sbf_updater(resource, compute_request), A + chain.wcet)


def max_busy_period(task):
    "?? I don't have any intuitive notion of what this does"
    resource = task.resource

    if task.typ == CBType.EVENT_SOURCE:
        compute_request = task.request_bound
    elif task.typ == CBType.TIMER:
        hp_tasks, _, lp_tasks = partition(
            resource.tasks, task, lambda t: t.scheduling_parameter)

        def compute_request(T):
            return (sum([t.request_bound(T) for t in hp_tasks])
                    + max([t.wcet for t in lp_tasks])
                    + task.request_bound(T))
    else:
        def compute_request(T):
            return sum([t.request_bound(T) for t in resource.tasks])

    return fixed_point(balance_rbf_sbf_updater(resource, compute_request), task.wcet)

# We do not inherit from analysis.Scheduler, since we use a different scheduling analysis approach
# The scheduler must fulfil the following interface:
#  compute_max_backlog(self, task, task_results, output_delay=0)
#  compute_bcrt(self, task, task_results=None)
#  compute_wcrt(self, task, task_results=None)
#
# All these tasks return their results and also put their results into the task_results dictionary
#  compute_wcrt additionally has to set b_wcrt and q_wcrt and also busy_times
# If one sets the JitterBminPropagationEventModel, one also needs:
#  b_min(self, task, q)
#
# Further, the scheduler must provide a function get_dependent_tasks(self, task),
# which returns additional dependent tasks besides the direct successors that need
# to be recomputed on a change (https://bitbucket.org/pycpa/pycpa/commits/1b90b92d8186545fd4869ed097d9de0aa367f7ab)


class DefaultScheduler:
    def __init__(self):
        pass

    def get_dependent_tasks(self, task):
        return set()

    def compute_max_backlog(self, task, task_results, output_delay=0):
        return 0

    def compute_bcrt(self, task, task_results=None):
        if task_results:
            task_results[task].bcrt = 0
        return 0

    def compute_wcrt(self, task, task_results=None):
        global disable_chain_analysis
        if disable_chain_analysis:
            chain = [task]
        else:
            chain = longest_unique_subchain(task, cross_resources=False)

        wcrt = 0
        max_A = max_busy_period(task)
        for A in task.request_bound_steps():
            if A >= max_A:
                break
            wcrt = max(wcrt, latest_chain_response_at(chain, A) - A)
        if task_results:
            task_results[task].wcrt = wcrt
            # These don't make sense in our analysis approach
            task_results[task].b_wcrt = task_results[task].q_wcrt = None
            task_results[task].busy_times = [0, task.wcet]
        return wcrt
