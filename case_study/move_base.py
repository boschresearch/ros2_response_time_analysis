# Copyright (c) 2019 Robert Bosch GmbH
# All rights reserved.
#
# This source code is licensed under the BSD-3-Clause license found in the
# LICENSE file in the root directory of this source tree.

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from fractions import Fraction
import math
import logging
from functools import lru_cache

import pycpa.model as model
from pycpa.junctions import ORJoin
import pycpa.analysis
from pycpa.analysis import NotSchedulableException
from pycpa.graph import graph_system

import ros
from ros import Callback, CBType, CBSReservation, DefaultScheduler

# The time unit is 100 microseconds. However, this is an implementation detail.
# Use these functions when calculating times instead.
def useconds(n):
    frac, i = math.modf(n / 100)
    assert frac == 0
    return int(i)
def mseconds(n):
    return useconds(1000 * n)
def seconds(n):
    return mseconds(1000 * n)
def Hz(n):
    return int(seconds(1 / n))


# Minimum worst-case jitter for input sensors.
# This models interrupt latency+ROS overhead, underestimated
min_jitter = useconds(200)


class PBurstModel (model.PJdEventModel):
    def __init__(self, P, burstlen, dmin, name='min', **kwargs):
        J = burstlen * P
        super().__init__(P, J, dmin, name=name, **kwargs)


wcets = {'local_costmap': mseconds(5),
         'global_costmap': mseconds(10),
         'local_planner': mseconds(18),
         'global_planner': mseconds(200),
         'goal_setter': min_jitter,
         'pose_estimator': mseconds(6),
         'sensor2mem': min_jitter}
input_topics = [('/scan', model.PJdEventModel(P=Hz(12.5), J=min_jitter)),
                ('/tF', model.PJdEventModel(P=Hz(12.5), J=min_jitter)),
                ('/odom', model.PJdEventModel(P=Hz(12.5), J=min_jitter)),
                ('/goal', PBurstModel(P=Hz(0.1), burstlen=0, dmin=mseconds(100)))]


class MoveBase(model.System):
    """The common parts of the move_base system model"""
    # Global dictionary of callback names -> callbacks
    callbacks = dict()
    reservations = []

    def __init__(self, num_reservations, *args, override_model={}, **kwargs):
        super().__init__(*args, **kwargs)
        for name, events in input_topics:
            if name in override_model:
                events = override_model[name]
            self.add_event_source(name, events)

        out = Callback('/cmd_vel', CBType.SUBSCRIBER, 0, wcet=0)
        res = CBSReservation('Res[cmd_vel]',
                             scheduler=DefaultScheduler(),
                             budget=0,
                             period=1)
        res.bind_task(out)
        self.bind_resource(res)
        self.register_callbacks((out,))

        for i in range(num_reservations):
            res = CBSReservation('R{}'.format(i),
                                 scheduler=DefaultScheduler(),
                                 budget=1,
                                 period=num_reservations)
            self.bind_resource(res)
            self.reservations.insert(i, res)

    def add_event_source(self, name, event_model):
        """Adds an event source with the given name and event model."""
        src = Callback(name, CBType.EVENT_SOURCE, 0, wcet=0)
        src.in_event_model = event_model
        res = CBSReservation('Res[{}]'.format(name),
                             scheduler=DefaultScheduler(),
                             budget=0,
                             period=1)
        res.bind_task(src)
        self.bind_resource(res)
        self.register_callbacks((src,))

    def register_callbacks(self, cbs):
        """Registers cbs, a list of callbacks, with the model.

        This allows referring to them by name in the other methods."""
        for cb in cbs:
            self.callbacks[cb.name] = cb

    def res_bind(self, resource, cbs):
        """Binds cbs (a list of callbacks) to resource.

        Callbacks might be callback objects or names (which must have
        been registered using register_callbacks before)."""
        cbs = [(self.callbacks[cb] if isinstance(cb, str) else cb)
               for cb in cbs]
        for cb in cbs:
            resource.bind_task(cb)

    def link(system, srcs, dsts, junctionStrategy=ORJoin):
        """Links multiple callbacks to each of multiple destinations.

        Each of srcs is linked to a junction, which is linked to each of dsts.
        junctionStrategy contains the constructor of the junction.
        srcs and dsts are sequences of callback objects or strings.
        For convenience, a single string or callback object is interpreted
        as a sequence of length 1."""

        # Wrap strings and non-sequences into a sequence of length 1
        if isinstance(srcs, str) or not hasattr(srcs, '__iter__'):
            srcs = (srcs,)
        if isinstance(dsts, str) or not hasattr(dsts, '__iter__'):
            dsts = (dsts,)

        # Look up callbacks by name, if necessary
        srcs = [system.callbacks[s] if isinstance(s, str) else s
                for s in srcs]
        dsts = [system.callbacks[d] if isinstance(d, str) else d
                for d in dsts]

        # Build a junction node to combine multiple sources, if necessary
        if len(srcs) == 1:
            junction = srcs[0]
        else:
            junction = model.Junction('Junction<{}>'.format(" ".join([s.name for s in srcs])),
                                      junctionStrategy())
            for s in srcs:
                s.link_dependent_task(junction)

        # Finally, link the destinations to the junction
        for d in dsts:
            junction.link_dependent_task(d)

    def e2e_local_chain(self):
        """Returns the end-to-end latency from /odom to /cmd_vel"""
        # Must be overridden by the child classes
        raise NotImplementedError


class EventDrivenMoveBase(MoveBase):
    """A model of move_base, where the components are triggered by ROS topics."""
    def __init__(self, num_reservations, **kwargs):
        super().__init__(num_reservations,
                         name='move_base.driver-event.{}'.format(num_reservations), **kwargs)
        # Since there is only one timer, the priorities don't matter. We just keep them different
        callbacks_ = [
            Callback('sensor2mem', CBType.SUBSCRIBER, 0, wcet=wcets['sensor2mem']),
            Callback('local_costmap', CBType.SUBSCRIBER, 4, wcet=wcets['local_costmap']),
            Callback('global_costmap', CBType.SUBSCRIBER, 5, wcet=wcets['global_costmap']),
            Callback('local_planner', CBType.SUBSCRIBER, 6, wcet=wcets['local_planner']),
            Callback('global_planner_goalset', CBType.SUBSCRIBER, 7, wcet=wcets['global_planner']),
            Callback('global_planner_timed', CBType.TIMER, 8, wcet=wcets['global_planner']),
            Callback('pose_estimator', CBType.SUBSCRIBER, 10, wcet=wcets['pose_estimator'])]

        self.register_callbacks(callbacks_)

        self.callbacks['global_planner_timed'].in_event_model = model.PJdEventModel(P=Hz(1), J=0)

        link = self.link
        link(['/scan', '/tF'], 'sensor2mem')
        link('pose_estimator', ['local_costmap', 'global_costmap'])
        link('local_costmap', 'local_planner')
        link('local_planner', '/cmd_vel')
        link('/odom', 'pose_estimator')
        link('/goal', 'global_planner_goalset')

        if num_reservations == 1:
            self.res_bind(self.reservations[0], callbacks_)
        elif num_reservations == 2:
            self.res_bind(self.reservations[0],
                          ['local_costmap', 'local_planner', 'pose_estimator', 'sensor2mem'])
            self.res_bind(self.reservations[1],
                          ['global_costmap', 'global_planner_timed', 'global_planner_goalset'])
        else:
            raise ValueError(
                "Unsupported reservation count: %s", num_reservations)

        assert all([cb.resource for cb in self.callbacks.values()])

    def e2e_local_chain(self):
        try:
            task_results = analyze_system(self)
        except NotSchedulableException:
            return None
        return ros.e2e_latency(ros.find_path(self.callbacks['/odom'], self.callbacks['/cmd_vel']), task_results)


class TimeDrivenMoveBase(MoveBase):
    """A model of move_base, where all components are triggered periodically."""
    def __init__(self, num_reservations, period=Hz(12.5), **kwargs):
        super().__init__(num_reservations,
                         name='move_base.time-driven{}'.format(num_reservations), **kwargs)

        # Prioritize the timers from source to destination. We want the source tasks to run first, so
        # the data can flow through the entire path in a single period.
        # We prioritize the local planner higher than the global planner components
        callbacks_ = [
            Callback('sensor2mem', CBType.SUBSCRIBER, 0, wcet=wcets['sensor2mem']),
            Callback('goal2mem', CBType.SUBSCRIBER, 0, wcet=wcets['sensor2mem']),
            Callback('local_costmap', CBType.TIMER, 4, wcet=wcets['local_costmap']),
            Callback('global_costmap', CBType.TIMER, 1, wcet=wcets['global_costmap']),
            Callback('local_planner', CBType.TIMER, 3, wcet=wcets['local_planner']),
            Callback('global_planner', CBType.TIMER, 5, wcet=wcets['global_planner']),
            Callback('pose_estimator', CBType.TIMER, 2, wcet=wcets['pose_estimator'])]

        self.register_callbacks(callbacks_)

        self.link(['/scan', '/tF', '/odom'], 'sensor2mem')
        self.link('/goal', 'goal2mem')
        self.link('local_planner', '/cmd_vel')
        self.callbacks['local_costmap'].in_event_model = model.PJdEventModel(P=period, J=0)
        self.callbacks['local_planner'].in_event_model = model.PJdEventModel(P=period, J=0)
        self.callbacks['pose_estimator'].in_event_model = model.PJdEventModel(P=period, J=0)
        self.callbacks['global_costmap'].in_event_model = model.PJdEventModel(P=period, J=0)
        self.callbacks['global_planner'].in_event_model = model.PJdEventModel(P=Hz(1), J=0)

        if num_reservations == 1:
            self.res_bind(self.reservations[0], callbacks_)
        elif num_reservations == 2:
            self.res_bind(self.reservations[0],
                          ['local_costmap', 'local_planner', 'pose_estimator', 'sensor2mem'])
            self.res_bind(self.reservations[1],
                          ['global_costmap', 'global_planner', 'goal2mem'])
        else:
            raise ValueError("Unsupported reservation count: %s", num_reservations)

        assert all([cb.resource for cb in self.callbacks.values()])

    def e2e_local_chain(self):
        # In the time-driven system, the E2E delay corresponds to the WCET of the local_planner task.
        # This is, because the predecessors of local_planner are prioritized higher than it and all trigger at
        # the same time.
        # However, the WCRT of the local_planner task is the E2E delay *counting from the task release*.
        # We therefore add the maximum delay between event arrival and task release (one period) to get
        # the maximum E2E latency from event arrival.
        try:
            task_results = analyze_system(self)
        except NotSchedulableException:
            return None

        sampled_model = self.callbacks['/odom'].in_event_model
        sampler_period = self.callbacks['pose_estimator'].in_event_model.P
        sampling_delay = sampler_period if sampled_model.J > 0 else 0
        return sampling_delay + task_results['local_planner'].wcrt


models = {'time-driven': TimeDrivenMoveBase,
          'event-driven': EventDrivenMoveBase}


def analyze_system(s):
    """Use names as keys instead of the callback objects, since the latter have equality issues"""
    r = pycpa.analysis.analyze_system(s)
    return {task.name: result for task, result in r.items()}


# Plotting/Analysis stuff

def isInteger(x):
    """Returns whether x is an integral number.

    This is independent of the type of x. 2.0 is considered an integral number"""
    return int(x) == x

def minimalP(alpha, min_budget=mseconds(1)):
    """Returns the smallest reservation period that allows a budget of alpha with min_budget granularity.

    Effectively, this computes the smallest P such that alpha*P is divisible by min_budget."""

    if alpha == 0:
        return min_budget

    alpha = Fraction(alpha).limit_denominator(100)

    # Multiply both sides by ceil(budget/numerator) to get a numerator
    # that is larger then min_budget.

    factor = min_budget / alpha.numerator
    if not isInteger(factor):
        factor = math.ceil(factor)
        assert isInteger(factor)
    return alpha.denominator * int(factor)


def make2res_alpha(alpha, model, P):
    """Creates a CPA model with two reservations, with relative budget alpha.

    model is a function that, given a reservation count, returns a model with that many reservations.
    The function then configures the budget such that reservation 0 receives alpha of the total budget,
    and reservation 1 receives (1-alpha) of the budget.
    P can be the reservation period length or None, which selects the minimum feasible period length."""
    s = model(2)

    if P is None:
        P = minimalP(alpha)
        logging.info('Computed P for alpha={} as {}'.format(alpha, P))

    alpha = Fraction(alpha).limit_denominator(P)
    assert isInteger(alpha * P)
    s.reservations[0].period = s.reservations[1].period = P
    s.reservations[0].budget = int(alpha * P)
    s.reservations[1].budget = P - s.reservations[0].budget

    # Sanity check
    for r in s.reservations:
        assert isInteger(r.period) and isInteger(r.budget)
        assert 0 <= r.budget <= r.period

    return s


@lru_cache(maxsize=None)
def e2e_per_alpha_local(alpha, model, P=None, disable_chain=False):
    """Sets up a 2-reservation model with the given budget distribution, and computes the e2e latency of the /odom -> /cmd_vel chain"""
    # The chain analysis decision is passed in from the outside as a parameter,
    # and the global variable modified here. This allows memoization of the function,
    # which cannot account for global variables otherwise
    with ros.chainAnalysis(disable_chain):
        return make2res_alpha(alpha, model, P).e2e_local_chain()


def convertTimes(times, unit_tick):
    """Takes a list of timestamps, and normalizes them to the given unit.

    For convenience, None is normalized to None. This is important to allow undefined points in the list."""
    return [None if x is None else x / unit_tick
            for x in times]


# The pyplot plotting style for the different models.
model_graphspec = {'time-driven': 'bo-',
                   'event-driven': 'gD-'}

def plot_e2e_per_alpha(P=None, **kwargs):
    """Plots the reservation budget assignment vs. the /odom->/cmd_vel end-to-end latencies.

    P is the reservation period, any other kwargs are passed to plt.plot"""

    # Step through the x axis in steps of 2.5.
    xs_pct = np.linspace(0, 100, num=41)
    plt.xlabel('Budget of the local reservation (percent)')
    plt.ylabel('End-to-end latency (ms)')
    plt.ylim(top=500)

    for mname, m in models.items():
        if P is not None:
            mname += '(P={})'.format(P / mseconds(1))

        latencies = [e2e_per_alpha_local(x / 100, P=P, model=m, disable_chain=False)
                     for x in xs_pct]
        latencies_nochain = [e2e_per_alpha_local(x / 100, P=P, model=m, disable_chain=True)
                             for x in xs_pct]
        plt.plot(xs_pct, convertTimes(latencies, mseconds(1)),
                 model_graphspec[mname], label=mname, **kwargs)
        if latencies != latencies_nochain:
            plt.plot(xs_pct, convertTimes(latencies_nochain, mseconds(1)),
                     model_graphspec[mname], linestyle=':', label=mname + ' (no chains)', **kwargs)
    plt.legend()

# We need to memoize this function in addition to e2e_per_alpha_local, since the memoization
# doesn't seem to be able to handle ad-hoc computed models.
@lru_cache(maxsize=None)
def jittered_lats(movebase, js, disable_chain=False):
    """Computes e2e_per_alpha_local at reservation alpha of 45% for each jitter in js."""

    def jittered_models(J):
        return {'/scan': model.PJdEventModel(P=Hz(12.5), J=J),
                '/tF': model.PJdEventModel(P=Hz(12.5), J=J),
                '/odom': model.PJdEventModel(P=Hz(12.5), J=J)}

    return [e2e_per_alpha_local(0.45,
                                lambda n: movebase(n, override_model=jittered_models(J)),
                                disable_chain=disable_chain)
            for J in js]


def plot_e2e_per_jitter():
    """Plots jitter values vs. jittered_lats."""
    js = range(0, mseconds(80), mseconds(5))

    plt.xlabel('Jitter on the input sensors (ms)')
    plt.ylabel('End-to-End latency (ms)')
    for mname, m in models.items():
        plt.plot(convertTimes(js, mseconds(1)),
                 convertTimes(jittered_lats(m, js), mseconds(1)),
                 model_graphspec[mname], label=mname)

    plt.plot(convertTimes(js, mseconds(1)),
             convertTimes(jittered_lats(EventDrivenMoveBase, js, disable_chain=True), mseconds(1)),
             model_graphspec[mname], linestyle=':', label='event-driven (no chains)')
    plt.legend()

def configure_mpl_for_tex():
    "Configures matplotlib for LaTeX embedding"
    # Inspired by https://jwalton.info/Embed-Publication-Matplotlib-Latex/

    # Width of the LIPICS a4paper column, determined by \the\columnwidth
    width_pts = 398.33858
    golden_ratio = (5**.5 - 1) / 2
    # Width of the figure relative to the page
    rel_width = .48

    inches_per_pt = 1 / 72.27

    # Figure width in inches
    width_in = width_pts * inches_per_pt * rel_width
    # Figure height in inches
    height_in = width_in * golden_ratio

    settings = {
        "figure.figsize": (width_in, height_in),
        "text.usetex": True,
        "font.family": "serif",
        "font.size": 6,
        "font.serif": ["computer modern roman"],
        "axes.labelsize": 6,
        "legend.fontsize": 7,
        "xtick.labelsize": 6,
        "ytick.labelsize": 6,
        "lines.markersize": 3.5
    }
    mpl.rcParams.update(settings)

def generate_charts_for_paper(dir='.'):
    """Generate the charts for the ECRTS paper."""
    plt.close()
    configure_mpl_for_tex()
    plot_e2e_per_alpha()
    plt.savefig(dir + '/latency_per_budget.pdf', bbox_inches='tight')

    plt.cla()
    plot_e2e_per_jitter()
    plt.savefig(dir + '/latency_per_jitter.pdf', bbox_inches='tight')

if __name__ == "__main__":
    for num_res in [1, 2]:
        for mname, m in models.items():
            s = m(num_res)
            graph_system(s, filename='{}-{}.pdf'.format(mname, num_res),
                         dotout='{}-{}.dot'.format(mname, num_res), show=False)

    generate_charts_for_paper()
