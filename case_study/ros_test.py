# Copyright (c) 2019 Robert Bosch GmbH
# All rights reserved.
#
# This source code is licensed under the BSD-3-Clause license found in the
# LICENSE file in the root directory of this source tree.

import unittest
import itertools
import ros
import pycpa.model as model

class TestArrivalCurveSteps(unittest.TestCase):
    models = [model.PJdEventModel(P=20, J=5, dmin=1),
              model.PJdEventModel(P=100, J=400, dmin=2),
              model.PJdEventModel(P=20, J=30, dmin=20),
              model.CTEventModel(c=5, T=10),
              model.CTEventModel(c=5,T=15,dmin=3)]

    def bruteforce_model_test(self, m):
        "Check that the bruteforce arrive_curve_steps predicts the first 100 steps of the arrival curve correctly"
        steps = list(itertools.islice(ros.arrival_curve_steps(m, optimized=False), 100))
        eta = [ m.eta_plus(i) for i in range(steps[-1]+1) ]

        self.assertIn(0, steps)
        for i in range(1,len(eta)):
            if i in steps:
                self.assertNotEqual(eta[i-1], eta[i])
            else:
                self.assertEqual(eta[i-1], eta[i])

    def specialized_model_test(self, m):
        specialized = itertools.islice(ros.arrival_curve_steps(m),100)
        forced = itertools.islice(ros.arrival_curve_steps(m,optimized=False), 100)
        for s,f in zip(specialized,forced):
            self.assertEqual(s,f)
    
    def test_bruteforce(self):
        for m in self.models:
            with self.subTest(m=m):
                self.bruteforce_model_test(m)
    def test_specialization(self):
        for m in self.models:
            with self.subTest(m=m):
                self.specialized_model_test(m)

if __name__ == '__main__':
    unittest.main()
