#!/usr/bin/env python3

import numpy as np


class JointSampler:

    def __init__(self, model):

        self.model = model

        self.lower = model.lowerPositionLimit
        self.upper = model.upperPositionLimit

    def random_configuration(self):
        """
        Generate one random valid joint configuration.
        """

        return np.random.uniform(self.lower, self.upper)

    def sample(self, n_samples):
        """
        Generate multiple random configurations.
        """

        return np.random.uniform(self.lower, self.upper, (n_samples, self.model.nq))
