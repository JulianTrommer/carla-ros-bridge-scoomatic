import numpy as np


class DomainTransformer():
    '''The base transformer class'''

    def __init__(self, **kwargs):
        pass

    def initialize(self, target_space):
        raise NotImplementedError

    def transform(self, target_space):
        raise NotImplementedError


class SequentialDomainReductionTransformer(DomainTransformer):
    """
    A sequential domain reduction transformer bassed on the work by Stander, N. and Craig, K:
    "On the robustness of a simple domain reduction scheme for simulation-based optimization"
    """

    def __init__(
        self,
        gamma_osc = 0.7,
        gamma_pan = 1.0,
        eta = 0.9
    ):
        self.gamma_osc = gamma_osc
        self.gamma_pan = gamma_pan
        self.eta = eta
        pass

    def initialize(self, target_space):
        """Initialize all of the parameters"""
        self.original_bounds = np.copy(target_space.bounds)
        self.bounds = [self.original_bounds]

        self.previous_optimal = np.mean(target_space.bounds, axis=1)
        self.current_optimal = np.mean(target_space.bounds, axis=1)
        self.r = target_space.bounds[:, 1] - target_space.bounds[:, 0]

        self.previous_d = 2.0 * \
            (self.current_optimal - self.previous_optimal) / self.r

        self.current_d = 2.0 * (self.current_optimal -
                                self.previous_optimal) / self.r

        self.c = self.current_d * self.previous_d
        self.c_hat = np.sqrt(np.abs(self.c)) * np.sign(self.c)

        self.gamma = 0.5 * (self.gamma_pan * (1.0 + self.c_hat) +
                            self.gamma_osc * (1.0 - self.c_hat))

        self.contraction_rate = self.eta + \
            np.abs(self.current_d) * (self.gamma - self.eta)

        self.r = self.contraction_rate * self.r

    def _update(self, target_space):

        # setting the previous
        self.previous_optimal = self.current_optimal
        self.previous_d = self.current_d

        self.current_optimal = target_space.params[
            np.argmax(target_space.target)
        ]

        self.current_d = 2.0 * (self.current_optimal -
                                self.previous_optimal) / self.r

        self.c = self.current_d * self.previous_d

        self.c_hat = np.sqrt(np.abs(self.c)) * np.sign(self.c)

        self.gamma = 0.5 * (self.gamma_pan * (1.0 + self.c_hat) +
                            self.gamma_osc * (1.0 - self.c_hat))

        self.contraction_rate = self.eta + \
            np.abs(self.current_d) * (self.gamma - self.eta)

        self.r = self.contraction_rate * self.r

    def _trim(self, new_bounds, global_bounds):
        for i, variable in enumerate(new_bounds):
            if variable[0] < global_bounds[i, 0]:
                variable[0] = global_bounds[i, 0]
            if variable[1] > global_bounds[i, 1]:
                variable[1] = global_bounds[i, 1]

        return new_bounds

    def _create_bounds(self, parameters, bounds):
	return {param: bounds[i, :] for i, param in enumerate(parameters)}

    def transform(self, target_space):

        self._update(target_space)

        new_bounds = np.array(
            [
                self.current_optimal - 0.5 * self.r,
                self.current_optimal + 0.5 * self.r
            ]
        ).T

        self._trim(new_bounds, self.original_bounds)
        self.bounds.append(new_bounds)
        return self._create_bounds(target_space.keys, new_bounds)
