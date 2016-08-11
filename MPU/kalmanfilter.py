import numpy

class KalmanFilter(object):

    def __init__(self, process_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def input_latest_noisy_measurement(self, measurement):
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate +
            self.process_variance

        blending_factor = priori_error_estimate / (priori_error_estimate +
            self.estimated_measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor *
            (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) *
            priori_error_estimate

    def get_latest_estimated_measurement(self):
        return self.posteri_estimate

    # in practice we would take our sensor, log some readings and get the
    # standard deviation
    measurement_standard_deviation = numpy.std([])

    # The smaller this number, the fewer fluctuations, but can also venture off
    # course...
    process_variance = 1e-3
    estimated_measurement_variance = measurement_standard_deviation ** 2  # 0.05 ** 2
    kalman_filter = KalmanFilter(process_variance, estimated_measurement_variance)
    posteri_estimate_graph = []

    for iteration in xrange(1, iteration_count):
        kalman_filter.input_latest_noisy_measurement(noisy_measurement[iteration])
        posteri_estimate_graph.append(kalman_filter.get_latest_estimated_measurement())
