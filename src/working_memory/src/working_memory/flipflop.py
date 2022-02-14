
import numpy as np


def generate_flipflop_trials(n_time, n_bits, p_flip, last_bit, rng):
    '''Generates synthetic data (i.e., ground truth trials) for the
    FlipFlop task. See comments following FlipFlop class definition for a
    description of the input-output relationship in the task.
    Args:
        None.
    Returns:
        dict containing 'inputs' and 'outputs'.
            'inputs': [n_batch x n_time x n_bits] numpy array containing
            input pulses.
            'outputs': [n_batch x n_time x n_bits] numpy array specifying
            the correct behavior of the FlipFlop memory device.'''

    # Randomly generate unsigned input pulses
    unsigned_inputs = rng.binomial(
        1, p_flip, [n_time, n_bits])

    # Ensure every trial is initialized with a pulse at time 0
    unsigned_inputs[0, :] = last_bit

    # Generate random signs {-1, +1}
    random_signs = 2 * rng.binomial(
        1, 0.5, [n_time, n_bits]) - 1

    # Apply random signs to input pulses
    inputs = np.multiply(unsigned_inputs, random_signs)

    # Allocate output
    output = np.zeros([n_time, n_bits])

    # Update inputs (zero-out random start holds) & compute output
    for bit_idx in range(n_bits):
        input_ = np.squeeze(inputs[:, bit_idx])
        t_flip = np.where(input_ != 0)
        for flip_idx in range(np.size(t_flip)):
            # Get the time of the next flip
            t_flip_i = t_flip[0][flip_idx]

            '''Set the output to the sign of the flip for the
            remainder of the trial. Future flips will overwrite future
            output'''
            output[t_flip_i:, bit_idx] = \
                inputs[t_flip_i, bit_idx]

    return {'inputs': inputs, 'output': output}
