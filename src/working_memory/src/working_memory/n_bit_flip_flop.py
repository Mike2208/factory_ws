import tensorflow as tf
import numpy as np
import numpy.random as npr
from tensorflow.keras.models import load_model
import os
import matplotlib.pyplot as plt


class Flipflopper(object):
    ''' Class for training an RNN to implement a 3-Bit Flip-Flop task as
    described in Sussillo, D., & Barak, O. (2012). Opening the Black Box:
    Low-Dimensional Dynamics in High-Dimensional Recurrent Neural Networks.
    Neural Computation, 25(3), 626–649. https://doi.org/10.1162/NECO_a_00409

    Task:
        A set of three inputs submit transient pulses of either -1 or +1. The
        task of the model is to return said inputs until one of them flips.
        If the input pulse has the same value as the previous input in a given
        channel, the output must not change. Thus, the RNN has to memorize
        previous input pulses. The number of input channels is not limited
        in theory.

    Usage:
        The class Flipflopper can be used to build and train a model of type
        RNN on the 3-Bit Flip-Flop task. Furthermore, the class can make use
        of the class FixedPointFinder to analyze the trained model.

    Hyperparameters:
        rnn_type: Specifies architecture of type RNN. Must be one of
        ['vanilla','gru', 'lstm']. Will raise ValueError if
        specified otherwise. Default is 'vanilla'.

        n_hidden: Specifies the number of hidden units in RNN. Default
        is: 24.

    '''

    def __init__(self, rnn_type: str = 'vanilla', n_hidden: int = 24):

        self.hps = {'rnn_type': rnn_type,
                    'n_hidden': n_hidden,
                    'model_name': 'flipflopmodel',
                    'verbose': False}
        self.data_hps = {'n_batch': 32,
                         'n_time': 256,
                         'n_bits': 1,
                         'p_flip': 0.2}
        self.verbose = self.hps['verbose']

        self.model, self.weights = self._build_model()
        self.rng = npr.RandomState(125)

    def _build_model(self):
        '''Builds model that can be used to train the n-Bit Flip-Flop task.

        Args:
            None.

        Returns:
            None.'''
        n_hidden = self.hps['n_hidden']
        name = self.hps['model_name']

        n_time, n_batch, n_bits = self.data_hps['n_time'], self.data_hps['n_batch'], self.data_hps['n_bits']

        inputs = tf.keras.Input(shape=(n_time, n_bits), batch_size=n_batch, name='input')

        if self.hps['rnn_type'] == 'vanilla':
            x = tf.keras.layers.SimpleRNN(n_hidden, name=self.hps['rnn_type'], return_sequences=True, stateful=True)(inputs)
        elif self.hps['rnn_type'] == 'gru':
            x = tf.keras.layers.GRU(n_hidden, name=self.hps['rnn_type'], return_sequences=True, stateful=True)(inputs)
        elif self.hps['rnn_type'] == 'lstm':
            x, state_h, state_c = tf.keras.layers.LSTM(n_hidden, name=self.hps['rnn_type'], return_sequences=True,
                                                       stateful=True, return_state=True,
                                                       implementation=1)(inputs)
        else:
            raise ValueError('Hyperparameter rnn_type must be one of'
                             '[vanilla, gru, lstm] but was %s', self.hps['rnn_type'])

        x = tf.keras.layers.Dense(1)(x)
        model = tf.keras.Model(inputs=inputs, outputs=x, name=name)
        weights = model.get_layer(self.hps['rnn_type']).get_weights()

        if self.verbose:
            model.summary()

        return model, weights

    def generate_flipflop_trials(self):
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
        data_hps = self.data_hps
        n_batch = data_hps['n_batch']
        n_time = data_hps['n_time']
        n_bits = data_hps['n_bits']
        p_flip = data_hps['p_flip']

        # Randomly generate unsigned input pulses
        unsigned_inputs = self.rng.binomial(
            1, p_flip, [n_batch, n_time, n_bits])

        # Ensure every trial is initialized with a pulse at time 0
        unsigned_inputs[:, 0, :] = 1

        # Generate random signs {-1, +1}
        random_signs = 2 * self.rng.binomial(
            1, 0.5, [n_batch, n_time, n_bits]) - 1

        # Apply random signs to input pulses
        inputs = np.multiply(unsigned_inputs, random_signs)

        # Allocate output
        output = np.zeros([n_batch, n_time, n_bits])

        # Update inputs (zero-out random start holds) & compute output
        for trial_idx in range(n_batch):
            for bit_idx in range(n_bits):
                input_ = np.squeeze(inputs[trial_idx, :, bit_idx])
                t_flip = np.where(input_ != 0)
                for flip_idx in range(np.size(t_flip)):
                    # Get the time of the next flip
                    t_flip_i = t_flip[0][flip_idx]

                    '''Set the output to the sign of the flip for the
                    remainder of the trial. Future flips will overwrite future
                    output'''
                    output[trial_idx, t_flip_i:, bit_idx] = \
                        inputs[trial_idx, t_flip_i, bit_idx]

        return {'inputs': inputs, 'output': output}

    def train(self, stim, epochs, save_model: bool = True):
        '''Function to train an RNN model This function will save the trained model afterwards.

        Args:
            stim: dict containing 'inputs' and 'output' as input and target data for training the model.

                'inputs': [n_batch x n_time x n_bits] numpy array containing
                input pulses.
                'outputs': [n_batch x n_time x n_bits] numpy array specifying
                the correct behavior of the FlipFlop memory device.

        Returns:
            None.'''

        self.model.compile(optimizer="adam", loss="mse", metrics=['accuracy'])
        history = self.model.fit(tf.convert_to_tensor(stim['inputs'], dtype=tf.float32),
                                 tf.convert_to_tensor(stim['output'], dtype=tf.float32),
                                 epochs=epochs)
        if save_model:
            self._save_model()
        return history

    def _save_model(self):
        '''Save trained model to JSON file.'''
        self.model.save(os.getcwd()+"/saved/"+self.hps['rnn_type']+"model.h5")
        print("Saved "+self.hps['rnn_type']+" model.")

    def load_model(self):
        """Load saved model from JSON file.
        The function will overwrite the current model, if it exists."""
        try:
            self.model = load_model(os.getcwd()+"//saved/"+self.hps['rnn_type']+"model.h5")
            print("Loaded "+self.hps['rnn_type']+" model.")
        except OSError:
            self.model = load_model(os.getcwd()+"//fixedpointfinder/saved/"+self.hps['rnn_type']+"model.h5")
            print("Loaded "+self.hps['rnn_type']+" model.")


def visualize_flipflop(prediction, stim):
    """Function to visualize the n-Bit flip flop task."""

    plt.plot(prediction[0, :, 0], c='r')
    plt.plot(stim['inputs'][0, :, 0], c='k')

    plt.yticks([-1, +1]), plt.grid()
    plt.xlabel('Time')

    plt.show()


if __name__ == "__main__":
    os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

    ############################################################
    # Create and train recurrent model on 3-Bit FlipFop task
    ############################################################
    # specify architecture e.g. 'vanilla' and number of hidden units
    rnn_type, n_hidden = 'gru', 12

    flopper = Flipflopper(rnn_type=rnn_type, n_hidden=n_hidden)
    stim = flopper.generate_flipflop_trials()

    flopper.train(stim, 4000, save_model=True)
    # if a trained model has been saved, it may also be loaded
    # flopper.load_model()

    # visualize a single batch after training
    prediction = flopper.model.predict(tf.convert_to_tensor(stim['inputs'], dtype=tf.float32))
    visualize_flipflop(prediction, stim)
