#!/usr/bin/env python

import os
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"  # execute tensorflow models on cpu for inference

import numpy as np
import tensorflow as tf

import tasks
import workmate
from n_bit_flip_flop import Flipflopper


class WorkingMemory(object):

    def __init__(self, wm_type: str = "gru"):
        """
            WorkingMemory - class to implement working memory module with the goal to equip
            a robot with memory to guide behavior.
        :param workmate: boolean flag to indicate whether to use workmate architecture or gru
        """

        self.wm_type = wm_type

        if self.wm_type == "workmate":
            self.wh = tasks.WareHouse(19, 1, 0.5)
            self.wh.reset()
            self.agent = workmate.WorkMATe(self.wh)  # for simplicity but will be connected to gazebo
            self.agent.load_model()

            self.stimdict = {1: 'h', 0: 'u', -1: 'n'}
        elif self.wm_type == "gru":
            self.wm_gru = Flipflopper(rnn_type='gru', n_hidden=12)
            self.wm_gru.load_model()
        else:
            raise NotImplementedError("working memory model must be of type workmate or gru.")

        self.o = None
        self.a = None

        self.t = 0
        self.dt = 0.5

    def input(self, o):
        """
            Function to receive and check input from gazebo environment.
        :param o: observation - can be one of three values: 1 = human detected,
                                                            0 = nothing detected,
                                                           -1 = no human signal detected
        :return: checked observation to working memory
        """
        assert o == -1 or o == 0 or o == 1, "observation must be one of -1, 0, 1"

        self.o = o
        self.t += 1

        if self.wm_type == "gru":
            o_tensor = tf.convert_to_tensor(np.reshape(np.array(o), (1, 1, 1)))
            a = self.wm_gru.model.predict(o_tensor)
            self.a = int(a[0].round(1))
            if self.a == 0:
                self.a = -1  # gru is expected to output 0 when input is 0 without earlier stimulus
        elif self.wm_type == "workmate":
            o = self.stimdict[o]
            self.agent.step_wh(o, 0)  # reward signal artificially kept at 0
            self.a = self.agent.action
            if self.a == 0:
                self.a = -1  # translate from workmate output to desired output

    def output(self):
        """
            Function to check and release working memory output to gazebo environment.
        :param a: action - can be one of two values: -1, 1
        :return: checked action from working memory
        """
        assert self.a == -1 or self.a == 1, "output must be one of -1, 1"

        return self.a

    def get_working_memory_time(self):
        return self.t * self.dt
