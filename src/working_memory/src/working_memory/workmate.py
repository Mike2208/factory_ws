#!/usr/bin/env python
"""
This file is part of the WorkMATe source code, provided as supplemental material
for the article:
    "Flexible Working Memory through Selective Gating and Attentional Tagging"
Wouter Kruijne, Sander M Bohte, Pieter R Roelfsema, Christian N L Olivers

Please see the README file for additional details.
Questions or comments regarding this file can be addressed to w.kruijne@vu.nl
-------------------
The class defined in this file implements the WorkMATe model;
Entry point for the model is the 'step()' function which defined what WorkMATe
does during a single time step; this entails: 
1-  the feedforward sweep to determine its  next action, 
2-  Learning: update the weights in response to any potentially obtained reward
3-  recurrent attentional feedback to place synaptic tags on connections 
    responsible for the selected action. 
4-  Executing the desired action, including possibly storing 
    an observation in memory.
"""

import os
import pickle

import numpy as np
import scipy.spatial.distance
from IPython import embed
import inputs


class WorkMATe(object):
    """
    Architecture schematic:
        x -- S
          \/
          h
          |
    [q_int|q_ext]
    """
    def __init__(self, env=None, nhidden=15, nblocks=2, block_size=15):
        super(WorkMATe, self).__init__()
        assert env is not None
        self.env=env

        ## member lambda functions:
        # sigmoid transfer function, offset at 2.5
        self.transfer = lambda x: 1 / ( 1. + np.exp(2.5 - x) )
        self.dtransfer= lambda x: x * (1. - x) # derivative
        # softmax normalization; for action selection - boltzmann controller
        self.softmaxnorm = lambda x: (
            np.exp( x - x.max() ) / np.exp( x - x.max() ) .sum() )

        ## init network architecture -- inputs and output shape from env
        # input and hidden
        nx = inputs.get_obs('a').size
        nh = nhidden
        # memory cell properties:
        self.nblocks = nblocks
        self.block_size = block_size
        nS = nblocks * block_size
        
        # output -- q layer consisting of 2 modules
        # module for n external actions, internal actions for nblocks + 1 (null) 
        mod_sz = env.n_actions, nblocks + 1
        nq = np.sum(mod_sz)
        # indices of module for each node:
        self.zmods = np.hstack( [ [i] * sz for i, sz in enumerate( mod_sz ) ] )

        ## init network layers (activations 0)
        # (x will be constructed when processing 'new_obs')
        self.S = np.zeros( nS )
        self.h = np.zeros( nh ) 
        self.q = np.zeros( nq ) 

        ## init weights, tags traces, (+1 indicates projection from bias node)
        wl, wh = -.5, .5
        # Memory projection (x > S)
        self.W_Sx  = np.random.sample( (nS, nx) )  * (wh-wl) + wl 
        # Note that time and sensory input cells are not separated in memory

        # PLASTIC CONNECTIONS (all except memory projection)
        wl, wh = -.25, .25
        # connections x->h; nx + match nodes + bias
        self.W_hx  = np.random.sample( (nh, nx + nblocks + 1) ) * (wh-wl) + wl
        # connections S->h
        self.W_hS  = np.random.sample( (nh, nS    ) ) * (wh-wl) + wl
        # connections h->q:
        self.W_qh  = np.random.sample( (nq, nh + 1) ) * (wh-wl) + wl
        # tags are shaped like weights but initialized at 0:
        zeros_ = np.zeros_like
        self.Tag_W_hx, self.Trace_W_hx = zeros_(self.W_hx), zeros_(self.W_hx)
        self.Tag_W_hS, self.Trace_W_hS = zeros_(self.W_hS), zeros_(self.W_hS)
        self.Tag_W_qh, self.Trace_W_qh = zeros_(self.W_qh), zeros_(self.W_qh)

        ## learning params (adopted from Rombouts et al., 2015)
        self.beta = 0.15
        self.gamma = 0.7  # 0.9
        self.L = 0.6      # 0.8
        # exploration:
        self.epsilon = 0.025

        # Init action state
        self.action = -1
        # (prev) predicted reward:
        self.qat_1 = self.qat = None
        self.t   = 0 
        return

    def _intertrial_reset(self):
        """
        Reset time, memory, tags and traces
        """
        self.t = 0 
        self.S *= 0

        # previous action = zeros 
        self.z *= 0
        # reset tags/traces for each Wmat
        zeros_ = np.zeros_like
        self.Tag_W_hx, self.Trace_W_hx = zeros_(self.W_hx), zeros_(self.W_hx)
        self.Tag_W_hS, self.Trace_W_hS = zeros_(self.W_hS), zeros_(self.W_hS)
        self.Tag_W_qh, self.Trace_W_qh = zeros_(self.W_qh), zeros_(self.W_qh)

        # reset 'current action', and the like
        self.action = -1
        self.qat = None
        return
    
    def step(self):
        # get observation and reward from env
        # print(self.action)
        self.obs, self.r = self.env.step(self.action)
        # print(self.obs)
        # do feedforward:
        self._feedforward()
        # learn from the obtained reward
        self._learn()
        # end of trial?
        if 'RESET' in self.obs:
            self._intertrial_reset()
            return self.r
        # do feedback (tag placement)
        self._feedback()
        # act (internal, external)
        self._act()
        self.t += 1
        return self.r

    def step_wh(self, o, r):
        # get observation and reward from env
        # print(self.action)
        self.obs, self.r = o, r
        # print(self.obs)
        # do feedforward:
        self._feedforward()
        # learn from the obtained reward
        self._learn()
        # end of trial?
        if 'RESET' in self.obs:
            self._intertrial_reset()
            return self.r
        # do feedback (tag placement)
        self._feedback()
        # act (internal, external)
        self._act()
        self.t += 1
        return self.r


    def _feedforward(self):
        # shift previous action
        self.qat_1 = self.qat
        if 'RESET' in self.obs:
            # no meaningful feedforward sweep:  qat is not computed
            self.qat = None
            return
        # else:
        # compute input, hidden, output:
        self.construct_input()
        self.compute_hidden()
        self.compute_output()
        # determine z from q (action selection)
        self.action_selection()
        # determine new qat
        self.qat = (self.z * self.q).sum()
        return

    def _learn(self):
        """
        Learn from the reward; compute RPE and update weights
        general form form delta = r + gamma * qat - qat_1
        ...but there are edge cases
        """
        r = self.r
        if self.qat and self.qat_1: # regular
            delta = r + (self.gamma * self.qat) - self.qat_1
        elif self.qat_1 is None: # first step
            delta = r + (self.gamma * self.qat) - self.qat
        else: # self.qa(t) is None (final step):
            delta = r - self.qat_1
        self.delta = delta
        self.update_weights()
        return

    def _feedback(self):
        # updates traces and tags, based on action selection:
        # traces and tags
        self.update_traces()
        self.update_tags()
        return

    def _act(self):
        # external and internal actions:
        zext =  self.z[self.zmods == 0]
        zint =  self.z[self.zmods == 1]
        self.action = np.argmax(zext)
        self.update_memory( zint )
        return

    @staticmethod
    def match(x,y):
        ons = np.maximum(y-x, 0.0 )
        off = np.maximum(x-y, 0.0 )
        return (ons+off).sum()/x.size

    def construct_input(self):
        """
        Turn obs into a vector; uses coding defined in 'inputs.py'
        """
        # input consists of: observation and time t
        self.x_sens = x_sens = inputs.get_obs(self.obs, self.t)
                
        # Compute match value:
        Sproj = self.W_Sx.dot(x_sens).reshape( (self.nblocks, self.block_size) )
        matches = 1- scipy.spatial.distance.cdist( 
            Sproj, self.S.reshape(Sproj.shape), 
            metric = WorkMATe.match) .diagonal()

        # add match nodes + bias to input vector
        self.x = np.r_ [x_sens, matches, 1.0]
        return

    def compute_hidden(self):
        # x->h  +  S->h
        self.S_out  = self.S
        # Compute Ha and h
        ha = self.W_hx.dot(self.x) + self.W_hS.dot(self.S_out)
        # print(self.x)
        # print(self.S_out)
        # print(ha)
        self.h_out  = np.r_[ self.transfer(ha), 1.0 ] # bias added
        # print(self.h_out)
        return

    def compute_output(self):
        # hidden output (has bias added)
        self.q = self.W_qh.dot(self.h_out)
        # (no transfer, q nodes are linear)

        return

    def action_selection(self):
        # using q, per module, determine z (based on argmax or exploration)
        self.z = np.zeros_like(self.q)
        # action selection for both modules separately
        for mod_idx in np.unique(self.zmods):
            qvec = self.q[self.zmods == mod_idx] # get the module's qvalues 
            # check exploration; if not just take argmax:
            if ( np.random.sample() >= self.epsilon ):
                action = np.argmax(qvec)
            else: # compute softmax over Q and explore:
                pvec = self.softmaxnorm(qvec)
                action = np.random.choice( range(qvec.size), p = pvec) 
            # set zvec: 1-hot code of actions:            
            zvec = np.zeros_like(qvec)
            zvec[ action ] = 1.0
            # place zvec into z at the right indices:
            self.z[self.zmods == mod_idx] = zvec
        return

    def update_weights(self):
        """
        all Weight-Trace pairs are updated with the same rule:
        w += beta * delta * tag
        """
        wt_pairs = ( ( self.W_hx, self.Tag_W_hx ), 
                     ( self.W_hS, self.Tag_W_hS ),
                     ( self.W_qh, self.Tag_W_qh ), )
        for W, Tag in wt_pairs:
            W += self.beta * self.delta * Tag
        return

    def update_traces(self):
        """
        Traces are the intermediate layers' markers
        The Traces are a relic from old AuGMEnT code, have no 'meaning' here
        """
        # Regulars, are replaced by new input:
        self.Trace_W_hx *= 0.0
        self.Trace_W_hS *= 0.0
        # add 1 x X vec to H x X matrix yields H copies of 1 x X vec
        self.Trace_W_hx += self.x.reshape(1, self.x.size) # this includes trace for bias
        self.Trace_W_hS += self.S_out.reshape(1, self.S.size)
        return

    def update_tags(self):
        # 1. old tag decay:
        alltags = ( self.Tag_W_hx, self.Tag_W_hS, self.Tag_W_qh)
        for Tag in alltags:
            Tag *= (self.L * self.gamma)

        # 2. form new tags:
        # tags onto output units: selected action.
        self.Tag_W_qh[self.z.astype('bool'), :] += self.h_out

        # feedback to hidden
        dh = self.dtransfer(self.h_out[:-1])
        fbh= self.W_qh[self.z.astype('bool'), :-1] # excluding the bias node
        fbh = fbh.sum(axis = 0 ) # summed contribution of all actions
        feedback = fbh * dh
        self.Tag_W_hx +=  np.expand_dims(feedback, 1) *  self.Trace_W_hx 
        self.Tag_W_hS +=  np.expand_dims(feedback, 1) *  self.Trace_W_hS
        return

    def update_memory(self, zvec):
        # final z is 'do not gate'; nothing happens then
        if not zvec[-1] == 1:
            # else:
            gate_idx = np.argmax(zvec)
            x = self.x_sens     # this excludes bias- and  match
            S_ = self.S.reshape( (self.nblocks, self.block_size) )
            W_ = self.W_Sx.reshape( S_.shape +  (x.size, ) ) 
            # project x->S (encode)
            Sproj = W_.dot(x)
            # store @ gated 'stripe'
            S_[gate_idx,:] = Sproj[gate_idx,:]
            # transform S back to its flat representation
            self.S = S_.reshape(self.S.shape)
        return

    def save_model(self):
        model = {'W_qh': self.W_qh,
                 'W_hx': self.W_hx,
                 'W_Sx': self.W_Sx,
                 'W_hS': self.W_hS}

        with open(os.getcwd()+"//saved/workmate.pkl", 'wb') as f:
            pickle.dump(model, f)

        print('Saved workmate')

    def load_model(self):

        with open(os.getcwd()+"//saved/workmate.pkl", 'rb') as f:
            model = pickle.load(f)

        self.W_qh = model['W_qh']
        self.W_hx = model['W_hx']
        self.W_Sx = model['W_Sx']
        self.W_hS = model['W_hS']