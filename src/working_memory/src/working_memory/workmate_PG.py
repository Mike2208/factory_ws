#!/usr/bin/env python
"""
This file is part of the WorkMATe source code, provided as supplemental material
for the article:
    "Flexible Working Memory through Selective Gating and Attentional Tagging"
Wouter Kruijne, Sander M Bohte, Pieter R Roelfsema, Christian N L Olivers

Please see the README file for additional details.
Questions or comments regarding this file can be addressed to w.kruijne@vu.nl
-------------------
The class defined in this file implements a special case of the WorkMATe model;
It has a policy for memory gating that is fixed, and appropriate for the 
Pro-/antisaccade task.
This illustrates the added complexity of gating.

This class inherits from 'normal' WorkMATe, but overwrites the action selection 
process, and the _act function
"""
import numpy as np 

import inputs
from workmate import WorkMATe

class WorkMATePG(WorkMATe):
    """
    To illustrate the added complexity of gating for Pro-/antisaccade
    replace gating actions by a preset gating policy
    -- This agent is ONLY valid for ggsa task!
    """
    def __init__(self, *args, **kwargs):
        super(WorkMATePG, self).__init__(*args, **kwargs)
        return 

    def action_selection(self):
        """
        # See super(); afterwards, z in gating module are all set to 0:
        """
        super(WorkMATePG, self).action_selection()
        self.z[self.zmods == 1] = 0
        return

    def _act(self):
        """
        zext -- for external actions -- is still the same.
        zint is here filled in based on the observation
        f goes into 0;  l,r go in store 1; 'g' is not encoded.
        """
        zext =  self.z[self.zmods == 0]
        zint =  self.z[self.zmods == 1] # these are always 0.
        # hard_code correct gating:
        zint[0] = 1 if self.obs in ['l', 'r'] else 0
        zint[1] = 1 if self.obs != ['l', 'r'] else 0
        if self.obs == 'g':
            zint *=0
            zint[-1] == 1
        self.action = np.argmax(zext)
        self.update_memory( zint )
        return


if __name__ == '__main__':
    from tasks import GGSA
    from runner import _run_trial

    env = GGSA(  )
    agent = WorkMATePG(env)
    
    total_buff      = np.zeros(100)
    trtype_buff     = np.zeros((2, total_buff.size))
    i = 0    
    totrew = 0
    while True:
        # run trial, get performance
        r = _run_trial(agent, env)
        # increase i
        i += 1
        # store 'correct' in buffer(s)
        tp = env.trial_type
        corr = ( r >= env.bigreward )
        total_buff[0] = corr
        total_buff = np.roll(total_buff, 1)
        trtype_buff[tp, 0] = corr
        trtype_buff[tp, :] = np.roll( trtype_buff[tp,:], 1 )

        # check whether crit is met:
        if np.all(np.mean(trtype_buff, axis=1) > .75) and np.mean(total_buff) > .85 or (i > 5e5):
            print('Done.', i)
            print(np.mean(trtype_buff, axis=1))
            break
        if i % 25 == 0:
            print("{}\t{}".format(i, np.mean(trtype_buff, axis=1) ))