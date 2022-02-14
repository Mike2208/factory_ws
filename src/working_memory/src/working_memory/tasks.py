#!/usr/bin/env python
"""
This file is part of the WorkMATe source code, provided as supplemental material
for the article:
    "Flexible Working Memory through Selective Gating and Attentional Tagging"
Wouter Kruijne, Sander M Bohte, Pieter R Roelfsema, Christian N L Olivers

Please see the README file for additional details.
Questions or comments regarding this file can be addressed to w.kruijne@vu.nl
-------------------
This file implements the task environments used in the simulations.
These classes at least implement:
    - __init__()    A constructor that at least defines the n_actions 
                    for the task
    - reset()       A function to initialize a new trial 
    - step(action)  A function that evaluates the agent's action, and returns 
                    a new observation and a reward in return.
                    if this observation is 'RESET', the agent assumes the trial
                    has ended and will reset its tags and memory content

For testing purposes, this file also implements a 'Manual_agent', which allows
one to take the role of a WorkMATe agent and reply to trial sequences  with
keyboard input

If the file is run as-is, it should run the pro-/antisaccade task (GGSA) 
using the manual-agent. Other tasks can be uncommented below (l. 476-480) 
for testing
"""

import numpy as np 
import itertools 
import string
from flipflop import generate_flipflop_trials


class GGSA(object):
    """Pro-antisaccade"""
    def __init__(self, maxdelay=3):
        super(GGSA, self).__init__()
        self.maxdelay = maxdelay
        self.n_actions  = 3
        self.minireward = 0.3 # shaping rew for fixation
        self.bigreward  = 1.5 # when trial is correct
        self.fixtimeout = 3   # wait for a fix on step 1 for this long
        self.gotimeout =  4    # wait time for fixation
        self.paswi     = False# option to test (re)learning with new mapping
        self.reset()
        return

    def reset(self):
        # determine type of trial:
        self.fixtype = np.random.choice(list('pa'))
        self.cue     = np.random.choice(list('lr'))
        # target is 'left' unless LA or RP
        self.target  = 0
        CF = self.cue + self.fixtype
        if CF == 'la' or CF == 'rp':
            self.target = 2
        if self.paswi:
            self.target = 2-self.target
        self.trial_type = int(self.fixtype == 'p') # 0/1 for pro/anti
        # functions to run a  step in each  state:
        self.statefuns = [self._waitfix, self._cue, self._delay, self._go]
        self.statefun = self.statefuns.pop(0)
        # timer, to keep track how long we are in each state
        self.timer = 0
        # delay state has variable time:
        self.delaytimeout = np.random.randint(1, self.maxdelay+1)
        return

    def step(self, action):
        # if agent is back in initial state, run the reset
        if action == -1:
            self.reset()
        # run one state_function
        newobs, reward = self.statefun(action)
        # if newobs is None, this means shift state and run next state:
        if newobs is None:
            # shift to new state:
            self.timer = 0
            self.statefun = self.statefuns.pop(0)
            # run new state and add potential reward
            newobs, reward2 = self.statefun(action)
            reward += reward2
        self.timer += 1
        # return newobs and total rew
        return newobs, reward

    def _waitfix(self, action):
        # if fixation made, give mireward and shift state
        if action == 1:
            return None, self.minireward
        # ... no fixation yet, check time elapsed:
        if self.timer == self.fixtimeout:
            return 'RESET', 0.0
        return self.fixtype, 0.0

    def _cue(self, action):
        # hold until time runs out.
        # during cue, if action is anything other than fixation, RESET without reward
        if action != 1:
            return 'RESET', 0.0
        # cue always only lasts 1 timestep; so if time has passed-> shift
        if self.timer > 0:
            return None, 0.0
        # else: just show the cue for a timestep
        # NB: orig augment presents Cue + Fix; that would be cheating with 
        # WorkMATe, because then it only has to encode once.
        return self.cue, 0.0

    def _delay(self, action):
        # hold until time runs out.
        # during delay, if action is anything other than fixation, RESET without reward
        if action != 1:
            return 'RESET', 0.0
        if self.timer == self.delaytimeout:
            # delay has passed, move on to go:
            return None, 0.0
        # else, we're in delay: return the fix still:
        return self.fixtype, 0.0

    def _go(self, action):
        # If time runs out, no reward:
        # print action, self.target
        if self.timer == self.gotimeout:
            return 'RESET', 0.0
        # else, there's still time; if action is still fix, no reward yet
        if action == 1:
            return 'g', 0.0
        # else: must've moved: correct or incorrect?
        if action == self.target:
            return 'RESET', 1.5
        return 'RESET', 0.0

class OneTwoAX(object):
    """12AX task -- trial based, via curriculum learning
    The rule is: 1..A-X -> GO or 2..B-Y is Go. 
    All other stimuli are nogo """
    def __init__(self, difficulty=0):
        super(OneTwoAX, self).__init__()
        self.difficulty = difficulty
        self.n_actions = 2

        self.minireward = 0.25
        self.bigreward = 1.5
        self.trial_at_highest_level = False
        self.reset()
        return

    def reset(self):
        # What kind of trial is it?
        matches     = ['1AX','2BY'] 
        nonmatches  = [''.join(i) for i in  itertools.product('12','AB','XY') if i not in matches]
        # match trial or nonmatch trial: random choice
        self.seq = np.random.choice( [np.random.choice(matches), np.random.choice(nonmatches)] )
        self.seq = list(self.seq)

        # make a choice about trial difficulty; 50% hardest diff, 50% other
        diff = self.difficulty
        if diff == 0: 
            lvl = 0
        elif diff==1:
            lvl = np.random.choice(diff+1)
        else:
            p = [.5/diff]*diff+[.5]
            lvl = np.random.choice(diff + 1, p=p)

        # record whether trial is at the highest level
        self.trial_at_highest_level = (lvl == diff)

        # difficulty 0: 1ax -- seq already determined:
        if lvl == 0:
            return
        
        ### Difficulty 1: add random 1 or 2 at the beginning 
        ###(i.e. learn to update ctxt)
        newseq = self.seq[:]
        newseq.insert(0, np.random.choice( list('12') ) )
        if lvl == 1:
            self.seq = newseq
            return
        elif np.random.choice([True,False]): 
            # random choice; either proceed with newseq or oldseq as basis
            self.seq = newseq

        ### Difficulty 2: up to 2 random distractors between ctxt and probe:
        ndist = np.random.randint(1,3)
        newseq = self.seq[:]
        for i in range(ndist):
            newseq.insert(-2, np.random.choice( list( 'QRAB' ) ) )
        if lvl == 2: 
            self.seq=newseq
            return
    
        ## difficulty 3: insert up to 2 distractor between probe and target:
        ndist = np.random.randint(1,3)

        newseq = self.seq[:]
        for i in range(ndist):
            newseq.insert(-1, np.random.choice( list( 'QRAB' ) ) )
        if lvl == 3: 
            self.seq=newseq
            return

        ## difficulty 4: insert 2-4 distractors anywhere
        newseq = self.seq # not a copy
        ndist = np.random.randint(2, 5)
        for d in range(ndist):
            pos = np.random.choice( len(newseq) - 1 ) 
            dist = np.random.choice(list('QRAB'))
            newseq.insert(pos, dist)
        self.seq=newseq
        return

    def step(self,action):
        reward = 0.0
        if action == -1:
            self.stimulus = '0' # fake 'current' stimulus (becomes inner loop)
            self.reset()
            newobs = self.seq.pop(0)
        elif action == 0: 
            # else, process action:
            newobs, reward = self._hold()
        else:
            newobs, reward = self._go()

        # update task parameters:
        if newobs in list('12'):
            self.outer_loop = newobs
        self.inner_loop = self.stimulus
        self.stimulus = newobs

        return newobs, reward

    def _hold(self):
        if ''.join([self.outer_loop, self.inner_loop, self.stimulus]) in ['1AX','2BY']:
            # incorrect hold! no reward:
            return 'RESET', 0.0
        if self.stimulus in 'XY':
            # correct hold on X/Y! big reward!
            return 'RESET', self.bigreward
        # else, update loops and return the state:
        newobs = self.seq.pop(0)
        return newobs, 0.0

    def _go(self):
        if ''.join([self.outer_loop, self.inner_loop, self.stimulus]) in ['1AX','2BY']:
            # correct go on XY! big reward!
            return 'RESET', self.bigreward
        # else incorrect go, no rew, abort tr
        return 'RESET', 0.0

class ABBA_recog(object):
    """ABAB Recognition task: 
        Dissociate ABAB, ABBA.., ABCA..,ABAC..
    """
    def __init__(self):
        super(ABBA_recog, self).__init__()
        self.n_stim = 3 # A,B,C
        stimuli = np.random.choice( list(string.ascii_uppercase),
            self.n_stim, replace=False)
        self.stimuli = stimuli
        # n_actions
        self.n_actions  =  2
        self.bigreward =  1.5
        self.minireward =  0.5
        self.punish     = -0.0 # not used here

        # relative probability of getting chosen as a trial type
        self.p = np.zeros(4)

        self.t = 0
        self.reset()
        return

    def switch_set(self):
        stimuli = np.random.choice( list(string.ascii_uppercase),
            self.n_stim, replace=False)
        self.stimuli = stimuli
        return

    def reset(self):
        self.t = 0
        stims = self.stimuli[:]
        np.random.shuffle(stims)
        a,b,c = stims
        self.match_trial = True

        # In this version, each trial type is equally likely
        probs = np.ones(4) * .25
        self.trial_type = np.random.choice(range(4), p=probs )
        if self.trial_type > 0:
            self.match_trial = False
        trseqs = [ [a,b,a,b], [a,b,b,a,a,b], [a,b,c,b,a,b], [a,b,a,c,a,b] ]
        self.seq = trseqs[self.trial_type]
        return

    def step(self, action):
        self.t += 1
        if action == -1:
            self.reset()
            newobs = self.seq[ self.t ]
            rew = 0.0
        elif action == 0:
            newobs, rew = self._hold()
        else:# action == 1:
            newobs, rew = self._release()
        return newobs, rew

    def _release(self):
        t = self.t
        rew = 0.0
        newobs = 'RESET' # release is always reset

        # release on t==1 or t == 2 means no reward
        # T3: Go on B is punish
        if t == 3 and (self.seq[1]==self.seq[2]):
            rew = self.punish
        
        # T4: Go on match is big reward
        elif t == 4 and self.match_trial:
            rew = self.bigreward
        # Go on swap trial is punishment:
        elif (t == 4) and (self.seq[1]==self.seq[2]) and (self.seq[0]==self.seq[3]):
            rew = self.punish
        
        elif t == 6:
            rew = self.bigreward + self.minireward

        return newobs, rew

    def _hold(self):
        t = self.t
        rew = 0.0
        # see what release gives -- if it gives reward then no
        if t == 1: # first hold, minireward:
            rew = self.minireward

        # Hold on match is punish
        elif t == 4 and self.match_trial:
            rew = self.punish
            return 'RESET', 0.0
        # Hold on swap trial is minireward:
        elif t == 4 and (self.seq[1]==self.seq[2]) and (self.seq[0]==self.seq[3]):
            rew = self.minireward

        if t == 6:
            return 'RESET', 0.0

        # else: still in the sequence, so yes, hold is good
        newobs = self.seq[t]
        return newobs, rew



class DMS(object):
    """Delayed Match to sample, with switches in stimset"""
    def __init__(self, n_stim=3, n_switches=3):
        super(DMS, self).__init__()
        self.n_actions = 3 # hold fixation, go left/mismatch or go right/match
        
        self.minireward = 0.2
        self.bigreward  = 1.5

        # determine new stimuli
        stims = np.random.choice(list(string.ascii_uppercase),
            n_stim * (n_switches + 1), replace=False)
        self.stimset = stims.reshape( (n_switches+1, n_stim) )
        self.setnr   = 0 
        self.stimuli = self.stimset[self.setnr, :]
        self._newstimuli= stims[-n_stim:]
        self.reset()
        return

    def reset(self):
        self.trial_type = np.random.choice([0,1]) # 0 = match
        self.target = self.trial_type * 2
        sample, probe = np.random.choice(self.stimuli, 2, replace=False)
        if self.trial_type == 0:
            probe = sample
        self.seq = list("p{}p{}".format( sample, probe) )
        self.t = 0
        return

    def switch_set(self, target_set=None):
        # Cycle through new- and old sets:
        if target_set is None:
            target_set = (self.setnr + 1) % self.stimset.shape[0]
        assert target_set < self.stimset.shape[0] # assert always new, unknown
        self.stimuli = self.stimset[target_set, :]
        self.setnr = target_set
        return

    def step(self,action):
        self.t += 1
        if action == -1:
            self.reset()
            newobs = self.seq[ self.t ]
            return newobs, 0.0
        # else:
        if action == 1:
            newobs, reward = self._hold()
        else:
            newobs, reward = self._go(action)
        return newobs, reward

    def _hold(self):
        try:
            newobs =  self.seq[ self.t ]
            return newobs, self.minireward
        except IndexError as e:
            # out of time, should've gone!
            pass
        return  'RESET', 0.0

    def _go(self, action):
        # is is the end of the sequence?
        if self.t < len(self.seq):
            return 'RESET', 0.0
        # was it on the target?
        elif action == self.target:
            return 'RESET', self.bigreward
        return 'RESET', -1 * self.minireward


class WareHouse(object):

    def __init__(self, n_time, n_bit, p_flip):
        self.n_actions = 2

        self.n_time = n_time
        self.n_bit = n_bit
        self.p_flip = p_flip

        self.data = dict()
        self.obs = 1
        self.t = None
        self.rng = np.random.RandomState(123)

    def reset(self):
        self.data = generate_flipflop_trials(self.n_time, self.n_bit, self.p_flip, self.obs, self.rng)
        self.t = 0
        obs = self.data['inputs'][self.t]

        return obs

    def step(self, action):

        target = self.data['output'][self.t]
        if action == -1:
            reward = 0
        else:
            if action == target:
                reward = 1
            elif action == (target + 1):
                reward = 1
            else:
                reward = -0.2

        self.t += 1
        if self.t == self.n_time:
            self.reset()
            o = 'RESET'
        else:
            self.obs = self.data['inputs'][self.t]
            if self.obs == 1:
                o = 'h'
            elif self.obs == 0:
                o = 'u'
            elif self.obs == -1:
                o = 'n'
            else:
                raise ValueError("o must be one of -1, 0, 1")

        return o, reward


class Manual_agent(object):
    """for easy testing of env objects"""
    def __init__(self, env):
        super(Manual_agent, self).__init__()
        self.env = env
        self.action = -1
        self.t = 0
        return

    def step(self):
        obs, r = self.env.step(self.action)
        print("OBSERVATION {}:\t{}".format(self.t, obs))
        print("REWARD\t\t{}".format(r))
        if obs == 'RESET':
            self.action = -1
            self.t = 0
            print("-----")
            print("NEW TRIAL")
            print("-----")
            return 

        print("Choose an action: {}".format(range(self.env.n_actions)))
        act =  input('? > ')
        self.action = int(act)
        self.t += 1
        return


if __name__ == '__main__':
    # env = GGSA()
    # # env = ABBA_recog()
    # # env = OneTwoAX(difficulty=0)
    # # env = DMS()
    # agent= Manual_agent(env)
    # env.reset()
    # while True:
    #     agent.step()
    env = WareHouse(9, 1, 0.3)
    env.reset()

    import matplotlib.pyplot as plt

    plt.figure(figsize=[16, 9])
    plt.subplot(211)
    plt.plot(env.data['inputs'])
    plt.xlabel('time [s]')
    plt.yticks([-1, 0, 1], ['no human', 'unchanged', 'human'])
    plt.grid()

    plt.subplot(212)
    plt.plot(env.data['output'])
    plt.xlabel('time [s]')
    plt.yticks([-1, 1], ['fast', 'slow'])
    plt.grid()

    plt.savefig('./example')
