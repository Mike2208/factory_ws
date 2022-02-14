#!/usr/bin/env python
"""
This file is part of the WorkMATe source code, provided as supplemental material
for the article:
    "Flexible Working Memory through Selective Gating and Attentional Tagging"
Wouter Kruijne, Sander M Bohte, Pieter R Roelfsema, Christian N L Olivers

Please see the README file for additional details.
Questions or comments regarding this file can be addressed to w.kruijne@vu.nl
-------------------
This file implements four functions for individual runs for the four tasks
tasks. The random seeds that each run get here yield 'illustrative'
convergence times.
"""
import numpy as np
import matplotlib.pyplot as plt

from  multiprocessing import Pool

import tasks
import workmate
import workmate_PG

np.random.seed(2187682668) # a randnum [0- 1e10]
ncore = 4

def _run_trial(agent, env,log_q=False, **kwargs):
    """
    Generic function to run a trial:
    agent   : the agent being run
    env     : the environment implementing the task
    env     : if log_q=True, function returns both the reward and the output
                values in the q-layer

    Additional kwargs can be passed to be set in the environment (not used here)
    """
    if not kwargs is None:
        env.__dict__.update(kwargs)
    # init trial results:
    totrew = 0.0
    qlog = []
    # step until obs == 'RESET', collect reward and q-values
    while True:
        totrew += agent.step()
        if log_q:
            print(agent.x)
            qlog.append (agent.q)
        if agent.obs == 'RESET':
            break
    # format result:

    retval = (totrew, np.array(qlog)) if log_q else totrew
    return retval

def run_dms(seed=1, viz_converged=True):
    np.random.seed(seed)

    n_switches = 5 # i.e. 6 stimulus sets, 3 stimuli per set
    dms = tasks.DMS(n_stim=3, n_switches=n_switches)
    # init the agent
    agent = workmate.WorkMATe(dms, nblocks=2)


    ### Initialize training:
    # buffer to store 'the moment of switching' (=convergence)
    iswi = np.zeros(n_switches+1)
    # buffer for last 500 trials:
    total_buff      = np.zeros(500) 
    # buffer for performance immediately after a switch
    swiperf = np.nan * np.ones((n_switches+1, total_buff.size))
    # counters
    i = 0 
    i_ = 0

    while True:
        # run trial, get performance
        r = _run_trial(agent, dms)
        # increase i
        i += 1
        # was the trial correct?
        corr = ( r >= dms.bigreward )
        total_buff[0] = corr
        total_buff = np.roll(total_buff, 1)

        # if the past 100 trials were 85% correct, set is 'learned'
        if np.mean(total_buff[:100]) >= .85:
            print('Convergence at {}\tSwitch to set {}'.format(i,dms.setnr+1))
            iswi[dms.setnr] = i
            # if criterion reached in less than 500 trials,
            # 'performanc post-switch' hasn't been logged yet -- do that now,
            # using only the trials with this set:
            if i < i_ + 500:
                swiperf[dms.setnr, :(i- i_)] = total_buff[:(i - i_)] # leaves nans for the rest of performance

            if dms.setnr == 5:
                break

            dms.switch_set()
            total_buff *= 0 # reset performance buffer
            i_ = i

        # @ iswi + 500: store post-switch performance:
        if i == i_ + 500:
            swiperf[dms.setnr, :] = total_buff

        # print progress:
        if i % 250 == 0:
            print(i, '\t', np.mean(total_buff))
    return (iswi, swiperf)

# TODO: variant 2?


def run_otax(seed=1):
    """
    12-AX, trial-based with curriculum learning
    """
    np.random.seed(seed)
    otax = tasks.OneTwoAX()
    agent = workmate.WorkMATe(otax)
    total_buff      = np.zeros(100)
    lesson_perf_buff = np.zeros((2, 5))

    i = 0    # how many trials were run?
    k = 0  # how many trials were run at the current level?

    while True:
        # run trial, get performance
        r = _run_trial(agent, otax)
        # increase i
        i += 1
        k += (1 if otax.trial_at_highest_level else 0)
        # store 'correct' in buffer(s)
        corr = ( r >= otax.bigreward )
        total_buff[0] = corr
        total_buff = np.roll(total_buff, 1)

        # check whether crit is met:
        if np.mean(total_buff) > .85:
            print('Converged at difficulty {}'.format(otax.difficulty))
            print('after {} /// {} trials'.format(i, k))
            lesson_perf_buff[:, otax.difficulty] = i,k
            print(lesson_perf_buff)

            if otax.difficulty == 4:
                break
            otax.difficulty += 1
            total_buff *= 0 
            i = 0
            k = 0
        # print progress.
        if (i>0) and (i % 250 == 0):
            print(i, '\t', np.mean(total_buff))
    return lesson_perf_buff

"""
ABAB ordered recognition task:
"""
def run_abba(seed):
    np.random.seed(seed)
    # create abba environment
    abba= tasks.ABBA_recog()
    # create agent:
    agent = workmate.WorkMATe(abba, nhidden=30)

    # buffers; total & per trial type
    total_buff      = np.zeros(100)
    trtype_buff     = np.zeros((4, total_buff.size))
    res = []
    i = 0 

    abba_i = np.zeros(5)
    while True:
        r = _run_trial(agent, abba, p = 1-np.mean(trtype_buff, axis=1))
        i += 1

        # store whether it was correct
        tp = abba.trial_type
        corr = ( r >= abba.bigreward )
        total_buff[0] = corr
        total_buff = np.roll(total_buff, 1)
        trtype_buff[tp, 0] = corr
        trtype_buff[tp, :] = np.roll( trtype_buff[tp,:], 1 )

        # 'convergence' on individual trial types:
        if abba_i[tp] == 0 and np.mean(trtype_buff[tp, :]) > 0.75:
            print("Done with ", tp)
            abba_i[tp] = i

        # criterion for full convergence
        if np.all(np.mean(trtype_buff, axis=1) > .75) and np.mean(total_buff) > .85:
            print('Done.', i)
            abba_i[-1] = i
            break

        # Uncomment this for dynamic condition:
        # if i % 3000== 0:
        #     abba.switch_set()

        # print progress.
        if i % 1000 == 0:
            print(i, '\t'.join([str(v) for v in np.mean(trtype_buff, axis=1)]))
            print('\t',np.mean(total_buff))
            step_arr = np.r_[np.mean(total_buff), np.mean(trtype_buff, axis=1)]
            res += [step_arr]
    # return np.array(res) 
    return abba_i


"""
Pro-/Antisaccade task
"""
def run_ggsa(seed=1,prefixed_gates=False):
    np.random.seed(seed)
    # create abba
    ggsa= tasks.GGSA()
    # create agent
    if prefixed_gates:
        agent = workmate_PG.WorkMATePG(ggsa)
    else:
        agent = workmate.WorkMATe(ggsa)

    # 2 buffers: overall & per trial-type
    total_buff      = np.zeros(100)
    trtype_buff     = np.zeros((2, total_buff.size))
    i = 0    
    while True:
        r = _run_trial(agent, ggsa)
        i += 1

        # store 'correct' in buffer(s)
        tp = ggsa.trial_type
        corr = ( r >= ggsa.bigreward )
        total_buff[0] = corr
        total_buff = np.roll(total_buff, 1)
        trtype_buff[tp, 0] = corr
        trtype_buff[tp, :] = np.roll( trtype_buff[tp,:], 1 )

        # check whether crit is met:
        separate_perf = np.mean(trtype_buff, axis=1)
        total_perf    = np.mean(total_buff)
        if (np.all(separate_perf) > .75) and (total_perf > .85):
            print('Converged after {}'.format(i))
            print(i, '\t'.join([str(v) for v in np.mean(trtype_buff, axis=1)]))
            print("=========")
            break
        # print progress.
        if i % 250 == 0:
            print(i, '\t'.join([str(v) for v in np.mean(trtype_buff, axis=1)]))
            print('\t',np.mean(total_buff))
    return i


def run_warehouse(seed=1):
    np.random.seed(seed)

    wh = tasks.WareHouse(19, 1, 0.6)
    wh.reset()

    agent = workmate.WorkMATe(wh)
    # agent.load_model()
    r_ = []
    i = 0
    while True:
        r = _run_trial(agent, wh)
        r_.append(r)
        i += 1
        if i % 200 == 0:
            print(np.round(np.mean(r_[-200:]), 3))
        # print(r)
        if i == 50000:
            print("done")
            break
    return agent


if __name__ == '__main__':
    # run_dms(seed=1)
    # run_otax(seed=4)
    # run_abba(seed=5)
    # run_ggsa(2)
    # run_ggsa(2,prefixed_gates=True)
    agent = run_warehouse(seed=1)

    o, r, a, t = [], [], [], []
    for i in range(100):
        agent.step()
        o.append(agent.env.obs)
        r.append(agent.r)
        a.append(agent.action)
        t.append(agent.env.data['output'][agent.env.t])

    a = np.vstack(np.array(a))
    o = np.vstack(np.array(o))
    t = np.vstack(np.array(t))

    o = o[a != -1]
    t = t[a != -1]
    a = a[a != -1]
    a[a == 0] = -1

    plt.figure()
    plt.subplot(211)
    plt.plot(o), plt.ylabel('observation [-]')
    plt.yticks([-1, 0, 1], ['nh', 'ns', 'h'])  # Signals: No Human, No trigger Signal, Human

    plt.subplot(212)
    plt.plot(a)
    plt.plot(t)
    plt.xlabel('time [s]'), plt.legend(['chosen action', 'desired action'])
    plt.ylabel('action [-]'), plt.yticks([-1, 1], ['fast', 'slow'])  # responses
    plt.show()

    agent.save_model()
