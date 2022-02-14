#!/usr/bin/env python
"""
This file is part of the WorkMATe source code, provided as supplemental material
for the article:
    "Flexible Working Memory through Selective Gating and Attentional Tagging"
Wouter Kruijne, Sander M Bohte, Pieter R Roelfsema, Christian N L Olivers

Please see the README file for additional details.
Questions or comments regarding this file can be addressed to w.kruijne@vu.nl
-------------------
This file implements a 'get_obs()' function, which is used by the agents
to convert a symbolic 'observation' at a time point t into an input vector 
code, combining an input representation for the time cells and an input 
representation for the stimulus offered by the environment.

If this file is run independently, it will plot two example input vectors 
for an entire Pro/antisaccade trial and for a 12-AX trial
"""

import numpy as np
import string

##################### time cell coding #####################
maxtime  = 20
# Time vectors are created by convolving a response vector 
# with an identity matrix, yielding [maxtime] rows of time cell responses,
# each peaking at a unique, consecutive time.
# z = [0.1, 0.25, 0.5, 1, 0.5, 0.25, 0.1]
z = [0.05, 0.075, 0.1, 0.125, 0.175, 0.25, 0.35, 1, 0.5, 0.35, 0.25, 0.175, 0.125, 0.1, 0.075, 0.05]
# the '3'-cropping here removes edge artefacts from convolution; 
# Time cell 0 (at row 0) peaks at the first moment in time (column 0).
tmat = np.vstack([np.convolve(z, t)[ 3:t.size + 3] for t in np.eye(maxtime)])


##################### stimulus coding  #####################
stimbits = 7 # (= (256 unique stimuli))
# construct binary stim_repres
binstr = '0{}b'.format(stimbits)
binstrings = [format(i, binstr) for i in range(2**stimbits)]
tobinarr = lambda s : np.array([float(c) for c in s])
Dx = np.vstack(  [tobinarr(i) for i in binstrings]  )

# Dx now is a matrix of 128 x 7 bits. 'stimbits' is a dict that will order the 
# first 52 of these in a lookup table, 
chars = string.ascii_lowercase + string.ascii_uppercase
stimdict = dict( zip( chars, Dx ) )

# Stimuli with these 5 letters are used in prosaccade/antisaccade, and here made
# linearly separable, cf. Rombouts et al., 2015
stimdict['g'] = np.zeros(stimbits)
stimdict['p'] = np.eye(stimbits)[0]
stimdict['a'] = np.eye(stimbits)[1]
stimdict['l'] = np.eye(stimbits)[2]
stimdict['r'] = np.eye(stimbits)[3]


# digits, used in 12-AX, are added to the stimdict in a similar manner
digdict = dict( 
    [(d,Dx[i + 2**(stimbits-1) ]) for i,d in enumerate(string.digits) ])
stimdict.update( digdict )

def get_obs(obs='A', t=0):
    # return time-stim vec:
    return np.r_[ tmat[t], stimdict[obs] ]

##################### visualization of stimuli  #####################
if __name__ == '__main__':
    import matplotlib.pyplot as plt
    ################ Example sequnece of Pro/Antisaccade  ###########
    ggsa_seq = 'plppg'
    O =  np.vstack([get_obs(s, t) for t,s in enumerate(ggsa_seq)])
    plt.imshow(O.T, interpolation='nearest', cmap='seismic', vmin=-1, vmax=1)
    plt.show()
    ################ Example sequnece of 12-AX  ###########
    axseq = '1RGY2SZABY'
    O =  np.vstack([get_obs(s, t) for t,s in enumerate(axseq)])
    plt.imshow(O.T, interpolation='nearest', cmap='seismic', vmin=-1, vmax=1)
    plt.show()

    plt.figure()
    plt.imshow(tmat)
    plt.colorbar(), plt.axis('off')
    plt.savefig('./tmat')

