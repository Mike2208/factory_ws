# Working Memory Module
This version of a working memory module is designed to support
the Proof of Concept of a warehouse robot navigation problem. 
Specifically, the working memory problem is to change behavior upon 
presentation of a stimulus and maintain the same behavior until a release 
stimulus is presented. 

## Installation
This module is built and tested with `Python3.6`. Requirements can be installed with

``pip install -r requirements.txt``

No GPU acceleration is required to run the code. 

## How to use the code
### `WorkMATe`
Large parts of the implementation are based on:

> Kruijne, W., Bohte, S. M., Roelfsema, P. R., & Olivers, C. N. L. (2021). Flexible Working Memory Through Selective Gating and Attentional Tagging. Neural Computation, 33(1), 1–40. [https://doi.org/10.1162/NECO_A_01339](https://doi.org/10.1162/NECO_A_01339)

The authors original implementation can be found here: https://osf.io/jrkdq/?view_only=e2251230b9bf415a9da837ecba3a7d64.

Notable additions are creating an additional task (`tasks.py`)for the warehouse problem as well as an appropriate 
execution script (`runner.py`). Execution of `runner.py` will train `WorkMATe` on 50,000 examples with sequences of 20 timesteps.

### `GRU`
The same problem has been solved as 1-bit flipflop sequence modeling task. Here a gated recurrent unit architecture learns to remember the last
seen stimulus until a new stimulus is presented. Training of this network is done by executing `n_bit_flip_flop.py`.


### `main.py`

`main.py` executes the `ros_working_memory` object. It allows to chose either 
`WorkMATe` or `GRU`. `GRU` achieves signifantly better performance. 

## Limitations and Considerations
- This is a simple Proof of Conecpt version and not a perfectly robust implementation of the working memory module.
- `WorkMATe` supports at maximum 20 timesteps, e.g. 10s with dt = 0.5s at the moment. This is due to behavior of the learning algorithm and can probably be improved in the future.
- `GRU` is more accurate than `WorkMATe` which occasionally displays undesired behavior. 