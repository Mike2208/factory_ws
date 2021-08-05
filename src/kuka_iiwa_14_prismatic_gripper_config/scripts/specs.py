import numpy as np
import cv2
import os
from scipy import optimize as op


# Find the experiment directory (because names may vary a bit)
exp_dir = os.environ['HOME']+'/.opt/nrpStorage/'
for f_name in os.listdir(exp_dir):
    if f_name.startswith('demonstrator6'):
        exp_dir += f_name+'/resources/'
        break  # only take the first matching directory

# Define the filters to find the target
pths  = [exp_dir+'fltr_'+c+'.bmp' for c in ['r', 'g', 'b', 'k']]
flts  = [cv2.imread(pth, cv2.IMREAD_GRAYSCALE) for pth in pths]
(flt_cols, flt_rows) = flts[0].shape
# colors = ['r', 'g', 'b', 'r+b=purple', 'r+g=yellow', 'g+b=turquoise', 'r+g*0.6=orange']


# Identify target location and orientation
def localize_target(img):

    # Prepare the probe
    probe = 255*img.numpy().transpose(1,2,0)
    probe = cv2.cvtColor(np.ascontiguousarray(probe, dtype=np.uint8), cv2.COLOR_RGB2GRAY)
 
    # Detect the best matching location
    max_val  = 0
    targ_pos = [np.nan, np.nan]
    for flt in flts:
        res = cv2.matchTemplate(probe, flt, cv2.TM_CCOEFF_NORMED)
        _, val, _, loc = cv2.minMaxLoc(res)
        if val > max_val and val > 0.65:
            max_val  = val
            targ_pos = loc

    # Return the target position (order = row, col)
    return (targ_pos[0] + flt.shape[0]//2, targ_pos[1] + flt.shape[1]//2)


# Highlight the target with a red box
def mark_target(img, targ_pos):

    # If the target position exists, mark it
    if not any([np.isnan(t) for t in targ_pos]):

        # Build the box boundaries
        (cntr_row, cntr_col) = (targ_pos[1], targ_pos[0])  # See comment line 61 of img_to_pred.py
        (frst_row, frst_col) = (cntr_row - flt_rows//2, cntr_col - flt_cols//2)
        (last_row, last_col) = (cntr_row + flt_rows//2, cntr_col + flt_cols//2)
        (frst_row, frst_col) = (max(frst_row, 0              ), max(frst_col, 0              ))
        (last_row, last_col) = (min(last_row, img.shape[-2]-1), min(last_col, img.shape[-1]-1))
        
        # Draw the box
        if frst_row < last_row and frst_col < last_col:
            img[:,[frst_row,last_row],  frst_col:last_col ] = 0.0
            img[:, frst_row:last_row , [frst_col,last_col]] = 0.0
            img[0,[frst_row,last_row],  frst_col:last_col ] = 1.0
            img[0, frst_row:last_row , [frst_col,last_col]] = 1.0
    
    # Return the marked images
    return img


# Identify target location and orientation
def complete_target_positions(targ_pos):

    # Build the existing data points
    time_data = [t+1  for t, p in enumerate(targ_pos) if not np.isnan(p[0])]
    row_data  = [p[0] for    p in           targ_pos  if not np.isnan(p[0])]
    col_data  = [p[1] for    p in           targ_pos  if not np.isnan(p[0])]
 
    # Fit curves to find the missing points
    pr, _ = op.curve_fit(f, time_data, row_data)
    pc, _ = op.curve_fit(f, time_data, col_data)

    # Complete the missing points
    time = [t+1 for t in range(len(targ_pos))]
    rows = [int(f(t, *pr)) if np.isnan(p[0]) else p[0] for (t,p) in zip(time, targ_pos)]
    cols = [int(f(t, *pc)) if np.isnan(p[1]) else p[1] for (t,p) in zip(time, targ_pos)]

    # Return the complete set of points to the simulation
    return zip(rows, cols)


# Simple function
def f(x, p0, p1):
    return p0*x + p1
