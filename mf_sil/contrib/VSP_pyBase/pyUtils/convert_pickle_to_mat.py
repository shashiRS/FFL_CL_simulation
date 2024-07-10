import pickle as pickle
import sys
import scipy.io
import os

import cmpy_helpers as helpers

sys.path.append( os.path.join( os.path.dirname(__file__), '..', 'pySimEval' ) )
import cmpyeval_lib

# get pickle file to convert
pickle_file = helpers.fileOpenDialog()
# figure out where to save
mat_file = '{0}.mat'.format(pickle_file[:-7])

# load pickle
with open(pickle_file,'r') as fh:
    dat=pickle.load(fh)

scipy.io.savemat(mat_file,dat)

print('Finish')