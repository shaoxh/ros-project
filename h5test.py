import h5py
f = h5py.File('/media/tongji-survey/zeran_ssd/backpack/calib1112/250/2020-11-12-17-58-42.h5', 'r')
print f['image250'].keys()