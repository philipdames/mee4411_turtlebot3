#!/usr/bin/env python3
# This script hold the model architecture
#------------------------------------------------------------------------------

import numpy as np
import os
import random
import torch
import torch.nn as nn
import yaml

NEW_LINE = "\n"

#------------------------------------------------------------------------------
#
# helper functions
#
#------------------------------------------------------------------------------

def set_seed(seed):
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    random.seed(seed)
    os.environ['PYTHONHASHSEED'] = str(seed)


def isPowerOfTwo(n):
    return (np.ceil(np.log2(n)) == np.floor(np.log2(n)))


def normalize(data, method, mu=None, std=None, min=None, max=None):
    '''
    mu and sigma are required for standardization
    min and max are required for min_max
    '''
    if method == 'standardization':
        assert not (mu is None or std is None)
        return (data - mu) / std

    elif method == 'min_max':
        assert not (min is None or max is None)
        return (2 * (data - min) / (max - min)) - 1


def unnormalize(data, method, mu=None, std=None, min=None, max=None):
    '''
    mu and sigma are required for standardization
    min and max are required for min_max
    '''
    if method == 'standardization':
        assert not (mu is None or std is None)
        return std * (data + mu)

    elif method == 'min_max':
        assert not (min is None or max is None)
        return ((max - min) / 2 * (data + 1)) + min


def normalize_scan(scan, method):
    # min: 0.1245, max: 3.5000, mu: 1.7127, std: 0.9558
    return normalize(scan, method, min=0.0, max=3.5000, mu=1.7127, std=0.9558)


def normalize_sub_goal(sub_goal, method):
    # min: -0.5000, max: 0.5000, mu: 0.0742, std: 0.3189
    return normalize(sub_goal, method, min=-0.5000, max=0.5000, mu=0.0742, std=0.3189)


def normalize_final_goal(final_goal, method):
    # min: -3.8780, max: 3.7970, mu: 0.1682, std: 1.2081
    return normalize(final_goal, method, min=-4.000, max=4.000, mu=0.1682, std=1.2081)


def normalize_velocities(vel):
    # MinMaxScaler: velocities
    # vx: min: -0.2200, max: 0.2200, mu: 0.0619, std: 0.1868
    # wz: min: -2.7500, max: 2.6954, mu: -0.0123, std: 0.3904
    vx_max = 0.22
    wz_max = 2.75
    vel[0] = normalize(vel[0], method='min_max', min=-vx_max, max=vx_max)
    vel[1] = normalize(vel[1], method='min_max', min=-wz_max, max=wz_max)
    return vel


def unnormalize_velocities(vel):
    # MinMaxScaler: velocities
    # vx: min: -0.2200, max: 0.2200, mu: 0.0619, std: 0.1868
    # wz: min: -2.7500, max: 2.6954, mu: -0.0123, std: 0.3904
    vx_max = 0.22
    wz_max = 2.75
    vel[0] = unnormalize(vel[0], method='min_max', min=-vx_max, max=vx_max)
    vel[1] = unnormalize(vel[1], method='min_max', min=-wz_max, max=wz_max)
    return vel


def loadDNNParams(file):
    params = yaml.load(open(file), Loader=yaml.FullLoader)

    assert 'goal_input' in params.keys()
    assert params['goal_input'] in ['final_goal', 'sub_goal']

    assert 'num_channels' in params.keys()
    assert isinstance(params['num_channels'], int) and (params['num_channels'] >= 4) and isPowerOfTwo(params['num_channels'])
    
    assert 'normalization_method' in params.keys()
    assert params['normalization_method'] in ['standardization', 'min_max']

    assert 'num_epochs' in params.keys()
    assert isinstance(params['num_epochs'], int) and (params['num_epochs'] > 0)

    assert 'batch_size' in params.keys()
    assert isinstance(params['batch_size'], int) and (0 < params['batch_size'] < 5000) and isPowerOfTwo(params['batch_size'])

    assert 'loss_function' in params.keys()
    assert params['loss_function'] in ['MAE', 'MSE']

    assert 'learning_rate' in params.keys()
    assert isinstance(params['learning_rate'], float) and (0.0 < params['learning_rate'] < 1.0)

    return params


#------------------------------------------------------------------------------
#
# data loader
#
#------------------------------------------------------------------------------

# function: get_data
#
# arguments: fp - file pointer
#            num_feats - the number of features in a sample
#
# returns: data - the signals/features
#          labels - the correct labels for them
#
# this method takes in a fp and returns the data and labels
#
class NavDataset(torch.utils.data.Dataset):
    def __init__(self, data_path, file_name, normalization_method='standardization', preload=False):
        # initialize the data and labels
        self.normalization_method = normalization_method
        self.preload = preload

        # read data
        self.scan_file_names, self.scan_data             = self.read_data(data_path, 'scans_local', file_name)
        self.sub_goal_file_names, self.sub_goal_data     = self.read_data(data_path, 'sub_goals_local', file_name)
        self.final_goal_file_names, self.final_goal_data = self.read_data(data_path, 'final_goals_local', file_name)
        self.vel_file_names, self.vel_data               = self.read_data(data_path, 'velocities', file_name)
        
        self.length = len(self.vel_file_names)
        print("dataset length: ", self.length)


    def read_data(self, data_path, folder, file_name):
        # Initialize lists
        file_names = []
        data = []

        # Get file containing all filenames
        fp = open(os.path.join(data_path, folder, file_name+'.txt'), 'r')

        # For each line of the file:
        for line in fp.read().split(NEW_LINE):
            if('.npy' in line):
                # Get filename
                fname = os.path.join(data_path, folder, line)
                file_names.append(fname)
                # Preload data, if desired
                if self.preload:
                    # Load raw data
                    d = np.load(fname)

                    # Normalize data
                    if 'scan' in folder:
                        d = normalize_scan(d, self.normalization_method)
                    elif 'sub_goal' in folder:
                        d = normalize_sub_goal(d, self.normalization_method)
                    elif 'final_goal' in folder:
                        d = normalize_final_goal(d, self.normalization_method)
                    elif 'vel' in folder:
                        d = normalize_velocities(d)
                    else:
                        raise Exception('Data type not defined')

                    # Save data
                    data.append(d)

        return file_names, np.array(data)


    def __len__(self):
        return self.length


    def __getitem__(self, idx):
        if self.preload:
            # Get data from stored arrays
            scan = self.scan_data[idx,:]
            sub_goal = self.sub_goal_data[idx,:]
            final_goal = self.final_goal_data[idx,:]
            vel = self.vel_data[idx,:]
        else:
            # Load data from files and normalize
            scan = self.normalize_scan(np.load(self.scan_file_names[idx]), self.normalization_method)
            sub_goal = self.normalize_sub_goal(np.load(self.sub_goal_file_names[idx]), self.normalization_method)
            final_goal = self.normalize_final_goal(np.load(self.final_goal_file_names[idx]), self.normalization_method)
            vel = self.normalize_velocities(np.load(self.vel_file_names[idx]))
            
        # Convert to pytorch tensor:
        return {
                'scan': torch.FloatTensor(scan),
                'sub_goal': torch.FloatTensor(sub_goal),
                'final_goal': torch.FloatTensor(final_goal),
                'velocity': torch.FloatTensor(vel), 
                }

#
# end of function


#------------------------------------------------------------------------------
#
# DNN model
#
#------------------------------------------------------------------------------

class DnnNet(nn.Module):

    # function: init
    #
    # arguments: input_size - int representing size of input
    #            hidden_size - number of nodes in the hidden layer
    #            num_classes - number of classes to classify
    #
    # return: none
    #
    # This method is the main function.
    #
    def __init__(self, in_channels: int, num_hiddens: int, num_scan_points: int = 360):

        # inherit the superclass properties/methods
        #
        super(DnnNet, self).__init__()
        # define the model
        #
         ################## CNN net model: ###################
        self.conv1 = nn.Sequential(nn.Conv1d(in_channels=in_channels,
                                            out_channels=num_hiddens//4,
                                            kernel_size=4,
                                            stride=2, 
                                            padding=1),
                                    nn.BatchNorm1d(num_hiddens//4),
                                    nn.ReLU(True),
                                    nn.AvgPool1d(kernel_size=4, 
                                            stride=2, 
                                            padding=1)
                                    )
        
        self.conv2 = nn.Sequential(nn.Conv1d(in_channels=num_hiddens//4,
                                            out_channels=num_hiddens//2,
                                            kernel_size=4,
                                            stride=2, 
                                            padding=1),
                                    nn.BatchNorm1d(num_hiddens//2),
                                    nn.ReLU(True),
                                    #nn.MaxPool1d(kernel_size=3, 
                                    #        stride=2, 
                                    #        padding=0)
                                    )
        
        self.conv3 = nn.Sequential(nn.Conv1d(in_channels=num_hiddens//2,
                                            out_channels=num_hiddens,
                                            kernel_size=3,
                                            stride=2, 
                                            padding=0),
                                    nn.BatchNorm1d(num_hiddens),
                                    nn.ReLU(True)
                                    )

        ################## full connect net model: ###################
        self.fc_net = nn.Sequential(
            nn.Linear(num_hiddens*22+2, 2),
            nn.Tanh()
            )

        self.num_scan_points = num_scan_points

        # initilization:
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.xavier_normal_(m.weight)
            elif isinstance(m, nn.Conv1d):
                nn.init.xavier_normal_(m.weight)
            elif isinstance(m, nn.BatchNorm1d):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)  
    #
    # end of function


    # function: forward
    #
    # arguments: data - the input to the model
    #
    # return: out - the output of the model
    #
    # This method feeds the data through the network
    #
    def forward(self, scan, goal):
        
        ###### Start of CNN net #######
        scan_in = scan.reshape(-1, 1, self.num_scan_points)
        # conv1:
        x = self.conv1(scan_in)
        # conv2: 
        x = self.conv2(x)
        # conv3: 
        x = self.conv3(x)
        scan_out = torch.flatten(x, 1)

        ###### Start of full connect net #######
        goal_in = goal.reshape(-1, 1, 2)
        goal_out = torch.flatten(goal_in, 1)

        # Combine
        fc_in = torch.cat((scan_out, goal_out), dim=1)
        fc_out = self.fc_net(fc_in)
        # return the output
        #
        return fc_out
    #
    # end of method
#
# end of class

#
# end of file
