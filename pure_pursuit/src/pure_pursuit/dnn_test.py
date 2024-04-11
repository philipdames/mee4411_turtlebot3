#!/usr/bin/env python3

# import pytorch modules
import torch
from tqdm import tqdm

# import the model and all of its variables/functions
from dnn_model import loadDNNParams, NavDataset, DnnNet
import numpy as np

# import modules
import sys
import os
from sklearn.metrics import explained_variance_score, mean_squared_error

#-----------------------------------------------------------------------------
#
# global variables are listed here
#
#-----------------------------------------------------------------------------
# parameters: do not modify
NUM_ARGS = 4

#------------------------------------------------------------------------------
#
# the main program starts here
#
#------------------------------------------------------------------------------
def main(argv):
    # ensure we have the correct amount of arguments
    if (len(argv) != NUM_ARGS):
        print("usage: python decode_demo_rmse_ev.py [PARAM_FILE] [MDL_PATH] [DEV_PATH]")
        exit(-1)

    # define local variables:
    odir = argv[0]
    param_file = argv[1]
    mdl_path = argv[2]
    pTest = argv[3]

    # if the odir doesn't exits, we make it
    if not os.path.exists(odir):
        os.makedirs(odir)

    # load DNN training params
    dnn_params = loadDNNParams(param_file)

    # set the device to use GPU if available:
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # get array of the data
    head_tail = os.path.split(pTest)
    file_name = head_tail[-1]
    eval_dataset = NavDataset(pTest, file_name, \
                              normalization_method=dnn_params['normalization_method'], \
                              preload=dnn_params['preload_all_data'])
    eval_dataloader = torch.utils.data.DataLoader(eval_dataset, batch_size=1, \
                                                   shuffle=False, drop_last=True) #, pin_memory=True)

    # instantiate a model:
    model = DnnNet(in_channels=1, 
                   num_hiddens=dnn_params['num_channels'])
    
    # moves the model to device (cpu in our case so no change):
    model.to(device)

    # set the model to evaluate
    model.eval()

    # load the weights
    checkpoint = torch.load(mdl_path, map_location=device)
    model.load_state_dict(checkpoint['model'])

    # for each batch in increments of batch size:
    running_rmse = 0
    running_ev = 0
    
    # get the number of batches (ceiling of train_data/batch_size):
    num_batches = int(len(eval_dataset)/eval_dataloader.batch_size)
    outputs = []
    targets = []
    with torch.no_grad():
        for batch in tqdm(eval_dataloader, total=num_batches):
            # collect the samples as a batch:
            scans = batch['scan'].to(device)
            goals = batch[dnn_params['goal_input']].to(device)
            velocities = batch['velocity'].to(device)
            
            # feed the network the batch
            output = model(scans, goals)

            # get the loss
            outputs.append(output.squeeze().data.cpu().numpy())
            targets.append(velocities.squeeze().data.cpu().numpy())

        running_ev = explained_variance_score(targets, outputs)
        running_rmse = np.sqrt(mean_squared_error(targets, outputs))
          
    val_rmse = running_rmse
    val_ev = running_ev

    # write wmse into np.array:
    val_res = np.array([val_rmse])
    val_res_odir = odir + '/rmse'
    if not os.path.exists(val_res_odir):
        os.makedirs(val_res_odir)
    val_res_name = val_res_odir + '/rmse'
    np.save(val_res_name, val_res)

    # write ssim into np.array:
    val_res = np.array([val_ev])
    val_res_odir = odir + '/ev'
    if not os.path.exists(val_res_odir):
        os.makedirs(val_res_odir)
    val_res_name = val_res_odir  + '/ev'
    np.save(val_res_name, val_res)

    # loss:
    print('Average RMSE: {:.4f}, Average EV: {:.4f}'.format(val_rmse, val_ev))
        
    # exit gracefully
    return True
# end of function


# begin gracefully
if __name__ == '__main__':
    main(sys.argv[1:])
#
# end of file