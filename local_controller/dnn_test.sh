#!/bin/sh

# define a base directory for the experiment
DL_EXP=`pwd`;
DL_SCRIPTS="$DL_EXP/src/dnn_controller";
DL_OUT="$DL_EXP/output";

# define parameter file
DL_PARAM_FILE="$DL_EXP/params/hyperparameters.yaml";

# define the output directories for training/decoding/scoring
DL_TRAIN_ODIR="$DL_EXP/model";
DL_MDL_PATH="$DL_TRAIN_ODIR/model_final.pth";

# define dataset directories
DL_DATA="$DL_EXP/turtlebot3_dataset";
DL_DATA_TEST="$DL_DATA/test";

# evaluate each data set that was specified
#
echo "... starting evaluation of $DL_DATA_EVAL ..."
$DL_SCRIPTS/dnn_test.py $DL_OUT $DL_PARAM_FILE $DL_MDL_PATH $DL_DATA_TEST | \
    tee $DL_OUT/01_decode_train.log | grep "Average"
echo "... finished evaluation of $DL_DATA_EVAL ..."


echo "======= end of results ======="

#
# exit gracefully
