#!/bin/sh

# define a base directory for the experiment
DL_EXP=`pwd`;

# scripts files
DL_SCRIPTS="$DL_EXP/src/pure_pursuit";

# output directory
DL_OUT="$DL_EXP/output";
if [ ! -d $DL_OUT ]; then
      mkdir $DL_OUT
fi

# define parameter file
DL_PARAM_FILE="$DL_EXP/params/hyperparameters.yaml";

# define the output directories for training/decoding/scoring
DL_TRAIN_ODIR="$DL_EXP/model";
if [ ! -d $DL_TRAIN_ODIR ]; then
      mkdir $DL_TRAIN_ODIR
fi

# define dataset directories
DL_DATA="$DL_EXP/turtlebot3_dataset";
DL_DATA_TRAIN="$DL_DATA/train";
DL_DATA_EVAL="$DL_DATA/dev";

# execute training: training must always be run
echo "... starting training on $DL_DATA_TRAIN ..."
$DL_SCRIPTS/dnn_train.py $DL_PARAM_FILE $DL_TRAIN_ODIR $DL_DATA_TRAIN $DL_DATA_EVAL | tee $DL_OUT/00_train.log | \
      grep "reading\|Step\|Average\|Warning\|Error" 
echo "... finished training on $DL_DATA_TRAIN ..."
