#!/usr/bin/env bash

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

cd $SCRIPTPATH/camera_pseudo/src/ && python setup.py build_ext --inplace; cd $SCRIPTPATH
cd $SCRIPTPATH/exercise_1/src/ && python setup.py build_ext --inplace; cd $SCRIPTPATH
cd $SCRIPTPATH/prediction/src/ && python setup.py build_ext --inplace; cd $SCRIPTPATH
