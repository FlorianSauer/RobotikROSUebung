#!/usr/bin/env bash

SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"

cd $SCRIPTPATH/prediction/src/ && python setup.py build_ext --inplace; cd $SCRIPTPATH
cd $SCRIPTPATH/sound/src/ && python setup.py build_ext --inplace; cd $SCRIPTPATH
