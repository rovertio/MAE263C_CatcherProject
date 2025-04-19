#!/bin/zsh

if [[ -d "venv" ]]; then
    source venv/bin/activate;
else
    python3 -m venv venv;
    source venv/bin/activate;
    pip install --upgrade pip setuptools;
    pip install pyyaml
    pip install ./mechae263C_helpers --no-warn-script-location;
    pip install ./dynamixel-controller --no-warn-script-location;
fi
clear;



