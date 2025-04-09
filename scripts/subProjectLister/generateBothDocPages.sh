#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

export SOFA_SRC=${SCRIPT_DIR}/../../

cp inTree-plugins.md 50_Activate_Plugins.md
python3 exportInTreePlugins.py >> 50_Activate_Plugins.md

cp supported-plugins.md 45_Suported_Plugins_List.md
python3 exportSupportedPlugins.py >> 45_Suuported_Plugins_List.md