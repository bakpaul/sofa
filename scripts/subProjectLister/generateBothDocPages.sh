#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SOFA_DIR=${SCRIPT_DIR}/../../


cp ${SCRIPT_DIR}/inTree-plugins.md ${SCRIPT_DIR}/50_Activate_Plugins.md
python3 ${SCRIPT_DIR}/exportInTreePlugins.py "$SOFA_DIR" >> ${SCRIPT_DIR}/50_Activate_Plugins.md

cp ${SCRIPT_DIR}/supported-plugins.md ${SCRIPT_DIR}/45_Suported_Plugins_List.md
python3 ${SCRIPT_DIR}/exportSupportedPlugins.py "$SOFA_DIR" >> ${SCRIPT_DIR}/45_Suported_Plugins_List.md