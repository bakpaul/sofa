echo "Generate stubfiles..."

if [ "$#" -ge 2 ]; then
    SRC_DIR="$(cd $1 && pwd)"
    INSTALL_DIR="$(cd $2 && pwd)"
    if [ "$#" -eq 2 ]; then
        SYSTEM_NAME=0
    else
        SYSTEM_NAME=$3
    fi

  echo "Inputs are"
  echo "- SRC_DIR       :${SRC_DIR}"
  echo "- INSTALL_DIR   :${INSTALL_DIR}"
  echo "- SYSTEM_NAME   :${SYSTEM_NAME}"
else
    echo "Usage: generate_stubfiles <SRC_DIR> <INSTALL_DIR> [SYSTEM_NAME = 0]"; exit 1
fi

if [ -e "$VM_PYTHON3_EXECUTABLE" ]; then
    if [ "$SYSTEM_NAME" = "Windows" ]; then
        pythonroot="$(dirname $VM_PYTHON3_EXECUTABLE)"
        pythonroot="$(cd "$pythonroot" && pwd)"
        export PATH="$pythonroot:$pythonroot/DLLs:$pythonroot/Lib:$PATH"
        PYTHON_SCRIPT=$(cd "$SRC_DIR/applications/plugins/SofaPython3/scripts" && pwd -W )\\generate_stubs.py
        PYTHON_INSTALL_SITE_PACKAGE_DIR=$(cd "$INSTALL_DIR/plugins/SofaPython3/lib/python3/site-packages" && pwd -W )
        echo "PATH=$PATH"
    else
        PYTHON_SCRIPT=$(cd "$SRC_DIR/applications/plugins/SofaPython3/scripts" && pwd )/generate_stubs.py
        PYTHON_INSTALL_SITE_PACKAGE_DIR=$(cd "$INSTALL_DIR/plugins/SofaPython3/lib/python3/site-packages" && pwd )
    fi

    export SOFA_ROOT="$INSTALL_DIR"
    export PYTHONPATH="$PYTHON_INSTALL_SITE_PACKAGE_DIR:$PYTHONPATH"

    echo "SOFA_ROOT=$SOFA_ROOT"
    echo "PYTHONPATH=$PYTHONPATH"

    #Create folder if not already created

    python_exe="$VM_PYTHON3_EXECUTABLE"
    if [ -n "$python_exe" ]; then
        echo "Launching the stub generation with '$python_exe ${PYTHON_SCRIPT} -d $PYTHON_INSTALL_SITE_PACKAGE_DIR -m Sofa --use_pybind11'"
        $python_exe "${PYTHON_SCRIPT}" -d "$PYTHON_INSTALL_SITE_PACKAGE_DIR" -m Sofa --use_pybind11
    fi
else
    echo "VM_PYTHON3_EXECUTABLE doe not point to an existing file. To generate stubfiles you should point this env var to the Python3.XX executable."
fi
echo "Generate stubfiles: done."