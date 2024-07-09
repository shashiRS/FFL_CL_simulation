#!/bin/bash

export current_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" || return ; pwd -P )
export venv_path=${current_path}/../venv

if [ ! -d ${venv_path} ]; then
    echo Creating python virtual environment... |& tee container.log
    python3 -m venv ${venv_path} |& tee container.log
    echo Installing VSP_pyBase dependencies... |& tee container.log
    source ${venv_path}/bin/activate |& tee container.log
    python -m pip install -r ${current_path}/../tests/VSP_PYBASE/requirements.txt |& tee container.log
    deactivate |& tee container.log
fi

source ${venv_path}/bin/activate |& tee container.log
python3 ${current_path}/../scripts/run_simulation.py $1 $2 $3 $4 $5 $6 $7 $8 $9 |& tee container.log
deactivate |& tee container.log
