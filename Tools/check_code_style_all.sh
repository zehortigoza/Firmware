#!/usr/bin/env bash
set -eu

if [[ "$@" == "--fix" ]]
then
    export PX4_ASTYLE_FIX=1
fi

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

${DIR}/files_to_check_code_style.sh | xargs -n 1 -P 8 -I % ${DIR}/check_code_style.sh %

if [ $? -eq 0 ]; then
    echo "Format checks passed"
    exit 0
fi
