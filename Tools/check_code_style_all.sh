#!/usr/bin/env bash
set -eu

if [[ "$@" == "--fix" ]]
then
    export PX4_ASTYLE_FIX=1
fi

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

HOOK_FILE="$DIR/../.git/hooks/pre-commit"
if [ ! -f $HOOK_FILE ] && [ "$CI" != "true" ]; then
	echo -e "\033[31mNinja tip: enable Git pre-commit hook to check for code style.\033[0m"
	echo ""
	echo ""
	echo " *******************************************************************************"
	echo " *   You do not seem to have a Git pre-commit hook installed that checks code"
	echo " *   style and whites-spaces on every commit you make."
	echo " *   Please allow me to install the pre-commit hook for you by entering 'y'."
	echo -e " *   (performs \033[94mcp ./Tools/pre-commit .git/hooks/pre-commit\033[0m)"
	echo " *   Otherwise just press enter."
	echo " *******************************************************************************"
	echo ""

	read user_cmd
	if [ "$user_cmd" == "y" ]; then
		cp $DIR/pre-commit $HOOK_FILE
		echo -e "\033[94mGreat, hook installed!\033[0m (checking style now)"
	else
		echo -e "\033[94mOk, I will remind you again later!\033[0m (checking style now)"
	fi
fi

${DIR}/files_to_check_code_style.sh | xargs -n 1 -P 8 -I % ${DIR}/check_code_style.sh %

if [ $? -eq 0 ]; then
    echo "Format checks passed"
    exit 0
fi
