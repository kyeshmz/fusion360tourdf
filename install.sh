#!/usr/bin/env bash

script_dir=${BASH_SOURCE[0]%/*}
if [[ "$script_dir" == "${BASH_SOURCE[0]}" ]]; then
    script_dir=.
fi
root=$(CDPATH= cd -- "$script_dir" && pwd) || exit 1
backend="$root/tools/install_fusion.py"
if [[ ! -f "$backend" ]]; then
    printf '%s\n' 'Install failed: missing tools/install_fusion.py. Use a complete checkout.' >&2
    exit 1
fi

export PYTHON_MANAGER_AUTOMATIC_INSTALL=false
unset PYLAUNCHER_ALLOW_INSTALL PYLAUNCHER_ALWAYS_INSTALL

for python in python3 python; do
    executable=$(type -P "$python") || continue
    if "$executable" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 11) else 1)' >/dev/null 2>&1; then
        exec "$executable" "$backend" "$@"
    fi
done

printf '%s\n' 'Install failed: Python 3.11 or newer is required. Install Python and put python3 or python on PATH.' >&2
exit 1
