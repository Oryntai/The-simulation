#!/bin/bash
set -eo pipefail
fixture_dir=$(cd -- "$(dirname -- "$0")" && pwd -P)
idf_dir=${ESP_PERIPHERALS_IDF_PATH:-$HOME/esp/esp-idf}
if [[ ! -r "$idf_dir/export.sh" ]]; then
    echo 'NotAnalyzed. Нужен установленный ESP-IDF v6.1.1.' >&2
    exit 2
fi

export PATH="/opt/homebrew/opt/python@3.13/libexec/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1
# ESP-IDF tools/cmake/project.cmake. Достаточно компонентов из локального SDK.
export IDF_COMPONENT_MANAGER=0
. "$idf_dir/export.sh" >&2
exec python "$fixture_dir/reproduce.py" "$@"
