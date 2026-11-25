#!/bin/bash
set -eo pipefail
if [[ -z ${SIM_ESP_SDK_TMPDIR:-} ]]; then
    echo 'Задайте SIM_ESP_SDK_TMPDIR существующим каталогом вне репозитория.' >&2
    exit 2
fi
fixture_dir=$(cd -- "$(dirname -- "$0")" && pwd -P)
idf_dir=${SIM_ESP_IDF_PATH:-$HOME/esp/esp-idf}
if [[ ! -r "$idf_dir/export.sh" ]]; then
    echo 'NotAnalyzed: нужен установленный ESP-IDF v6.1.1 (SIM_ESP_IDF_PATH).' >&2
    exit 2
fi
export PATH="/opt/homebrew/opt/python@3.13/libexec/bin:$PATH"
export PYTHONDONTWRITEBYTECODE=1
# ESP-IDF tools/cmake/project.cmake: примеры собираются из установленных компонентов.
export IDF_COMPONENT_MANAGER=0
. "$idf_dir/export.sh" >&2
exec python "$fixture_dir/capture.py" "$@"
