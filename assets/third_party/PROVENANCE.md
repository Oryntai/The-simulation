# Сторонние данные и библиотеки — происхождение

Скачано 21.11.2026 с разрешения владельца. Внутри этих папок — чужие файлы как есть;
наши правки перечислены ниже. Ничего отсюда не копируется в код `crates/`.

| Папка | Что | Откуда | Версия / коммит | Лицензия | Наши изменения |
|---|---|---|---|---|---|
| `mujoco/` | MuJoCo — физический движок, `mujoco.framework` (macOS, arm64 + x86_64), LICENSE, THIRD_PARTY_NOTICES | https://github.com/google-deepmind/mujoco/releases/tag/4.0.1, файл `mujoco-4.0.1-macos-universal2.dmg` | 4.0.1; sha256 образа `2616bb30cf48e5603e09f42ef7898e1302c2f99cdaaea650714f0fcfa3b57722` — совпал с официальным; подпись «Developer ID Application: Google LLC (EQHXZ8M8AV)» валидна | Apache-2.0 | из образа взяты только framework и файлы лицензий |
| `ms_human_700/` | MS-Human-700 — модель всего тела: 80 тел, 85 степеней свободы, 700 мышц (MJCF + меши) | https://github.com/LNSGroup/MS-Human-700 | коммит `2d686957aefd5739cf4d2859a6acfd94c8f84400` (28.02.2026) | Apache-2.0 | удалены `.git` и `Pictures/` (демо-гифки) |
| `myo_sim/` | MyoSuite myo_sim — MyoArm (63 мышцы руки), MyoLeg, туловище, кисть (MJCF-фрагменты + меши) | https://github.com/MyoHub/myo_sim | коммит `5cd36ee4d823bced6cbd7c02424458ed0fefb2b2` (10.11.2026) | Apache-2.0; **данные руки — из MoBL (Stanford), только некоммерческое использование** | удалены `.git`, `CLAUDE.md` и `AGENTS.md` (инструкции для ИИ-агентов разработчиков myo_sim — не для нас) |

Исходный образ MuJoCo (`mujoco-4.0.1-macos-universal2.dmg`, 58 МБ) после распаковки удалён;
скачать заново можно по ссылке выше и проверить по контрольной сумме.

Ссылки на статьи (цитировать при публикации):
* MS-Human-700: Zuo и др., ICRA 2024, https://arxiv.org/abs/2312.05473
* MyoArm: модель MoBL — Saul и др., «Benchmarking of dynamic simulation predictions in
  two software platforms using an upper limb musculoskeletal model»; Holzbaur и др., 2005,
  https://doi.org/10.1007/s10439-005-3320-7
* MuJoCo: Todorov и др., IROS 2012

## ngspice 47 (внешняя программа для сверки схем)

* Источник: https://sourceforge.net/projects/ngspice/files/ng-spice-rework/47/ngspice-47.tar.gz,
  скачано 21.11.2026, sha256 `894e649651f1838a14095e5a5439e7d3aa63e87ede14d283173fda4fcdef675f`.
* Собрано из исходников на этой машине: `configure --without-x --with-readline=no --disable-debug
  --disable-openmp`, Apple clang. Здесь лежит только бинарь `bin/ngspice` (зависит лишь от
  системных libSystem/libc++) и наш минимальный `share/ngspice/scripts/spinit` без кодовых
  моделей XSPICE/OSDI (они нам не нужны; модуль `table.cm` под GPLv2 сознательно не берём).
* Лицензия: Modified BSD для основной части (см. `ngspice/COPYING`; исключения — LGPL у KLU и
  numparam, GPLv2 у xspice/icm/table). Используется только как внешняя программа: запускаем
  `ngspice -b` на тех же нетлистах и сравниваем числа с нашим решателем `sim-circuit`.
  Код ngspice в наш код не переносится.
