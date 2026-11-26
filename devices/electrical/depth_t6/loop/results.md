# Т6.5. Численная сверка трёх локальных стендов

Команды выполняются из корня рабочей копии.

```sh
cargo test --locked --offline --manifest-path devices/electrical/depth_t6/loop/Cargo.toml \
  --target-dir target -- --include-ignored --test-threads=1 --nocapture
assets/third_party/ngspice/bin/ngspice -b devices/electrical/depth_t6/loop/rl.cir
```

Стенд использует `OPENEMS` или `~/opt/openEMS/bin/openEMS`. Лимит одного процесса в
тесте 60 с. Вызов и временный каталог принадлежат адаптеру 6.4. Стенд хранит каждый
точный входной XML, версии openEMS/CSXCAD, число шагов, размер шага и комплексные
отсчёты. Числа ниже относятся к openEMS 0.37 (`305df56`) и CSXCAD `c46e320`.

| Стенд | Полоса и точки, МГц | Порты и нагрузки | Сигналы |
|---|---|---|---|
| Петля | 100, 150, 200, 250, 300 | Два зазора по 25 Ом, противоположные стороны прямоугольника 24 × 16 мм | Ток источника и напряжение зазора общего возврата |
| Две связанные полоски | 200, 300, 400, 500, 600 | Четыре окончания по 35 Ом, `x=0` и `x=36 мм` к общей плоскости | Напряжения всех четырёх портов |
| Синфазный кабель | 200, 300, 400, 500, 600 | Два окончания по 100 Ом, `x=0` и `x=40 мм` к общей плоскости | Токи обоих портов, дальний со знаком по фактической нормали |

Портовые зонды проверяются по фактическим индексам Yee. Для источника
`E = U + Rₛ I`, где положительный ток от нижней к верхней координате внутри
зазора. На пассивном окончании проверяется комплексное `U = −RI`. Напряжения
нормированы в В/В, токи в А/В. Сравнение использует максимум
`|20 log₁₀(|Xполе|/|Xаналитика|)|` по всем частотам и наблюдаемым портам.

Отдельно проверяются разность амплитуд на двух вложенных сетках до 1 дБ,
невязка комплексного закона Ома до 10% и отступ от изменения спектра при
продлении хвоста минимум 20 дБ. Порог численного сопоставления 3 дБ включительно.
Отдельно проверяются покрытие полосы возбуждением и достаточность данных адаптера. На
первой паре сеток изменение наводки связанных полосок было 1,505 дБ, поэтому
постановка получила `NotAnalyzed`. Рабочая серия уточняет сетку вокруг полосок
при тех же проводниках, окончаниях и полосе сравнения. Для обоих рабочих
расчётов выбран общий импульс, при котором нет предупреждения openEMS о
числе шагов на отсчёт Найквиста.

| Стенд | Сравнений | Максимум аналитика ↔ поле, дБ | Сдвиг сетки, дБ | Невязка порта | Минимальный запас над хвостом, дБ | Шаги: базовая / уточнённая / продлённая |
|---|---:|---:|---:|---:|---:|---:|
| Петля | 10 | 0,784233 | 0,247103 | 0,313944% | 60,525951 | 5109 / 10244 / 13396 |
| Две связанные полоски | 20 | 1,068232 | 0,536213 | 0,695490% | 63,840680 | 6951 / 17440 / 27032 |
| Синфазный кабель | 10 | 0,279685 | 0,146532 | 2,079141% | 93,560751 | 2460 / 5264 / 7567 |

Ручная модель петли использует внешнюю индуктивность PEC по интегралу Неймана.
Замкнутая формула даёт 35,738699289 нГн, независимая квадратура даёт
35,738699160 нГн. Для пяти частот ngspice 47 с квадратурной индуктивностью
расходится с комплексным делителем не более чем на 8,975·10⁻¹⁰ В/В.
Для кабеля дополнительная сверка складывает падающую и отражённую волны
при `Z₀ = 123,72057415 Ом`.

Область данных — идеальная PEC-геометрия в вакууме. Максимальная электрическая
длина петли 0,080 λ по периметру, полосок 0,072 λ, кабеля 0,080 λ по продольной
оси. Точки между частотами, моды вне полосы, характеристики реального
материала, монтаж и физические шумы требуют стендового ряда. Статус ЭМС
`NotAnalyzed` запрашивает замер на тех же портах и в той же полосе.

Формулы и привязка отсчёта описаны в источниках. [Rosa–Cohen, NBS 5(1)](https://nvlpubs.nist.gov/nistpubs/bulletin/05/nbsbulletinv5n1p1_A2b.pdf),
[Kirschning–Jansen](https://doi.org/10.1109/TMTT.1984.1132616),
[MIT 6.013, §14.5](https://web.mit.edu/6.013_book/www/chapter14/14.5.html),
[openEMS, сетка Yee](https://github.com/thliebig/openEMS/blob/305df56/FDTD/operator.cpp),
[openEMS, токовый зонд](https://github.com/thliebig/openEMS/blob/305df56/Common/processcurrent.cpp).

## Итоговые строки тестов

Стенд Т6.5

```text
test result: ok. 6 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 64.09s
test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
```

`cargo test --offline -p sim-circuit -- --include-ignored`

```text
test result: ok. 24 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
test result: ok. 3 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
test result: ok. 12 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 3.41s
test result: ok. 7 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.01s
test result: ok. 6 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.01s
test result: ok. 9 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.05s
test result: ok. 3 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.66s
test result: ok. 5 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.87s
test result: ok. 4 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.01s
test result: ok. 11 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.01s
test result: ok. 8 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
test result: ok. 9 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
test result: ok. 11 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
test result: ok. 5 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 2.80s
test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
```

`cargo test --offline -p sim-verify --test platform_boundary`

```text
test result: ok. 4 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.17s
```
