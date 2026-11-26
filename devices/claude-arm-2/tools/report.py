"""Разбор JSON-отчёта задания «налив»: исход, нарушения, вода, где встала бутылка.

Запуск: [LAYOUT=b] python3 tools/report.py отчёт.json [журнал контроллера]
LAYOUT — раскладка стола из make_arm.LAYOUTS (по умолчанию основная).
"""
import json, math, os, re, sys

from make_arm import LAYOUTS

BOTTLE_AT, MUG_AT, _ = LAYOUTS[os.environ.get("LAYOUT", "")]


def main():
    d = json.load(open(sys.argv[1]))
    ev = d.get("evidence", {})
    print("статус:", d.get("status"), "| контур:", d.get("closed_loop"), "| исход:", ev.get("outcome"),
          "| покрытие:", json.dumps(d.get("coverage"), ensure_ascii=False))
    cases = ev.get("cases") or [{}]
    executed = [c for c in cases if c.get("metrics")]
    print(f"выполнено случаев: {len(executed)} из {len(cases)}")
    case = executed[0] if executed else cases[0]
    print("случай:", json.dumps(case.get("case"), ensure_ascii=False), "| исход:", case.get("outcome"))
    print(f"дошёл до {(case.get('reached_ns') or 0) / 1e9:.2f} с, завершён: {case.get('completed')}")
    if case.get("runtime_error"):
        print("ошибка прогона:", case["runtime_error"])
    for item in case.get("violations") or []:
        print("нарушение:", item)
    missing = [m for m in case.get("missing") or [] if not m.startswith("ограничение модели")]
    if missing:
        print(f"недоказано ({len(case.get('missing') or [])}, без ограничений моделей):")
        for m in missing[:12]:
            print("  -", m)
    metrics = {m["signal"]["name"]: m for m in case.get("metrics") or []}
    last = lambda n: metrics[n]["last"] if n in metrics else None
    # Что увидела камера — из журнала контроллера (строка «вижу:»): приёмка судит только истину.
    if len(sys.argv) > 2:
        seen = [l for l in open(sys.argv[2]) if "вижу:" in l]
        for line in seen[:1]:
            nums = [float(v) for v in re.findall(r"\((-?[\d.]+), (-?[\d.]+)\)", line) for v in v]
            for (what, truth), (sx, sy) in zip((("бутылка", BOTTLE_AT), ("кружка", MUG_AT)), (nums[0:2], nums[2:4])):
                print(f"камера: {what} в ({sx:.1f}, {sy:.1f}) мм, на деле ({truth[0] * 1000:.1f}, {truth[1] * 1000:.1f}): "
                      f"ошибка {math.hypot(sx - truth[0] * 1000, sy - truth[1] * 1000):.2f} мм")
    for name in ("вода/кружка", "вода/мимо", "вода/цель"):
        m = metrics.get(name)
        if m:
            print(f"{name}: последнее {m['last']:.4f}, наибольшее {m['max']:.4f} в {m['max_at_ns'] / 1e9:.2f} с")
    for name, m in metrics.items():
        if name.endswith("winding_c"):
            print(f"{name}: до {m['max']:.1f} °C")
    bx, by, bz = (last(f"бутылка {a}") for a in "xyz")
    if bx is not None and by is not None:
        print(f"бутылка в конце: ({bx * 1000:.1f}, {by * 1000:.1f}) мм, исходно ({BOTTLE_AT[0] * 1000:.0f}, {BOTTLE_AT[1] * 1000:.0f}), "
              f"сдвиг {math.hypot(bx - BOTTLE_AT[0], by - BOTTLE_AT[1]) * 1000:.1f} мм; z от {metrics['бутылка z']['min'] * 1000:.2f} до "
              f"{metrics['бутылка z']['max'] * 1000:.1f} мм")
    if last("бутылка ось z") is not None and bz is not None:
        print(f"бутылка в конце наклонена на {math.degrees(math.acos(max(-1.0, min(1.0, last('бутылка ось z') - bz)))):.2f}°")
    mx, my = last("кружка x"), last("кружка y")
    if mx is not None:
        print(f"кружка сдвинута на {math.hypot(mx - MUG_AT[0], my - MUG_AT[1]) * 1000:.2f} мм")
    for name, bounds in (case.get("process_metrics") or {}).items():
        print(f"свидетель «{name}»: {bounds}")
    if len(sys.argv) > 2:
        print("--- контроллер:")
        print("".join(l for l in open(sys.argv[2]) if l.startswith("контроллер"))[-3000:])


if __name__ == "__main__":
    main()
