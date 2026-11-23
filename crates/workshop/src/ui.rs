//! Интерфейс: слева — сценарии и параметры, справа — сверка с ручным расчётом, выводы,
//! график и эталонные проверки физики. Посередине — 3D.

use bevy::camera::Viewport;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::egui::{self, Color32, RichText};
use bevy_egui::{EguiContexts, egui::FontDefinitions};
use egui_plot::{Line, LineStyle, Plot, PlotPoints, Span};
use sim_scenarios::{FormItem, FormValue, FormWidget, Level, SeriesStyle, Status};

use crate::camera::SceneCamera;
use crate::{ReferenceChecks, Workshop};

const OK: Color32 = Color32::from_rgb(70, 200, 110);
const WARN: Color32 = Color32::from_rgb(240, 190, 50);
const BAD: Color32 = Color32::from_rgb(240, 80, 70);
const DIM: Color32 = Color32::from_rgb(150, 156, 168);
const INFO: Color32 = Color32::from_rgb(120, 170, 240);

fn c32(c: &[f32; 4]) -> Color32 {
    Color32::from_rgba_unmultiplied(
        (c[0] * 255.0) as u8,
        (c[1] * 255.0) as u8,
        (c[2] * 255.0) as u8,
        (c[3] * 255.0) as u8,
    )
}

fn install_fonts(ctx: &egui::Context) {
    // Встроенные шрифты egui покрывают кириллицу; системный Arial Unicode — запасной
    // для математических знаков (√, ≈, ↔, Σ, ⁴ …), если он есть на маке.
    let mut fonts = FontDefinitions::default();
    for path in [
        "/System/Library/Fonts/Supplemental/Arial Unicode.ttf",
        "/Library/Fonts/Arial Unicode.ttf",
    ] {
        if let Ok(bytes) = std::fs::read(path) {
            fonts.font_data.insert(
                "arial_unicode".into(),
                egui::FontData::from_owned(bytes).into(),
            );
            for fam in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
                fonts
                    .families
                    .entry(fam)
                    .or_default()
                    .push("arial_unicode".into());
            }
            break;
        }
    }
    ctx.set_fonts(fonts);
    ctx.all_styles_mut(|s| {
        s.spacing.item_spacing = egui::vec2(8.0, 5.0);
        s.spacing.slider_width = 170.0;
    });
}

fn fmt(v: f64, decimals: usize) -> String {
    if v.is_nan() {
        "—".into()
    } else {
        format!("{v:.decimals$}")
    }
}

fn status_chip(status: Status) -> RichText {
    match status {
        Status::Ok => RichText::new(" совпало ")
            .color(Color32::BLACK)
            .background_color(OK),
        Status::Explained => RichText::new(" расходится — причина известна ")
            .color(Color32::BLACK)
            .background_color(WARN),
        Status::Mismatch => RichText::new(" РАСХОДИТСЯ — верить нельзя ")
            .color(Color32::WHITE)
            .background_color(BAD),
        Status::Pending => RichText::new(" ещё рано сравнивать ")
            .color(Color32::BLACK)
            .background_color(DIM),
        Status::WithinLimit => RichText::new(" требование выполнено ")
            .color(Color32::BLACK)
            .background_color(OK),
        Status::OverLimit => RichText::new(" ТРЕБОВАНИЕ НАРУШЕНО ")
            .color(Color32::WHITE)
            .background_color(BAD),
        Status::NotAnalyzed => RichText::new(" НЕ ПОСЧИТАНО ")
            .color(Color32::BLACK)
            .background_color(WARN),
    }
    .strong()
}

fn card(ui: &mut egui::Ui, accent: Color32, add: impl FnOnce(&mut egui::Ui)) {
    egui::Frame::default()
        .fill(Color32::from_rgb(30, 33, 40))
        .stroke(egui::Stroke::new(1.0, accent.gamma_multiply(0.6)))
        .corner_radius(6.0)
        .inner_margin(8.0)
        .show(ui, |ui| {
            ui.set_width(ui.available_width());
            add(ui)
        });
}

pub fn ui_system(
    mut contexts: EguiContexts,
    mut ws: ResMut<Workshop>,
    checks: Res<ReferenceChecks>,
    mut fonts_ready: Local<bool>,
    mut camera: Query<&mut Camera, With<SceneCamera>>,
    window: Query<&Window, With<PrimaryWindow>>,
    offscreen: Option<Res<crate::camera::OffscreenTarget>>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    if !*fonts_ready {
        install_fonts(ctx);
        if offscreen.is_some() {
            ctx.set_zoom_factor(crate::camera::OFFSCREEN_UI_ZOOM);
        }
        *fonts_ready = true;
    }
    let mut root = egui::Ui::new(
        ctx.clone(),
        "workshop_root".into(),
        egui::UiBuilder::new()
            .layer_id(egui::LayerId::background())
            .max_rect(ctx.viewport_rect()),
    );

    let left = egui::Panel::left("left_panel")
        .resizable(true)
        .default_size(370.0)
        .show(&mut root, |ui| left_panel(ui, &mut ws))
        .response
        .rect
        .width();
    let right = egui::Panel::right("right_panel")
        .resizable(true)
        .default_size(470.0)
        .show(&mut root, |ui| right_panel(ui, &mut ws, &checks))
        .response
        .rect
        .width();

    // 3D-камере — область между панелями (в физических пикселях цели рендера).
    let ppp = ctx.pixels_per_point();
    let target = match &offscreen {
        Some(off) => Some(off.size),
        None => window
            .single()
            .ok()
            .map(|w| UVec2::new(w.physical_width(), w.physical_height())),
    };
    if let (Ok(mut cam), Some(size)) = (camera.single_mut(), target) {
        let (l, r) = ((left * ppp) as u32, (right * ppp) as u32);
        if size.x > l + r + 16 && size.y > 16 {
            cam.viewport = Some(Viewport {
                physical_position: UVec2::new(l, 0),
                physical_size: UVec2::new(size.x - l - r, size.y),
                ..default()
            });
        }
    }
    Ok(())
}

pub(crate) fn left_panel(ui: &mut egui::Ui, ws: &mut Workshop) {
    egui::ScrollArea::vertical()
        .id_salt("left_scroll")
        .show(ui, |ui| {
            ui.add_space(4.0);
            ui.label(RichText::new("Мастерская устройств").size(22.0).strong());
            ui.label(
                RichText::new("Сначала проверяем в симуляции — потом собираем в железе.")
                    .color(DIM),
            );
            ui.separator();

            ui.label(RichText::new("Сценарии").strong());
            let titles: Vec<String> = ws.scenarios.iter().map(|s| s.title()).collect();
            for (i, t) in titles.iter().enumerate() {
                if ui.selectable_label(ws.active == i, t.as_str()).clicked() {
                    ws.active = i;
                }
            }
            ui.separator();

            let sc = ws.current_mut();
            ui.label(RichText::new(sc.title()).size(17.0).strong());
            ui.label(sc.summary());
            ui.add_space(4.0);

            let presets = sc.presets();
            if !presets.is_empty() {
                ui.horizontal_wrapped(|ui| {
                    for (i, p) in presets.iter().enumerate() {
                        if ui.button(RichText::new(p).strong()).clicked() {
                            sc.apply_preset(i);
                        }
                    }
                });
            }
            for ch in sc.choices() {
                let mut sel = ch.selected;
                egui::ComboBox::from_id_salt(ch.id)
                    .width(260.0)
                    .selected_text(ch.options[sel].clone())
                    .show_ui(ui, |ui| {
                        for (i, o) in ch.options.iter().enumerate() {
                            ui.selectable_value(&mut sel, i, o.as_str());
                        }
                    });
                if sel != ch.selected {
                    sc.set_choice(ch.id, sel);
                }
            }
            ui.add_space(4.0);
            for p in sc.params() {
                let mut v = p.value;
                ui.label(&p.label);
                let r = ui.add(
                    egui::Slider::new(&mut v, p.min..=p.max)
                        .fixed_decimals(p.decimals)
                        .suffix(format!(" {}", p.unit)),
                );
                if r.changed() && (v - p.value).abs() > 0.0 {
                    sc.set_param(p.id, v);
                }
            }
            let actions = sc.actions();
            if !actions.is_empty() {
                ui.add_space(4.0);
                ui.horizontal_wrapped(|ui| {
                    for a in &actions {
                        let text = if a.active {
                            RichText::new(&a.label).strong().color(WARN)
                        } else {
                            RichText::new(&a.label)
                        };
                        if ui.button(text).on_hover_text(&a.hint).clicked() {
                            sc.run_action(a.id);
                        }
                    }
                });
            }
            form_panel(ui, ws);
            text_fields(ui, ws);
            ui.separator();

            ui.horizontal(|ui| {
                let label = if ws.paused {
                    "▶ Пуск"
                } else {
                    "⏸ Пауза"
                };
                if ui.button(label).clicked() {
                    ws.paused = !ws.paused;
                }
                if ui.button("⟲ Сначала").clicked() {
                    ws.current_mut().reset();
                }
                ui.label("Скорость:");
                for s in [0.25, 0.5, 1.0, 2.0] {
                    if ui
                        .selectable_label((ws.speed - s).abs() < 1e-9, format!("×{s}"))
                        .clicked()
                    {
                        ws.speed = s;
                    }
                }
            });
            ui.add_space(4.0);

            ui.label(RichText::new("Показания").strong());
            egui::Grid::new("readouts")
                .striped(true)
                .num_columns(2)
                .show(ui, |ui| {
                    for r in ws.current().readouts() {
                        ui.label(&r.label);
                        ui.label(
                            RichText::new(format!("{} {}", fmt(r.value, r.decimals), r.unit))
                                .monospace(),
                        );
                        ui.end_row();
                    }
                });
            for (pi, plot) in ws.current().plots().into_iter().enumerate() {
                ui.add_space(6.0);
                ui.label(RichText::new(&plot.title).size(17.0).strong());
                // Легенда — над графиком, чтобы не закрывать линии.
                ui.horizontal_wrapped(|ui| {
                    for s in &plot.series {
                        let mark = if s.style == SeriesStyle::Dashed {
                            "╌╌"
                        } else {
                            "━━"
                        };
                        ui.label(RichText::new(format!("{mark} {}", s.name)).color(c32(&s.color)));
                    }
                    for b in &plot.bands {
                        let mut c = b.color;
                        c[3] = 0.9;
                        ui.label(RichText::new(format!("▇ {}", b.label)).color(c32(&c)));
                    }
                });
                Plot::new(format!("plot_{}_{pi}", ws.current().id()))
                    .height(220.0)
                    .x_axis_label(plot.x_label.clone())
                    .y_axis_label(plot.y_label.clone())
                    .include_y(0.0)
                    .allow_scroll(false)
                    .show(ui, |pui| {
                        for b in &plot.bands {
                            pui.span(
                                Span::new("", b.lo..=b.hi)
                                    .axis(egui_plot::Axis::Y)
                                    .fill(c32(&b.color)),
                            );
                        }
                        for s in &plot.series {
                            let mut line =
                                Line::new(s.name.clone(), PlotPoints::from(s.points.clone()))
                                    .color(c32(&s.color))
                                    .width(2.0);
                            if s.style == SeriesStyle::Dashed {
                                line = line.style(LineStyle::dashed_loose());
                            }
                            pui.line(line);
                        }
                    });
            }

            let console = ws.current().console();
            if !console.is_empty() {
                ui.add_space(6.0);
                egui::CollapsingHeader::new(RichText::new("Журнал (UART, события)").strong())
                    .default_open(true)
                    .show(ui, |ui| {
                        egui::ScrollArea::vertical()
                            .id_salt("console_scroll")
                            .max_height(200.0)
                            .stick_to_bottom(true)
                            .show(ui, |ui| {
                                for line in &console {
                                    ui.label(RichText::new(line).monospace().size(12.0));
                                }
                            });
                    });
            }
            ui.separator();
            egui::CollapsingHeader::new("Управление камерой")
                .default_open(false)
                .show(ui, |ui| {
                    ui.label("Правая кнопка мыши (держать) — осмотреться");
                    ui.label("W A S D — идти, Q / E — вниз / вверх, Shift — быстрее");
                    ui.label(
                        "Колесо — скорость шага, F — взгляд на устройство, R — вид по умолчанию",
                    );
                });
        });
}

/// Редакторы текста сценария: черновик живёт в `Workshop::drafts`, в сценарий уходит по кнопке.
/// Форма сценария: сворачиваемые группы полей. Изменения применяются после отрисовки,
/// потому что от них может поменяться сам состав формы.
fn form_panel(ui: &mut egui::Ui, ws: &mut Workshop) {
    ws.current_mut().poll_background();
    let items = ws.current().form();
    if items.is_empty() {
        return;
    }
    let active = ws.active;
    let mut changes: Vec<(String, FormValue)> = Vec::new();
    let drafts = &mut ws.form_drafts;
    let mut i = 0;
    while i < items.len() {
        if items[i].widget != FormWidget::Heading {
            form_item(ui, &items[i], drafts, &mut changes);
            i += 1;
            continue;
        }
        let start = i + 1;
        let mut end = start;
        while end < items.len() && items[end].widget != FormWidget::Heading {
            end += 1;
        }
        egui::CollapsingHeader::new(RichText::new(&items[i].label).strong())
            .id_salt(("form_group", active, items[i].id.as_str()))
            .default_open(matches!(items[i].id.as_str(),
                "g.project" | "g.check" | "g.use" | "g.links" | "g.link"))
            .show(ui, |ui| {
                for it in &items[start..end] {
                    form_item(ui, it, drafts, &mut changes);
                }
            });
        i = end;
    }
    changes.sort_by_key(|(_, value)| matches!(value, FormValue::Pressed));
    for (id, value) in changes {
        ws.current_mut().set_form(&id, value);
    }
}

fn form_item(
    ui: &mut egui::Ui,
    it: &FormItem,
    drafts: &mut std::collections::HashMap<String, String>,
    changes: &mut Vec<(String, FormValue)>,
) {
    match &it.widget {
        FormWidget::Heading => {}
        FormWidget::Info(text) => {
            ui.horizontal_wrapped(|ui| {
                if !it.label.is_empty() {
                    ui.label(RichText::new(&it.label).color(DIM));
                }
                ui.label(text);
            });
        }
        FormWidget::Text(value) => {
            let draft = drafts.entry(it.id.clone()).or_insert_with(|| value.clone());
            ui.horizontal(|ui| {
                ui.label(&it.label);
                let r = ui.add(egui::TextEdit::singleline(draft).desired_width(190.0));
                if r.lost_focus() && *draft != *value {
                    changes.push((it.id.clone(), FormValue::Text(draft.clone())));
                } else if !r.has_focus() && *draft != *value {
                    *draft = value.clone();
                }
            });
        }
        FormWidget::LiveText(value) => {
            let draft = drafts.entry(it.id.clone()).or_insert_with(|| value.clone());
            let id = ui.make_persistent_id(("form_live", it.id.as_str()));
            if !ui.memory(|m| m.has_focus(id)) && *draft != *value {
                *draft = value.clone();
            }
            ui.horizontal(|ui| {
                ui.label(&it.label);
                let r = ui.add(egui::TextEdit::singleline(draft).id(id).desired_width(190.0));
                if r.changed() {
                    changes.push((it.id.clone(), FormValue::Text(draft.clone())));
                }
            });
        }
        FormWidget::Number { value, min, max, unit, decimals } => {
            ui.horizontal(|ui| {
                ui.label(&it.label);
                let mut v = *value;
                let r = ui.add(
                    egui::DragValue::new(&mut v)
                        .range(*min..=*max)
                        .fixed_decimals(*decimals)
                        .suffix(format!(" {unit}")),
                );
                if r.changed() && v != *value {
                    changes.push((it.id.clone(), FormValue::Number(v)));
                }
            });
        }
        FormWidget::Choice { options, selected } => {
            ui.horizontal(|ui| {
                ui.label(&it.label);
                let mut sel = *selected;
                let text = options.get(sel).cloned().unwrap_or_default();
                egui::ComboBox::from_id_salt(("form_choice", it.id.as_str()))
                    .width(190.0)
                    .selected_text(text)
                    .show_ui(ui, |ui| {
                        for (k, o) in options.iter().enumerate() {
                            ui.selectable_value(&mut sel, k, o.as_str());
                        }
                    });
                if sel != *selected {
                    changes.push((it.id.clone(), FormValue::Choice(sel)));
                }
            });
        }
        FormWidget::Toggle(on) => {
            let mut v = *on;
            if ui.checkbox(&mut v, &it.label).changed() {
                changes.push((it.id.clone(), FormValue::Toggle(v)));
            }
        }
        FormWidget::Button { hint } => {
            if ui.button(&it.label).on_hover_text(hint).clicked() {
                changes.push((it.id.clone(), FormValue::Pressed));
            }
        }
    }
}

fn text_fields(ui: &mut egui::Ui, ws: &mut Workshop) {
    let active = ws.active;
    let fields = ws.current().text_fields();
    for f in fields {
        ui.add_space(6.0);
        ui.label(RichText::new(&f.label).strong());
        let key = (active, f.id);
        let draft = ws.drafts.entry(key).or_insert_with(|| f.text.clone());
        egui::ScrollArea::vertical()
            .id_salt(("text_field", f.id))
            .max_height(f.rows as f32 * 16.0)
            .show(ui, |ui| {
                ui.add(
                    egui::TextEdit::multiline(draft)
                        .font(egui::TextStyle::Monospace)
                        .desired_rows(f.rows)
                        .desired_width(f32::INFINITY),
                );
            });
        let changed = *draft != f.text;
        let mut apply = None;
        let mut revert = false;
        ui.horizontal(|ui| {
            let label = if changed {
                "▶ Применить"
            } else {
                "Применено"
            };
            if ui.add_enabled(changed, egui::Button::new(label)).clicked() {
                apply = Some(draft.clone());
            }
            if changed && ui.button("Отменить правку").clicked() {
                revert = true;
            }
        });
        if revert {
            ws.drafts.insert(key, f.text.clone());
        }
        if let Some(text) = apply {
            ws.current_mut().set_text(f.id, text);
            // Сценарий мог нормализовать текст — берём его версию.
            if let Some(nf) = ws
                .current()
                .text_fields()
                .into_iter()
                .find(|x| x.id == f.id)
            {
                ws.drafts.insert(key, nf.text);
            }
        }
    }
}

fn right_panel(ui: &mut egui::Ui, ws: &mut Workshop, checks: &ReferenceChecks) {
    egui::ScrollArea::vertical().id_salt("right_scroll").show(ui, |ui| {
        let sc = ws.current();
        // Сначала — опасности и выводы (по убыванию серьёзности), потом — сверки.
        let mut notes = sc.notes();
        notes.sort_by_key(|n| match n.level {
            Level::Danger => 0,
            Level::Warn => 1,
            Level::Good => 2,
            Level::Info => 3,
        });
        if !notes.is_empty() {
            ui.add_space(4.0);
            ui.label(RichText::new("Выводы и опасности").size(17.0).strong());
            for n in notes {
                let col = match n.level {
                    Level::Info => INFO,
                    Level::Good => OK,
                    Level::Warn => WARN,
                    Level::Danger => BAD,
                };
                card(ui, col, |ui| {
                    ui.label(RichText::new(&n.text).color(if n.level == Level::Info { Color32::from_gray(210) } else { col }));
                });
                ui.add_space(2.0);
            }
        }
        ui.add_space(6.0);
        ui.add_space(4.0);
        ui.label(RichText::new("Сверка: симуляция ↔ ручной расчёт").size(17.0).strong());
        ui.label(RichText::new("Расхождение не прячется: оба числа, разница и её причина.").color(DIM));
        ui.add_space(4.0);
        for (index, c) in sc.comparisons().into_iter().enumerate() {
            let accent = match c.status {
                Status::Ok | Status::WithinLimit => OK,
                Status::Explained | Status::NotAnalyzed => WARN,
                Status::Mismatch | Status::OverLimit => BAD,
                Status::Pending => DIM,
            };
            card(ui, accent, |ui| {
                ui.horizontal_wrapped(|ui| {
                    ui.label(RichText::new(&c.title).strong());
                    ui.label(status_chip(c.status));
                });
                // У проверки без числа (данных нет или сверяется факт) нули ничего не значат.
                let no_numbers = c.sim == 0.0
                    && c.reference == 0.0
                    && matches!(c.status, Status::NotAnalyzed | Status::WithinLimit | Status::OverLimit);
                if !no_numbers {
                    egui::Grid::new(("cmp", index)).num_columns(2).show(ui, |ui| {
                        ui.label(RichText::new(&c.sim_label).color(DIM));
                        ui.label(RichText::new(format!("{} {}", fmt(c.sim, c.decimals), c.unit)).monospace().size(15.0).strong());
                        ui.end_row();
                        ui.label(RichText::new(&c.ref_label).color(DIM));
                        ui.label(RichText::new(format!("{} {}", fmt(c.reference, c.decimals), c.unit)).monospace().size(15.0));
                        ui.end_row();
                        if !c.sim.is_nan() {
                            ui.label(RichText::new("разница").color(DIM));
                            let d = c.sim - c.reference;
                            let rel = if c.reference.abs() > 0.0 { format!("  ({:+.3}%)", 100.0 * c.rel_diff()) } else { String::new() };
                            ui.label(RichText::new(format!("{:+.decimals$} {}{rel}", d, c.unit, decimals = c.decimals)).monospace());
                            ui.end_row();
                        }
                    });
                }
                if !c.explanation.is_empty() {
                    ui.label(RichText::new(&c.explanation).italics());
                }
            });
            ui.add_space(3.0);
        }

        ui.add_space(8.0);
        ui.separator();
        let guard = checks.0.lock().unwrap();
        match guard.as_ref() {
            None => {
                ui.label(RichText::new("Эталонные проверки физики: считаются…").color(DIM));
            }
            Some(groups) => {
                let total: usize = groups.iter().map(|g| g.checks.len()).sum();
                let passed: usize = groups
                    .iter()
                    .map(|g| g.checks.iter().filter(|c| c.passed()).count())
                    .sum();
                let head = format!("Эталонные проверки физики: {passed} из {total} пройдено");
                let col = if passed == total { OK } else { BAD };
                egui::CollapsingHeader::new(RichText::new(head).strong().color(col)).default_open(false).show(ui, |ui| {
                    ui.label(RichText::new("Те же проверки, что в `cargo test` и `cargo verify`: наша модель против второго независимого способа.").color(DIM));
                    for g in groups {
                        let ok = g.checks.iter().filter(|c| c.passed()).count();
                        let gc = if ok == g.checks.len() { OK } else { BAD };
                        egui::CollapsingHeader::new(RichText::new(format!("{} — {ok} из {}", g.name, g.checks.len())).color(gc))
                            .id_salt(g.name)
                            .default_open(false)
                            .show(ui, |ui| {
                                for c in &g.checks {
                                    let (mark, mc) = if c.passed() { ("✓", OK) } else { ("✗", BAD) };
                                    ui.horizontal_wrapped(|ui| {
                                        ui.label(RichText::new(mark).color(mc).strong());
                                        ui.label(&c.title);
                                    });
                                    ui.label(
                                        RichText::new(format!("   ошибка {}  (допуск {}) — {}", c.error_text(), c.tolerance_text(), c.reference_method))
                                            .color(DIM)
                                            .small(),
                                    );
                                }
                            });
                    }
                });
            }
        }
    });
}
