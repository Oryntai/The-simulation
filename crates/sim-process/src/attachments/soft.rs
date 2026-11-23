use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;

use sim_signal::{Rng, Time};

use crate::contact::ContactFrame;
use crate::work::{HazardEvidence, HazardKind};
use crate::{Analysis, BodyLoad, Interval, ProcessError, SourcedRange, StepModel};

use super::ForceCurve;
use super::SoftObjectMeasure;
use super::math::{
    Vector, add, analyzed, check_interval, cross, dot, finite, invalid, length_and_unit, load,
    norm, positive, range, scale, sub, unavailable,
};

#[derive(Clone, Debug, PartialEq)]
pub struct CableSpan {
    pub rest_length_m: SourcedRange,
    /// EA, осевая жёсткость сечения в ньютонах.
    pub tensile_rigidity_n: SourcedRange,
    /// Вязкая составляющая осевого усилия на единицу скорости деформации.
    pub axial_viscosity_ns: SourcedRange,
    /// Допустимый диапазон относительного удлинения, включая ослабленные ветви.
    pub strain: SourcedRange,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CableBend {
    /// EI, жёсткость сечения на изгиб.
    pub rigidity_nm2: SourcedRange,
    /// Предельная кривизна, проверенная для выбранного материала и дискретизации.
    pub maximum_curvature_inv_m: SourcedRange,
    /// Калиброванная верхняя граница угла между соседними ветвями, строго меньше π.
    pub maximum_angle_rad: SourcedRange,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CableLaw {
    pub spans: Vec<CableSpan>,
    pub bends: Vec<CableBend>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct RubberLaw {
    pub free_length_m: SourcedRange,
    pub compression: ForceCurve,
    pub damping_ns_m: SourcedRange,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FoamLaw {
    pub free_length_m: SourcedRange,
    pub loading: ForceCurve,
    /// Форма разгрузки и повторной нагрузки ниже предыдущего максимума.
    /// Масштабируется по силе в точке разворота на кривой первичного сжатия.
    pub unloading: ForceCurve,
}

#[derive(Clone, Debug, PartialEq)]
pub enum SoftLaw {
    Cable(CableLaw),
    Rubber(RubberLaw),
    Foam(FoamLaw),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SoftBinding {
    pub name: String,
    /// Имена тел в порядке узлов. Несколько узлов могут принадлежать одному телу.
    pub bodies: Vec<String>,
    /// Геометрии, для которых за интервал проверены и разрешены все возможные пары.
    /// Для кабеля геометрия обязательна на каждой ветви, в порядке ветвей.
    pub geometries: Vec<String>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftNode {
    pub point_m: [f64; 3],
    pub velocity_m_s: [f64; 3],
    pub com_m: [f64; 3],
}

#[derive(Clone, Debug, PartialEq)]
pub struct SoftContactReaction {
    pub geometry: String,
    /// Сумма модулей сил отдельных контактов данной геометрии, Н.
    /// Ноль означает наблюдённое отсутствие контакта, включая проверку таблицы пар.
    pub reaction_n: f64,
}

impl SoftBinding {
    /// Суммирует реакции движка по геометриям. Перед запуском сборщик должен
    /// проверить покрытие всех допустимых пар. Здесь проверяются пары,
    /// фактически возникшие за интервал, включая контакт ветвей друг с другом.
    pub fn observe_contacts(
        &self,
        interval: Interval,
        frame: &ContactFrame,
        resolved_pairs: &[[String; 2]],
    ) -> Result<Analysis<Vec<SoftContactReaction>>, ProcessError> {
        check_interval(interval, None)?;
        if frame.at != interval.end {
            return Err(invalid("реакции контакта относятся к другому интервалу"));
        }
        let mut totals: BTreeMap<_, f64> =
            self.geometries.iter().map(|g| (g.as_str(), 0.0)).collect();
        fn key(mut names: [&str; 2]) -> [&str; 2] {
            names.sort_unstable();
            names
        }
        let mut covered = BTreeSet::new();
        for pair in resolved_pairs {
            if pair[0] == pair[1]
                || pair.iter().any(|name| name.trim().is_empty())
                || !covered.insert(key(pair.each_ref().map(String::as_str)))
            {
                return Err(invalid("пустая или повторная пара контакта"));
            }
        }
        let mut missing = None;
        for sample in &frame.contacts {
            finite(
                sample
                    .point_m
                    .into_iter()
                    .chain(sample.normal)
                    .chain(sample.force_n)
                    .chain(sample.torque_nm)
                    .chain([sample.separation_m]),
            )?;
            let names = sample.geometries.each_ref().map(String::as_str);
            if names[0] == names[1] || names.iter().any(|name| name.trim().is_empty()) {
                return Err(invalid("недопустимые геометрии контакта"));
            }
            if names.iter().any(|name| totals.contains_key(name)) && !covered.contains(&key(names))
            {
                missing = Some(names);
            }
            let reaction = norm(sample.force_n);
            for name in names {
                if let Some(total) = totals.get_mut(name) {
                    *total += reaction;
                    finite([*total])?;
                }
            }
        }
        if let Some(pair) = missing {
            return Ok(unavailable(format!(
                "нет разрешённой пары материала для {pair:?}"
            )));
        }
        Ok(analyzed(
            totals
                .into_iter()
                .map(|(geometry, reaction_n)| SoftContactReaction {
                    geometry: geometry.into(),
                    reaction_n,
                })
                .collect(),
            "контакты подтверждённых пар за завершённый интервал",
        ))
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SoftInput {
    pub nodes: Vec<SoftNode>,
    pub contacts: Analysis<Vec<SoftContactReaction>>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SoftResponse {
    /// Для кабеля это суммарное изменение длины, для резины и пены — сжатие.
    pub deformation_m: f64,
    /// Максимум натяжений ветвей или сила сжатия.
    pub reaction_n: f64,
    pub stored_energy_j: f64,
    /// Сумма по наблюдаемым геометриям. Контакт внутри предмета учитывается на обеих сторонах.
    pub contact_reaction_n: f64,
}

impl SoftResponse {
    fn validate(self) -> Result<(), ProcessError> {
        finite([
            self.deformation_m,
            self.reaction_n,
            self.stored_energy_j,
            self.contact_reaction_n,
        ])?;
        if [
            self.reaction_n,
            self.stored_energy_j,
            self.contact_reaction_n,
        ]
        .into_iter()
        .any(|v| v < 0.0)
        {
            return Err(invalid(
                "отрицательная реакция или запас энергии мягкого предмета",
            ));
        }
        Ok(())
    }

    fn difference(a: Self, b: Self) -> Self {
        Self {
            deformation_m: (a.deformation_m - b.deformation_m).abs(),
            reaction_n: (a.reaction_n - b.reaction_n).abs(),
            stored_energy_j: (a.stored_energy_j - b.stored_energy_j).abs(),
            contact_reaction_n: (a.contact_reaction_n - b.contact_reaction_n).abs(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct SoftStep {
    pub name: String,
    pub at: Time,
    pub response: Analysis<SoftResponse>,
    /// Осевые натяжения по ветвям кабеля. У объёмной компрессионной модели ветвей нет.
    pub tensions_n: Analysis<Vec<f64>>,
    /// Мгновенные вязкие потери. Для пены задано вместо них необратимое накопление работы.
    pub dissipated_power_w: Analysis<f64>,
    /// Накопленная необратимая работа на диаграмме пены.
    pub foam_loss_j: Analysis<f64>,
    /// Упругие и вязкие нагрузки. Контакт уже приложен движком и сюда не входит.
    pub loads: Vec<BodyLoad>,
    pub hazards: Vec<HazardEvidence>,
}

impl SoftStep {
    pub fn measure(&self) -> SoftObjectMeasure {
        let name = self.name.clone();
        match &self.response {
            Analysis::Analyzed { value, scope } => SoftObjectMeasure {
                name,
                deformation_m: analyzed(value.deformation_m, scope),
                reaction_n: analyzed(value.reaction_n, scope),
                stored_energy_j: analyzed(value.stored_energy_j, scope),
            },
            Analysis::NotAnalyzed { reason } => SoftObjectMeasure {
                name,
                deformation_m: unavailable(reason),
                reaction_n: unavailable(reason),
                stored_energy_j: unavailable(reason),
            },
        }
    }
}

#[derive(Clone, Debug)]
pub struct SoftModel {
    binding: SoftBinding,
    law: SoftLaw,
    previous: Option<Time>,
    foam_peak: f64,
    foam_loss: f64,
    history_lost: bool,
}

impl SoftModel {
    pub fn new(binding: SoftBinding, law: SoftLaw) -> Result<Self, ProcessError> {
        let count = match &law {
            SoftLaw::Cable(cable) => {
                cable.validate()?;
                if binding.geometries.len() != cable.spans.len() {
                    return Err(invalid(
                        "для каждой ветви кабеля нужна разрешённая геометрия контакта",
                    ));
                }
                cable.spans.len() + 1
            }
            SoftLaw::Rubber(rubber) => {
                positive(&rubber.free_length_m)?;
                range(&rubber.damping_ns_m, 0.0, f64::MAX)?;
                rubber.compression.compression()?;
                check_compression_length(&rubber.compression, &rubber.free_length_m)?;
                2
            }
            SoftLaw::Foam(foam) => {
                foam.validate()?;
                2
            }
        };
        if binding.name.trim().is_empty()
            || binding.bodies.len() != count
            || binding.bodies.iter().any(|n| n.trim().is_empty())
            || binding.geometries.is_empty()
            || binding.geometries.iter().any(|n| n.trim().is_empty())
            || binding.geometries.iter().collect::<BTreeSet<_>>().len() != binding.geometries.len()
            || (!matches!(law, SoftLaw::Cable(_)) && binding.bodies[0] == binding.bodies[1])
        {
            return Err(invalid(
                "неразрешённые тела или повторяющиеся геометрии мягкого предмета",
            ));
        }
        Ok(Self {
            binding,
            law,
            previous: None,
            foam_peak: 0.0,
            foam_loss: 0.0,
            history_lost: false,
        })
    }

    pub fn binding(&self) -> &SoftBinding {
        &self.binding
    }

    pub fn hazards(&self) -> Vec<HazardEvidence> {
        [HazardKind::Pinch, HazardKind::StoredEnergy]
            .into_iter()
            .map(|kind| HazardEvidence {
                subject: self.binding.name.clone(),
                kind,
                coverage: unavailable(
                    "реакция и запас энергии требуют оценки опасности во всей сцене",
                ),
            })
            .collect()
    }

    fn contact_sum(&self, input: &SoftInput) -> Result<Analysis<f64>, ProcessError> {
        let value = match &input.contacts {
            Analysis::Analyzed { value, .. } => value,
            Analysis::NotAnalyzed { reason } => return Ok(unavailable(reason)),
        };
        let mut measured = BTreeMap::new();
        for sample in value {
            finite([sample.reaction_n])?;
            if sample.reaction_n < 0.0
                || !self.binding.geometries.contains(&sample.geometry)
                || measured
                    .insert(&sample.geometry, sample.reaction_n)
                    .is_some()
            {
                return Err(invalid(
                    "лишняя, отрицательная или повторная контактная реакция",
                ));
            }
        }
        if measured.len() != self.binding.geometries.len() {
            return Ok(unavailable(
                "есть геометрия без проверенного контакта или пары материалов",
            ));
        }
        let sum = measured.values().sum();
        finite([sum])?;
        Ok(analyzed(sum, "силы контакта за завершённый интервал"))
    }
}

impl CableLaw {
    fn validate(&self) -> Result<(), ProcessError> {
        if self.spans.is_empty() || self.bends.len() != self.spans.len() - 1 {
            return Err(invalid(
                "кабель требует ветвей и одной изгибной характеристики на внутренний узел",
            ));
        }
        for span in &self.spans {
            positive(&span.rest_length_m)?;
            positive(&span.tensile_rigidity_n)?;
            range(&span.axial_viscosity_ns, 0.0, f64::MAX)?;
            range(&span.strain, -1.0, f64::MAX)?;
            if span.strain.min > 0.0 || span.strain.max <= 0.0 {
                return Err(invalid(
                    "диапазон калибровки кабеля должен включать покой и растяжение",
                ));
            }
        }
        for bend in &self.bends {
            positive(&bend.rigidity_nm2)?;
            positive(&bend.maximum_curvature_inv_m)?;
            range(&bend.maximum_angle_rad, 0.0, PI)?;
            if bend.maximum_angle_rad.min <= 0.0 || bend.maximum_angle_rad.max >= PI {
                return Err(invalid(
                    "не задан калиброванный диапазон углов изгиба меньше π",
                ));
            }
        }
        Ok(())
    }

    fn evaluate(&self, nodes: &[SoftNode]) -> Result<Analysis<Deformation>, ProcessError> {
        let mut d = Deformation::new(nodes.len());
        let mut spans = Vec::with_capacity(self.spans.len());
        for (i, span) in self.spans.iter().enumerate() {
            let [a, b] = [nodes[i], nodes[i + 1]];
            let separation = sub(b.point_m, a.point_m);
            finite(separation)?;
            let Some((length, direction)) = length_and_unit(separation) else {
                return Ok(unavailable("вырожденная геометрия ветви кабеля"));
            };
            let rest = span.rest_length_m.nominal;
            let dx = length - rest;
            let rate = dot(sub(b.velocity_m_s, a.velocity_m_s), direction);
            let strain = dx / rest;
            finite([rate, strain])?;
            if !(span.strain.min..=span.strain.max).contains(&strain) {
                return Ok(unavailable("удлинение ветви вне калиброванного диапазона"));
            }
            let (tension, energy, loss) = if dx < 0.0 {
                // Ослабленная ветвь не передаёт осевого сжатия.
                (0.0, 0.0, 0.0)
            } else {
                let elastic = span.tensile_rigidity_n.nominal * strain;
                let trial = elastic + span.axial_viscosity_ns.nominal * rate / rest;
                finite([trial])?;
                let tension = trial.max(0.0);
                (tension, 0.5 * elastic * dx, (tension - elastic) * rate)
            };
            d.forces[i] = add(d.forces[i], scale(direction, tension));
            d.forces[i + 1] = add(d.forces[i + 1], scale(direction, -tension));
            d.deformation_m += dx;
            d.reaction_n = d.reaction_n.max(tension);
            d.energy_j += energy;
            d.loss_w += loss;
            d.tensions.push(tension);
            spans.push((length, direction));
        }
        for (i, bend) in self.bends.iter().enumerate() {
            let (left_length, left) = spans[i];
            let (right_length, right) = spans[i + 1];
            let dual_length = self.spans[i].rest_length_m.nominal * 0.5
                + self.spans[i + 1].rest_length_m.nominal * 0.5;
            let plane = cross(left, right);
            let theta = norm(plane).atan2(dot(left, right));
            let curvature = theta / dual_length;
            finite([curvature])?;
            if theta > bend.maximum_angle_rad.min || curvature > bend.maximum_curvature_inv_m.min {
                return Ok(unavailable("кривизна кабеля вне калиброванного диапазона"));
            }
            if let Some((_, normal)) = length_and_unit(plane) {
                // Дискретизация интеграла EI·κ²/2 по двойственному отрезку.
                // Силы — отрицательный градиент энергии угла по трём положениям.
                let moment = bend.rigidity_nm2.nominal * curvature;
                let first = scale(cross(left, normal), moment / left_length);
                let last = scale(cross(normal, right), -moment / right_length);
                let middle = scale(add(first, last), -1.0);
                d.forces[i] = add(d.forces[i], first);
                d.forces[i + 1] = add(d.forces[i + 1], middle);
                d.forces[i + 2] = add(d.forces[i + 2], last);
                d.energy_j += 0.5 * moment * theta;
            }
        }
        Ok(analyzed(
            d,
            "растяжение, осевые потери и дискретный изгиб кабеля",
        ))
    }
}

fn check_compression_length(curve: &ForceCurve, free: &SourcedRange) -> Result<(), ProcessError> {
    if curve.points().last().unwrap().displacement_m.max >= free.min {
        return Err(invalid(
            "кривая сжатия подразумевает нулевую или отрицательную толщину",
        ));
    }
    Ok(())
}

impl FoamLaw {
    fn validate(&self) -> Result<(), ProcessError> {
        positive(&self.free_length_m)?;
        self.loading.compression()?;
        self.unloading.compression()?;
        check_compression_length(&self.loading, &self.free_length_m)?;
        check_compression_length(&self.unloading, &self.free_length_m)?;
        if self.loading.maximum_m() != self.unloading.maximum_m() {
            return Err(invalid(
                "нагружение и разгрузка должны покрывать один диапазон сжатия",
            ));
        }
        let mut knots: Vec<_> = [&self.loading, &self.unloading]
            .into_iter()
            .flat_map(|curve| {
                curve
                    .points()
                    .iter()
                    .skip(1)
                    .map(|p| p.displacement_m.nominal)
            })
            .collect();
        knots.sort_by(f64::total_cmp);
        let mut previous = 0.0;
        for x in knots {
            let ratio = self.unloading.at(x)?.0 / self.loading.at(x)?.0;
            finite([ratio])?;
            // На каждом линейном интервале отношение двух линейных функций монотонно.
            // Неубывающий U/L гарантирует, что масштабированная ветвь не поднимется над L.
            if ratio <= 0.0 || ratio < previous {
                return Err(invalid(
                    "ветвь разгрузки допускает появление энергии на цикле",
                ));
            }
            previous = ratio;
        }
        Ok(())
    }

    fn response(
        &self,
        x: f64,
        old_peak: f64,
        old_loss: f64,
    ) -> Result<(f64, f64, f64, f64), ProcessError> {
        if x == 0.0 {
            return Ok((0.0, 0.0, 0.0, old_loss));
        }
        let peak = old_peak.max(x);
        let (peak_force, peak_work) = self.loading.at(peak)?;
        let (turn_force, turn_work) = self.unloading.at(peak)?;
        let (unload_force, unload_work) = self.unloading.at(x)?;
        let scale = peak_force / turn_force;
        let mut loss = old_loss;
        if peak > old_peak {
            let previous = if old_peak == 0.0 {
                0.0
            } else {
                let (force, work) = self.loading.at(old_peak)?;
                let (turn_force, turn_work) = self.unloading.at(old_peak)?;
                work - force / turn_force * turn_work
            };
            let next = peak_work - scale * turn_work;
            // Потеря есть разность работ. Нуль ограничивает вычитание близких чисел.
            loss += (next - previous).max(0.0);
        }
        let force = if x == peak {
            peak_force
        } else {
            scale * unload_force
        };
        let energy = scale * unload_work;
        finite([force, energy, loss])?;
        Ok((force, energy, peak, loss))
    }
}

struct Deformation {
    forces: Vec<Vector>,
    deformation_m: f64,
    reaction_n: f64,
    energy_j: f64,
    loss_w: f64,
    tensions: Vec<f64>,
    foam: Option<(f64, f64)>,
}

impl Deformation {
    fn new(nodes: usize) -> Self {
        Self {
            forces: vec![[0.0; 3]; nodes],
            deformation_m: 0.0,
            reaction_n: 0.0,
            energy_j: 0.0,
            loss_w: 0.0,
            tensions: Vec::new(),
            foam: None,
        }
    }
}

impl SoftModel {
    fn compression(
        &self,
        nodes: &[SoftNode],
        free: f64,
    ) -> Result<Option<(f64, f64, Vector)>, ProcessError> {
        let [a, b] = [nodes[0], nodes[1]];
        let separation = sub(b.point_m, a.point_m);
        finite(separation)?;
        let Some((length, direction)) = length_and_unit(separation) else {
            return Ok(None);
        };
        let rate = -dot(sub(b.velocity_m_s, a.velocity_m_s), direction);
        finite([rate])?;
        // До касания односторонняя компрессионная модель не передаёт усилия.
        Ok(Some(((free - length).max(0.0), rate, direction)))
    }

    fn evaluate(&self, input: &SoftInput) -> Result<Analysis<Deformation>, ProcessError> {
        let (free, curve) = match &self.law {
            SoftLaw::Cable(cable) => return cable.evaluate(&input.nodes),
            SoftLaw::Rubber(law) => (law.free_length_m.nominal, &law.compression),
            SoftLaw::Foam(law) => (law.free_length_m.nominal, &law.loading),
        };
        let Some((x, rate, direction)) = self.compression(&input.nodes, free)? else {
            return Ok(unavailable("вырожденное положение поверхностей сжатия"));
        };
        let Some((force, work)) = curve.sample(x)? else {
            return Ok(unavailable("нет участка кривой для наблюдаемого сжатия"));
        };
        let mut d = Deformation::new(input.nodes.len());
        d.deformation_m = x;
        match &self.law {
            SoftLaw::Rubber(law) => {
                let trial = force + law.damping_ns_m.nominal * rate;
                finite([trial])?;
                d.reaction_n = if x > 0.0 { trial.max(0.0) } else { 0.0 };
                d.energy_j = work;
                d.loss_w = (d.reaction_n - force) * rate;
            }
            SoftLaw::Foam(law) => {
                if self.previous.is_none() && x > 0.0 {
                    return Ok(unavailable("начальная история сжатой пены неизвестна"));
                }
                if self.history_lost {
                    return Ok(unavailable(
                        "история разгрузки пены потеряна вне области модели",
                    ));
                }
                let (force, energy, peak, loss) =
                    law.response(x, self.foam_peak, self.foam_loss)?;
                d.reaction_n = force;
                d.energy_j = energy;
                d.foam = Some((peak, loss));
            }
            SoftLaw::Cable(_) => unreachable!(),
        }
        d.forces = vec![
            scale(direction, -d.reaction_n),
            scale(direction, d.reaction_n),
        ];
        Ok(analyzed(
            d,
            "номинальное одностороннее сжатие между заданными поверхностями",
        ))
    }
}

impl StepModel for SoftModel {
    type Input = SoftInput;
    type Output = SoftStep;

    fn advance(
        &mut self,
        interval: Interval,
        input: &Self::Input,
        _: &mut Rng,
    ) -> Result<Self::Output, ProcessError> {
        check_interval(interval, self.previous)?;
        if input.nodes.len() != self.binding.bodies.len() {
            return Err(invalid(
                "количество положений не совпадает с разбиением предмета",
            ));
        }
        for node in &input.nodes {
            finite(
                node.point_m
                    .into_iter()
                    .chain(node.velocity_m_s)
                    .chain(node.com_m),
            )?;
        }
        let contact = self.contact_sum(input)?;
        let computed = match &contact {
            Analysis::Analyzed { .. } => self.evaluate(input)?,
            Analysis::NotAnalyzed { reason } => unavailable(reason),
        };
        let mut step = SoftStep {
            name: self.binding.name.clone(),
            at: interval.end,
            response: unavailable("нет результатов модели"),
            tensions_n: unavailable("натяжение ветвей относится к кабелю"),
            dissipated_power_w: unavailable("мгновенный вязкий член не задан"),
            foam_loss_j: unavailable("история диаграммы разгрузки относится к пене"),
            loads: Vec::new(),
            hazards: self.hazards(),
        };
        match computed {
            Analysis::Analyzed { value: d, scope } => {
                let Analysis::Analyzed {
                    value: contact_reaction_n,
                    ..
                } = contact
                else {
                    unreachable!("деформация рассчитана только с проверенным контактом");
                };
                let response = SoftResponse {
                    deformation_m: d.deformation_m,
                    reaction_n: d.reaction_n,
                    stored_energy_j: d.energy_j,
                    contact_reaction_n,
                };
                response.validate()?;
                finite(std::iter::once(d.loss_w).chain(d.tensions.iter().copied()))?;
                let mut loads = BTreeMap::<&str, (Vector, Vector)>::new();
                for ((body, node), force) in
                    self.binding.bodies.iter().zip(&input.nodes).zip(d.forces)
                {
                    let wrench = load(body, node.point_m, node.com_m, force, [0.0; 3])?;
                    let entry = loads.entry(body).or_insert(([0.0; 3], [0.0; 3]));
                    entry.0 = add(entry.0, wrench.force_n);
                    entry.1 = add(entry.1, wrench.torque_nm);
                }
                for (body, (force_n, torque_nm)) in loads {
                    finite(force_n.into_iter().chain(torque_nm))?;
                    step.loads.push(BodyLoad {
                        body: body.into(),
                        force_n,
                        torque_nm,
                    });
                }
                step.response = analyzed(response, &scope);
                if !d.tensions.is_empty() {
                    step.tensions_n = analyzed(d.tensions, &scope);
                }
                if let Some((peak, loss)) = d.foam {
                    self.foam_peak = peak;
                    self.foam_loss = loss;
                    step.foam_loss_j = analyzed(loss, &scope);
                } else {
                    step.dissipated_power_w = analyzed(d.loss_w, &scope);
                }
            }
            Analysis::NotAnalyzed { reason } => {
                self.history_lost |= matches!(self.law, SoftLaw::Foam(_));
                step.response = unavailable(&reason);
                step.tensions_n = unavailable(&reason);
                step.dissipated_power_w = unavailable(&reason);
                step.foam_loss_j = unavailable(reason);
            }
        }
        self.previous = Some(interval.end);
        Ok(step)
    }
}

/// Измерение на сетке одной и той же задачи с одинаковыми параметрами и временем.
#[derive(Clone, Debug, PartialEq)]
pub struct SpatialSample {
    pub elements: usize,
    pub at: Time,
    pub response: Analysis<SoftResponse>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SpatialTolerance {
    pub deformation_m: SourcedRange,
    pub reaction_n: SourcedRange,
    pub stored_energy_j: SourcedRange,
    pub contact_reaction_n: SourcedRange,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SpatialComparison {
    pub differences: [SoftResponse; 2],
    pub within_tolerance: bool,
}

/// Сравнивает две соседние разности на трёх вложенных разбиениях одной геометрии.
/// Совпадение окон, времени, возбуждения и калибровки трёх прогонов гарантирует вызывающий.
pub fn compare_spatial(
    samples: &[SpatialSample; 3],
    tolerance: &SpatialTolerance,
) -> Result<Analysis<SpatialComparison>, ProcessError> {
    let limits = [
        &tolerance.deformation_m,
        &tolerance.reaction_n,
        &tolerance.stored_energy_j,
        &tolerance.contact_reaction_n,
    ];
    for value in limits {
        range(value, 0.0, f64::MAX)?;
    }
    for entry in samples {
        if let Analysis::Analyzed { value, .. } = entry.response {
            value.validate()?;
        }
    }
    if samples[0].elements == 0
        || samples
            .windows(2)
            .any(|p| p[0].at != p[1].at || p[0].elements.checked_mul(2) != Some(p[1].elements))
    {
        return Err(invalid(
            "нужны три вложенных разбиения в один физический момент",
        ));
    }
    let [a, b, c] = samples.each_ref().map(|s| &s.response);
    let (
        Analysis::Analyzed { value: a, .. },
        Analysis::Analyzed { value: b, .. },
        Analysis::Analyzed { value: c, .. },
    ) = (a, b, c)
    else {
        return Ok(unavailable("отсутствует измерение на одном из разбиений"));
    };
    let differences = [
        SoftResponse::difference(*a, *b),
        SoftResponse::difference(*b, *c),
    ];
    let mut within_tolerance = true;
    for difference in differences {
        difference.validate()?;
        within_tolerance &= [
            difference.deformation_m,
            difference.reaction_n,
            difference.stored_energy_j,
            difference.contact_reaction_n,
        ]
        .into_iter()
        .zip(limits)
        .all(|(delta, limit)| delta <= limit.min);
    }
    if !within_tolerance {
        return Ok(unavailable(format!(
            "пространственная несходимость: {differences:?}"
        )));
    }
    Ok(analyzed(
        SpatialComparison {
            differences,
            within_tolerance,
        },
        "сходимость метрик при удвоении числа элементов",
    ))
}
