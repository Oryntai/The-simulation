use std::ptr::{NonNull, from_ref};

#[cfg(test)]
use sim_mujoco::{JointInfo, JointKind};
use sim_mujoco::{Sim, ffi};

/// Отдельный кэш инерции. После mj_step у рабочего mjData qpos уже
/// новая, а матрица масс ещё может относиться к предыдущей позе.
#[derive(Default)]
pub(crate) struct MassProbe {
    scratch: Option<MassData>,
    tendon_inertia: Option<bool>,
    projection: Vec<f64>,
    response: Option<ResponseData>,
    /// Обобщённые силы шага вне этой группы приводов (стимулы, усилия без привода,
    /// процессы, другие группы). Пустой вектор означает нули.
    external: Vec<f64>,
}

impl Clone for MassProbe {
    fn clone(&self) -> Self {
        // Ветви и снимки не делят изменяемый mjData. Кэш восстановится из позы.
        Self::default()
    }
}

struct MassData {
    data: NonNull<ffi::mjData>,
    rhs: Vec<f64>,
    solution: Vec<f64>,
    ready: bool,
}

impl Drop for MassData {
    fn drop(&mut self) {
        // SAFETY: это отдельный mjData, которым владеет только MassData.
        unsafe { ffi::mj_deleteData(self.data.as_ptr()) };
    }
}

struct ResponseData {
    data: NonNull<ffi::mjData>,
    nonlinear: bool,
    reuse_velocity_stage: bool,
}

impl Drop for ResponseData {
    fn drop(&mut self) {
        // SAFETY: указатель принадлежит этой копии состояния.
        unsafe { ffi::mj_deleteData(self.data.as_ptr()) };
    }
}

impl MassProbe {
    /// Блок M⁻¹ по приводным осям, по строкам. Остальные оси свободны.
    pub(crate) fn projection(&self) -> &[f64] {
        &self.projection
    }

    pub(crate) fn free_velocity(&mut self, sim: &Sim, dofs: &[usize]) -> Result<Vec<f64>, String> {
        let m = from_ref(sim.model());
        if self.response.is_none() {
            // SAFETY: движок владеет живой моделью.
            let data = NonNull::new(unsafe { ffi::mj_makeData(m) })
                .ok_or("MuJoCo не создал данные для расчёта отклика оси")?;
            self.response = Some(ResponseData {
                data,
                nonlinear: false,
                // Для простого Euler второй этап меняет состояние интегратора,
                // а кинематика и ограничения остаются в исходных qpos и qvel.
                // Нативные актуаторы, датчики, плагины и гибкие тела сохраняют
                // полный пробный шаг со своей копией всего состояния.
                reuse_velocity_stage: sim.model().opt.integrator as u32
                    == ffi::mjtIntegrator::mjINT_EULER
                    && sim.model().opt.enableflags == 0
                    && sim.model().nu == 0
                    && sim.model().nsensor == 0
                    && sim.model().nplugin == 0
                    && sim.model().nflex == 0,
            });
        }
        let response = self.response.as_mut().expect("данные созданы");
        let ptr = response.data.as_ptr();
        // SAFETY: все указатели относятся к живым данным той же модели;
        // массивы qfrc_applied, qacc и dof_damping имеют nv элементов.
        let velocities = unsafe {
            reset_response(sim, ptr, dofs, &[], &self.external);
            ffi::mj_forward(m, ptr);
            let d = ptr.as_ref().expect("mjData существует");
            check_response(d)?;
            response.nonlinear =
                d.nefc != 0 || (0..sim.nv()).any(|i| *sim.model().dof_damping.add(i) != 0.0);
            dofs.iter()
                .map(|&dof| sim.timestep().mul_add(*d.qacc.add(dof), sim.qvel()[dof]))
                .collect::<Vec<_>>()
        };
        if !velocities.iter().copied().all(f64::is_finite) {
            return Err("неконечный отклик оси в MuJoCo".into());
        }
        Ok(velocities)
    }

    /// Внешние силы, которые подействуют на следующем шаге вместе с усилиями группы.
    pub(crate) fn set_external(&mut self, forces: &[f64]) {
        self.external.clear();
        self.external.extend_from_slice(forces);
    }

    pub(crate) fn uses_trial_step(&self) -> bool {
        self.response.as_ref().is_some_and(|probe| probe.nonlinear)
    }

    pub(crate) fn respond(
        &self,
        sim: &Sim,
        dofs: &[usize],
        free: &[f64],
        efforts: &[f64],
    ) -> Result<Vec<f64>, String> {
        if !efforts.iter().copied().all(f64::is_finite) {
            return Err("сумма усилий на оси вне численного диапазона".into());
        }
        let response = self.response.as_ref().expect("свободный отклик рассчитан");
        let velocities: Vec<f64> = if response.nonlinear {
            let ptr = response.data.as_ptr();
            // SAFETY: данные пробы принадлежат MassProbe и не пересекаются с Sim.
            unsafe {
                let m = from_ref(sim.model());
                if response.reuse_velocity_stage {
                    // Каждый пробный импульс стартует из того же состояния,
                    // в том числе с тем же начальным приближением решателя.
                    let live = sim.data();
                    let d = &mut *ptr;
                    d.time = live.time;
                    for (src, dst, len) in [
                        (live.qpos, d.qpos, sim.nq()),
                        (live.qvel, d.qvel, sim.nv()),
                        (live.qacc_warmstart, d.qacc_warmstart, sim.nv()),
                    ] {
                        std::ptr::copy_nonoverlapping(src, dst, len);
                    }
                    set_trial_efforts(d, sim.nv(), dofs, efforts, &self.external);
                    ffi::mj_step2(m, ptr);
                } else {
                    reset_response(sim, ptr, dofs, efforts, &self.external);
                    ffi::mj_step(m, ptr);
                }
                let d = ptr.as_ref().expect("mjData существует");
                check_response(d)?;
                if d.time != sim.time() + sim.timestep() || d.time <= sim.time() {
                    return Err("MuJoCo сбросил состояние в пробном шаге отклика".into());
                }
                dofs.iter().map(|&dof| *d.qvel.add(dof)).collect()
            }
        } else {
            self.projection()
                .chunks_exact(dofs.len())
                .zip(free)
                .map(|(row, &velocity)| {
                    let acceleration = row.iter().zip(efforts).map(|(m, f)| m * f).sum::<f64>();
                    sim.timestep().mul_add(acceleration, velocity)
                })
                .collect()
        };
        if !velocities.iter().copied().all(f64::is_finite) {
            return Err("скорость отклика на усилие вне численного диапазона".into());
        }
        Ok(velocities)
    }

    /// Матрица измеряется в текущей позе, в неизменном порядке выбранных осей.
    pub(crate) fn measure(
        &mut self,
        sim: &Sim,
        dofs: &[usize],
        result: &mut [(f64, f64)],
    ) -> Result<(), String> {
        let m = sim.model();
        let nv = sim.nv();
        self.projection.resize(dofs.len() * dofs.len(), 0.0);
        let tendon_inertia = *self
            .tendon_inertia
            .get_or_insert_with(|| has_tendon_inertia(sim));
        if nv == 1 && !tendon_inertia {
            // Единственная степень свободы без инерции сухожилий: постоянная
            // масса поступательного движения или момент вокруг фиксированной оси.
            let j = diagonal(m, sim.data(), 0)?;
            result[0] = (j, 1.0 / j);
            self.projection[0] = result[0].1;
            return Ok(());
        }
        if self.scratch.is_none() {
            // SAFETY: сам движок владеет живой моделью, mj_makeData её читает.
            let data = NonNull::new(unsafe { ffi::mj_makeData(from_ref(m)) })
                .ok_or("MuJoCo не создал данные для проверки матрицы масс")?;
            self.scratch = Some(MassData {
                data,
                rhs: vec![0.0; nv],
                solution: vec![0.0; nv],
                ready: false,
            });
        }
        let scratch = self.scratch.as_mut().expect("данные созданы");
        let ptr = scratch.data.as_ptr();
        // SAFETY: все массивы взяты из двух mjData для одной модели и имеют
        // размеры из этой модели. Запись идёт только в принадлежащий нам mjData.
        unsafe {
            let d = ptr.as_ref().expect("mjData существует");
            let q = sim.qpos();
            let old_q = std::slice::from_raw_parts_mut(d.qpos, sim.nq());
            let mut changed = !scratch.ready || old_q != q;
            if changed {
                old_q.copy_from_slice(q);
            }
            // Положение кинематической цели может менять моментные плечи сухожилия.
            if tendon_inertia && m.nmocap > 0 {
                for (src, dst, stride) in [
                    (sim.data().mocap_pos, d.mocap_pos, 3),
                    (sim.data().mocap_quat, d.mocap_quat, 4),
                ] {
                    let len = stride * m.nmocap as usize;
                    let from = std::slice::from_raw_parts(src, len);
                    let to = std::slice::from_raw_parts_mut(dst, len);
                    if from != to {
                        to.copy_from_slice(from);
                        changed = true;
                    }
                }
            }
            if !changed {
                return Ok(());
            }
            ffi::mj_kinematics(from_ref(m), ptr);
            ffi::mj_comPos(from_ref(m), ptr);
            if tendon_inertia {
                ffi::mj_tendon(from_ref(m), ptr);
                check_tendon_terms(m, ptr.as_ref().expect("mjData существует"))?;
            }
            ffi::mj_makeM(from_ref(m), ptr);
            if m.nC as usize > nv {
                ffi::mj_factorM(from_ref(m), ptr);
            }
        }
        for (column, (&index, pair)) in dofs.iter().zip(result).enumerate() {
            // SAFETY: ptr принадлежит кэшу и сохраняет посчитанную выше матрицу.
            let j = diagonal(
                m,
                unsafe { ptr.as_ref().expect("mjData существует") },
                index,
            )?;
            let inverse = if m.nC as usize == nv {
                self.projection[column * dofs.len() + column] = 1.0 / j;
                1.0 / j
            } else {
                scratch.rhs[index] = 1.0;
                // SAFETY: векторы имеют nv элементов, факторизация относится
                // к той же модели. Ни один вектор не пересекается с mjData.
                unsafe {
                    ffi::mj_solveM(
                        from_ref(m),
                        ptr,
                        scratch.solution.as_mut_ptr(),
                        scratch.rhs.as_ptr(),
                        1,
                    );
                }
                scratch.rhs[index] = 0.0;
                for (row, &dof) in dofs.iter().enumerate() {
                    let value = scratch.solution[dof];
                    if !value.is_finite() {
                        return Err(format!("неконечный элемент M⁻¹ по осям {dof}, {index}"));
                    }
                    self.projection[row * dofs.len() + column] = value;
                }
                scratch.solution[index]
            };
            if !inverse.is_finite() || inverse <= 0.0 {
                return Err(format!("ось {index}: некорректная диагональ M⁻¹ {inverse}"));
            }
            *pair = (j, inverse);
        }
        scratch.ready = true;
        Ok(())
    }
}

// Независимый пробный шаг. Runtime добавляет свои обобщённые
// силы отдельно; их будущее значение интерфейс Object не передаёт.
unsafe fn reset_response(
    sim: &Sim,
    ptr: *mut ffi::mjData,
    dofs: &[usize],
    efforts: &[f64],
    external: &[f64],
) {
    // SAFETY: ptr создан для этой модели, индексы dofs проверены при привязке.
    unsafe {
        ffi::mj_copyData(ptr, from_ref(sim.model()), from_ref(sim.data()));
        set_trial_efforts(&*ptr, sim.nv(), dofs, efforts, external);
    }
}

unsafe fn set_trial_efforts(
    d: &ffi::mjData,
    nv: usize,
    dofs: &[usize],
    efforts: &[f64],
    external: &[f64],
) {
    // SAFETY: для пробного mjData этот массив имеет nv элементов и принадлежит только пробе.
    unsafe {
        let forces = std::slice::from_raw_parts_mut(d.qfrc_applied, nv);
        // Внешние силы шага входят в пробу так же, как в основной шаг рантайма.
        if external.len() == nv {
            forces.copy_from_slice(external);
        } else {
            forces.fill(0.0);
        }
        for (&dof, &effort) in dofs.iter().zip(efforts) {
            forces[dof] += effort;
        }
    }
}

fn check_response(data: &ffi::mjData) -> Result<(), String> {
    use ffi::mjtWarning as w;
    let warnings = [
        w::mjWARN_BADQPOS,
        w::mjWARN_BADQVEL,
        w::mjWARN_BADQACC,
        w::mjWARN_CONTACTFULL,
        w::mjWARN_CNSTRFULL,
    ];
    if warnings
        .iter()
        .all(|&kind| data.warning[kind as usize].number == 0)
    {
        Ok(())
    } else {
        Err("MuJoCo: сбой или переполнение при пробном расчёте отклика".into())
    }
}

fn has_tendon_inertia(sim: &Sim) -> bool {
    // SAFETY: поле взято из живой модели и содержит ntendon элементов.
    (0..sim.ntendon()).any(|i| unsafe { *sim.model().tendon_armature.add(i) != 0.0 })
}

// В инерцию входит armature·J_tᵀJ_t. В поставляемом движке между
// некоторыми несвязанными ветвями для этого нет места в разреженной M.
// Молчаливое выпадение связи меняет эффективную инерцию.
fn check_tendon_terms(m: &ffi::mjModel, d: &ffi::mjData) -> Result<(), String> {
    for t in 0..m.ntendon as usize {
        // SAFETY: все строки и индексы построены MuJoCo для одной модели.
        unsafe {
            let inertia = *m.tendon_armature.add(t);
            if inertia == 0.0 {
                continue;
            }
            if !inertia.is_finite() || inertia < 0.0 {
                return Err(format!("некорректная armature сухожилия {t}"));
            }
            let start = *m.ten_J_rowadr.add(t) as usize;
            let len = *m.ten_J_rownnz.add(t) as usize;
            if len == 0 {
                continue;
            }
            let columns = std::slice::from_raw_parts(m.ten_J_colind.add(start), len);
            let values = std::slice::from_raw_parts(d.ten_J.add(start), len);
            for (i, (&row, &value)) in columns.iter().zip(values).enumerate() {
                if !value.is_finite() {
                    return Err(format!("неконечное плечо сухожилия {t}"));
                }
                if value == 0.0 {
                    continue;
                }
                let start = *m.M_rowadr.add(row as usize) as usize;
                let len = *m.M_rownnz.add(row as usize) as usize;
                let present = std::slice::from_raw_parts(m.M_colind.add(start), len);
                if columns[..i]
                    .iter()
                    .zip(values)
                    .any(|(column, &arm)| arm != 0.0 && present.binary_search(column).is_err())
                {
                    return Err(format!(
                        "armature сухожилия {t}: MuJoCo пропускает связь оси {row} с другой ветвью в матрице масс"
                    ));
                }
            }
        }
    }
    Ok(())
}

/// Нижняя граница эффективной инерции во всех позах. Используется
/// только чтобы пропустить повторное разложение матрицы с большим запасом.
#[cfg(test)]
fn inertia_floor(sim: &Sim, joint: &JointInfo, dof: usize) -> f64 {
    let m = sim.model();
    let body = joint.body;
    // SAFETY: JointInfo взят из этого Sim. Все индексы принадлежат модели.
    let (armature, single_joint, parent) = unsafe {
        (
            *m.dof_armature.add(dof),
            *m.body_jntnum.add(body) == 1,
            *m.body_parentid.add(body) as usize,
        )
    };
    let mut floor = armature;
    if single_joint && joint.kind == JointKind::Hinge {
        let min_inertia = |b| {
            sim.body_inertia(b)
                .into_iter()
                .fold(f64::INFINITY, f64::min)
        };
        let child = min_inertia(body);
        if parent == 0 {
            floor += child;
        } else {
            // Скорость одноосного шарнира — проекция разности угловых
            // скоростей. Неравенство Коши для энергии двух тел даёт
            // 1/J_eff ≤ 1/I_min_child + 1/I_min_parent. Armature добавляет
            // положительную энергию непосредственно на данной координате.
            let parent = min_inertia(parent);
            if child > 0.0 && parent > 0.0 {
                floor += 1.0 / (1.0 / child + 1.0 / parent);
            }
        }
    }
    // Запас округления для вычисления нижней границы.
    floor * (1.0 - 8.0 * f64::EPSILON)
}

fn diagonal(m: &ffi::mjModel, d: &ffi::mjData, index: usize) -> Result<f64, String> {
    if index >= m.nv as usize {
        return Err(format!("нет строки {index} в матрице масс"));
    }
    // SAFETY: разреженная нижняя часть M хранится в строках, заканчивающихся
    // диагональю. Размеры полей заданы в mjmodel.h и mjdata.h.
    let (column, value) = unsafe {
        let row = *m.M_rowadr.add(index) as usize;
        let count = *m.M_rownnz.add(index) as usize;
        let last = row
            .checked_add(count)
            .filter(|&end| count > 0 && end <= m.nC as usize)
            .ok_or("некорректная строка матрицы масс MuJoCo")?
            - 1;
        (*m.M_colind.add(last), *d.M.add(last))
    };
    if column as usize != index || !value.is_finite() || value <= 0.0 {
        return Err(format!("ось {index}: некорректная диагональ M {value}"));
    }
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reused_velocity_stage_matches_full_euler_with_constraints_and_damping() {
        let mut sim = Sim::from_xml(
            r#"<mujoco><option timestep="0.001" gravity="0 0 0"/>
                <worldbody><body name="каретка">
                  <joint type="slide" axis="1 0 0" limited="true" range="0 1" damping="2"/>
                  <geom type="sphere" size="0.05" mass="1"/>
                </body></worldbody></mujoco>"#,
        )
        .unwrap();
        // Каретка выходит за предел на 1 мм со скоростью 1 м/с.
        // Реальный шаг под нагрузкой заполняет начальное приближение решателя.
        sim.qpos_mut()[0] = -0.001;
        sim.qvel_mut()[0] = -1.0;
        sim.qfrc_applied_mut()[0] = -2.0;
        sim.step();
        assert_ne!(sim.qpos()[0], 0.0);
        let mut probe = MassProbe::default();
        let mut full = MassProbe::default();
        let free = probe.free_velocity(&sim, &[0]).unwrap();
        full.free_velocity(&sim, &[0]).unwrap();
        assert!(probe.uses_trial_step());
        let response = full.response.as_mut().unwrap();
        // Убедимся, что сверяется именно активное ограничение и два разных пути.
        assert!(unsafe { response.data.as_ref().nefc } > 0);
        assert!(probe.response.as_ref().unwrap().reuse_velocity_stage);
        response.reuse_velocity_stage = false;
        let main = sim.state_hash();
        // Прямой и обратный пробные импульсы по 1 кг·м/с, затем свободный отклик.
        let step = sim.timestep();
        let mut velocities = Vec::new();
        for force in [-1.0 / step, 1.0 / step, 0.0, -1.0 / step] {
            let actual = probe.respond(&sim, &[0], &free, &[force]).unwrap()[0];
            let expected = full.respond(&sim, &[0], &free, &[force]).unwrap()[0];
            assert_eq!(actual.to_bits(), expected.to_bits(), "усилие {force}");
            velocities.push(actual);
        }
        assert_ne!(velocities[0], velocities[1]);
        assert_eq!(velocities[0], velocities[3]);
        assert_eq!(sim.state_hash(), main);
    }

    #[test]
    fn reads_current_inertia_and_accounts_for_motion_of_the_other_axis() {
        // Два стержня длиной 1 м, массы по 1 кг, J_cm и armature по 0.25.
        let mut sim = Sim::from_xml(
            r#"<mujoco><worldbody><body name="стержень">
                <joint name="ось" axis="0 0 1" armature="0.25"/>
                <inertial pos="0 0 0" mass="1" diaginertia="0.25 0.25 0.25"/>
                <body name="блок" pos="1 0 0">
                    <joint name="вторая ось" axis="0 0 1" armature="0.25"/>
                    <inertial pos="1 0 0" mass="1" diaginertia="0.25 0.25 0.25"/>
                </body></body></worldbody></mujoco>"#,
        )
        .unwrap();
        let mut probe = MassProbe::default();
        let mut pairs = [(0.0, 0.0); 2];
        let joints = sim.joints();
        let floors = [
            inertia_floor(&sim, &joints[0], 0),
            inertia_floor(&sim, &joints[1], 1),
        ];
        let check = |actual: f64, expected: f64| {
            assert!(
                (actual - expected).abs() < 1e-12,
                "{actual} против {expected}"
            );
        };
        for angle in [0.0, std::f64::consts::PI, std::f64::consts::FRAC_PI_2] {
            // Меняем qpos напрямую, чтобы матрица живого движка устарела.
            sim.qpos_mut()[1] = angle;
            probe.measure(&sim, &[0, 1], &mut pairs).unwrap();
            let [a, b, d] = [2.75 + 2.0 * angle.cos(), 1.25 + angle.cos(), 1.5];
            check(pairs[0].0, a);
            check(pairs[1].0, d);
            check(pairs[0].1, d / (a * d - b * b));
            check(pairs[1].1, a / (a * d - b * b));
            let det = a * d - b * b;
            for (&actual, expected) in probe.projection().iter().zip([d, -b, -b, a]) {
                check(actual, expected / det);
            }
            assert!(
                pairs
                    .iter()
                    .zip(floors)
                    .all(|((_, inverse), floor)| inverse * floor <= 1.0)
            );
            sim.forward();
            for (actual, expected) in sim.full_mass_matrix().into_iter().zip([a, b, b, d]) {
                check(actual, expected);
            }
            assert!(probe.scratch.is_some() && probe.clone().scratch.is_none());
        }
    }

    #[test]
    fn mass_probe_checks_tendon_armature_against_the_jacobian() {
        // Массы 1 и 2 кг, armature осей по 0.25 кг. Сухожилие
        // с инерцией 0.5 и вектором плеч (1, 2) добавляет 0.5·u·uᵀ.
        let xml = r#"<mujoco>
            <worldbody>
                <body><joint name="a" type="slide" armature="0.25"/>
                    <inertial pos="0 0 0" mass="1" diaginertia="0.25 0.25 0.25"/>
                <body><joint name="b" type="slide" armature="0.25"/>
                    <inertial pos="0 0 0" mass="2" diaginertia="0.25 0.25 0.25"/></body></body>
            </worldbody>
            <tendon><fixed armature="0.5"><joint joint="a" coef="1"/><joint joint="b" coef="2"/></fixed></tendon>
            </mujoco>"#;
        let serial = Sim::from_xml(xml).unwrap();
        let mut measured = [(0.0, 0.0); 2];
        MassProbe::default()
            .measure(&serial, &[0, 1], &mut measured)
            .unwrap();
        let det = 3.75 * 4.25 - 3.0 * 3.0;
        for ((j, inv), (expected_j, expected_inv)) in measured
            .into_iter()
            .zip([(3.75, 4.25 / det), (4.25, 3.75 / det)])
        {
            assert!((j - expected_j).abs() < 1e-12);
            assert!((inv - expected_inv).abs() < 1e-12);
        }
        assert_eq!(serial.full_mass_matrix(), vec![3.75, 3.0, 3.0, 4.25]);
        // При разнесении осей по ветвям кузовной крестовой член исчезает,
        // но сухожилие всё ещё даёт M_ab=1. Движок теряет этот член.
        let parallel = Sim::from_xml(
            &xml.replace(
                r#"<body><joint name="b""#,
                r#"</body><body><joint name="b""#,
            )
            .replace("</body></body>", "</body>"),
        )
        .unwrap();
        assert_eq!(parallel.full_mass_matrix(), vec![1.75, 0.0, 0.0, 4.25]);
        let error = MassProbe::default()
            .measure(&parallel, &[0, 1], &mut measured)
            .unwrap_err();
        assert!(
            error.contains("armature сухожилия") && error.contains("ветвью"),
            "{error}"
        );
    }
}
