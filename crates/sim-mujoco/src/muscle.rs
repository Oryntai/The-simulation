//! Мышцы: модель MuJoCo (упрощённая модель Хилла с жёстким сухожилием), переписанная на Rust
//! и сверенная с оригиналом (`mju_muscleGain`, `mju_muscleBias`), плюс доступ к длинам,
//! плечам и силам мышц в данных MuJoCo.
//!
//! Нормированная длина мышцы L = r₀ + (ℓ − ℓ_min) / L₀, где L₀ = (ℓ_max − ℓ_min) / (r₁ − r₀),
//! ℓ — длина мышцы-сухожилия, [ℓ_min, ℓ_max] — допустимый диапазон (lengthrange),
//! [r₀, r₁] — рабочий диапазон нормированной длины (по умолчанию 0.75…1.05).
//! Сила = −F₀·FL(L)·FV(V)·a − F₀·FP(L) (знак минус: мышца тянет, укорачивая путь).
//! Документация: https://mujoco.readthedocs.io/en/stable/modeling.html#muscles

use crate::{Sim, ffi};

/// Параметры мышцы MuJoCo (gaintype = muscle).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MuscleModel {
    pub lengthrange: [f64; 2],
    pub acc0: f64,
    /// range₀, range₁, force, scale, lmin, lmax, vmax, fpmax, fvmax
    pub prm: [f64; 9],
}

impl MuscleModel {
    /// Пиковая активная сила F₀, Н (если в модели не задана — scale / acc0).
    pub fn f0(&self) -> f64 {
        if self.prm[2] < 0.0 {
            self.prm[3] / self.acc0
        } else {
            self.prm[2]
        }
    }

    /// Длина, соответствующая нормированной единице (оптимальная длина в терминах MuJoCo), м.
    pub fn l0(&self) -> f64 {
        (self.lengthrange[1] - self.lengthrange[0]) / (self.prm[1] - self.prm[0])
    }

    pub fn norm_length(&self, len: f64) -> f64 {
        self.prm[0] + (len - self.lengthrange[0]) / self.l0()
    }

    pub fn norm_velocity(&self, vel: f64) -> f64 {
        vel / (self.l0() * self.prm[6])
    }

    /// Активная сила при единичной активации (со знаком MuJoCo, ≤ 0), Н.
    pub fn gain(&self, len: f64, vel: f64) -> f64 {
        let l = self.norm_length(len);
        let v = self.norm_velocity(vel);
        -self.f0() * fl(l, self.prm[4], self.prm[5]) * fv(v, self.prm[8])
    }

    /// Пассивная сила (со знаком MuJoCo, ≤ 0), Н.
    pub fn bias(&self, len: f64) -> f64 {
        -self.f0() * fp(self.norm_length(len), self.prm[5], self.prm[7])
    }

    /// То же, что `gain`, но посчитанное самим MuJoCo — для сверки.
    pub fn gain_mujoco(&self, len: f64, vel: f64) -> f64 {
        unsafe {
            ffi::mju_muscleGain(
                len,
                vel,
                self.lengthrange.as_ptr(),
                self.acc0,
                self.prm.as_ptr(),
            )
        }
    }

    pub fn bias_mujoco(&self, len: f64) -> f64 {
        unsafe { ffi::mju_muscleBias(len, self.lengthrange.as_ptr(), self.acc0, self.prm.as_ptr()) }
    }
}

/// Сила–длина (активная): гладкий «колокол» из четырёх парабол, 0 вне (lmin, lmax), 1 при L = 1.
pub fn fl(l: f64, lmin: f64, lmax: f64) -> f64 {
    let a = 0.5 * (lmin + 1.0);
    let b = 0.5 * (1.0 + lmax);
    if l <= lmin || l >= lmax {
        0.0
    } else if l <= a {
        let x = (l - lmin) / (a - lmin);
        0.5 * x * x
    } else if l <= 1.0 {
        let x = (1.0 - l) / (1.0 - a);
        1.0 - 0.5 * x * x
    } else if l <= b {
        let x = (l - 1.0) / (b - 1.0);
        1.0 - 0.5 * x * x
    } else {
        let x = (lmax - l) / (lmax - b);
        0.5 * x * x
    }
}

/// Сила–скорость: 0 при V ≤ −1 (максимальная скорость укорочения), 1 при V = 0,
/// до fvmax при удлинении.
pub fn fv(v: f64, fvmax: f64) -> f64 {
    let c = fvmax - 1.0;
    if v <= -1.0 {
        0.0
    } else if v <= 0.0 {
        (v + 1.0) * (v + 1.0)
    } else if v <= c {
        fvmax - (c - v) * (c - v) / c
    } else {
        fvmax
    }
}

/// Пассивная сила–длина: 0 до L = 1, полупарабола до b = (1 + lmax)/2, дальше — прямая
/// с тем же наклоном. (Первая версия по памяти была кубической — тест против `mju_muscleBias`
/// поймал расхождение до 80% F₀; формула исправлена по поведению оригинала.)
pub fn fp(l: f64, lmax: f64, fpmax: f64) -> f64 {
    let b = 0.5 * (1.0 + lmax);
    if l <= 1.0 {
        0.0
    } else if l <= b {
        let x = (l - 1.0) / (b - 1.0);
        0.5 * fpmax * x * x
    } else {
        let x = (l - b) / (b - 1.0);
        fpmax * (0.5 + x)
    }
}

impl Sim {
    fn out_adr(&self, actuator: usize) -> usize {
        Self::arr(self.model().actuator_outadr, self.nactuator())[actuator] as usize
    }

    fn nout(&self) -> usize {
        self.model().nout as usize
    }

    pub fn muscle_model(&self, actuator: usize) -> MuscleModel {
        let m = self.model();
        let o = self.out_adr(actuator);
        let lr = Self::arr(m.actuator_lengthrange, 2 * self.nout());
        let acc0 = Self::arr(m.actuator_acc0, self.nout());
        let gp = Self::arr(m.actuator_gainprm, ffi::mjNGAIN as usize * self.nactuator());
        let k = ffi::mjNGAIN as usize * actuator;
        let mut prm = [0.0; 9];
        prm.copy_from_slice(&gp[k..k + 9]);
        MuscleModel {
            lengthrange: [lr[2 * o], lr[2 * o + 1]],
            acc0: acc0[o],
            prm,
        }
    }

    /// Длина мышцы-сухожилия в текущей позе, м.
    pub fn actuator_length(&self, actuator: usize) -> f64 {
        Self::arr(self.data().actuator_length, self.nout())[self.out_adr(actuator)]
    }

    /// Сила привода (для мышцы — отрицательна: тянет), Н.
    pub fn actuator_force(&self, actuator: usize) -> f64 {
        Self::arr(self.data().actuator_force, self.nout())[self.out_adr(actuator)]
    }

    pub fn qfrc_actuator(&self) -> &[f64] {
        Self::arr(self.data().qfrc_actuator, self.nv())
    }

    /// Активация мышцы (0…1); действует после `forward`.
    pub fn set_activation(&mut self, actuator: usize, a: f64) {
        let adr = Self::arr(self.model().actuator_actadr, self.nactuator())[actuator];
        if adr >= 0 {
            let na = self.model().na as usize;
            unsafe { std::slice::from_raw_parts_mut((*self.data_ptr()).act, na)[adr as usize] = a };
        }
    }

    pub fn clear_activations(&mut self) {
        let na = self.model().na as usize;
        unsafe { std::slice::from_raw_parts_mut((*self.data_ptr()).act, na).fill(0.0) };
    }

    /// Строка якобиана сухожилия: (степень свободы, dℓ/dq). Плечо мышцы = −dℓ/dq.
    pub fn tendon_jacobian(&self, tendon: usize) -> Vec<(usize, f64)> {
        let m = self.model();
        let nt = self.ntendon();
        let nnz = Self::arr(m.ten_J_rownnz, nt)[tendon] as usize;
        let adr = Self::arr(m.ten_J_rowadr, nt)[tendon] as usize;
        let col = Self::arr(m.ten_J_colind, m.nJten as usize);
        let val = Self::arr(self.data().ten_J, m.nJten as usize);
        (adr..adr + nnz)
            .map(|k| (col[k] as usize, val[k]))
            .collect()
    }

    /// Центр масс тела в мире.
    pub fn body_com(&self, body: usize) -> [f64; 3] {
        let p = Self::arr(self.data().xipos, 3 * self.nbody());
        [p[3 * body], p[3 * body + 1], p[3 * body + 2]]
    }

    pub fn body_parent(&self, body: usize) -> usize {
        Self::arr(self.model().body_parentid, self.nbody())[body] as usize
    }

    /// Признак «тело лежит в поддереве `root`» для всех тел.
    pub fn subtree_mask(&self, root: usize) -> Vec<bool> {
        (0..self.nbody())
            .map(|b| {
                let mut x = b;
                loop {
                    if x == root {
                        return true;
                    }
                    if x == 0 {
                        return false;
                    }
                    x = self.body_parent(x);
                }
            })
            .collect()
    }

    /// Точки пути мышцы с телами, к которым они относятся (для расчёта сил на кости).
    /// Точки идут в том же порядке, что `tendon_segments`; шкивы разрывают путь (`None`).
    pub fn tendon_path_bodies(&self, tendon: usize) -> Vec<Option<([f64; 3], usize)>> {
        let m = self.model();
        let d = self.data();
        let nt = self.ntendon();
        let nwrap = m.nwrap as usize;
        // Объекты пути в модели: тип и id (сайт или геом).
        let madr = Self::arr(m.tendon_adr, nt)[tendon] as usize;
        let mnum = Self::arr(m.tendon_num, nt)[tendon] as usize;
        let wtype = Self::arr(m.wrap_type, nwrap);
        let wid = Self::arr(m.wrap_objid, nwrap);
        let site_body = Self::arr(m.site_bodyid, m.nsite as usize);
        let geom_body = Self::arr(m.geom_bodyid, self.ngeom());
        let sites: Vec<usize> = (madr..madr + mnum)
            .filter(|&k| wtype[k] as u32 == ffi::mjtWrap::mjWRAP_SITE)
            .map(|k| wid[k] as usize)
            .collect();
        let adr = Self::arr(d.ten_wrapadr, nt)[tendon] as usize;
        let num = Self::arr(d.ten_wrapnum, nt)[tendon] as usize;
        let obj = Self::arr(d.wrap_obj, 2 * nwrap);
        let xpos = Self::arr(d.wrap_xpos, 6 * nwrap);
        let mut next_site = 0usize;
        let mut out = Vec::with_capacity(num);
        for j in adr..adr + num {
            let p = [xpos[3 * j], xpos[3 * j + 1], xpos[3 * j + 2]];
            match obj[j] {
                -2 => out.push(None),
                -1 => {
                    let s = sites.get(next_site).copied();
                    next_site += 1;
                    out.push(s.map(|s| (p, site_body[s] as usize)));
                }
                g => out.push(Some((p, geom_body[g as usize] as usize))),
            }
        }
        out
    }

    pub(crate) fn data_ptr(&self) -> *mut ffi::mjData {
        self.d
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn curves_have_expected_landmarks() {
        assert!((fl(1.0, 0.5, 1.6) - 1.0).abs() < 1e-15);
        assert_eq!(fl(0.5, 0.5, 1.6), 0.0);
        assert!((fv(0.0, 1.2) - 1.0).abs() < 1e-15);
        assert_eq!(fv(-1.0, 1.2), 0.0);
        assert_eq!(fp(1.0, 1.6, 1.3), 0.0);
    }
}
