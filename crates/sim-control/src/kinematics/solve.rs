use super::math::{finite, norm, sub};
use super::{
    CoordinateLimit, Jacobian, Joint, KinematicError, LinkId, Pose, Result, TaskFrame, Tree, Vec3,
    twist,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TaskSpace {
    Position,
    Pose,
}

impl TaskSpace {
    fn rows(self) -> usize {
        match self {
            Self::Position => 3,
            Self::Pose => 6,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct RankOptions {
    pub space: TaskSpace,
    /// Вес линейных строк, обратная характерная длина задачи.
    pub position_scale: f64,
    /// Вес угловых строк, обратный характерный угол задачи.
    pub angle_scale: f64,
    /// Порог относительно максимальной исходной нормы строки.
    pub relative_tolerance: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rank {
    pub rank: usize,
    pub task_dimension: usize,
    pub movable_coordinates: usize,
}

impl Rank {
    /// Можно ли локально изменять все компоненты выбранной задачи.
    pub fn full_task_rank(self) -> bool {
        self.rank == self.task_dimension
    }
    /// Вырождение относительно максимально возможного ранга по числу строк и столбцов.
    pub fn rank_deficient(self) -> bool {
        self.rank < self.task_dimension.min(self.movable_coordinates)
    }
}

#[derive(Clone, Copy, Debug)]
pub enum IkTarget {
    Position(Vec3),
    Pose(Pose),
}

impl IkTarget {
    fn space(self) -> TaskSpace {
        match self {
            Self::Position(_) => TaskSpace::Position,
            Self::Pose(_) => TaskSpace::Pose,
        }
    }

    fn translation(self) -> Vec3 {
        match self {
            Self::Position(p) => p,
            Self::Pose(p) => p.translation(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct IkOptions {
    pub position_tolerance_m: f64,
    pub angle_tolerance_rad: f64,
    /// Веса ошибки. Выравнивают масштабы метров и радиан.
    pub position_scale: f64,
    pub angle_scale: f64,
    pub damping: f64,
    /// Верхняя граница приращения за итерацию в единицах каждой координаты.
    pub max_coordinate_step: f64,
    pub max_iterations: usize,
    pub max_line_search: usize,
}

#[derive(Clone, Copy, Debug)]
pub struct IkResidual {
    pub position_m: f64,
    pub angle_rad: Option<f64>,
}

#[derive(Clone, Copy, Debug)]
pub struct ReachBound {
    /// Верхняя граница расстояния от начала выбранной системы отсчёта.
    pub upper_distance_m: f64,
    /// Нижняя граница расстояния до цели по максимуму модулей компонент.
    pub target_lower_distance_m: f64,
}

#[derive(Clone, Debug)]
pub enum IkResult {
    Solved {
        coordinates: Vec<f64>,
        residual: IkResidual,
        iterations: usize,
    },
    Unreachable(ReachBound),
    NoConvergence {
        best_coordinates: Vec<f64>,
        residual: IkResidual,
        iterations: usize,
    },
}

#[derive(Clone, Copy, Debug)]
pub struct GridAxis {
    pub min: f64,
    pub max: f64,
    /// Число равномерных интервалов. Для фиксированной точки задаётся ноль.
    pub subdivisions: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WorkspaceMethod {
    /// Декартово произведение узлов сетки, включая концы каждой оси.
    UniformCoordinateGrid,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WorkspaceCoverage {
    /// Только перечисленные конфигурации. Промежутки и точки вне сетки не обследованы.
    SampledConfigurationsOnly,
}

#[derive(Clone, Debug)]
pub struct WorkspacePoint {
    pub coordinates: Vec<f64>,
    pub pose: Pose,
}

#[derive(Clone, Debug)]
pub struct Workspace {
    pub task: TaskFrame,
    pub method: WorkspaceMethod,
    pub coverage: WorkspaceCoverage,
    pub axes: Vec<GridAxis>,
    pub points: Vec<WorkspacePoint>,
}

impl Tree {
    /// Численный ранг по координатам ветвей задачи, без жёстко зафиксированных координат.
    /// Односторонние ограничения на активной границе анализируются отдельно.
    pub fn rank(&self, q: &[f64], task: TaskFrame, options: RankOptions) -> Result<Rank> {
        if !positive(options.position_scale)
            || !positive(options.angle_scale)
            || !positive(options.relative_tolerance)
            || options.relative_tolerance >= 1.0
        {
            return Err(KinematicError::Invalid(
                "неверные веса или порог ранга".into(),
            ));
        }
        let jacobian = self.jacobian(q, task)?;
        let rows = options.space.rows();
        let weights = weights(options.position_scale, options.angle_scale);
        let paths = self.task_paths(task)?;
        let movable: Vec<_> = paths
            .links()
            .flat_map(|id| self.link_range(id))
            .filter(|&index| !self.limits[index].locked())
            .collect();
        let matrix = (0..rows)
            .map(|row| {
                movable
                    .iter()
                    .map(|&index| jacobian.columns[index][row] * weights[row])
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        Ok(Rank {
            rank: row_rank(matrix, options.relative_tolerance)?,
            task_dimension: rows,
            movable_coordinates: movable.len(),
        })
    }

    /// Локальный поиск с демпфированием, проекцией на пределы и контролем фактической ошибки.
    /// Недостижимость выдаётся только при наличии консервативной геометрической границы.
    pub fn inverse(
        &self,
        task: TaskFrame,
        target: IkTarget,
        initial: &[f64],
        options: IkOptions,
    ) -> Result<IkResult> {
        let scales = weights(options.position_scale, options.angle_scale);
        if !finite(&target.translation())
            || ![
                options.position_tolerance_m,
                options.angle_tolerance_rad,
                options.position_scale,
                options.angle_scale,
                options.damping,
                options.max_coordinate_step,
            ]
            .into_iter()
            .all(positive)
            || !positive(options.damping * options.damping)
            || options.max_iterations == 0
            || options.max_line_search == 0
        {
            return Err(KinematicError::Invalid(
                "неверная цель или параметры обратной задачи".into(),
            ));
        }
        let mut coordinates = initial.to_vec();
        let (pose, mut jacobian) = self.pose_and_jacobian(&coordinates, task)?;
        let mut current = measure(pose, target, &scales)?;
        if let Some(bound) = self.reach_bound(task, target.translation())
            && bound.target_lower_distance_m
                > (bound.upper_distance_m + options.position_tolerance_m).next_up()
        {
            return Ok(IkResult::Unreachable(bound));
        }

        for iterations in 0..=options.max_iterations {
            if current.residual.position_m <= options.position_tolerance_m
                && current
                    .residual
                    .angle_rad
                    .is_none_or(|angle| angle <= options.angle_tolerance_rad)
            {
                return Ok(IkResult::Solved {
                    coordinates,
                    residual: current.residual,
                    iterations,
                });
            }
            if iterations < options.max_iterations {
                let step = self.damped_step(
                    &jacobian,
                    &current.vector,
                    &scales,
                    target.space().rows(),
                    options.damping,
                )?;
                if let Some((new_coordinates, measured)) = search(
                    self,
                    &coordinates,
                    &step,
                    &current,
                    &Search {
                        task,
                        target,
                        scales,
                        options: &options,
                    },
                )? {
                    coordinates = new_coordinates;
                    current = measured;
                    jacobian = self.jacobian(&coordinates, task)?;
                    continue;
                }
                // При упоре в ограничение проекция шага может мешать другим координатам.
                // Проецированный градиент даёт запасное направление у границы.
                let gradient = projected_gradient(&jacobian, &current.vector, &scales)?;
                if let Some((new_coordinates, measured)) = search(
                    self,
                    &coordinates,
                    &gradient,
                    &current,
                    &Search {
                        task,
                        target,
                        scales,
                        options: &options,
                    },
                )? {
                    coordinates = new_coordinates;
                    current = measured;
                    jacobian = self.jacobian(&coordinates, task)?;
                    continue;
                }
            }
            return Ok(IkResult::NoConvergence {
                best_coordinates: coordinates,
                residual: current.residual,
                iterations,
            });
        }
        unreachable!("цикл завершает каждую последнюю итерацию")
    }

    fn damped_step(
        &self,
        jacobian: &Jacobian,
        error: &[f64; 6],
        scales: &[f64; 6],
        rows: usize,
        damping: f64,
    ) -> Result<Vec<f64>> {
        let mut normal = [[0.0; 6]; 6];
        let columns: Vec<_> = jacobian
            .columns
            .iter()
            .zip(&self.limits)
            .map(|(column, limit)| {
                std::array::from_fn::<_, 6, _>(|i| {
                    if i < rows && !limit.locked() {
                        column[i] * scales[i]
                    } else {
                        0.0
                    }
                })
            })
            .collect();
        for row in 0..rows {
            for col in 0..rows {
                normal[row][col] = columns.iter().map(|j| j[row] * j[col]).sum();
            }
            normal[row][row] += damping * damping;
        }
        let dual = solve_linear(normal, *error, rows)?;
        let step: Vec<f64> = columns
            .iter()
            .map(|j| j.iter().zip(dual).map(|(a, b)| a * b).sum())
            .collect();
        if !finite(&step) {
            return Err(KinematicError::Numerical(
                "переполнение шага обратной задачи".into(),
            ));
        }
        Ok(step)
    }

    fn reach_bound(&self, task: TaskFrame, target: Vec3) -> Option<ReachBound> {
        let paths = self.task_paths(task).ok()?;
        let mut radius = upper_add(
            upper_l1(task.site.offset.translation()),
            upper_l1(paths.reference_offset.translation()),
        );
        for link in paths.links() {
            radius = upper_add(radius, self.link_bound(link)?);
        }
        if !radius.is_finite() {
            return None;
        }
        let target_lower_distance_m = target.into_iter().map(f64::abs).fold(0.0, f64::max);
        Some(ReachBound {
            upper_distance_m: radius,
            target_lower_distance_m,
        })
    }

    fn link_bound(&self, id: LinkId) -> Option<f64> {
        let link = &self.links[id.0];
        let max_abs = |limit: CoordinateLimit| Some(limit.min?.abs().max(limit.max?.abs()));
        let motion = match link.joint {
            // Норма коллинеарного смещения не превосходит |q| для единичной оси.
            Joint::Slide { limit, .. } => max_abs(limit)?,
            Joint::Free { translation, .. } => {
                let [x, y, z] = translation;
                upper_l1([max_abs(x)?, max_abs(y)?, max_abs(z)?])
            }
            Joint::Fixed | Joint::Hinge { .. } | Joint::Ball { .. } => 0.0,
        };
        Some(upper_add(
            upper_add(upper_l1(link.parent_to_joint.translation()), motion),
            upper_l1(link.joint_to_link.translation()),
        ))
    }

    /// Пользователь задаёт область сканирования, разрешение и бюджет точек явно.
    pub fn workspace(
        &self,
        task: TaskFrame,
        axes: &[GridAxis],
        max_points: usize,
    ) -> Result<Workspace> {
        if axes.len() != self.limits.len() || max_points == 0 {
            return Err(KinematicError::Invalid(
                "неверный размер сетки или её бюджет".into(),
            ));
        }
        let mut count = 1_usize;
        for (axis, limit) in axes.iter().zip(&self.limits) {
            if !limit.contains(axis.min)
                || !limit.contains(axis.max)
                || axis.min > axis.max
                || (axis.min == axis.max) != (axis.subdivisions == 0)
                || !(axis.max - axis.min).is_finite()
            {
                return Err(KinematicError::Invalid(
                    "неверная область или разбиение сетки".into(),
                ));
            }
            count = (axis.subdivisions as usize)
                .checked_add(1)
                .and_then(|n| count.checked_mul(n))
                .filter(|&n| n <= max_points)
                .ok_or_else(|| KinematicError::Invalid("сетка превышает бюджет точек".into()))?;
        }
        let mut points = Vec::new();
        points
            .try_reserve_exact(count)
            .map_err(|_| KinematicError::Invalid("не удалось выделить память под сетку".into()))?;
        let mut indices = vec![0_u32; axes.len()];
        for _ in 0..count {
            let coordinates: Vec<_> = axes
                .iter()
                .zip(&indices)
                .map(|(axis, &i)| {
                    if i == 0 {
                        axis.min
                    } else if i == axis.subdivisions {
                        axis.max
                    } else {
                        ((axis.max - axis.min) * (f64::from(i) / f64::from(axis.subdivisions))
                            + axis.min)
                            .clamp(axis.min, axis.max)
                    }
                })
                .collect();
            points.push(WorkspacePoint {
                pose: self.pose(&coordinates, task)?,
                coordinates,
            });
            for (index, axis) in indices.iter_mut().zip(axes).rev() {
                if *index < axis.subdivisions {
                    *index += 1;
                    break;
                }
                *index = 0;
            }
        }
        Ok(Workspace {
            task,
            method: WorkspaceMethod::UniformCoordinateGrid,
            coverage: WorkspaceCoverage::SampledConfigurationsOnly,
            axes: axes.to_vec(),
            points,
        })
    }
}

// Верхние границы считаются по неравенству треугольника, L1 ≥ L2, с округлением вверх.
fn upper_l1(v: Vec3) -> f64 {
    v.into_iter().map(f64::abs).fold(0.0, upper_add)
}
fn upper_add(a: f64, b: f64) -> f64 {
    if a == 0.0 || b == 0.0 {
        a + b
    } else {
        (a + b).next_up()
    }
}

fn positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

fn row_rank(mut rows: Vec<Vec<f64>>, relative_tolerance: f64) -> Result<usize> {
    if !rows.iter().all(|r| finite(r)) {
        return Err(KinematicError::Numerical(
            "переполнение масштаба якобиана".into(),
        ));
    }
    let largest = rows.iter().flatten().map(|x| x.abs()).fold(0.0, f64::max);
    if largest == 0.0 {
        return Ok(0);
    }
    rows.iter_mut().flatten().for_each(|x| *x /= largest);
    let mut best = rows.iter().map(|row| length(row)).fold(0.0, f64::max);
    let threshold = relative_tolerance * best;
    let mut rank = 0;
    let max_rank = rows[0].len().min(rows.len());
    // Модифицированный Грам — Шмидт с перестановкой строк и повторной ортогонализацией.
    while rank < max_rank && best > threshold {
        let Some(index) = rows.iter().position(|row| length(row) == best) else {
            break;
        };
        let mut basis = rows.swap_remove(index);
        basis.iter_mut().for_each(|x| *x /= best);
        for row in &mut rows {
            for _ in 0..2 {
                let projection = row.iter().zip(&basis).map(|(a, b)| a * b).sum::<f64>();
                for (a, b) in row.iter_mut().zip(&basis) {
                    *a -= b * projection;
                }
            }
        }
        rank += 1;
        best = rows.iter().map(|row| length(row)).fold(0.0, f64::max);
    }
    Ok(rank)
}

fn length(v: &[f64]) -> f64 {
    v.iter().fold(0.0, |norm, &x| norm.hypot(x))
}

fn weights(position: f64, angle: f64) -> [f64; 6] {
    [position, position, position, angle, angle, angle]
}

struct Measurement {
    vector: [f64; 6],
    residual: IkResidual,
    cost: f64,
}

fn measure(pose: Pose, target: IkTarget, scales: &[f64; 6]) -> Result<Measurement> {
    let position = sub(target.translation(), pose.translation());
    let angle = match target {
        IkTarget::Position(_) => None,
        IkTarget::Pose(target) => {
            let target_rotation = Pose::new([0.0; 3], target.quaternion())?;
            let rotation = Pose::new([0.0; 3], pose.quaternion())?;
            Some(
                target_rotation
                    .compose(rotation.inverse()?)?
                    .rotation_vector(),
            )
        }
    };
    let residual = IkResidual {
        position_m: norm(position),
        angle_rad: angle.map(norm),
    };
    let mut vector = twist(position, angle.unwrap_or([0.0; 3]));
    for (value, scale) in vector.iter_mut().zip(scales) {
        *value *= scale;
    }
    let cost = length(&vector);
    if ![residual.position_m, cost].into_iter().all(f64::is_finite) {
        return Err(KinematicError::Numerical(
            "переполнение ошибки обратной задачи".into(),
        ));
    }
    Ok(Measurement {
        vector,
        residual,
        cost,
    })
}

fn solve_linear(mut matrix: [[f64; 6]; 6], mut rhs: [f64; 6], n: usize) -> Result<[f64; 6]> {
    let failed = || {
        KinematicError::Numerical("вырожденная или переполненная система обратной задачи".into())
    };
    for col in 0..n {
        let pivot = (col..n)
            .max_by(|&a, &b| matrix[a][col].abs().total_cmp(&matrix[b][col].abs()))
            .ok_or_else(failed)?;
        matrix.swap(col, pivot);
        rhs.swap(col, pivot);
        let diagonal = matrix[col][col];
        if !positive(diagonal.abs()) {
            return Err(failed());
        }
        let pivot_row = matrix[col];
        for row in col + 1..n {
            let multiplier = matrix[row][col] / diagonal;
            for (next, &p) in matrix[row][col + 1..n]
                .iter_mut()
                .zip(&pivot_row[col + 1..n])
            {
                *next -= multiplier * p;
            }
            rhs[row] -= multiplier * rhs[col];
        }
    }
    let mut answer = [0.0; 6];
    for row in (0..n).rev() {
        let known = matrix[row][row + 1..n]
            .iter()
            .zip(&answer[row + 1..n])
            .map(|(a, b)| a * b)
            .sum::<f64>();
        answer[row] = (rhs[row] - known) / matrix[row][row];
    }
    if !finite(&answer) {
        return Err(failed());
    }
    Ok(answer)
}

fn projected_gradient(
    jacobian: &Jacobian,
    error: &[f64; 6],
    scales: &[f64; 6],
) -> Result<Vec<f64>> {
    let result: Vec<f64> = jacobian
        .columns
        .iter()
        .map(|column| {
            column
                .iter()
                .zip(error)
                .zip(scales)
                .map(|((v, e), s)| v * s * e)
                .sum()
        })
        .collect();
    if !finite(&result) {
        return Err(KinematicError::Numerical("переполнение градиента".into()));
    }
    Ok(result)
}

struct Search<'a> {
    task: TaskFrame,
    target: IkTarget,
    scales: [f64; 6],
    options: &'a IkOptions,
}

fn search(
    tree: &Tree,
    q: &[f64],
    direction: &[f64],
    current: &Measurement,
    search: &Search<'_>,
) -> Result<Option<(Vec<f64>, Measurement)>> {
    let mut direction = direction.to_vec();
    for ((d, &x), limit) in direction.iter_mut().zip(q).zip(&tree.limits) {
        if (Some(x) == limit.min && *d < 0.0) || (Some(x) == limit.max && *d > 0.0) {
            *d = 0.0;
        }
    }
    let greatest = direction.iter().map(|x| x.abs()).fold(0.0, f64::max);
    if greatest == 0.0 {
        return Ok(None);
    }
    let mut step = greatest.min(search.options.max_coordinate_step);
    for _ in 0..search.options.max_line_search {
        let trial: Vec<_> = q
            .iter()
            .zip(&direction)
            .zip(&tree.limits)
            .map(|((&value, &d), limit)| limit.project(value + (d / greatest) * step))
            .collect();
        if !finite(&trial) {
            return Err(KinematicError::Numerical(
                "переполнение пробных координат".into(),
            ));
        }
        if trial == q {
            break;
        }
        let measured = measure(
            tree.pose(&trial, search.task)?,
            search.target,
            &search.scales,
        )?;
        if measured.cost < current.cost {
            return Ok(Some((trial, measured)));
        }
        step *= 0.5;
    }
    Ok(None)
}
