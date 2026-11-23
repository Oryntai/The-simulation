use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, TAU};

use sim_control::kinematics::*;
use sim_signal::Time;

const UNBOUNDED: CoordinateLimit = CoordinateLimit {
    min: None,
    max: None,
};
const X: Vec3 = [1., 0., 0.];
const Y: Vec3 = [0., 1., 0.];
const Z: Vec3 = [0., 0., 1.];

fn limit(min: f64, max: f64) -> CoordinateLimit {
    CoordinateLimit {
        min: Some(min),
        max: Some(max),
    }
}

fn translation(t: Vec3) -> Pose {
    Pose::new(t, [1., 0., 0., 0.]).unwrap()
}

fn link(name: &str, parent: Option<usize>, joint: Joint) -> Link {
    Link {
        name: name.into(),
        parent: parent.map(LinkId),
        joint,
        parent_to_joint: Pose::IDENTITY,
        joint_to_link: Pose::IDENTITY,
    }
}

fn task(link: usize, offset: Pose, reference: Frame) -> TaskFrame {
    TaskFrame {
        site: Site {
            link: LinkId(link),
            offset,
        },
        reference,
    }
}

fn close<const N: usize>(got: [f64; N], expected: [f64; N], tolerance: f64) {
    for (actual, expected) in got.into_iter().zip(expected) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual:.16e} != {expected:.16e}, допуск {tolerance}"
        );
    }
}

type Matrix<const N: usize> = [[f64; N]; N];

fn multiply<const N: usize>(a: Matrix<N>, b: Matrix<N>) -> Matrix<N> {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..N).map(|k| a[i][k] * b[k][j]).sum()))
}

fn transpose<const N: usize>(a: Matrix<N>) -> Matrix<N> {
    std::array::from_fn(|i| std::array::from_fn(|j| a[j][i]))
}

fn homogeneous(r: Matrix<3>, t: Vec3) -> Matrix<4> {
    std::array::from_fn(|i| {
        std::array::from_fn(|j| match (i, j) {
            (3, 3) => 1.,
            (3, _) => 0.,
            (_, 3) => t[i],
            _ => r[i][j],
        })
    })
}

fn rotation(p: Pose) -> Matrix<3> {
    // Независимая формула: действительная часть q⊗(0,eᵢ)⊗q* равна нулю.
    let [w, a, b, c] = p.quaternion();
    let q = [a, b, c];
    let skew = [[0., -c, b], [c, 0., -a], [-b, a, 0.]];
    let scalar = w * w - a * a - b * b - c * c;
    std::array::from_fn(|i| {
        std::array::from_fn(|j| {
            2. * q[i] * q[j] + 2. * w * skew[i][j] + if i == j { scalar } else { 0. }
        })
    })
}

#[test]
fn forward_agrees_with_independent_homogeneous_multiplication() {
    // Фикстура: известные повороты на 90° вокруг X/Z, произвольные смещения в метрах.
    let rx = [[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]];
    let rz = [[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]];
    let i = [X, Y, Z];
    let mut carriage = link(
        "каретка",
        Some(3),
        Joint::Slide {
            axis: [0., 4., 0.],
            limit: limit(-0.5, 1.),
        },
    );
    carriage.parent_to_joint = translation([0.2, 0.3, 0.4]);
    let mut block = link("блок", Some(2), Joint::Fixed);
    block.parent_to_joint = translation([-1., 0., 0.]);
    let mut root = link("основание", None, Joint::Fixed);
    root.parent_to_joint = Pose::new([1., 2., 3.], [FRAC_1_SQRT_2, 0., 0., FRAC_1_SQRT_2]).unwrap();
    let mut rod = link(
        "стержень",
        Some(2),
        Joint::Hinge {
            axis: [0., 0., 2.],
            limit: UNBOUNDED,
        },
    );
    rod.parent_to_joint = Pose::new([2., -1., 0.], [FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0., 0.]).unwrap();
    rod.joint_to_link = translation([0.75, 0., 0.]);
    let tree = Tree::new(vec![carriage, block, root, rod]).unwrap();
    assert_eq!(tree.coordinates_for(LinkId(2)).unwrap(), 1..1);
    assert_eq!(tree.coordinates_for(LinkId(3)).unwrap(), 1..2);

    let root = homogeneous(rz, [1., 2., 3.]);
    let rod = [
        root,
        homogeneous(rx, [2., -1., 0.]),
        homogeneous(rz, [0.; 3]),
        homogeneous(i, [0.75, 0., 0.]),
    ]
    .into_iter()
    .reduce(multiply)
    .unwrap();
    let carriage = multiply(
        multiply(rod, homogeneous(i, [0.2, 0.3, 0.4])),
        homogeneous(i, [0., 0.5, 0.]),
    );
    let block = multiply(root, homogeneous(i, [-1., 0., 0.]));
    let poses = tree.forward(&[0.5, FRAC_PI_2]).unwrap();
    let mut max_error: f64 = 0.;
    for (p, expected) in poses.iter().zip([carriage, block, root, rod]) {
        let matrix = homogeneous(rotation(*p), p.translation());
        for (row, expected) in matrix.into_iter().zip(expected) {
            for (a, b) in row.into_iter().zip(expected) {
                max_error = max_error.max((a - b).abs());
            }
            // Допуск на накопление округлений при нескольких преобразованиях.
            close(row, expected, 1e-13);
        }
    }
    println!("Максимальная ошибка матрицы прямой задачи {max_error:.3e}");
}

fn branched_tree() -> Tree {
    let mut free = link(
        "основание",
        None,
        Joint::Free {
            translation: [UNBOUNDED; 3],
            rotation: [UNBOUNDED; 3],
        },
    );
    free.parent_to_joint = Pose::new([0.3, -0.5, 0.2], [0.5, 0.5, 0.5, 0.5]).unwrap();
    free.joint_to_link = translation([0., 0.3, 0.3]);
    let mut hinge = link(
        "стержень",
        Some(0),
        Joint::Hinge {
            axis: [2., -1., 2.],
            limit: UNBOUNDED,
        },
    );
    hinge.parent_to_joint = translation([0., 1., 0.]);
    let mut ball = link(
        "блок",
        Some(1),
        Joint::Ball {
            rotation: [UNBOUNDED; 3],
        },
    );
    ball.joint_to_link = translation([0., 1., 0.5]);
    let mut carriage = link(
        "каретка",
        Some(0),
        Joint::Slide {
            axis: [0., 1., -1.],
            limit: UNBOUNDED,
        },
    );
    carriage.parent_to_joint =
        Pose::new([0.4, 0., 0.], [FRAC_1_SQRT_2, 0., FRAC_1_SQRT_2, 0.]).unwrap();
    carriage.joint_to_link = translation([0., 1., 0.]);
    Tree::new(vec![free, hinge, ball, carriage]).unwrap()
}

#[test]
fn both_jacobians_match_central_differences_for_all_joint_types() {
    let tree = branched_tree();
    let q = [0.2, -0.1, 0.4, 0.2, -0.3, 0.4, 0.45, -0.2, 0.1, 0.7, 0.3];
    let site = translation([0.2, 0.5, 0.1]);
    // Шаг порядка ε^(1/3) уравновешивает ошибку округления и остаток центральной разности.
    let h = f64::EPSILON.cbrt();
    let mut max_error: f64 = 0.;
    for site_link in [2, 3] {
        for reference in [Frame::World, Frame::Link(LinkId(3))] {
            let task = task(site_link, site, reference);
            let jacobian = tree.jacobian(&q, task).unwrap();
            if let Frame::Link(_) = reference {
                assert_eq!(
                    jacobian.columns[..6],
                    [[0.; 6]; 6],
                    "общее движение отменяется"
                );
            }
            for (j, column) in jacobian.columns.iter().enumerate() {
                let mut lo = q;
                let mut hi = q;
                lo[j] -= h;
                hi[j] += h;
                let a = tree.pose(&lo, task).unwrap();
                let b = tree.pose(&hi, task).unwrap();
                let dr = multiply(rotation(b), transpose(rotation(a)));
                let dp = std::array::from_fn::<_, 3, _>(|i| {
                    (b.translation()[i] - a.translation()[i]) / (2. * h)
                });
                let dw = [
                    (dr[2][1] - dr[1][2]) / (4. * h),
                    (dr[0][2] - dr[2][0]) / (4. * h),
                    (dr[1][0] - dr[0][1]) / (4. * h),
                ];
                let [x, y, z] = dp;
                let [rx, ry, rz] = dw;
                let expected = [x, y, z, rx, ry, rz];
                for (a, b) in column.iter().zip(expected) {
                    max_error = max_error.max((a - b).abs());
                }
                close(*column, expected, 2e-8);
            }
        }
    }
    println!("Максимальная ошибка столбца по центральным разностям {max_error:.3e}");
}

fn ik_options() -> IkOptions {
    IkOptions {
        position_tolerance_m: 1e-9,
        angle_tolerance_rad: 1e-9,
        position_scale: 1.,
        angle_scale: 1.,
        damping: 0.01,
        max_coordinate_step: 0.3,
        max_iterations: 100,
        max_line_search: 15,
    }
}

fn rank_options(space: TaskSpace) -> RankOptions {
    RankOptions {
        space,
        position_scale: 1.,
        angle_scale: 1.,
        relative_tolerance: 1e-10,
    }
}

fn carriage_tree() -> Tree {
    Tree::new(vec![link(
        "каретка",
        None,
        Joint::Slide {
            axis: X,
            limit: limit(0., 1.),
        },
    )])
    .unwrap()
}

#[test]
fn inverse_position_honors_limits_and_distinguishes_unreachable_from_no_convergence() {
    let tree = carriage_tree();
    let task = task(0, Pose::IDENTITY, Frame::World);
    let options = ik_options();
    let IkResult::Solved {
        coordinates,
        residual,
        ..
    } = tree
        .inverse(task, IkTarget::Position([1., 0., 0.]), &[0.], options)
        .unwrap()
    else {
        panic!("решение на верхнем пределе")
    };
    assert!(coordinates[0] <= 1.);
    close(
        tree.pose(&coordinates, task).unwrap().translation(),
        [1., 0., 0.],
        options.position_tolerance_m,
    );
    println!(
        "Ошибка обратной задачи каретки {:.3e} м",
        residual.position_m
    );
    let IkResult::Unreachable(bound) = tree
        .inverse(task, IkTarget::Position([2., 0., 0.]), &[0.5], options)
        .unwrap()
    else {
        panic!("цель вне доказанной области")
    };
    assert_eq!(bound.upper_distance_m, 1.);
    assert_eq!(bound.target_lower_distance_m, 2.);
    let IkResult::NoConvergence {
        best_coordinates,
        residual,
        ..
    } = tree
        .inverse(task, IkTarget::Position([0.5, 0.5, 0.]), &[0.25], options)
        .unwrap()
    else {
        panic!("по этой оценке недостижимость не доказана")
    };
    close(best_coordinates.try_into().unwrap(), [0.5], 1e-8);
    close([residual.position_m], [0.5], 1e-12);
    assert!(matches!(
        tree.inverse(task, IkTarget::Position([0.; 3]), &[-0.01], options),
        Err(KinematicError::JointLimit { coordinate: 0 })
    ));
}

#[test]
fn inverse_pose_matches_target_after_forward_substitution() {
    let tree = branched_tree();
    let task = task(2, translation([0.2, 0., 0.]), Frame::World);
    // Цель задана независимо от прямой задачи: поворот на 90° вокруг Y.
    let target = Pose::new([0.5, -0.3, 0.7], [FRAC_1_SQRT_2, 0., FRAC_1_SQRT_2, 0.]).unwrap();
    let options = ik_options();
    let IkResult::Solved {
        coordinates,
        residual,
        ..
    } = tree
        .inverse(task, IkTarget::Pose(target), &[0.; 11], options)
        .unwrap()
    else {
        panic!("свободное основание даёт достижимую позу")
    };
    let obtained = tree.pose(&coordinates, task).unwrap();
    close(
        obtained.translation(),
        target.translation(),
        options.position_tolerance_m,
    );
    for (a, b) in rotation(obtained).into_iter().zip(rotation(target)) {
        close(a, b, options.angle_tolerance_rad);
    }
    println!(
        "Невязка позы {:.3e} м, {:.3e} рад",
        residual.position_m,
        residual.angle_rad.unwrap()
    );
    assert!(
        tree.inverse(
            task,
            IkTarget::Position([f64::NAN, 0., 0.]),
            &[0.; 11],
            options
        )
        .is_err()
    );
}

#[test]
fn rank_detects_straight_rod_coordinate_singularity_and_locked_joints() {
    let mut a = link(
        "стержень",
        None,
        Joint::Hinge {
            axis: Z,
            limit: UNBOUNDED,
        },
    );
    a.joint_to_link = translation(X);
    let mut b = a.clone();
    b.name = "второе звено".into();
    b.parent = Some(LinkId(0));
    let tree = Tree::new(vec![a, b]).unwrap();
    let site = task(1, Pose::IDENTITY, Frame::World);
    let rank = tree
        .rank(&[0., 0.], site, rank_options(TaskSpace::Position))
        .unwrap();
    assert_eq!(rank.rank, 1);
    assert!(rank.rank_deficient());
    let bent = tree
        .rank(&[0., FRAC_PI_2], site, rank_options(TaskSpace::Position))
        .unwrap();
    assert_eq!(bent.rank, 2);
    assert!(!bent.rank_deficient());
    assert!(!bent.full_task_rank());
    let free = branched_tree();
    assert!(
        free.rank(
            &[0.; 11],
            task(0, Pose::IDENTITY, Frame::World),
            rank_options(TaskSpace::Pose)
        )
        .unwrap()
        .full_task_rank()
    );
    let ball = Tree::new(vec![link(
        "шар",
        None,
        Joint::Ball {
            rotation: [UNBOUNDED; 3],
        },
    )])
    .unwrap();
    assert_eq!(
        ball.rank(
            &[TAU, 0., 0.],
            task(0, Pose::IDENTITY, Frame::World),
            rank_options(TaskSpace::Pose)
        )
        .unwrap()
        .rank,
        1
    );
    let locked = Tree::new(vec![link(
        "каретка",
        None,
        Joint::Slide {
            axis: X,
            limit: limit(0., 0.),
        },
    )])
    .unwrap();
    assert_eq!(
        locked
            .rank(
                &[0.],
                task(0, Pose::IDENTITY, Frame::World),
                rank_options(TaskSpace::Pose)
            )
            .unwrap()
            .rank,
        0
    );
}

#[test]
fn malformed_trees_coordinates_poses_and_limits_are_rejected() {
    let root = link("корень", None, Joint::Fixed);
    let child = link("блок", Some(8), Joint::Fixed);
    assert!(Tree::new(vec![root.clone(), child]).is_err());
    assert!(Tree::new(vec![root.clone(), root.clone()]).is_err());
    assert!(Tree::new(vec![root.clone(), link("другой", None, Joint::Fixed)]).is_err());
    let a = link("стержень", Some(2), Joint::Fixed);
    let b = link("каретка", Some(1), Joint::Fixed);
    assert!(Tree::new(vec![root, a, b]).is_err());
    for axis in [[0.; 3], [f64::INFINITY, 0., 0.]] {
        assert!(
            Tree::new(vec![link(
                "стержень",
                None,
                Joint::Hinge {
                    axis,
                    limit: UNBOUNDED
                }
            )])
            .is_err()
        );
    }
    for range in [limit(2., 1.), limit(0., f64::NAN)] {
        assert!(
            Tree::new(vec![link(
                "каретка",
                None,
                Joint::Slide {
                    axis: X,
                    limit: range
                }
            )])
            .is_err()
        );
    }
    assert!(Pose::new([0.; 3], [0.; 4]).is_err());
    assert!(Pose::new([0.; 3], [1., 0., f64::NAN, 0.]).is_err());
    assert!(Pose::new([0., f64::INFINITY, 0.], [1., 0., 0., 0.]).is_err());
    let tree = carriage_tree();
    assert!(tree.forward(&[f64::NAN]).is_err());
    assert!(tree.forward(&[]).is_err());
    assert!(
        tree.pose(&[0.], task(0, Pose::IDENTITY, Frame::Link(LinkId(9))))
            .is_err()
    );
}

#[test]
fn workspace_lists_only_sampled_configurations_and_enforces_budget() {
    let tree = carriage_tree();
    let task = task(0, Pose::IDENTITY, Frame::World);
    let grid = [GridAxis {
        min: 0.,
        max: 1.,
        subdivisions: 4,
    }];
    let result = tree.workspace(task, &grid, 5).unwrap();
    assert_eq!(result.method, WorkspaceMethod::UniformCoordinateGrid);
    assert_eq!(
        result.coverage,
        WorkspaceCoverage::SampledConfigurationsOnly
    );
    assert_eq!(result.axes[0].subdivisions, 4);
    assert_eq!(result.points.len(), 5);
    for (point, x) in result.points.iter().zip([0., 0.25, 0.5, 0.75, 1.]) {
        assert_eq!(point.coordinates, [x]);
        assert_eq!(point.pose.translation(), [x, 0., 0.]);
    }
    assert!(tree.workspace(task, &grid, 4).is_err());
    assert!(
        tree.workspace(
            task,
            &[GridAxis {
                min: -1.,
                ..grid[0]
            }],
            5
        )
        .is_err()
    );
    assert!(
        tree.workspace(
            task,
            &[GridAxis {
                max: f64::NAN,
                ..grid[0]
            }],
            5
        )
        .is_err()
    );
    assert!(
        tree.workspace(
            task,
            &[GridAxis {
                subdivisions: 0,
                ..grid[0]
            }],
            5
        )
        .is_err()
    );
}

#[test]
fn trajectory_matches_analytic_midpoint_and_respects_continuous_motion_bounds() {
    let tree = carriage_tree();
    let s = |seconds| Time::from_secs(seconds).unwrap();
    let waypoints = vec![
        Waypoint {
            at: s(1.),
            coordinates: vec![0.],
        },
        Waypoint {
            at: s(3.),
            coordinates: vec![1.],
        },
        Waypoint {
            at: s(5.),
            coordinates: vec![0.],
        },
    ];
    // Ручной расчёт для 1 м за 2 с: максимум скорости 15/16, ускорения 2.5/√3.
    let limits = [MotionLimit {
        max_speed: 0.94,
        max_acceleration: 1.45,
    }];
    let trajectory = Trajectory::new(&tree, waypoints.clone(), &limits).unwrap();
    for (t, p) in [(1., 0.), (3., 1.), (5., 0.)] {
        let point = trajectory.sample(s(t)).unwrap();
        assert_eq!(point.coordinates, [p]);
        assert_eq!(point.velocity, [0.]);
        assert_eq!(point.acceleration, [0.]);
    }
    for (t, sign) in [(2., 1.), (4., -1.)] {
        let middle = trajectory.sample(s(t)).unwrap();
        assert_eq!(middle.coordinates, [0.5]);
        assert_eq!(middle.velocity, [sign * 15. / 16.]);
        assert_eq!(middle.acceleration, [0.]);
    }
    let maximum = s(1. + (3. - 3_f64.sqrt()) / 3.);
    close(
        [trajectory.sample(maximum).unwrap().acceleration[0]],
        [2.5 / 3_f64.sqrt()],
        1e-12,
    );
    for step in 0..=400 {
        let sample = trajectory.sample(s(1. + f64::from(step) / 100.)).unwrap();
        assert!((0. ..=1.).contains(&sample.coordinates[0]));
        assert!(sample.velocity[0].abs() <= limits[0].max_speed);
        assert!(sample.acceleration[0].abs() <= limits[0].max_acceleration);
    }
    for invalid in [
        MotionLimit {
            max_speed: 0.93,
            ..limits[0]
        },
        MotionLimit {
            max_acceleration: 1.44,
            ..limits[0]
        },
        MotionLimit {
            max_speed: f64::NAN,
            ..limits[0]
        },
    ] {
        assert!(Trajectory::new(&tree, waypoints.clone(), &[invalid]).is_err());
    }
    assert!(trajectory.sample(s(0.99)).is_err());
    let mut wrong = waypoints;
    wrong[1].coordinates[0] = 1.01;
    assert!(Trajectory::new(&tree, wrong.clone(), &limits).is_err());
    wrong[1].coordinates[0] = 1.;
    wrong[1].at = s(1.);
    assert!(Trajectory::new(&tree, wrong, &limits).is_err());
}

#[test]
fn shared_base_translation_cancels_before_relative_arithmetic_and_unrelated_joints_stay_out_of_rank()
 {
    // Большое смещение выбрано так, что шаг f64 значительно крупнее 1 м.
    let root = link(
        "основание",
        None,
        Joint::Free {
            translation: [UNBOUNDED; 3],
            rotation: [UNBOUNDED; 3],
        },
    );
    let moving = link(
        "каретка",
        Some(0),
        Joint::Slide {
            axis: X,
            limit: limit(0., 1.),
        },
    );
    let other = link(
        "блок",
        Some(0),
        Joint::Slide {
            axis: Y,
            limit: limit(0., 1.),
        },
    );
    let tree = Tree::new(vec![root, moving, other]).unwrap();
    let q = [1e20, -1e20, 0., 0.7, -0.2, 0.3, 0.5, 1.];
    let base = Frame::Site(Site {
        link: LinkId(0),
        offset: Pose::new([0.25, 0., 0.], [FRAC_1_SQRT_2, 0., 0., FRAC_1_SQRT_2]).unwrap(),
    });
    let relative = task(1, translation([1., 0., 0.]), base);
    close(
        tree.pose(&q, relative).unwrap().translation(),
        [0., -1.25, 0.],
        1e-14,
    );
    let j = tree.jacobian(&q, relative).unwrap();
    close(j.columns[6], [0., -1., 0., 0., 0., 0.], 1e-14);
    assert_eq!(j.columns[..6], [[0.; 6]; 6]);
    assert_eq!(j.columns[7], [0.; 6]);
    let r = tree
        .rank(&q, relative, rank_options(TaskSpace::Position))
        .unwrap();
    assert_eq!((r.rank, r.movable_coordinates), (1, 1));
    assert!(!r.rank_deficient());

    let fixed = Frame::Fixed(translation([0.25, 0., 0.]));
    let fixed_task = task(0, Pose::IDENTITY, fixed);
    assert_eq!(
        carriage_tree()
            .pose(&[0.5], fixed_task)
            .unwrap()
            .translation(),
        [0.25, 0., 0.]
    );
}
