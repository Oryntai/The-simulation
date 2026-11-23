use super::{Interval, RegionId, Source};
use crate::Sex;

const CDC: &str = "https://www.cdc.gov/nchs/data/series/sr_03/sr03-046-508.pdf";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BaseVariant {
    /// Закреплённое основание, физические суставы свободны.
    MsHuman700,
    /// Пространственное свободное основание, для прогона с физическими опорами.
    MsHuman700Free,
}

#[derive(Clone, Debug)]
pub struct ParticipantSpec {
    pub base: BaseVariant,
    pub population: Sex,
    pub stature_m: f64,
    pub mass_kg: f64,
    pub proportions: Vec<Proportion>,
}

#[derive(Clone, Debug)]
pub struct Proportion {
    pub region: RegionId,
    pub measure: ProportionMeasure,
    pub source: String,
}

#[derive(Clone, Copy, Debug)]
pub enum ProportionMeasure {
    /// Габариты внешнего участка по осям его тела в нейтральной рамке, м.
    LocalDimensions([f64; 3]),
    /// Наружная длина по локальной Y и обхват эллиптического центрального
    /// сечения XZ. Применимо к выровненным эллипсоидам.
    LengthAndCircumference { length_m: f64, circumference_m: f64 },
}

#[derive(Clone, Copy, Debug)]
pub struct Marginals {
    /// Отдельные маргинальные 5, 50 и 95 перцентили.
    pub stature_m: [f64; 3],
    pub mass_kg: [f64; 3],
    pub stature_source: Source<'static>,
    pub mass_source: Source<'static>,
}

pub fn marginals(population: Sex) -> Marginals {
    let (stature_m, mass_kg, stature_table, mass_table) = match population {
        Sex::Female => (
            [1.498, 1.613, 1.725],
            [49.8, 73.1, 119.6],
            "табл. 9",
            "табл. 3",
        ),
        Sex::Male => (
            [1.628, 1.754, 1.874],
            [61.7, 87.4, 130.3],
            "табл. 11",
            "табл. 5",
        ),
    };
    Marginals {
        stature_m,
        mass_kg,
        stature_source: Source {
            reference: CDC,
            locator: stature_table,
        },
        mass_source: Source {
            reference: CDC,
            locator: mass_table,
        },
    }
}

/// Полное декартово произведение маргинальных уровней, без вероятностных весов.
pub fn marginal_matrix(population: Sex) -> Vec<ParticipantSpec> {
    let table = marginals(population);
    table
        .stature_m
        .into_iter()
        .flat_map(|stature_m| {
            table
                .mass_kg
                .into_iter()
                .map(move |mass_kg| ParticipantSpec {
                    base: BaseVariant::MsHuman700,
                    population,
                    stature_m,
                    mass_kg,
                    proportions: Vec::new(),
                })
        })
        .collect()
}

#[derive(Clone, Debug)]
pub struct Anthropometry {
    pub population: Sex,
    pub stature_m: f64,
    pub neutral_proxy_height_m: f64,
    pub mass_kg: f64,
    pub reference_stature_m: f64,
    pub reference_mass_kg: f64,
    pub uniform_geometric_scale: Option<f64>,
    pub mass_scale: f64,
    pub stature_in_marginal_range: bool,
    pub mass_in_marginal_range: bool,
    pub joint_distribution_known: bool,
    pub measured_proportions: Vec<Proportion>,
    pub strength_coverage: Option<Interval>,
    pub soft_tissue_coverage: Option<Interval>,
    pub limitations: [&'static str; 3],
}
