use std::{collections::BTreeMap, fs, path::PathBuf};

use sim_cli::electrical::SchematicInput;
use sim_task::{
    Fingerprint, Signal, Source, SourceKind,
    campaign::{
        Campaign, CampaignPlan, Grid, RequiredPath, ResolvedInputs, ResolvedReference, WorldInputs,
    },
    verdict::{
        CampaignEvidence, ElectricalEvidence, ResolvedInventory, VerdictReport, assess_campaign,
        derive_requirements_with_electrical,
    },
};

/// Неподвижный блок служит носителем ссылки на схему. Электрический опыт
/// занимает фиксированную часть вердикта, для него не нужны прогоны механики.
pub struct ElectricalStand {
    directory: StandDirectory,
    plan: CampaignPlan,
    inventory: ResolvedInventory,
}

struct StandDirectory(PathBuf);

impl Drop for StandDirectory {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

impl ElectricalStand {
    pub fn new(test: &str) -> Self {
        let directory =
            std::env::temp_dir().join(format!("electrical-{}-{test}", std::process::id()));
        fs::create_dir(&directory).unwrap();
        let directory = StandDirectory(directory);
        fs::write(directory.0.join("body.toml"), r#"
format = 3
name = "Блок"
schematic = "scheme.toml"
[[links]]
name = "корпус"
joint = { kind = "slide", axis = [1.0, 0.0, 0.0] }
# Единичный куб массой 6 кг имеет I = m·a²/6 = 1 кг·м².
mass = { kind = "explicit", mass_kg = 6.0, com_m = [0.0, 0.0, 0.0], inertia_kg_m2 = [1.0, 1.0, 1.0, 0.0, 0.0, 0.0], provenance = { measured = "условия модели куба" } }
"#).unwrap();
        fs::write(
            directory.0.join("world.toml"),
            r#"
format = 3
name = "Блок"
seed = 1
physics_hz = 1000
# Неподвижный блок в схеме электрического стенда.
gravity_m_s2 = [0.0, 0.0, 0.0]
[environment]
density_kg_m3 = 0.0
viscosity_pa_s = 0.0
wind_m_s = [0.0, 0.0, 0.0]
[[devices]]
name = "блок"
file = "body.toml"
"#,
        )
        .unwrap();
        let catalog = directory.0.join("catalog.toml");
        fs::write(&catalog, "version = 1").unwrap();
        let campaign_file = directory.0.join("campaign.toml");
        fs::write(&campaign_file, r#"
version = 1
world = "world.toml"
catalog = "catalog.toml"
seed = 1
[stations.block]
experiment = { kind = "scenario" }
[stations.block.modes.rest]
# Один такт неподвижного блока. Для электрических находок он не исполняется.
horizon_ns = 1000000
source = { kind = "measured", reference = "постановка синтетического блока" }
criteria = [{ id = "покой", window = "whole", limit = { kind = "range", signal = { name = "qpos[0]", unit = "m" }, min = 0.0, max = 0.0, source = { kind = "measured", reference = "начальное состояние без сил" } } }]
[stations.block.modes.rest.windows.whole]
start_ns = 0
end_ns = 1000000
max_gap_ns = 1000000
"#).unwrap();
        let campaign = Campaign::load(campaign_file).unwrap();
        let source = Source {
            kind: SourceKind::Measured,
            reference: "постановка синтетического блока".into(),
        };
        let plan = campaign
            .plan(&ResolvedInputs {
                world: WorldInputs::load(campaign.world_path()).unwrap(),
                references: campaign
                    .references()
                    .map(|entry| {
                        (
                            entry.path.clone(),
                            ResolvedReference {
                                kind: entry.kind,
                                file_fingerprint: entry.file_fingerprint,
                                graph_fingerprint: Fingerprint::of(b"version = 1"),
                            },
                        )
                    })
                    .collect(),
                paths: vec![RequiredPath {
                    station: "block".into(),
                    mode: "rest".into(),
                    path: "покой".into(),
                    source: source.clone(),
                    grids: vec![Grid {
                        id: "1000".into(),
                        hz: 1000,
                        source,
                    }],
                    faults: Default::default(),
                    combinations: Vec::new(),
                    unavailable: None,
                }],
                targets: Default::default(),
                signals: [Signal {
                    name: "qpos[0]".into(),
                    unit: "m".into(),
                }]
                .into(),
                model_parameters: Default::default(),
                initial_velocity: false,
            })
            .unwrap();
        let inventory = ResolvedInventory {
            build: Some(Fingerprint::of("электрический стенд".as_bytes())),
            ..Default::default()
        };
        Self {
            directory,
            plan,
            inventory,
        }
    }

    pub fn assess(&self, input: &SchematicInput) -> VerdictReport {
        fs::write(self.directory.0.join("scheme.toml"), &input.bytes).unwrap();
        let evidence = ElectricalEvidence::evaluate(
            "блок",
            &input.bytes,
            &input.compiled,
            &input.modes,
            self.inventory.build.unwrap(),
        )
        .unwrap();
        let requirements = derive_requirements_with_electrical(
            &self.inventory,
            &self.plan,
            &BTreeMap::from([("блок".into(), evidence)]),
        )
        .unwrap();
        assess_campaign(&requirements, &CampaignEvidence::new(&requirements)).unwrap()
    }
}
