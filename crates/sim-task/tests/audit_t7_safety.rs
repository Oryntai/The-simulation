use sim_task::campaign::depth_factors::DepthFactors;

#[test]
fn декларация_не_исключает_фактор_отказа_из_безопасности() {
    let fixture: toml::Value = toml::from_str(include_str!(
        "../../../devices/electrical/depth_t7/fixtures/block.toml"
    ))
    .unwrap();
    let mut description = fixture["description"].clone();
    description["factors"]["cut"]["safety"] = toml::Value::Boolean(false);
    let factors: DepthFactors = description.try_into().unwrap();
    assert!(factors.factors()["cut"].safety);
    assert!(!factors.factors()["grid"].safety);
}
