struct Stand(PathBuf);

impl Stand {
    fn new(label: &str) -> Self {
        let path = std::env::temp_dir().join(format!("audit-t3-t5-{}-{label}", std::process::id()));
        fs::create_dir(&path).unwrap();
        let stand = Self(path);
        for directory in ["electrical", "drives", "sensors", "boards"] {
            fs::create_dir_all(stand.0.join("catalog").join(directory)).unwrap();
        }
        stand
    }

    fn write(&self, path: &str, text: &str) {
        fs::write(self.0.join(path), text).unwrap();
    }

    fn save(&self, path: &str, value: &Toml) {
        self.write(path, &toml::to_string(value).unwrap());
    }

    fn parse(&self, path: &str) -> Toml {
        parse(&fs::read_to_string(self.0.join(path)).unwrap())
    }

    fn electrical(&self, extra: &str) -> Value {
        self.write("electrical.toml", &format!(
            "electrical_format = 1\ncatalog = 'catalog'\n[assemblies.audit]\nworld = 'world.toml'\n{extra}"
        ));
        self.run("electrical.toml")["assemblies"]["audit"].clone()
    }

    fn launch(&self, manifest: &str) -> Output {
        Command::new(env!("CARGO_BIN_EXE_sim-run"))
            .args(["--campaign", manifest, "--json", "--jobs", "1"])
            .current_dir(&self.0)
            .output()
            .unwrap()
    }

    fn run(&self, manifest: &str) -> Value {
        let output = self.launch(manifest);
        assert!(output.status.success(), "{output:?}");
        serde_json::from_slice(&output.stdout).unwrap_or_else(|error| {
            panic!(
                "{error}: {}, {}",
                output.status,
                String::from_utf8_lossy(&output.stderr)
            )
        })
    }

    fn t3(label: &str, code: &str, variant: &str) -> Self {
        let stand = Self::new(label);
        for name in ["world.toml", "device.toml", "scheme.toml"] {
            stand.write(name, &corpus(&format!("depth_t3/{code}/{variant}/{name}")));
        }
        for name in ["parts.toml", "ratings.toml"] {
            let path = format!("catalog/electrical/{name}");
            stand.write(&path, &corpus(&format!("depth_t3/{path}")));
        }
        stand
    }
}

impl Drop for Stand {
    fn drop(&mut self) {
        fs::remove_dir_all(&self.0).unwrap();
    }
}

fn corpus(path: &str) -> String {
    fs::read_to_string(
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../devices/electrical")
            .join(path),
    )
    .unwrap()
}

fn parse(text: &str) -> Toml {
    toml::from_str(text).unwrap()
}
