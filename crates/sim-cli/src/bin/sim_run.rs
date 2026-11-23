fn main() -> std::process::ExitCode {
    let code = sim_cli::run(
        std::env::args_os().skip(1),
        std::io::stdin().lock(),
        std::io::stdout().lock(),
        std::io::stderr().lock(),
    );
    code.into()
}
