//! Мир и устройства как данные, без окна. Контракт с рантаймом — [`CompiledWorld`].
//! STEP находится за границей импорта, сетки читаются в STL и OBJ.

mod compile;
mod compiled;
mod contact_options;
mod fragments;
mod import;
mod legacy;
mod loader;
mod mesh;
mod physics;
mod schema;
mod strength;
mod verify;

pub use compile::{DeviceInput, compile_world, compile_world_with_contact_options};
pub use compiled::{
    CompiledBody, CompiledDrive, CompiledJoint, CompiledJointKind, CompiledSensor, CompiledWorld,
    EffortChannel, Provenance, Simplification,
};
pub use contact_options::{
    ConstraintSolver, ContactBuildOptions, ContactPair, ContactSettings, ContactSolver,
    ContactStep, FrictionCone, MAX_CONTACT_IMPEDANCE, MIN_CONTACT_FRICTION, MIN_CONTACT_IMPEDANCE,
};
pub use fragments::{
    FragmentBindings, FragmentEffort, FragmentInertia, FragmentPort, ModelFragment, ResolvedPort,
    compile_with_fragment_bindings, compile_with_fragments,
};
pub use import::{import_mjcf, import_mjcf_world, import_urdf};
pub use legacy::{LegacyCatalog, extract_v2, migrate_v2, migrate_v2_device};
pub use loader::{
    load_and_compile, load_and_compile_with_catalog,
    load_and_compile_with_catalog_and_contact_options, load_and_compile_with_contact_options,
    load_device, parse_device, parse_world,
};
pub use mesh::{Mesh, load_mesh};
pub use schema::*;
pub use strength::{SectionAxis, StrengthSection, strength_section};
pub use verify::{CheckStatus, CompileCheck, set_attachment_active, verify_compiled};
