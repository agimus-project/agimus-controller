# crocoddyl-toml: OCP descriptions in TOML

Ruling R2 (2026-11-25, revised on PR #334 2026-11-26): OCPs are described
in TOML and loaded by **crocoddyl-toml**, a new library in its own
repository (`CT/`).

- The file format is designed for TOML. It is not a copy of crocoddyl_yaml's
  schema.
- **Every type is defined once, in a schema file.** The typed C++ structs,
  their parsers and their registration are generated from the schema files
  at build time, the way ROS generates code from message definitions. A
  JSON Schema of the whole format is generated too, so editors (TOML
  language servers) and LLMs can check a file. A downstream library adds a
  type with a schema file and one `build` function; it writes no parsing or
  registration code.
- crocoddyl_yaml is not upgraded, extended or used.
- **Nothing application-specific goes into `CT/`.** crocoddyl-toml is a
  loco-3d library. The first user, agimus, documents its own use in
  [ocp_files.md](ocp_files.md); briefs for `CT/` tasks do not include that
  file, and no agimus name, key, type or example appears in `CT/`.

## Acknowledgment: crocoddyl_yaml

crocoddyl-toml starts from **crocoddyl_yaml** (PAL GitLab
`control/crocoddyl_yaml`, written by PepMS between 2024-12-16 and
2024-12-20). That library showed that a crocoddyl OCP can be described in a
data file and built by one factory per model family:

- actuation;
- activation;
- residual;
- cost item;
- constraint;
- contact;
- differential and integrated action model;
- problem.

crocoddyl-toml keeps that architecture and its idea of reusable stages. It
re-implements the rest:

- TOML instead of YAML;
- schema files compiled into typed C++ code, instead of yaml-schema-cpp's
  run-time validation;
- the crocoddyl 3 and pinocchio 4 APIs;
- types added by other libraries through the same schema files.

crocoddyl_yaml's type list is the starting point for the built-in types
below.

Reviewing crocoddyl_yaml for this plan turned up defects. Each becomes a
requirement here, with a test:

| Found in crocoddyl_yaml | Requirement in crocoddyl-toml |
|---|---|
| The contact case falls through to `default: throw` (missing `break`, `src/factories/diff_action.cpp:92-96`) | Every built-in type has a test that builds it |
| Every node of a stage, and the terminal node, share one action-model object (`src/factories/problem.cpp:58-83`) | Every `build_*` call returns new objects; nothing is shared between nodes |
| `integrator` is parsed and then discarded (`src/trajectory.cpp:83-89`) | Every key is either used or rejected; a test per key |
| The node count `int(duration / dt)` truncates (`0.19 / 0.01` gives 18) | Node counts are given, never computed from a duration |
| Error messages copied between factories ("Residual ContactFrictionCone" in contacts) | Error messages are generated from the key path, never written by hand |
| Dead, uncompilable files; `ProblemFactory` without a test | `-Werror` in CI; every public function has a test |

The YAML-versus-TOML decision came from the gepetto maintainers (PR #334).
YAML 1.1's implicit typing turns `NO` into false and `3.10` into 3.1, YAML
parsers disagree with each other, and Python's standard library reads TOML
(`tomllib`) but not YAML.

## Repository and build

- Repository: [github.com/loco-3d/crocoddyl-toml](https://github.com/loco-3d/crocoddyl-toml)
  (shorthand `CT/`), next to crocoddyl. It was created empty on 2026-11-25,
  with no license yet; crocoddyl is BSD-3-Clause, and the user picks the
  license.
- **Packaging follows the Gepetto/loco-3d packages, not R6.** R6's plain
  CMake applies to `CPP/` only. The template is
  [gepetto/example-adder](https://github.com/gepetto/example-adder):
  - `CMakeLists.txt` with jrl-cmakemodules found as an installed package
    (`find_package(jrl-cmakemodules REQUIRED CONFIG)`, then `base.cmake`),
    not a git submodule. It uses `PROJECT_NAME crocoddyl-toml`,
    `PROJECT_URL https://github.com/loco-3d/crocoddyl-toml`,
    `PROJECT_USE_CMAKE_EXPORT TRUE`,
    `compute_project_args`, `check_minimal_cxx_standard(17 ENFORCE)`, and
    jrl's install and export. The container has jrl-cmakemodules 2.3.0
    (`ros-lyrical-jrl-cmakemodules`).
  - `package.xml` format 3 with `build_type` cmake and
    `build_depend jrl_cmakemodules`, so colcon and the ROS buildfarm can
    build it.
  - `flake.nix` with `inputs.gepetto.url = "github:gepetto/nix"` and
    `gepetto.lib.mkFlakoboros`, overriding the package's `src` with the
    local file set; `.envrc` with `use flake .`. The override needs the
    package to exist in gepetto/nix, so T5.1 also writes
    `pkgs/crocoddyl-toml.nix` for gepetto/nix (modelled on
    `pkgs/force-feedback-mpc.nix`). Opening that pull request is outward
    facing: the user approves it first.
  - The CI workflows, `.pre-commit-config.yaml` and `.mergify.yml` are
    copied from example-adder (`nix.yml`, `ci_ros2.yml`,
    `update-flake-lock.yml`).
  - Tests use doctest (R6's test rule), registered with jrl's
    `add_unit_test`. Gepetto packages usually use Boost.Test; if the
    loco-3d maintainers ask for it, switch before T5.2, while there are few
    tests.
  - No Python bindings in this plan (R1). The layout reserves `python/`
    and the `BUILD_PYTHON_INTERFACE` option (default OFF until bindings
    exist), because crocoddyl's users are mostly in Python and the
    maintainers will likely ask for them.
- Dependencies: crocoddyl, pinocchio, Eigen and toml++ 3.x; Python 3 (standard
  library only) at build time, for the code generator. toml++ is packaged as
  `libtomlplusplus-dev` in Ubuntu and as `tomlplusplus` in nixpkgs. The
  versions available on every supported platform (PLAN.md, R9) are the
  minimum to support.
  - It has no dependency on colmpc, force_feedback_mpc or any application.
    Those add their types from outside (see Schema files).
  - toml++ is header-only and heavy to compile. Compile it once, in
    `src/toml_impl.cpp` with `TOML_HEADER_ONLY=0`, so every translation unit
    stays under 2 GB.
- **Every name comes from the package name**, never from the GitHub
  organisation (which appears only in `PROJECT_URL`):
  - CMake project and exported target: `crocoddyl-toml::crocoddyl-toml`
    (jrl's `PROJECT_NAME::PROJECT_NAME`);
  - headers: `include/crocoddyl-toml/` (jrl's `CUSTOM_HEADER_DIR`);
  - C++ namespace: `crocoddyl_toml`, the package name with `-` replaced by
    `_`, because a C++ identifier cannot contain `-`.

## File format

Principles:

- **Named things are tables keyed by their name**, not arrays of tables:
  `[models.running.costs.state_reg]`, not `[[stages.costs]]`. crocoddyl
  stores costs and constraints by name anyway, so order does not matter.
- **Short sub-objects are inline tables** that fit on one line (TOML 1.0
  inline tables cannot span lines): `residual = { type = "state" }`. Longer
  ones use a sub-table, `[models.running.costs.x.residual]`.
- **Every object with variants has a `type` key**, written in `snake_case`.
- **Times are seconds**, read as `double` (`dt = 0.01`), as crocoddyl and the
  solvers use them.
- **Infinite bounds use TOML's native `inf` and `-inf`.**
- **A vector can be given as a scalar**, which is broadcast to the size the
  model needs, as in Python (`weights = 1.0`).
- **Poses** are `{ translation = [x, y, z], quaternion = [x, y, z, w] }`. The
  quaternion order is written out in the docs because it differs between
  libraries. Identity by default.
- **Strict parsing.** An unknown key, a wrong type or a wrong size is an
  error. The error message gives the file, line, column and key path:
  `goal.toml:14:1: models.running.costs.goal_tracking.activation.weights:
  expected 6 values (nr of frame_placement), got 3`.
- **Versioned.** The first key is `format = 1`, and the loader rejects other
  versions.
- **Checkable in an editor.** A file may start with
  `#:schema <path or URL of crocoddyl-toml.schema.json>`, which TOML language
  servers (taplo, Even Better TOML) use to validate it as it is typed.

Example: a goal-reaching OCP for example-robot-data's panda.

```toml
format = 1

[models.running]
dynamics = { type = "free_fwd" }
integrator = { type = "euler" }
actuation = { type = "full" }

[models.running.costs.control_reg]
weight = 1.0
residual = { type = "control" }
activation = { type = "weighted_quad", weights = 1.0 }

[models.running.costs.state_reg]
weight = 1.0
residual = { type = "state" }
activation = { type = "weighted_quad", weights = 1.0 }

[models.running.costs.goal_tracking]
weight = 1.0
residual = { type = "frame_placement", frame = "panda_hand" }
activation = { type = "weighted_quad", weights = 1.0 }

[models.terminal]
dynamics = { type = "free_fwd" }
integrator = { type = "euler" }
actuation = { type = "full" }

[models.terminal.costs.state_reg]
weight = 1.0
residual = { type = "state" }
activation = { type = "weighted_quad", weights = 1.0 }

[models.terminal.costs.goal_tracking]
weight = 1.0
residual = { type = "frame_placement", frame = "panda_hand" }
activation = { type = "weighted_quad", weights = 1.0 }
```

An optional `[problem]` table describes a whole horizon, for users that do
not build it themselves:

```toml
[problem]
nodes = [
  { model = "running", count = 20, dt = 0.01 },
  { model = "running", count = 10, dt = 0.02 },
]
terminal = "terminal"
```

### Application tables

crocoddyl-toml parses only `format`, `models` and `problem`. It returns every
other top-level table untouched in `Document::extra`, so an application keeps
its own keys in the same file (for instance `[my_controller]`). One OCP is
one file.

## C++ API

```cpp
namespace crocoddyl_toml {

struct Document {
  std::map<std::string, ModelSpec, std::less<>> models;
  std::optional<ProblemSpec> problem;
  toml::table extra;                       // application tables, untouched
  std::filesystem::path source;            // for error messages
};

// Setup only. Throws ParseError (a std::runtime_error) with file:line:col and
// the key path. Validates every key, including registered types' keys.
Document load_file(const std::filesystem::path&);
Document load_string(std::string_view toml, std::string_view source_name);

struct BuildContext {
  std::shared_ptr<crocoddyl::StateMultibody> state;
  std::shared_ptr<crocoddyl::ActuationModelAbstract> actuation;  // nullptr: from the model's `actuation`
  std::shared_ptr<pinocchio::GeometryModel> geometry;            // nullptr if unused
};

// New objects on every call (one per node). `dt` in seconds. Throws
// BuildError (key path) on an unknown frame or a size mismatch.
std::shared_ptr<crocoddyl::ActionModelAbstract> build_model(
    const Document&, std::string_view model, const BuildContext&, double dt);
std::shared_ptr<crocoddyl::ShootingProblem> build_problem(
    const Document&, const BuildContext&, const Eigen::VectorXd& x0);

}  // namespace crocoddyl_toml
```

- `ModelSpec`, `CostSpec`, `ConstraintSpec` and one struct per type
  (`StateResidualSpec { std::optional<Eigen::VectorXd> xref; }`, ...) hold
  the parsed values. They and their `parse` functions are generated from the
  schema files (next section); defaults come from the schema.
- Sizes that depend on the robot (nv, nr) are checked in `build_model`,
  where they are known. Everything else is checked in `load_*`.
- `load_*` validates the keys of every type registered so far, including
  types added by other libraries: register them before loading.

## Schema files and generated code

Endorsed on PR #334 (reviewer Kotochleb, the user): one schema file per type
is the single source of truth, and code is generated from it, as ROS does
with message definitions.

- **Format: JSON Schema** (a restricted subset), one file per type:
  `schemas/<family>/<type>.schema.json`, for instance
  `schemas/residual/frame_placement.schema.json`. JSON Schema is what TOML
  language servers read, and what LLMs know best.
  - Allowed keywords: `type` (`object`, `number`, `integer`, `boolean`,
    `string`, `array`), `properties`, `required`, `default`, `enum`,
    `items`, `minItems`, `maxItems`, `description`,
    `additionalProperties: false` (always).
  - Extensions, prefixed `x-`: `x-size` for a vector whose size depends on
    the model (`"nv"`, `"ndx"`, `"nu"`, `"nr"`), checked at build time;
    `x-scalar-broadcast: true` when a scalar may stand for the vector;
    `x-cpp-type` when the default mapping is not enough (`"pinocchio::SE3"`
    for a pose).
- **Generator:** `cmake/crocoddyl_toml_generate.py` (Python 3, standard
  library only), run at build time through the CMake function
  `crocoddyl_toml_add_types(<target> FAMILY <family> SCHEMAS <files...>)`.
  For each schema it generates:
  - the C++ struct, with the schema's defaults;
  - its `parse(const toml::table&, const KeyPath&)`, with the strict rules of
    the File format section (unknown key, wrong type or size: `ParseError`
    naming file:line:col and the key path);
  - its registration, which ties the type name to the struct and to a
    hand-written build function
    `std::shared_ptr<...> build(const <Struct>&, const BuildContext&)`;
  - a `register_<target>_types()` function for the target, idempotent
    (`std::call_once`).

  The generated files go to the build tree and are never edited or
  committed.
- **Downstream types** use the same CMake function from their own project,
  with their own schema files and build functions. That is the only way to
  add a type; there is no hand-written parsing or registration API.
- **Editor schema:** the build also writes `crocoddyl-toml.schema.json`, the
  JSON Schema of a whole file (format, models, problem, every built-in
  type), installed under `share/crocoddyl-toml/`. A downstream project's
  `crocoddyl_toml_add_types` call writes its own extended copy that also
  knows its types.
- **Registry underneath:** the generated code fills a run-time registry
  (type name to parse and build functions). Registering a name twice, or a
  built-in name, throws. An unknown `type` is a `ParseError` that lists the
  known types for that family.
- **Unit tests** (T5.3): each type has a test that loads a TOML file,
  builds the model and compares it with the same model built by hand in
  C++ (same `calc`/`calcDiff` results on random x and u). T5.2 also checks
  with taplo that the generated JSON Schema accepts the test files and
  rejects a file with an unknown key.

## Built-in types

Each is a schema file plus a build function. The first set is what the first
user needs, all plain crocoddyl. The rest of
crocoddyl_yaml's list (rigid contacts 1D–6D, friction cone, frame axis
alignment, frame collision, smooth norms, flat exp/log activations, RK
integrators) is added when someone needs it, each with its test.

| Family | `type` | crocoddyl class | Keys (default) |
|---|---|---|---|
| dynamics | `free_fwd` | `DifferentialActionModelFreeFwdDynamics` | none |
| integrator | `euler` | `IntegratedActionModelEuler` | `with_cost_residual` (true) |
| actuation | `full` | `ActuationModelFull` | none |
| actuation | `floating_base` | `ActuationModelFloatingBase` | none |
| residual | `state` | `ResidualModelState` | `xref` (neutral, zero velocity) |
| residual | `control` | `ResidualModelControl` | `uref` (zero) |
| residual | `control_grav` | `ResidualModelControlGrav` | none |
| residual | `frame_placement` | `ResidualModelFramePlacement` | `frame`, `pose` (identity) |
| residual | `frame_translation` | `ResidualModelFrameTranslation` | `frame`, `translation` (zero) |
| residual | `frame_rotation` | `ResidualModelFrameRotation` | `frame`, `quaternion` (identity) |
| residual | `frame_velocity` | `ResidualModelFrameVelocity` | `frame`, `twist` (zero), `reference_frame` (`"world"`; or `"local"`, `"local_world_aligned"`) |
| activation | `quad` | `ActivationModelQuad` | none (the default when `activation` is absent) |
| activation | `weighted_quad` | `ActivationModelWeightedQuad` | `weights` (scalar or vector of size nr) |
| constraint | `residual` (default) | `ConstraintModelResidual` | `residual`, `lower` (-inf), `upper` (inf), `terminal` (true) |
| constraint | `control_limit` | `ConstraintModelResidual` on a control residual | bounds from the model's `effortLimit` (Python `ConstraintModelControlLimit`) |

Cost item keys: `weight` (1.0), `active` (true), `residual`, `activation`.
Constraint item keys: `active` (true), plus the constraint's keys.
