# Supervision protocol

Two roles:

- The **supervisor** (a strong model) owns the plan. It writes task briefs,
  reviews, runs the judge (golden files), decides tolerances, and is the only
  one who talks to the user.
- **Implementers** (cheaper models) take one task at a time, each in a fresh
  context.

The loop is subagent-driven development: brief, implement, review, record.

## Loop per task

1. **Brief.** The supervisor copies the template below into the
   implementer's prompt and fills it in from PLAN.md. The brief is the
   implementer's only context, so it names exact files and line ranges, never
   "see the Python code".
2. **Implement.** The implementer works test-first:
   - port or write the tests (from the Python tests or the golden files) and
     see them fail for the right reason;
   - implement until they pass;
   - run the full package build and `ctest`;
   - run `pre-commit run --files <changed files>`;
   - commit (R7).

   It ends with a report: what changed, the test output, deviations, open
   questions.
3. **Review.** The supervisor reruns build and tests itself (trust the run,
   not the report), then applies the checklist below. Findings go back to the
   implementer, concrete and numbered. After two rounds on the same task, the
   supervisor fixes it itself or splits the task.
4. **Record.** The supervisor appends to `doc/ledger.md`:

   ```
   ## T<id> <title>: done (<commit>)
   - Rulings: <decisions made during the task that later tasks must follow>
   - Deviations: <differences from the plan, and why>
   - Open: <questions left for the user>
   ```

   A ruling that changes design.md is also edited into design.md (single
   source of truth). The ledger only points to it.

## Brief template

```
Task T<id>: <title>   (repo: <agimus_controller | crocoddyl_yaml>)
Goal: <one sentence>
Read first: doc/design.md#<section>, doc/rt_safety.md (if any @rt code),
  <Python sources with line ranges>, <Python tests>
Create/modify: <exact paths>
Tests to write: <names, incl. Rt* tests and golden sets>
Done when: <copied from PLAN.md>
Rulings in force: <ledger entries that matter here>
Stop and report instead of guessing if: any stop condition in doc/supervision.md
Build/test commands: <from doc/environment.md>
```

## Stop conditions

The implementer stops, commits nothing that is half-done, and reports when:

1. An `Rt*` test fails inside third-party code (rt_safety.md, third-party
   section).
2. Progress needs an edit to an existing crocoddyl_yaml `.schema` file.
3. The Python behaviour is ambiguous or looks wrong, and design.md's
   behaviour-change table does not cover it.
4. A dependency or header is missing, or the installed API differs from what
   the brief assumes.
5. A golden comparison fails beyond tolerance after one debugging attempt.
6. A translation unit needs more than 2 GB to compile (check with
   `/usr/bin/time -v`).
7. The task appears to need a public API not in design.md.

The supervisor resolves 3, 5 and 7 when it can, and records a ruling. It
escalates 1, 2, 4 and unresolved cases to the user.

## Review checklist

Each item is a yes/no question. A "no" is a finding.

- **Scope.** Only the files in the brief were touched. Nothing from the
  "not ported" list (R1) was added.
- **Design.** Names, files, signatures and ownership follow design.md. No
  new class hierarchy. No `std::map` or `std::string` in RT paths.
- **RT.** Every function reachable from an `@rt` function obeys
  rt_safety.md. Every `@rt` tag has an `Rt*` test covering its early-return
  branches, and `check_rt_coverage` passes. The tests run after warm-up with
  realistic sizes.
- **Parity.** Every Python test of the module is ported (or the ledger says
  why not). Golden comparisons use the tolerances from design.md. Each
  behaviour change B1–B9 the code touches has its test.
- **Setup errors.** Bad input throws with a message naming the item. There
  is a test for each throw ported from a Python assert or `ValueError`.
- **Simplicity.** No speculative options, templates or abstractions beyond
  the brief. No dead code. Comments say why, not what.
- **Speed.** Hot-path code follows design.md#performance. From T4.7 on,
  every task that touches the hot path reruns `bench_mpc` (Release, RT
  checks OFF), and the ledger records median and p99 against the last
  baseline. A regression of more than 3 % is a finding unless the task
  explains it. An optimization stays only with before/after numbers that
  meet R8.
- **Build.** Plain CMake, no new dependency outside the brief, warnings
  clean, pre-commit clean, no `-march` outside T10.7.
- **Git.** Atomic commits, short messages, no AI attribution lines, nothing
  pushed.

## Model assignment

PLAN.md marks tasks S, M or H.

- **S** tasks suit a small model (e.g. Haiku).
- **M** tasks suit a mid model (e.g. Sonnet).
- **H** tasks are done by the supervisor, or by a strong model under
  close review. They are the golden generator, `Ocp`, `Mpc`, the
  stage-template builder, the colmpc integration and force feedback.

Independent tasks may run in parallel in separate git worktrees: P5–P6
alongside P2–P4, and T2.1–T2.4 among themselves. Never run two tasks in
parallel that touch the same file.

## Skills

Found by an online search on 2026-11-24. All URLs were checked to exist;
none are installed yet (T0.3).

| Skill | Source | Use here |
|---|---|---|
| `subagent-driven-development`, `executing-plans`, `verification-before-completion`, `systematic-debugging`, `using-git-worktrees` | github.com/obra/superpowers | The supervisor loop above. Install these and skip its `writing-plans` (this plan replaces it) and its TDD skill (use the installed `mattpocock-skills:tdd`, so there is one TDD skill only). Its SDD skill runs without check-ins by default: keep the stop conditions above. |
| `cmake` | github.com/mohitmishra786/low-level-dev-skills (`skills/build-systems/cmake`) | Target-based CMake for T1.1 and T5.x. |
| `heaptrack` / `valgrind` | the same repo (`skills/profilers/`) | Second opinion when the allocation hook reports something hard to locate. |
| Rulebook and judge method | github.com/anthropics/code-migration-kit-with-claude-code | Background only. design.md is the rulebook and the golden files are the judge; do not run the kit's pipeline (it assumes same-architecture ports). |
| `rt-safe-cpp` (to write, T1.3) | this repo | No public skill covers real-time C++, Eigen or Pinocchio. The PickNik MoveIt Pro realtime guidelines (docs.picknik.ai) were the closest model. |
| already installed: `mattpocock-skills:tdd`, `code-review`, `simplify`, `mattpocock-skills:writing-for-agents` | — | TDD for implementers; `code-review` for T9.5; `writing-for-agents` for T1.3. |
