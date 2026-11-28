---
name: rt-safe-cpp
description: Real-time rules and allocation harness for agimus_controller_cpp. Use when writing, reviewing or testing a function tagged `@rt`, any code on the MPC tick path, or an `Rt*` test, and when an `Rt*` test goes red.
---

# Real-time C++ in agimus_controller_cpp

The single source of truth is
[agimus_controller_cpp/doc/rt_safety.md](../../../agimus_controller_cpp/doc/rt_safety.md).
This skill is the order in which to use it. An `@rt` function is proven
RT-safe by a test that goes **red** on any allocation, free or Eigen
malloc inside it; unproven code is not RT code.

## Steps

1. **Read the rules.** Read rt_safety.md, sections "Rules for `@rt` code"
   and "Guard and macros", before writing the first line. Done when you can
   say, for each RT call you plan, where its storage is sized at setup.
2. **Split setup from tick.** Setup (constructors, `finalize`, builders)
   sizes every buffer, resolves types, caches raw pointers and may throw.
   The tick path writes into that storage and returns a `Status`
   (`include/agimus_controller/status.hpp`). Setup ends with one full call
   of the tick path (warm-up, rule 7).
3. **Tag.** Put `/// @rt Type::method` (or `/// @rt name` for a free
   function) on the declaration in the header.
4. **Write the `Rt*` test.** Name it `RtType.method: ...` (`RtFree.name: ...`
   for a free function). Build the object at realistic sizes, warm up, then
   wrap each call in `CHECK_RT_SAFE(...)` and check the result *outside*
   the macro. Cover every early-return branch. Pattern:
   `test/test_trajectory_buffer.cpp`, `RtTrajectoryBuffer.push_back`.
5. **Run** configuration A and B
   (rt_safety.md, "Build configurations"). Done when every `Rt*` test and
   the `check_rt_coverage` ctest pass in both.

## When an `Rt*` test is red

- An abort with an Eigen assertion is an Eigen allocation: a dynamic-size
  temporary or a resize (rule 2).
- A count from `CHECK_RT_SAFE`: rerun the single test with
  `AGIMUS_RT_ABORT_ON_ALLOC=1` under gdb and read the backtrace.
- The first frame outside `agimus_controller_cpp` is a third-party
  library: follow rt_safety.md, "Third-party violations". The test stays
  red; stop and report.
