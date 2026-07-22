# Commander uORB Test Dependency Design

## Problem

A clean `make tests TESTFILTER=M2006TxPolicy` configures a parallel Ninja build.
`unit-CommanderHybridStatus` can compile before generated uORB headers exist and
fails in `HybridStatusGuard.hpp` because
`uORB/topics/hybrid_vehicle_status.h` is missing. Re-running after header
generation succeeds, proving an undeclared build dependency rather than a source
or compiler defect.

`CommanderHybridStatusTest.cpp` was added after the existing Commander build
graph. Its `px4_add_unit_gtest` target neither links a target that owns generated
uORB headers nor declares a direct dependency on `uorb_headers`.

## Design

Add one explicit CMake dependency immediately after the test declaration:

```cmake
px4_add_unit_gtest(SRC CommanderHybridStatusTest.cpp)
add_dependencies(unit-CommanderHybridStatus uorb_headers)
```

This makes Ninja generate all uORB topic headers before compiling the test. It
does not add a runtime or link dependency, change the common GoogleTest macro,
or affect production Commander behavior.

## Alternatives Rejected

- Linking `uorb_msgs` would establish an indirect dependency but add an unused
  library to a header-only unit test.
- Adding `uorb_headers` to every `px4_add_unit_gtest` target would broaden build
  ordering and cost across the repository.
- Serializing the build would hide the missing dependency instead of fixing it.

## Verification

The two existing clean-build failures are the RED reproduction. After the CMake
change:

1. Remove only `build/px4_sitl_test`.
2. Run the same Linux-only-PATH command with
   `TESTFILTER=M2006TxPolicy`, redirecting the full output.
3. Require exit 0, successful compilation of `unit-CommanderHybridStatus`, and
   `unit-M2006TxPolicy` passing 1/1.
4. Run `git diff --check` and confirm only the intended CMake file changed for
   this fix.

The fix is complete only if the test command succeeds from an absent SITL test
build directory; an incremental rerun is insufficient.
