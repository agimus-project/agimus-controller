{
  lib,
  stdenv,
  cmake,
  coal,
  doctest,
  eigen,
  example-robot-data,
  pinocchio,
  python3,
}:

stdenv.mkDerivation (finalAttrs: {
  pname = "agimus-controller-cpp";
  version = "0.0.0";

  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./CMakeLists.txt
      ./cmake
      ./include
      ./package.xml
      ./src
      ./test
    ];
  };

  nativeBuildInputs = [ cmake ];
  propagatedBuildInputs = [
    coal
    eigen
    pinocchio
  ];
  checkInputs = [
    doctest
    example-robot-data
  ];
  nativeCheckInputs = [ python3 ]; # test/check_rt_coverage.py

  # Configuration B of doc/rt_safety.md: optimised, with debug info and the
  # RT checks (on by default when tests are built).
  cmakeBuildType = "RelWithDebInfo";
  cmakeFlags = [ (lib.cmakeBool "BUILD_TESTING" finalAttrs.finalPackage.doCheck) ];
  # A value with a space must go through cmakeFlagsArray.
  preConfigure = ''
    cmakeFlagsArray+=("-DCMAKE_CXX_FLAGS_RELWITHDEBINFO=-O3 -g")
  '';

  doCheck = true;

  meta = {
    description = "Real-time C++ port of agimus_controller";
    homepage = "https://github.com/agimus-project/agimus-controller";
    license = lib.licenses.bsd3;
    platforms = lib.platforms.unix;
  };
})
