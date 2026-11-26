{
  description = "Whole Body Model Predictive Control in the AGIMUS architecture";

  inputs.gepetto.url = "github:gepetto/nix";

  # Same as gepetto.lib.mkFlakoboros, plus a perSystem devShell for C++.
  outputs =
    inputs:
    inputs.gepetto.inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.gepetto.inputs.systems;
        imports = [ inputs.gepetto.flakeModule ];
        flakoboros = {
          packages = {
            agimus-controller-cpp = ./agimus_controller_cpp/package.nix;
          };
          rosOverrideAttrs = {
            agimus-controller = {
              src = lib.fileset.toSource {
                root = ./.;
                fileset = ./agimus_controller;
              };
            };
            agimus-controller-ros = {
              src = lib.fileset.toSource {
                root = ./.;
                fileset = ./agimus_controller_ros;
              };
            };
          };
        };
        perSystem =
          { pkgs, self', ... }:
          {
            # `nix develop .#cpp`: only what agimus_controller_cpp needs. The
            # default devShell is the whole ROS stack.
            devShells.cpp = pkgs.mkShell {
              inputsFrom = [ self'.packages.agimus-controller-cpp ];
            };
          };
      }
    );
}
