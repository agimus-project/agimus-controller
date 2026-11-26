{
  description = "Whole Body Model Predictive Control in the AGIMUS architecture";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    systems.follows = "gepetto/systems";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gepetto.flakeModule
          {
            flakoboros = {
              rosPackages = {
                agimus-controller-doc = ./agimus_controller_doc/default.nix;
              };
              rosOverrideAttrs = {
                agimus-controller = _: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_controller
                    ];
                  };
                };
                agimus-controller-ros = _: {
                  src = lib.fileset.toSource {
                    root = ./.;
                    fileset = lib.fileset.unions [
                      ./agimus_controller_ros
                    ];
                  };
                };
              };
            };
          }
        ];
        perSystem =
          { config, pkgs, ... }:
          {
            packages = {
              agimus-controller-doc = pkgs.callPackage ./agimus_controller_doc/default.nix {
                inherit (pkgs.rosPackages.humble) agimus-controller;
              };
            };
            checks = {
              # Build packages during nix flake check
              inherit (config.packages) agimus-controller-doc;
            };
          };
      }
    );
}
