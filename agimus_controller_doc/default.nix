{
  lib,
  stdenv,
  python3Packages,
  agimus-controller,
}:

stdenv.mkDerivation (finalAttrs: {
  pname = "agimus-controller-doc";
  version = "0.1.0";

  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./docs
    ];
  };

  nativeBuildInputs = with python3Packages; [
    python
    sphinx
    sphinx-rtd-theme
    myst-parser
    sphinx-autodoc-typehints
  ];

  # Needed at build time so that sphinx autodoc can import the module.
  buildInputs = [ agimus-controller ];

  buildPhase = ''
    runHook preBuild
    sphinx-build -b html docs _build/html
    runHook postBuild
  '';

  installPhase = ''
    runHook preInstall
    mkdir -p $out/share/doc/${finalAttrs.pname}
    cp -r _build/html $out/share/doc/${finalAttrs.pname}/html
    runHook postInstall
  '';

  meta = {
    description = "Sphinx HTML documentation for agimus_controller";
    license = lib.licenses.bsd3;
    # Follows agimus-controller, which nix-ros-overlay only provides on linux.
    inherit (agimus-controller.meta) platforms;
  };
})
