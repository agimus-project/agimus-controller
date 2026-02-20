{
  stdenv,
  python3Packages,
  lib,
<<<<<<< HEAD
  agimus-controller,
=======
  fetchFromGitHub ? null,
>>>>>>> cf4d68b (Add agimus-controller-doc Nix derivation and expose package in flake)
  ...
}:

stdenv.mkDerivation rec {
  pname = "agimus-controller-doc";
  version = "0.1.0";

  src = ./.;

  buildInputs = with python3Packages; [
    sphinx
    sphinx-rtd-theme
    myst-parser
    sphinx-autodoc-typehints
<<<<<<< HEAD
    agimus-controller
=======
>>>>>>> cf4d68b (Add agimus-controller-doc Nix derivation and expose package in flake)
  ];

  buildPhase = ''
    mkdir -p _build/html
    sphinx-build -b html ${src}/docs _build/html
  '';

  installPhase = ''
    mkdir -p $out/share/doc/${pname}
    cp -r _build/html $out/share/doc/${pname}/html
  '';

  meta = with lib; {
    description = "Sphinx HTML documentation for agimus_controller";
<<<<<<< HEAD
    license = licenses.bsd3;
=======
    license = licenses.unfree; # docs only
>>>>>>> cf4d68b (Add agimus-controller-doc Nix derivation and expose package in flake)
    platforms = platforms.unix;
  };
}
