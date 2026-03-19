{ pkgs ? import <nixpkgs> {} }:

let
  # PX4 NuttX targets require GCC 10 arm-none-eabi toolchain
  gccPkgs = import (builtins.fetchTarball {
    url = "https://github.com/NixOS/nixpkgs/archive/05ae01fcea6c7d270cc15374b0a806b09f548a9a.tar.gz";
    sha256 = "1c629ncdqdd1y5h8b3pm3cn2sa0gyinlam4jncbrp1m7pvsr02ji";
  }) {};

  # PX4 format target requires astyle <= 3.1
  astyle31 = pkgs.stdenv.mkDerivation rec {
    pname = "astyle";
    version = "3.1";
    src = pkgs.fetchurl {
      url = "mirror://sourceforge/astyle/astyle/astyle%20${version}/astyle_${version}_linux.tar.gz";
      sha256 = "1ms54wcs7hg1bsywqwf2lhdfizgbk7qxc9ghasxk8i99jvwlrk6b";
    };
    sourceRoot = "astyle/build/gcc";
    makeFlags = [ "prefix=${placeholder "out"}" ];
    installPhase = ''
      install -Dm755 bin/astyle $out/bin/astyle
    '';
  };

in pkgs.mkShell {
  nativeBuildInputs = with pkgs; [
    cmake
    ninja
    gcc
    gnumake
    git
    pkg-config
  ];

  buildInputs = with pkgs; [
    python3
    python3Packages.pip
    python3Packages.virtualenv

    # ARM cross-compiler
    gccPkgs.gcc-arm-embedded

    # NuttX build dependencies
    automake
    bison
    flex
    genromfs
    gettext
    gperf
    libelf
    expat
    gmp
    isl
    libmpc
    mpfr
    ncurses
    libtool
    texinfo
    ubootTools

    # General build tools
    astyle31
    cppcheck
    gdb
    lcov
    libxml2
    rsync
    shellcheck
    unzip
    zip
  ];

  shellHook = ''
    export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath [ pkgs.stdenv.cc.cc.lib pkgs.zlib ]}''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export LOCALE_ARCHIVE="${pkgs.glibcLocales}/lib/locale/locale-archive"
    export CMAKE_ARGS="-DCMAKE_POLICY_VERSION_MINIMUM=3.5''${CMAKE_ARGS:+ $CMAKE_ARGS}"
    VENV_DIR="$PWD/.venv"
    if [ ! -d "$VENV_DIR" ]; then
      echo "Creating Python venv..."
      python3 -m venv "$VENV_DIR"
    fi
    source "$VENV_DIR/bin/activate"

    if ! python3 -c "import em" 2>/dev/null; then
      echo "Installing Python dependencies..."
      sed 's/>=3\.0\.\*/>=3.0/' Tools/setup/requirements.txt > /tmp/px4-requirements.txt
      pip install --quiet -r /tmp/px4-requirements.txt
      pip install --quiet 'setuptools<81'
      rm -f /tmp/px4-requirements.txt
    fi

    echo ""
    echo "PX4-ATS dev shell ready"
    echo "  Build: make cubepilot_cubeorange [upload]"
  '';
}
