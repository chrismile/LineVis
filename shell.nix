let
  nixpkgs = fetchTarball "https://github.com/NixOS/nixpkgs/tarball/nixos-25.11";
  pkgs = import nixpkgs { config = {}; overlays = []; };
in

pkgs.mkShell {
  packages = with pkgs; [
    cmake
    git
    curl
    pkg-config
    patchelf
    boost
    icu
    glm
    libarchive
    tinyxml-2
    libpng
    sdl3
    sdl3-image
    glew-egl
    vulkan-headers
    vulkan-loader
    vulkan-validation-layers
    shaderc
    glslang
    opencl-headers
    ocl-icd
    jsoncpp
    eigen
    python3
    zeromq
    cppzmq
    netcdf
    openexr_3
    eccodes
  ];

  BUILD_USE_NIX = "ON";

  shellHook = ''
    echo "Run ./build.sh to build the application with Nix."
  '';
}
