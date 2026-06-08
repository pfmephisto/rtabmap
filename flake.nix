{
  description = "Name";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  }; # end of inputs

  outputs = {
    self,
    nixpkgs,
    flake-utils,
  }:
  # flake-utils.lib.eachDefaultSystem (system:
    flake-utils.lib.eachSystem ["x86_64-linux"] (
      system: let
        inherit (nixpkgs) lib;

        # Import nixpkgs with custom configurations and overlays
        pkgs = import nixpkgs {
          inherit system;

          # Set systm comfigurations such as CUDA support and unfree packages
          config = {
            cudaSupport = true;
            allowUnfree = true;
            hardware.nvidia.open = false;
            allowUnfreePredicate = pkg:
              builtins.elem (lib.getName pkg) [
              ];
          };

          # Set overlays and custom fixes for broken packages
          #overlays =
          #  if builtins.pathExists ./pkgs then
          #    [
          #      (
          #        final: prev:
          #        (prev.lib.packagesFromDirectoryRecursive {
          #          callPackage = prev.lib.callPackageWith final;
          #          directory = ./pkgs;
          #        })
          #      )
          #    ]
          #  else
          #    [ ];
        };
      in {
        formatter = nixpkgs.legacyPackages.${system}.alejandra;

        #packages =
        #  if builtins.pathExists ./pkgs
        #  then
        #    pkgs.lib.packagesFromDirectoryRecursive {
        #      callPackage = pkgs.lib.callPackageWith pkgs;
        #      directory = ./pkgs;
        #    }
        #  else {}; # end of packages

        devShells = {
          default = pkgs.mkShell {
            # inputsFrom = [
            #   self.packages.${system}.default
            # ];

            packages = with pkgs; let
              pcl' = pcl.override {vtk = vtkWithQt6;};

              # GTSAM is not packaged in nixpkgs, so build it from source (borglab 4.2.1).
              # Using the system Eigen is required so it matches the Eigen used by RTAB-Map.
              gtsam = stdenv.mkDerivation rec {
                pname = "gtsam";
                version = "4.2.1";
                src = fetchFromGitHub {
                  owner = "borglab";
                  repo = "gtsam";
                  rev = version;
                  hash = "sha256-POuU6u7v9ElprjBuAggtpW+7hPgKmsmQtAsfcEUuRok=";
                };
                nativeBuildInputs = [cmake];
                propagatedBuildInputs = [onetbb];
                buildInputs = [boost eigen metis];
                cmakeFlags = [
                  (lib.cmakeFeature "CMAKE_POLICY_VERSION_MINIMUM" "3.5")
                  "-DGTSAM_USE_SYSTEM_EIGEN=ON"
                  "-DGTSAM_USE_SYSTEM_METIS=ON"
                  "-DGTSAM_BUILD_TESTS=OFF"
                  "-DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF"
                  "-DGTSAM_BUILD_PYTHON=OFF"
                ];
              };
            in [
              cmake
              ninja

              qt6.wrapQtAppsHook
              pkg-config
              wrapGAppsHook3

              ## Required
              opencv
              opencv.cxxdev
              pcl'
              liblapack
              xorg.libSM
              xorg.libICE
              xorg.libXt

              ## Optional
              libusb1
              eigen
              g2o
              gtsam # graph optimization backend (enables -DWITH_GTSAM)
              boost # needed on the include path when compiling against GTSAM headers
              onetbb # GTSAM is built with TBB
              ceres-solver
              yaml-cpp
              libnabo
              libpointmatcher
              octomap
              freenect
              libdc1394
              librealsense
              qt6.qtbase
              qt6.qtsvg # required by guilib (GraphViewer uses QtSvg/QSvgGenerator)
              libGL
              libGLU
              zed-open-capture
              hidapi

              claude-code
              gh
            ];

            shellHook = ''
              echo "Entering dev shell"
              export VIRTUAL_ENV_PROMPT="Name"
            '';
          }; # end of default shell
        }; # end of devShells
      }
    ); # end of outputs
}
