{
  description = "mc_mujoco: integration between mc_rtc and the MuJoCo simulator";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.mc-rtc-nix.flakeModule
          {
            # Configuration for the mc-rtc-nix module
            # Activate overlays, disable ros, etc
            mc-rtc-nix = { };

            # If you need a superbuild environment configure it here
            mc-rtc-superbuild = { };

            # Override dependencies with flakoboros module
            flakoboros = {
              overrideAttrs.mc-rtc-imgui =
                { pkgs-final, ... }:
                {
                  src = pkgs-final.fetchFromGitHub {
                    owner = "mc-rtc";
                    repo = "mc_rtc-imgui";
                    rev = "6ac125c00ca5e18c7da80e0049dc5697f4f36a23";
                    hash = "sha256-J9uQpfn08Yv5ROxQmxHYwiVRdvZsVOp2klZmgnw90Bg=";
                  };
                };
              overrideAttrs.mc-mujoco =
                { drv-prev, pkgs-final, ... }:
                {
                  src = lib.cleanSource ./.;
                  nativeBuildInputs = drv-prev.nativeBuildInputs ++ [ pkgs-final.cli11 ];
                };
            };
          }
        ];
      }
    );
}
