# Entry Point

Launch files and stuff that are used to start up the robot.

## Param Env Vars

Env vars expected as params to the launch process.

* `WROVER_LOCAL: bool` -- if true, the entire system is launched on the local machine (e.g. if testing without the physical rover)
* `WROVER_HW: "REAL" | "MOCK"` -- the type of robot hardware to use for the launch

## Set Env Vars

Env vars set by the launch process that are usable by nodes.

* `WROVER_MODE: "ERDM" | "EQ_SERVICE" | "AUTO_NAV" | "SCIENCE"` -- the current robot configuration
