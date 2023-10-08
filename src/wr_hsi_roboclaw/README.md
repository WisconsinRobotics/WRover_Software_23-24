# wr_hsi_roboclaw

@defgroup wr_hsi_roboclaw wr_hsi_roboclaw
@brief Configuration of the WRover for the `wroboclaw` package

This package controls the WRover configuration for the motors controlled by `wroboclaw`, including the mock mode and Freefall.

## Launching

* Real Mode: launch from `roboclaw_real.launch`
* Mock Mode: launch from `roboclaw_mock.launch`
* Freefall Mode: launch from `roboclaw_freefall.launch`

All three of these launch files can take a launch argument to `use_encoders`, which defaults to `true` (except for Freefall).
