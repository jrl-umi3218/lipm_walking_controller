# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

## [1.7.1] - 2026-07-04

## [1.7.0] - 2026-07-04

## [1.6.0] - 2019/12/11

### Added

- Footstep interpolator

### Fixed

- Terminology regarding pressure and foot force ratios
- ZMP frame initialization

## [1.5.0] - 2019/12/10

### Added

- Docker image with the complete controller environment
- Documentation: write build instructions
- Stabilizer: set wrench distribution QP weights from GUI

### Fixed

- Contact: read ankle transform from robot model (remove hard-coded values)
- Improved solve time of wrench distribution QP from 200-400 us to 5-20 us
- Sole dimensions for JVRC-1 model

### Changed

- Removed dependency on ``eigen-lssol``
- Stabilizer: switched solver of wrench distribution QP from LSSOL to QuadProg

## [1.4.0] - 2019/11/18

### Fixed

- Stabilizer: distribute wrenches w.r.t. [measured rather than reference
  CoM](https://github.com/stephane-caron/lipm_walking_controller/issues/28)

## [1.3.0] - 2019/11/14

### Added

- [JVRC-1](https://github.com/jvrc/model/) robot model configuration
- Doxygen [API documentation](https://scaron.info/doc/lipm_walking_controller/)
- ModelPredictiveControl: log ZMP and CoM velocity references
- Stabilizer: alternative DCM gain tuning by pole placement under ZMP lag model

### Fixed

- Add ``EIGEN_MAKE_ALIGNED_OPERATOR_NEW`` to all structs
- Anti-aliasing condition for EMA and LPF filters
- Handle corner case where the DSP duration is zero
- Initialization of realCom and realComd

### Changed

- Contact: removed ``t()``, ``b()`` and ``n()`` shorthands
- Moved torso pitch setting to "torso" dictionary of robot config
- Moved world definitions to ``utils/world.h``
- Renamed ModelPredictiveControlSolution to Preview
- Switch license to BSD 2-clause for compatibility with other mc\_rtc projects
- Updated dependencies

## [1.2.0] - 2019/10/11

The main change in this release is the switch from ZMP to DCM derivative term
in DCM tracking, following [Biped locomotion control for uneven terrain with
narrow support region](https://doi.org/10.1109/SII.2014.7028007) (Morisawa _et
al._, 2014). The DCM derivative is computed from DCM and ZMP measurements using
a model-based estimator.

### Added

- Default footstep plans and settings for HRP-2Kai
- Model-based DCM derivative estimator ("derivator")
- Sample curved footstep plans
- StationaryOffsetFilter class used by DCM derivator

### Fixed

- Initialization of reference pendulum ZMP
- Stabilizer: damping term of foot force difference control
- Stabilizer: formula of proportional term in desired CoM acceleration

### Changed

- Cleaned up legacy code
- Footstep plans are now robot-specific
- Removed legacy mass estimation phase
- Stabilizer: switch from ZMP to DCM derivative term in DCM tracking
- Updated CMake configuration
- Updated copra and mc\_rtc dependencies

## [1.1.0] - 2019/04/15

This release corresponds to the stair climbing and walking control that ran in
the final demonstrator of the COMANOID project.

## [1.0.0] - 2018/10/17

First public release of the controller.
