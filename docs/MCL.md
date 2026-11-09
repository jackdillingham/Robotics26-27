# MCL setup

The LemLib project creates the four-sensor MCL instance in `src/main.cpp`, starts
its background task after chassis calibration, and resets/enables it at the start
of each driving autonomous through `start_mcl_auton()`. Disabled mode, driver
control, the lift-only test, and routine completion disable corrections.

Sensor entries are `{sensor, forward_inches, right_inches, heading_degrees}`.
All `0, 0` mounting offsets are placeholders. Measure from the odometry center
to each sensor lens and verify the headings: front 0, right 90, back 180, left 270.
Only then set `MCL_GEOMETRY_CONFIGURED = true`. Until then autonomous continues
with ordinary LemLib odometry and MCL performs no corrections.

Field coordinates are centered: +Y forward at heading 0, +X at heading 90.
`MCL_METAL_PERIMETER` selects 140.50-inch metal (walls at +/-70.25) or
140.40-inch portable plastic (walls at +/-70.20). These are nominal inside
wall-to-wall dimensions from the final perimeter specification drawings in
[VEX Override v2.1 Appendix A](https://manuals.vex.com/v5/override/v2.1/appendix-a/).
Actual assembled fields have tolerances. Select the perimeter used at the event.

Review: the class converts millimeters to inches, rotates sensor offsets and
beams consistently with LemLib, intersects the nearest wall, handles heading
wrap, requires nonparallel agreeing sensors, limits correction magnitude, and
serializes filter resets/updates. Existing start poses lie inside both sizes.
`src/mcl.cpp` and `include/mcl.hpp` needed no algorithm changes.

This is a local filter around a known start pose, not global localization. It
models perimeter walls only. Mount sensors so their beams see the wall panels;
obstacles, seams, and readings outside the configured 2–78-inch range can prevent
corrections. In particular, diagonal wall distances near field center exceed that
range. Check measured sensor geometry and tune on the robot before competition.

Validation: all robot sources compiled and the hot/cold PROS packages linked.
The unused `static/example.txt` asset remains on disk but is excluded from the
build because its exported size symbol conflicts with the cold library image.
