# wacohtech_force_torque_sensor
Device driver for Wacohtech 6-axis force/touque sensor written in Rust.  
This driver can deal with WDF-6M200-3, a sensor by Wacohtech under Linux environment.

# Install
1. Download this repository.
1. Setup udev rule for Wacohtech sensor via ```setup_udev_rule.sh```.
1. Connect a sensor to Linux PC.
1. Build by ```cargo build``` where this repository downloaded.
1. Run a demonstration by ```cargo run --example demo```.
