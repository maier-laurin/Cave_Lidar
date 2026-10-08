// Cave Lidar - Engineering Documentation
// Build: typst compile cave_lidar.typ     Live: typst watch cave_lidar.typ
// Put images and diagrams in ./figures

#set document(title: "Cave Lidar - Engineering Documentation", author: "Laurin Maier")
#set page(paper: "a4", margin: (x: 20mm, y: 2.5cm), numbering: "1")
#set text(font: "New Computer Modern", size: 11pt, lang: "en")
#set par(justify: true)
#set heading(numbering: "1.1")
#show link: set text(fill: blue)

// Marker for things still to be filled in
#let todo(body) = text(fill: red, weight: "bold")[TODO: #body]

#align(center)[
  #text(size: 22pt, weight: "bold")[Cave Lidar] \
  #v(2pt)
  #text(size: 14pt)[Engineering Documentation] \
  #v(6pt)
  Laurin Maier \
  #datetime.today().display("[day].[month].[year]")
]

#v(1em)
#outline(indent: auto)
#pagebreak()

= Overview

A mobile rotating lidar rig for mapping caves. A spinning lidar is rotated about a second axis by a stepper drive, and the sensor pose (IMU) is fused with the lidar data to reconstruct a point cloud. This document describes the hardware, the control software, and the reconstruction and viewing software.

#todo[Project goals, requirements, intended use, system block diagram.]

= Hardware

== Lidar

Velodyne HDL lidar on a 3D-printed mount, rotated about the horizontal shaft axis.

#todo[Exact lidar model, mass, power supply, interface.]

== Rotation mechanism

The lidar is mounted on a shaft carried by two 3D-printed bearing brackets on an aluminium T-slot frame. A stepper motor with planetary gearbox drives the shaft through a toothed belt (GT2-type, marked 250-6). A slip ring at the end of the shaft carries power and data to the rotating lidar without limiting the rotation angle.

#figure(
  image("figures/Drawings-welle.pdf", width: 170mm),
  caption: [Main rotation axis, section view. Dimensions in mm. Drawn at 1:1 (170 mm wide).],
) <fig-axis>

#block(breakable: false, table(
  columns: (auto, 1fr),
  align: (center, left),
  stroke: 0.5pt + gray,
  [*Item*], [*Description*],
  [A], [Bearing at the drive end of the shaft],
  [B], [Toothed belt pulley (driven by the stepper)],
  [C], [Hollow shaft, 25 mm diameter],
  [D], [Index flag for the light barrier],
  [E], [Bearing with sleeve on the lidar side],
  [F], [Lidar mount, angled at $165 degree$ to the shaft axis],
))
#todo[Confirm item descriptions, bearing type and size, shaft material and wall thickness.]

The shaft is carried by two bearings 130 mm apart (@fig-axis). The lidar mount extends 217.1 mm beyond the lidar-side bearing.

#figure(
  rect(width: 100%, height: 5cm, stroke: 0.5pt + gray)[
    #align(center + horizon)[#todo[Insert photo of the assembled rig, e.g. `image("figures/rig_photo.jpg")`]]
  ],
  caption: [Assembled rotation mechanism.],
)

=== Drive and ratio

The total reduction from motor to lidar shaft (gearbox and belt together) was measured with the index sensor:

#table(
  columns: (auto, auto),
  align: (left, right),
  stroke: 0.5pt + gray,
  [*Quantity*], [*Value*],
  [Microstepping], [16 (3200 steps per motor revolution)],
  [Measured steps per lidar revolution], [31999 (mean of 5 revolutions)],
  [Total reduction], [10.00 : 1],
  [Revolution-to-revolution spread], [25 steps $approx 0.28 degree$],
)

#todo[Gearbox ratio, pulley tooth counts, belt length and width, motor type and rated torque.]

=== Belt tensioning

The motor sits on a slotted 3D-printed mount that slides on the extrusion, so the belt centre distance can be adjusted and locked.

#todo[Measured belt tension / pluck frequency, final motor mount design.]

== Index sensor

A light barrier detects the passage of a flag on the lidar shaft once per revolution. It sits after the belt on the lidar shaft, so skipped belt teeth appear as a changed step count per revolution. The sensor edge occurs when the lidar is $93 degree$ (8266 steps) before vertical in the scan direction.

== Electronics

#todo[Teensy 4.1, MKS SERVO42C driver, BNO085 IMU, power supply, battery.]

== Rig wiring

#todo[Wiring diagram.]

The Teensy (3.3 V logic, not 5 V tolerant) is powered over USB only. Never connect it to the 12 V supply and never apply 5 V to any of its pins. Teensy GND and the 12 V supply GND must be connected.

=== Stepper driver (MKS SERVO42C)

Closed-loop driver on the back of a NEMA 17 stepper (ZT-42HS3922-1503, 1.8°, about 1.5 A). The motor coils (A+, A−, B+, B−) connect to the driver through its 4-pin connector.

#table(
  columns: (auto, auto, auto, 1fr),
  stroke: 0.5pt + gray,
  [*Driver pin*], [*Label*], [*Connect to*], [*Notes*],
  [V+], [VIN], [12 V supply +], [12–24 V allowed],
  [Gnd], [GND], [12 V supply − and Teensy GND], [Common ground required],
  [Com], [VCC], [not connected], [Reads about 3 V floating. If steps are missed, connect to Teensy 3.3 V],
  [En], [EN], [Teensy pin 4], [Ignored while menu `En = Hold`],
  [Stp], [CLK], [Teensy pin 2], [Step pulses],
  [Dir], [DIR], [Teensy pin 3], [HIGH/LOW selects direction],
)

=== IMU (BNO085, I2C)

Adafruit BNO085, mounted rigidly on the frame next to the motor mount, so it measures the frame pose. I2C address `0x4A` (`0x4B` if the address jumper is bridged). BT, P0, P1, DI, CS and 3Vo stay unconnected.

#table(
  columns: (auto, auto, auto, 1fr),
  stroke: 0.5pt + gray,
  [*BNO085 pin*], [*STEMMA QT wire*], [*Teensy*], [*Notes*],
  [VIN], [red], [3.3 V], [Board has its own regulator],
  [GND], [black], [GND], [],
  [SDA], [blue], [pin 18 (SDA0)], [`Wire`, 400 kHz],
  [SCL], [yellow], [pin 19 (SCL0)], [],
  [RST], [grey], [pin 17], [Hardware reset, used by the library],
  [INT], [purple], [pin 16], [Wired but unused, never driven],
)

*Known issue:* after uploading a new sketch the BNO085 sends no data (even though `begin_I2C` succeeds) until the USB cable is unplugged and replugged. A cold start works. The reset pin is verified working, so the cause is the BNO state after an upload. Workaround: replug USB after each upload.

=== Light barrier (index sensor)

One pulse per lidar-shaft revolution, used for homing and the index edge.

#table(
  columns: (auto, auto, 1fr),
  stroke: 0.5pt + gray,
  [*Wire*], [*Teensy*], [*Notes*],
  [red (power)], [3.3 V], [Not 5 V: the Teensy input is not 5 V tolerant],
  [black (GND)], [GND], [],
  [green (signal)], [pin 5], [`INPUT_PULLUP`, interrupt on the falling edge (output goes LOW at the index)],
)

=== Velodyne VLP-16 GPS simulation

The Teensy imitates a GPS receiver so the VLP-16 timestamps share one clock with the IMU. Simulated time starts at 2000-01-01 00:00:00 UTC at Teensy boot.

#table(
  columns: (auto, auto, auto, auto, 1fr),
  stroke: 0.5pt + gray,
  [*Signal*], [*Jumper*], [*Velodyne GPS pin*], [*Teensy 4.1*], [*Notes*],
  [PPS], [green], [1], [pin 10], [3.3 V, 10 ms pulse every second, rising edge = top of the second],
  [GND], [white], [3], [GND], [],
  [NMEA], [yellow], [4], [pin 1 (TX1, `Serial1`)], [`$GPRMC` at 9600 baud 8N1, sent 100 ms after each PPS edge],
)

`Serial1` is a hardware UART separate from the USB port, so programming and the PC data stream are not affected.

= Control software

Firmware for the Teensy, in `firmware/`. The control sketch is `firmware/lidar_ctrl/lidar_ctrl.ino`.

== Step generation and calibration

The firmware generates step pulses with a ramped step rate. On start-up the number of steps per lidar revolution is known from calibration (31999). It can be re-measured with the `c` command, which averages 5 revolutions between index edges.

== Homing and zero position

The `h` command runs to the index edge and then moves a fixed offset to bring the lidar vertical. The offset was found by jogging the lidar with the serial commands until vertical:

$ theta_"offset" = 93 degree = 8266 "steps" $

== Operation

Continuous rotation at 6 rpm (the lidar supports 5 to 10 rpm). Each revolution the firmware prints the step count and its deviation from the calibrated value. A persistent deviation indicates belt slip or skipped steps.

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt + gray,
  [*Command*], [*Function*],
  [`c`], [Calibrate steps per revolution],
  [`h`], [Home to the index edge, move offset, hold],
  [`+` `-` `[` `]`], [Jog and adjust the offset by $plus.minus 1 degree$ / $plus.minus 10 degree$],
  [`g`], [Continuous rotation at 6 rpm],
  [`x`], [Stop and hold],
  [`r`], [Reverse direction],
)

#todo[Time synchronisation between lidar packets, IMU and step counter; data output format.]

= Reconstruction and viewing software

Post-processing in `PostProcessing/` (Julia). Poses are represented with dual quaternions.

#todo[Coordinate frames, calibration of the lidar-to-IMU transform, fusion of IMU orientation with the shaft angle, point cloud reconstruction pipeline, viewer.]

= Appendix

== Change log

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt + gray,
  [*Date*], [*Change*],
  [2026-10-04], [Index sensor, step calibration (10:1) and vertical offset (93°) measured.],
)
