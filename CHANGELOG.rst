^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_component_models
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-11-08)
------------------
* The first release: display models of DiffBot's swivel caster (three meshes), a 360 degree
  laser scanner of the RPLIDAR A2's size, the Raspberry Pi 4 Model B, the Jetson Nano Developer
  Kit B01, the Teensy 4.0 with two 14-pin headers, a 400-point breadboard and the Raspberry Pi
  Camera Module v2. Each mesh comes as STL, COLLADA and glTF (coloured), all in metres, in a
  local frame documented in the manifest and the part's provenance page
* The Raspberry Pi 4 Model B and the Jetson Nano Developer Kit B01 in detail: metal and
  solder mask materials with highlights, FR-4 board edges, connector shells with frames and
  spring fingers, pointed header pins, small parts with end caps, and the Jetson's heatsink
  after its drawing
* The boards' connector insides after the connector makers' drawings: metal walls, insulators,
  tongues with their contacts, the USB 2.0 springs, the RJ45 contact springs and the DC jack's
  centre pin and sleeve spring
* Both boards with every part a named node, so viewers can show them exploded: the
  processors' dies under their lids, the wireless chip under its shield, and the Jetson
  module's board, processor, memory and power parts, thermal pad, heatsink and screws
* Both boards checked against product photos, sizes and positions in the provenance pages:
  the Pi's processor lid, AV jack and flat cable connectors, the Jetson's black module, fan
  header, capacitors, J48, camera connectors and port widths after the photos
* The CadQuery generators, the sources of every model, meshes/manifest.yaml with checksums,
  units, frames and bounding boxes, and checks in CI
