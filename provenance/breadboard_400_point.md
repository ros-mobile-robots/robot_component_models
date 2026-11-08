# 400-point solderless breadboard

A display model of a half-size solderless breadboard: 300 terminal holes (30 columns, rows a to e and f to j on both sides of the centre channel) and four 25-hole power rails with red and blue lines, dovetail tabs and slots, on its adhesive foam backing. Remo's Teensy and IMU boards plug into it. The model was created independently, from BusBoard's public BB400 datasheet, the 0.1 in standard and the positions of the boards that plug into it. No geometry was imported, converted, traced or sampled from another 3D model, and no third-party 3D model was downloaded. No numbers, letters, "+" or "-" signs and no logos.

Generator: [`generators/breadboard_400_point.py`](../generators/breadboard_400_point.py).

## Mesh

`breadboard_400_point`: one mesh with 5 materials: the white body, dark hole walls and floors (`hole`), the red and blue lines and the foam backing (`tape`). Its sub-parts are the body, the backing and the rail lines.

## Frame

Millimetres in the STL file, metres in the COLLADA and glTF files. The bottom of the foam backing is at z = 0 and the top at z = 9.5 mm. The centre channel runs along y at x = -0.16 mm. The hole columns lie at y = -0.335 mm + n × 2.54 mm. The dovetail tabs are on the -x and -y sides, the slots on +x and +y.

The frame is the frame of the breadboard mesh Remo used before. Only that mesh's bounding box was read, to align the frame: (-28.826, -46.371, 0.000) to (27.254, 36.639, 9.510) mm. This mesh spans (-28.810, -46.795, 0.000) to (27.090, 37.105, 9.530) mm: 82.5 mm long instead of about 81.7 mm. The earlier box reaches 1.3 mm further on -x and -y than on +x and +y, so the tabs are on -x and -y.

### Hole grid

The hole grid isn't taken from the earlier mesh. It puts the header pins of the Teensy 4.0 and the BNO055 IMU into holes, at their positions in Remo's URDF:

- Teensy: PJRC's 0.1 in pin grid, placed with Remo's `mcu` joint (`-0.0414 -0.0206 0.012`, yaw -90°). Its two rows land at x = -9.052 and +6.188 mm in the breadboard's frame, its columns at y = -7.941 - 2.54 k mm.
- BNO055: the pin positions in Adafruit's published board layout (header JP1 with 6 pins at y = 1.27 mm, JP2 with 4 pins at y = 19.05 mm), moved by (-13.717, -10.217) mm into the frame of Remo's IMU mesh and placed with Remo's `imu_xyz` (`-0.0401 0.0234 0.013`, yaw -90°). Rows at x = -9.047 and +8.733 mm, columns at y = 30.132 - 2.54 k and 27.592 - 2.54 k mm.
- On a standard board the rows sit 0.15, 0.25 … 0.55 in from the channel's middle. Both boards then put the channel's middle at x = -0.157 to -0.162 mm: `CHANNEL_X = -0.16`. The columns: `COLUMN_Y0 = -0.335`, the mean of the two boards' grid offsets. The 30 columns run from y = -40.975 to 32.685 mm.

Result: in the horizontal plane, all 28 Teensy pins lie within 0.027 mm and all 10 BNO055 pins within 0.013 mm of a hole's centre. The check is 2D: the BNO055's height on the breadboard is the URDF's.

## Sources

| Fact | Value used | Source |
|---|---|---|
| Layout | 300-point terminal area (30 × 10, a to e and f to j), four 25-point power rails, 0.1 in (2.54 mm) pitch, square holes, 0.025 in posts | BusBoard BB400 datasheet, Rev 7, [BPS-DAT-(BB400)-Datasheet.pdf](https://www.busboard.com/documents/datasheets/BPS-DAT-(BB400)-Datasheet.pdf), pages 1 and 2. Product page [busboard.com/BB400](https://www.busboard.com/BB400) |
| Rows across | a to e span 0.4 in, e to f 0.3 in (the channel), so the rows sit 0.15 to 0.55 in from the middle | BB400 drawing "Measurements" (page 2): 0.400" and 0.300" |
| Rails | Two per side, 0.1 in apart, at 0.8 and 0.9 in from the middle. 25 holes in 5 groups of 5, centred lengthwise, half a pitch off the terminal columns | BB400 drawing: 0.100", the 0.370" rail strip and the 1.36" terminal area. The rail positions and the half-pitch offset measured on the drawing, scaled by its 0.4 in and 2.9 in spans |
| Strip seams | Between the terminal area and the rail strips, 1.36 in apart | BB400 drawing: 1.36" (34.54 mm) |
| Lines | Red and blue, one on each side of each rail pair, at 18.5 and 24.8 mm from the middle | Positions measured on the BB400 drawing. BusBoard prints red and black lines. The model uses red and blue, the common colouring |
| Size | 82.5 × 54.5 mm, body 8.5 mm | A typical generic half-size board, chosen. BB400: 84 × 54.3 × 8.5 mm |
| Foam backing | 1.0 mm, 0.3 mm inset, so the top is at 9.5 mm | BB400: "Peelable adhesive tape backing". The thickness is assumed, so that the top stays at Remo's 9.5 mm (the boards lie 2.5 mm above it, at z = 12 mm) |
| Channel | 2.6 mm wide, 4.5 mm deep | Width from the BB400 drawing (about 0.1 in), depth assumed |
| Dovetails | Tabs 1.4 mm deep, 2.6 mm at the neck, 3.6 mm at the tip, 7.5 mm high. 2 on the long side, 4 on the short side, matching slots opposite | Typical, from BB400's photos. Sizes and count assumed |
| Pin positions of the boards on it | Teensy: 0.1 in grid from PJRC's [dimension drawing](https://www.pjrc.com/teensy/dimensions_teensy40.png). BNO055: the pin positions in Adafruit's board layout `Adafruit BNO055.brd` in [Adafruit-BNO055-Breakout-PCB](https://github.com/adafruit/Adafruit-BNO055-Breakout-PCB) | Remo's URDF in [remo_description](https://github.com/ros-mobile-robots/remo_description) for their positions |

## Simplified or assumed

- The holes are straight 1.1 mm squares, dark from 1.6 mm down. The spring contacts aren't modelled. Real openings have a small funnel.
- The red and blue order on each side follows BusBoard's drawing: on the -x strip red is outside, on the +x strip blue is outside.
- The rail strips aren't separate parts, only marked by seams. The underside is flat.
- The brand and exact size of Remo's breadboard are unknown. The model is a generic half-size board.

## Measurements that would improve the model

1. The length, width and height of Remo's breadboard, with and without the foam backing (calipers), and the brand if printed on it.
2. A top photo of the breadboard on Remo: which side the red line is on and where the tabs are.
