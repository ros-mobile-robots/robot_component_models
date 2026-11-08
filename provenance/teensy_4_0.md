# Teensy 4.0 with two 14-pin headers

A display model of PJRC's Teensy 4.0 with two 14-pin male headers soldered into its long pin rows, as the board sits on Remo's breadboard. The model was created independently, from PJRC's public dimension drawing, parts placement drawings and product photos, and from the chip makers' public package drawings. No geometry was imported, converted, traced or sampled from another 3D model, and no vendor or third-party 3D model was downloaded. No logos, no pin labels and no other printed text. PJRC's photos were measured where they are published and aren't copied into this repository.

Generator: [`generators/teensy_4_0.py`](../generators/teensy_4_0.py).

## Mesh

`teensy_4-0`: one mesh with 20 materials (green solder mask, FR4 edges, gold pads and pins, copper test pads, solder, black ICs with pin-1 dimples, silver leads, the USB shell, grey latch slots, beige capacitors, black resistors, the button's cream cap, pale LEDs and others). Its sub-parts are the board, the pads, the header, the processor (`mcu`), the bootloader chip, the leaded parts, the inductor block, the passives on top and on the bottom, the crystals, the USB connector, the button, the test pads and the bottom's other features. Every part's solid is closed with outward normals.

## Frame

Millimetres in the STL file, metres in the COLLADA and glTF files. The board's bottom face is at z = 0, the components on +z. The long axis runs along x with the USB connector at -x: the board lies from x = -13.929 to 21.631 mm, centred at y = -0.032 mm. The row with the inner pin hole is at +y. The header's plastic spacer ends at z = -2.5 mm and the pins at z = -8.5 mm.

The frame is the frame of the Teensy mesh Remo used before. Only that mesh's bounding box was read, to align the frame: (-15.119, -8.882, 0.000) to (21.631, 8.818, 4.125) mm. This mesh's board ends at the same x = 21.631 mm, sits on the same centre line and has its bottom at the same z = 0. Its USB connector stands 0.7 mm beyond the board, as in PJRC's drawing, so its x minimum is -14.629 mm. Its top is 0.23 mm higher (the USB flange). The header below the board is new.

With Remo's URDF, the 28 header pins lie within 0.027 mm of the centres of the breadboard model's holes, in the horizontal plane.

## Sources

| Fact | Value used | Source |
|---|---|---|
| Board size and thickness | 35.56 × 17.78 mm (1.4 × 0.7 in), 1.57 mm | PJRC's dimension drawing [dimensions_teensy40.png](https://www.pjrc.com/teensy/dimensions_teensy40.png), on [pjrc.com/store/teensy40.html](https://www.pjrc.com/store/teensy40.html) |
| Pin grid | 2 × 14 holes at 2.54 mm, 1.27 mm from the edges. 5 holes across the far end, with the corner pins a column of 7. 1 inner hole one pitch in from the +y row, column 2 | The same drawing, top view |
| Hole diameter | 0.965 mm, 34 locations | teensy40.html, "Dimensions": "Finished hole diameter is 0.965mm (34 locations)" |
| Bottom pads | Pins 24 to 33: 2.794 × 0.889 mm, two columns 24.59 mm from the first pin column and 4.17 mm apart, 10.16 mm span. USB host 3.048 × 1.016 mm. SD 2.000 × 0.600 mm, 8 pads over 7.0 mm, 7.9 mm from the first pin column | The same drawing, bottom view, and the "Bottom pad sizes" text |
| Micro-USB | 0.7 mm beyond the board's edge, 7.5 mm wide, 2.5 mm high. Flange 8.06 × 3.07 mm, 0.6 mm thick. The shell's top ends at u = 4.3 mm, with the five signal leads behind it, two side tabs, two rear legs and two latch slots in the top | The drawing's top and side views, the rest from the front photo |
| Push button | Centre 29.21 mm from the first pin column (30.48 mm from the USB end). Metal frame 4.3 × 3.68 mm with four legs, plunger 3.3 × 2.7 mm | The same drawing, and the front photo |
| Processor | i.MX RT1062, 10 × 10 × 1.3 mm MAPBGA (1.43 mm max) | NXP IMXRT1060CEC Rev. 4, Figure 53 (page 91), [IMXRT1060CEC_rev4.pdf](https://www.pjrc.com/teensy/IMXRT1060CEC_rev4.pdf), linked from teensy40.html |
| Flash | W25Q16JVUXIM, USON-8 2 × 3 × 0.6 mm, U3 | Winbond's datasheet, section 10.1, [winbond_w25q16jvuxim.pdf](https://www.pjrc.com/teensy/winbond_w25q16jvuxim.pdf). U3 in PJRC's [schematic40.png](https://www.pjrc.com/teensy/schematic40.png) |
| Other parts | U2 (the bootloader chip), U4 TLV75733P (2 × 2 mm), Q1 DMG2305UX (SOT-23), D1, Y1 24 MHz, Y2 32.768 kHz, L1 4.7 µH, D3 (pin 13 LED), D4, S1, capacitors and resistors on both sides | Names from PJRC's schematic and the parts placement drawings [parts_placement_teensy40_top.png](https://www.pjrc.com/teensy/parts_placement_teensy40_top.png) and [parts_placement_teensy40_bottom.png](https://www.pjrc.com/teensy/parts_placement_teensy40_bottom.png). Positions, orientations, lead counts and pin-1 marks measured on PJRC's photos of the front, [teensy40_front.jpg](https://www.pjrc.com/store/teensy40_front.jpg), and of the back, [dimensions_teensy40_photo2.jpg](https://www.pjrc.com/teensy/dimensions_teensy40_photo2.jpg) (next section). Standard package sizes: SOT-23 2.9 × 1.3 mm, 0603 1.6 × 0.8 mm, 0402 1.0 × 0.5 mm, crystals 3.2 × 2.5 and 3.2 × 1.5 mm |
| Pin header | 0.64 mm (0.025 in) square posts at 2.54 mm, a 2.5 mm plastic spacer under the board, 6.0 mm of pin below the spacer, 1.43 mm above the board's top | A standard 0.1 in male header. The 2.5 mm spacer is in Remo's URDF (the comment on the `mcu` joint). The pin lengths are a common 11.5 mm header, assumed |
| Colours | Green solder mask 0.10 0.42 0.22 (Remo's colour for the Teensy), gold, solder, black ICs | PJRC's photos |

## How the parts were measured

Board coordinates: u from the USB end, v from the +y edge (the top of PJRC's front photo), in mm.

- **Photo frames**: each photo's pixel frame is an affine map fitted to its 28 long-row pin holes, whose positions the drawing gives. Front: x = 20.3234 + 12.2208 u + 0.0205 v, y = 2.7140 - 0.0029 u + 12.2345 v, radial RMS 0.16 px. Back: x = 20.4725 + 7.8765 u + 0.0193 v, y = 150.2520 + 0.0276 u - 7.8907 v, radial RMS 0.22 px.
- **Profiles** (capacitors, LEDs, resistors, Y2 and the inductors): two profiles through the part's centre, along u and v, each averaged over a 0.3 mm band of the four times upsampled photo. A pixel belongs to a part if its colour differs from the nearby board by more than about half the part's contrast. The outline is the part with the solder on its pads.
- **Edges** (the bodies of U1, U2 and Q1): each body edge is the first point, going outwards, brighter than halfway between the body and its surroundings, so the print on the chip doesn't count. The leads are left out.
- **Grid** (S1, D1, U3, U4, C2, Y1 and, on the back, R3, X1, C23 and C15): read by eye on 0.1 mm grid crops, where neighbouring parts merge or silkscreen lines lie next to a part.
- **Uncertainty**: one photo pixel is 0.082 mm on the front and 0.127 mm on the back. Expect about ±0.1 mm for any part's centre or size, more on the back where neighbours merge. An independent second reading of 20 parts differed by at most 0.08 mm in centre and 0.10 mm in size.

| Part | Side | Centre u, v (mm) | Outline u × v (mm) | Method |
|---|---|---|---|---|
| D4 | front | 5.82, 5.08 | 0.70 × 1.66 | profiles |
| C25 | front | 5.84, 8.90 | 0.92 × 2.31 | profiles |
| C8 | front | 5.87, 11.19 | 0.74 × 1.49 | profiles |
| C7 | front | 8.39, 7.46 | 1.55 × 0.74 | profiles |
| R4 | front | 8.85, 12.07 | 1.55 × 0.71 | profiles |
| C26 | front | 8.82, 13.37 | 2.35 × 0.98 | profiles |
| C1 | front | 22.94, 4.54 | 0.49 × 1.47 | profiles |
| C2 | front | 22.91, 6.14 | 0.53 × 1.43 | grid |
| R5 | front | 26.97, 5.08 | 0.74 × 1.51 | profiles |
| C3 | front | 28.61, 4.04 | 1.51 × 0.74 | profiles |
| C4 | front | 30.41, 4.04 | 1.51 × 0.74 | profiles |
| C27 | front | 26.28, 8.99 | 2.19 × 0.92 | profiles |
| C28 | front | 29.31, 12.54 | 0.98 × 2.08 | profiles |
| D3 | front | 32.29, 5.07 | 0.98 × 2.35 | profiles |
| L2 | back | 12.74, 7.82 | 0.89 × 2.38 | profiles |
| F1 | back | 13.19, 5.24 | 1.08 × 2.54 | profiles |
| R2 | back | 14.81, 5.52 | 1.71 × 0.67 | profiles |
| R1 | back | 15.78, 4.44 | 1.62 × 0.67 | profiles |
| R3 | back | 16.32, 12.89 | 1.49 × 0.62 | grid |
| X1 | back | 17.53, 12.92 | 0.56 × 0.50 | grid |
| L3 | back | 22.62, 13.85 | 2.44 × 0.73 | profiles |
| C29 | back | 22.60, 12.10 | 2.41 × 0.92 | profiles |
| C17 | back | 23.62, 10.74 | 0.64 × 1.49 | profiles |
| C6 | back | 15.99, 9.47 | 1.55 × 0.63 | profiles |
| C5 | back | 17.77, 9.47 | 1.55 × 0.63 | profiles |
| C16 | back | 21.28, 9.48 | 1.59 × 0.63 | profiles |
| C23 | back | 14.31, 8.24 | 1.51 × 0.70 | grid |
| C12 | back | 16.00, 8.17 | 1.55 × 0.57 | profiles |
| C13 | back | 17.77, 8.19 | 1.55 × 0.60 | profiles |
| C14 | back | 19.52, 8.16 | 1.55 × 0.67 | profiles |
| C15 | back | 21.29, 8.22 | 1.56 × 0.62 | grid |
| C22 | back | 15.71, 6.87 | 1.62 × 0.76 | profiles |
| C11 | back | 19.14, 6.83 | 1.62 × 0.67 | profiles |
| C10 | back | 21.03, 6.87 | 1.59 × 0.60 | profiles |
| C9 | back | 22.97, 6.04 | 1.59 × 0.63 | profiles |
| C20 | back | 22.99, 4.72 | 1.55 × 0.73 | profiles |
| C21 | back | 20.91, 4.91 | 1.52 × 0.67 | profiles |
| C24 | back | 20.52, 13.33 | 1.55 × 0.67 | profiles |
| C18 | back | 20.54, 12.18 | 1.59 × 0.57 | profiles |
| L1 | front | 26.89, 12.30 | 2.17 × 3.04 | profiles |
| Q1 | front | 8.49, 9.49 | 2.90 × 1.31 | edges |
| D1 | front | 31.36, 12.44 | 1.32 × 1.10 | grid |
| U3 | front | 24.08, 12.03 | 2.00 × 3.07 | grid |
| U4 | front | 5.84, 13.54 | 1.94 × 2.02 | grid |
| U2 | front | 8.81, 5.06 | 3.09 × 3.00 | edges |
| U1 | front | 17.18, 8.87 | 9.94 × 9.91 | edges |
| Y1 | front | 24.88, 5.56 | 2.88 × 3.60 | grid |
| Y2 | front | 29.49, 5.57 | 3.58 × 1.84 | profiles |
| S1 | front | 30.46, 8.87 | 5.03 × 3.67 | grid |

The generator's lists (`TOP_PASSIVES`, `BOTTOM_PASSIVES`, `TOP_BLOCKS`, `LEADED`, `CRYSTAL_*`, `BUTTON*`) hold these values. Every two-terminal part, the crystals and L1 have a flat solder outline as large as their outline above, under a body of the real package's size. R1 to R4 are 0402 parts. C25 to C29, D3, L2 and L3 are 0603 parts. X1, a small light feature on the back without a label in PJRC's drawing, is modelled as a small light part: the photo doesn't show whether it is a component or a test pad.

## Simplified or assumed

- No silkscreen (pin labels, outlines, pin-1 dots, logo), no traces, only the larger via pads on the back.
- Small parts: positions and outlines measured on the photos, standard package sizes, boxes with metal ends on a flat solder outline. The processor has no balls, only a dark layer under its substrate.
- The USB receptacle is simplified: shell, chamfered opening, tongue, tabs and legs as boxes.
- Headers on the two long rows only, 28 pins. The five end pins and the inner pin have no header. The pin lengths are assumed.
- Corner radius 0.4 mm, assumed.

## Measurements that would improve the model

1. A photo of the Teensy from the side and from below, off the breadboard: which pins carry headers.
2. The header's total pin length and spacer height, and whether the long or the short ends go into the breadboard: a side photo with a ruler.
