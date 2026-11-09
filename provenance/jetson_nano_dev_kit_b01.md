<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# Jetson Nano Developer Kit B01

A display model of the NVIDIA Jetson Nano Developer Kit B01: the carrier board P3449-B01 with the module P3448 and its heatsink. Remo carries one. The model was created independently, from NVIDIA's public documents and pictures, a public forum post, the public drawing of the kit's heatsink, the connector makers' public drawings for the insides of the connectors and Waveshare's product photo for a position and size check. No geometry was imported, converted, traced or sampled from another 3D model. NVIDIA's own 3D model of the kit is behind a developer login and wasn't downloaded or looked at. No logos and no printed text.

Generator: [`generators/jetson_nano_dev_kit_b01.py`](../generators/jetson_nano_dev_kit_b01.py).

## Mesh

`jetson_nano_dev_kit_b01`: one mesh with 18 materials: semi-gloss black solder mask on the carrier and the module, pale FR-4 on the boards' cut edges, black anodised aluminium on the heatsink, nickel connector shells and screws, gold pins, contacts and pads, tin solder joints and contact rows, brass standoffs, black plastic, chip packages, the processor's silicon die and substrate, the inductors, the grey thermal pad, blue USB 3.0 tongues, brown latches, the capacitors' grey polarity stripe and two LED colours. The metals are metallic in the glTF file. In the COLLADA file they have strong, small highlights. How strong they look depends on the viewer's lights: clear in Gazebo, flatter in RViz's default light.

Each sub-part is a named node in the COLLADA and glTF files, so a viewer can move the parts apart for an exploded view: lift the heatsink off the module, the module out of its connector, the thermal pad off the die.

- `carrier`
- port row: `dc_jack`, `hdmi_dp`, `usb_stack_j32`, `usb_stack_j33`, `ethernet`, `micro_usb`, `power_led`
- module: `sodimm_connector`, `module_board` (with its gold fingers), `module_sd_slot` under it, `soc_substrate` and `soc_die` (the processor), `memory`, `module_power` (a power management chip and two inductors), `module_standoffs`, `module_screws`, `carrier_screws` (the standoffs' screws from below the carrier)
- heatsink: `thermal_pad`, `heatsink`, `heatsink_screws` (four M2 screws from below the module)
- carrier parts: `gpio_header`, `poe_header`, `power_select_header` (J48), `fan_header` (J15), `capacitors`, `camera_connector_j13`, `camera_connector_j49`, `m2_slot`, `m2_standoff`, `solder_joints`

Every part's solid is closed with outward normals.

## Frame

The files are in metres. Positions on this page are in millimetres. The carrier's bottom face is at z = 0. The carrier lies from x = -28.982 to 71.018 mm and from y = -38.847 to 40.153 mm. The port row is at +y and the 40-pin header at -x. While drawing, the generator uses the carrier's own coordinates: u from the left edge (DC jack side) to the right edge (40-pin header side), v from the port edge inwards. They map to (71.018 - u, 40.153 - v): a turn by 180° about z and a shift.

**Frame alignment:** the reference Jetson Nano mesh's bounding box, origin and axes were read to position the model:

| | min (mm) | max (mm) |
|---|---|---|
| Reference mesh | (-28.982, -38.847, -2.230) | (71.018, 42.175, 28.276) |
| This model | (-28.982, -38.847, -1.950) | (71.018, 42.223, 28.250) |

Independent check: the centres of the four standoff bores of Remo's printed Jetson deck lie within 0.051 mm of this model's mounting holes, at the URDF's position of the kit. This checks the holes' positions, not the clearance: the deck's standoffs end 7.1 mm above the deck and the URDF puts the carrier's bottom 7.0 mm above it, so they overlap by 0.1 mm.

## Sources

1. **NVIDIA Jetson Nano Developer Kit User Guide**, DA_09402_003, 17 Dec 2019: <https://developer.download.nvidia.com/embedded/L4T/r32-3-1_Release_v1.0/Jetson_Nano_Developer_Kit_User_Guide.pdf>. Page 2, "Developer kit module and carrier board": front, rear and top views. The figure shows the A02 carrier. The port row, the 40-pin header, the PoE header, the M.2 slot, the SO-DIMM connector and the module's standoffs are the same on B01. The fan header J15 sits elsewhere on B01 (source 10). Page 3: J25 is 9.5 mm deep for a 2.1 × 5.5 mm plug, J32 and J33 are stacks of two USB 3.0 Type A, J6 is the HDMI and DisplayPort stack.
2. **NVIDIA Jetson Nano Developer Kit data sheet** (2022), as hosted by Farnell: <https://www.farnell.com/datasheets/3799946.pdf>. "Mechanical 100mm x 79mm x 30.21mm (height includes carrier board, module, and thermal solution.)", two 15-pin MIPI CSI-2 camera connectors, 4 USB 3.0 Type-A, 1 USB 2.0 Micro-B, 40-pin header, 4-pin fan header, 4-pin PoE header.
3. **NVIDIA, "Get Started With Jetson Nano Developer Kit"**: <https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit>. Its B01 picture shows the connector order and colours, and the heatsink's fins along the port edge with two flat-topped fins that carry the fan holes. Its microSD picture shows the microSD slot under the module's back edge.
4. **NVIDIA Jetson Nano product page**: <https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-nano/product-development/>. Module 69.6 × 45 mm, 260-pin SO-DIMM.
5. **Mounting holes**: NVIDIA developer forum, "Jetson Nano Dev kit mounting holes?" (2023): <https://forums.developer.nvidia.com/t/jetson-nano-dev-kit-mounting-holes/239242>. The dimensioned board layout posted there: holes Ø2.75 mm at (0, 0), (86, 0), (0, 58) and (86, 58), the board from (-4, -17) to (96, 62), so 100 × 79 mm, the ports along the -17 edge. Only these numbers were used.
6. **B01 changes**: CNX Software, "NVIDIA Jetson Nano Developer Kit-B01 Gets an Extra Camera Connector", 17 Jan 2020: <https://www.cnx-software.com/2020/01/17/nvidia-jetson-nano-developer-kit-b01-gets-an-extra-camera-connector/>. Its B01 photo gives the two camera connectors along the left edge, the moved J48 jumper and the removed serial and button headers.
7. **Heatsink**: the dimensioned drawing and photos of "The Official Heatsink for Jetson Nano" (Waveshare part 21599, <https://www.waveshare.com/nano-heatsink.htm>, and the same drawing in Botland's listing, <https://botland.store/nvidia/21614-official-heatsink-for-jetson-nano-waveshare-21599.html>, image <https://cdn1.botland.store/img/art/inne/21614_5.jpg>). Outline 58.7 × 39.4 × 17.3 mm, the base 38.2 mm wide, four Ø2.7 mm fan holes 32 × 32 mm apart, 13.35 mm from the ends and 3.7 mm from the long edges, and the end view of the extruded profile. The photos show the black anodised finish and the flat flanges with the holes along both long edges, as on NVIDIA's B01 picture.
8. **Fan holes, cross-check**: Noctua's FAQ, <https://www.noctua.at/cn/support/faqs/which-is-the-best-fan-for-the-nvidia-jetson-nano>: 40 mm fans fit the heatsink's holes (unthreaded, Ø2.5 mm there, where the drawing gives Ø2.7 mm). A 40 mm fan's holes are 32 mm apart.
9. **Look of the connectors**: the B01 photo of source 6 shows the DisplayPort receptacle in a metal shell above the HDMI receptacle in a black housing with a metal frame round its opening, the USB shells with spring fingers in the top, the Ethernet shield and the brass-coloured module standoffs.
10. **Waveshare's B01 photos**, on its product page <https://www.waveshare.com/jetson-nano-developer-kit-b01.htm>: the top view <https://www.waveshare.com/img/devkit/Jetson-Nano-Developer-Kit-B01/Jetson-Nano-Developer-Kit-B01-5_960.jpg> (960 × 720 pixels, about 6.7 pixels per mm) and the bottom view in the product gallery (800 × 800 pixels, about 5.7 pixels per mm) are the references of the [photo check](#photo-check). It also shows the black module board with gold pads round its standoff holes, the black fan header J15 with its latch ramp, two electrolytic capacitors (560 µF, 6.3 V), the 2-pin header J48 and the camera connectors' solder tails on their inner side.
11. **Insides of the USB-A receptacles**: GCT's drawing of the stacked USB 3.0 Type-A receptacle USB1100, <https://gct.co/files/drawings/usb1100.pdf>, sheet 1. The opening 12.50 × 5.12 mm, the tongue 11.10 × 1.84 mm with its top 0.40 mm below the opening's top, the rear housing 10.25 mm behind the front, the ports 8.50 mm apart, the four USB 2.0 contacts (pairs 2.0 and 7.0 mm apart, about 0.95 mm wide, 0.35 mm below the tongue) and the five SuperSpeed contacts within 3.0 mm of the tongue's tip.
12. **Insides of the HDMI receptacle**: Same Sky's drawings of the HDMI Type A receptacles HD05-19-TH-TR, <https://www.sameskydevices.com/product/resource/hd05-19-th-tr.pdf>, page 2 (the opening 14.00 × 4.55 mm, the tongue 10.80 × 0.90 mm centred in it, 10 contacts on its top face and 9 below at 1.00 mm, offset by 0.50 mm), and HD10-19-MSMT-TR, <https://www.sameskydevices.com/product/resource/hd10-19-msmt-tr.pdf>, page 2 (the insulator 6.80 mm behind the front, the tongue 4.80 mm long).
13. **Insides of the DisplayPort receptacle**: JAE's drawing SJ108326 of the DisplayPort receptacle DP1RD20JQ1, in JAE's DP1 series catalog, page 6, as hosted by DigiKey: <https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/567/DP1_Series.pdf>. The opening 16.1 × 4.76 mm, the tongue 12 mm wide with its centre 0.5 mm off the opening's and its centreline 3.4 mm above the board, its root 8.88 mm behind the front and 4.8 mm long, 20 contacts, the odd ones on its top face. Scaled from the front view (30.7 pixels per mm): the opening's bottom 0.88 mm above the board, so the tongue's centreline 2.52 mm above it, and the tongue about 1.0 mm thick.
14. **Insides of the micro-USB receptacle**: GCT's drawing of the micro-USB B receptacle USB3131, <https://gct.co/files/drawings/usb3131.pdf>, sheet 1. The opening 6.90 mm wide at the top, 5.20 mm at the bottom, 1.85 mm high and 3.50 mm deep, the tongue 3.50 mm wide and 0.60 mm thick with the five contacts under it at 0.65 mm, its tip 0.80 mm behind the front.
15. **Insides of the DC jack**: Same Sky's drawing of the 5.5 mm DC jack PJ-002A, <https://www.sameskydevices.com/product/resource/pj-002a.pdf>, page 2. The body 9.00 mm wide, the opening Ø6.50 mm, the centre pin Ø2.00 mm and 8.00 mm long from the opening's floor, the sleeve contact a leaf spring along the bottom of the opening.

## How the numbers were read

Board coordinates: u from the left edge (DC jack side), v from the port edge inwards, z from the carrier's bottom face.

- **Top view (user guide)**: the carrier's outline at x = 201.5 and 709.5 px (100 mm, 5.08 px/mm), the back edge at y = 47.5 px, the port edge at y = 449 px (79.0 mm, as in the 2022 data sheet). u = (x - 201.5) / 5.08, v = (449 - y) / 5.08. Results in mm: J25 u 1.67 to 10.53, v -1.97 to 12.8. J6 u 15.0 to 33.0, v -1.52 to 16.3. J32 u 35.8 to 49.6. J33 u 52.9 to 66.6. J43 u 68.9 to 85.0, v -1.97 to 19.8. J28 u 88.3 to 95.8, v -0.94. DS3 at u 96 to 98.5, v 11. J41 pins at u 88.8 and 91.3, pitch 2.54 from v 70.0. J38 u 93.8 to 98.5, v 72.3 to 77.2. J15 u 72.7 to 82.9, v 21.5 to 27.4 (A02: on B01 it sits elsewhere, see the [photo check](#photo-check)). J2 u 11.5 to 84.7 with its latches, v 28.0 to 36.6. Module standoffs at (16.3, 75.4) and (79.9, 75.4). J18 u 69.2 to 78.4, v 46.1 to 68.0. M.2 screw at (44.4, 57.3).
- **Front view (user guide)**: scale 3.71 px/mm, from J25's left edge (u 1.67) to J28's right edge (u 95.8). Heights above the carrier's top face: J25 11.05, J6 18.6, USB stacks 16.0, J43 13.7, J28 3.0, heatsink 26.5 mm. The HDMI opening from 1.6 to 5.7 mm and the DisplayPort opening from 12.7 to 17.5 mm above the top face.
- **Rear view (user guide)**: scale 3.80 px/mm. Heatsink top 26.8 mm above the top face, heatsink u 18.4 to 77.4, module board z 9.0 to 10.0, microSD slot u 37.1 to 48.4.
- **Heatsink profile (drawing of source 7)**: the end view's 17.3 mm height spans 118 px (6.83 px/mm), its 39.4 mm width 269 px. Measured from the profile's centre line (w) and the heatsink's bottom (h): base 3.8 mm thick and 38.2 mm wide, nine fins about 0.9 mm thick. The centre fin stands upright to h 16.4. The others splay out in pairs, root at the base → tip: w ±4.8 → ±6.8 (h 16.4), ±13.2 → ±16.6 (h 11.7) and ±16.9 → ±19.2 (h 7.6). Two webs, ±8.9 → ±12.5 (h 16.1), carry the 0.9 mm flanges from w ±12.0 to the long edges at the top (h 16.4 to 17.3). The heatsink is centred on the module's u = 47.9 (u 18.55 to 77.25, as the rear view's 18.4 to 77.4) with its back edge on the module's back edge (v 38.0 to 77.4), its top at 28.25 mm, so its bottom at 10.95 mm, 0.75 mm above the module.
- **Heights**: heatsink top = 1.6 + (26.5 + 26.8) / 2 = 28.25 mm. With the data sheet's 30.21 mm total height, the solder tails reach 1.96 mm below the carrier.
- **Holes**: the forum layout's (0, 0) to (86, 58), in a board from (-4, -17), gives u 4 and 90, v 17 and 75. The 40-pin header's pins at u 88.8 and 91.3 lie on the line of the right holes, as in that layout.
- **Camera connectors (B01)**: in the CNX photo, the pin rows of the two connectors lie 7.8 to 22.1 mm and 33.0 to 46.2 mm from the back left hole along the left edge, scaled by the 58 mm between the two left holes. Centres at v ≈ 60.0 and 35.4.
- **Heatsink underside (drawing of source 7)**: four M2 holes 33.43 × 14.9 mm apart, 21.74 mm from one end and 6.9 mm from one long edge of the 38.2 mm base. A 2.0 mm deep pocket with 4 mm corners, reaching 18.95 mm from the same end, inside a rim along the end and the sides (1.0 mm, estimated). Between the holes, a square of about 11.5 mm marks the contact area over the processor, centred about 38.8 mm from that end and 12.1 mm from that edge (scaled from the drawing at 6.95 pixels per mm). The model puts that end at the 40-pin header's side (u 77.25) and that edge at the port side, which places the processor at (38.45, 50.7), the M2 holes at u 22.08 and 55.51, v 45.5 and 60.4, and the pocket over u 58.3 to 77.25.

## Photo check

The model was rendered through the camera of Waveshare's top photo (source 10) and compared with it: side by side, as a 50 % overlay and as outlines. The camera was fitted to the four mounting holes and the tips of the 40-pin header: they fit within 0.02 and 0.07 mm (RMS). Each part's top face was then resampled from the photo at 20 pixels per mm, and its outline compared by eye. At 6.7 pixels per mm in the photo, an edge reads to about ±0.3 mm. The photo itself isn't part of this repository.

| Part | Model (u, v in mm) | Photo | Result |
|---|---|---|---|
| DC jack J25 | u 1.6 to 10.6 | 9.0 to 9.8 mm wide | Same Sky's 9.0 mm body. The jack's top edges are rounded, its sides dark. |
| HDMI and DisplayPort J6 | u 15.8 to 32.6 | u 16.0 to 32.5 | From the photo |
| USB stacks J32, J33 | u 36.15 to 49.25 and 53.2 to 66.3 | 12.6 and 12.8 mm wide | From the photo, the 13.1 mm a shell round a 12.5 mm opening needs |
| Ethernet J43, micro-USB J28 | as in the user guide | within 0.2 mm (Ethernet) and 0.45 mm (micro-USB, 7.5 mm wide with its ears) | User guide |
| Fan header J15 | u 67.26 to 77.46, v 22.2 to 27.95, pins at v 24.21 | the pins at u 68.6 to 76.1 (2.5 mm apart), v 24.2 | From the photo: black, its latch ramp at the module side |
| Capacitors | Ø6.6 mm at (82.3, 24.5) and (51.55, 18.75) | Ø6.6 to 6.75 mm | From the photo |
| J48 | pins at u 12.99, v 16.58 and 19.12 | pins at u 12.99, v 16.59 and 19.10 | From the photo: the pins one behind the other along v |
| Camera connectors J13, J49 | u 0.4 to 7.2, 21.9 mm long | u 0.4 to 7.2, 21.5 and 22.3 mm long | From the photo: the solder tails on the inner side (high u), the latch at the board's edge |
| SO-DIMM connector, 40-pin and PoE headers | as in the user guide | within 0.45 mm | User guide |
| Heatsink | u 18.55 to 77.25, v 38.0 to 77.4 | within 0.4 mm | Drawing and user guide |
| Module | black board, gold pads Ø5.6 mm round the standoff holes | black, pads about Ø5.8 mm | From the photo |

The bottom view was mapped by the plane of the carrier's bottom face, fitted to the four mounting holes (exact), and compared the same way:

| Part | Model (u, v in mm) | Photo | Result |
|---|---|---|---|
| 40-pin and PoE headers' tails | as on the top | within 0.3 mm | User guide, with solder cones round the tails |
| Fan header J15's tails | pins at u 68.6 to 76.2, v 24.21 | within 0.7 mm | From the top photo |
| Button header J50 | 12 holes Ø1.0 mm with Ø1.7 mm pads, pin 1 at u 49.71, v 69.47, 2.54 mm apart towards the DC jack | the same | From the photo, unpopulated as on the kit |
| Module standoffs' screws | under the carrier at the standoffs, Ø4.0 mm heads, Ø5.8 mm copper pads | u 16.2 and 79.5, v 76.3 | The user guide's v 75.4 is kept: 76.3 would put the module's standoff holes (Ø2.8 mm) 0.1 mm from its back edge. Both photos read 75.6 to 76.3. |
| M.2 standoff | (44.4, 57.25) | (42.7, 54.8), a press-fit standoff with a copper ring | The user guide's place is kept, in line with the M.2 slot, which neither photo shows on B01. |
| Port connectors' solder joints | four approximate joints a connector | about 150 joints in groups | Simplified |
| Coin cell holder J45 | not modelled | an unpopulated outline with two pads | Not modelled |

## Estimated or simplified

- **Carrier thickness 1.6 mm** (not published), corner radius 1.0 mm.
- **Heatsink**: the drawing's profile with straight, constant-thickness fins (the real ones taper a little and have rounded tips). The printed warning sign isn't modelled.
- **Connector details**, shaped after the photos, sizes estimated, inside the connectors' outlines from the user guide except the USB fronts' 0.35 mm ears and the Ethernet LEDs' 0.1 mm:
  - the USB stacks' front frames, spring fingers, dimples and snap recesses
  - the HDMI receptacle's black housing (to 10.2 mm above the carrier) with a 0.45 mm metal frame round its opening, set into the housing, and the DisplayPort shell above it with two spring fingers
  - the Ethernet shield's rounded front edge, the DC jack's front recess and side contact
  - the SO-DIMM connector's 130 contacts at 0.5 mm along its front, hexagonal brass standoffs and cross-head screws for the module
- **Connector insides**, from the drawings of sources 11 to 15: the inside walls of the metal shells show up to a black insulator, the tongues carry their contacts, the USB 2.0 contacts are springs that bulge below the tongue and the USB 3.0 tongues also have their five flat contacts near the tip. The kit's own connectors may differ from these makers' parts in details the photos don't show. Estimated: the contacts' widths where the drawings give only the solder tails (HDMI and DisplayPort 0.5 mm, micro-USB 0.35 mm), the DisplayPort tongue's 1.0 mm thickness and height (scaled, source 13), the SuperSpeed contacts' 2.3 mm pitch and the RJ45 springs: eight 0.4 mm contacts at the modular plug's 1.02 mm pitch, from the top of the opening at the front down and back at about 23 degrees, their ends in a black comb.
- **Module**: a 69.6 × 45 × 1.0 mm black board from z 9.2 to 10.2 mm, with its microSD slot under the back edge. The rear view reads 9.0 to 10.0 mm at ±0.3 mm per pixel. 0.2 mm higher leaves room for a 1.4 mm microSD socket underneath. Gold fingers on both faces where it sits in the connector, 130 a face at 0.5 mm, 71 and 59 beside the key notch at u 51.0 (the split and the notch's place estimated).
- **Module parts under the heatsink**: the heatsink hides them, and NVIDIA's module drawings with them aren't public. Placed by the heatsink's underside (see above) and sized as typical packages, all estimated: the processor's 23 × 23 × 0.45 mm substrate with a 10.9 × 10.9 × 0.25 mm bare die, the 10 × 14.5 × 1.0 mm memory package, a 6 × 6 mm power management chip and two 4 × 4 × 1.8 mm inductors under the pocket. The heatsink's base lies 0.75 mm above the module (from the kit's 30.21 mm height), so the substrate, die and 0.05 mm thermal pad fill that gap. The drawing doesn't show which end of the heatsink faces which side of the kit. Turned the other way, the processor and the screws would mirror about u 47.9.
- **Heatsink screws**: four M2 pan-head screws (Ø3.5 mm heads) from below the module into the heatsink's holes, 2.8 mm deep.
- **Camera connectors**: two 15-pin flat cable connectors (21.9 × 6.8 × 2.5 mm) with a brown latch at the board's edge and their solder tails on the inner side. Their centres along the edge are read from the CNX photo (source 6), their sizes from Waveshare's (source 10).
- **J48** is a 2-pin header without a jumper cap, its pins at u 12.99, v 16.58 and 19.12 mm (source 10). The B01's 12-pin button header J50 isn't modelled: its position isn't in the sources.
- **Capacitors**: 8.0 mm high (estimated), Ø6.6 mm from the photo.
- **Small surface parts** of the carrier aren't modelled: none are in the public drawings.
- **Bottom**: the header tails with solder cones, the module standoffs' screws with their pads, J50's twelve plated holes, and four approximate solder joints under each port connector.
- **No M.2 card, fan or camera cables**: the kit as shipped.

## Measurements that would improve the model

- Whether Remo's kit has the heatsink of the drawing (58.7 × 39.4 × 17.3 mm) and where it sits on the module, front to back: a top photo with a ruler.
- The module's height above the carrier and the base of the heatsink: calipers from the carrier to the module's edge.
- A photo of the left edge, for the exact places of the two camera connectors, J48 and J50.
- A photo of the module without its heatsink, for the processor's, memory's and power parts' places and sizes, and which end of the heatsink faces the 40-pin header.
- A ruler photo of the kit without the module, for the module's standoffs and the M.2 slot on B01.
