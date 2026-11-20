# Ipswich Kart Club (Willowbank, QLD) - measured replica

Ipswich Motorsport Precinct, 56 Champions Way, Willowbank QLD. Layouts: `IpswichKartClub_B.json` (B Track, clockwise,
the championship direction) and `IpswichKartClub_K.json` (K Track, anticlockwise). Built only from public data.

## Data sources
| Data | Source | Licence / attribution |
|---|---|---|
| Seed centre line, links, buildings | OpenStreetMap way 176471927 (+ 4 link ways, 13 building footprints) | (c) OpenStreetMap contributors, ODbL |
| Track edges, width, kerbs, start line, trees | Queensland Government aerial imagery, `Basemaps/LatestStateProgram_AllUsers` ImageServer, 0.25 m/px | (c) State of Queensland |
| Elevation, camber, terrain | Queensland Government LiDAR DTM, `Elevation/QldDem` ImageServer, 0.5 m, float32 | (c) State of Queensland |
| Layout names / directions, lap records | IKC lap-record sheets (B Track clockwise, K Track anticlockwise, updated 02/05/2026) | ipswichkartclub.org.au |
| Published length (validation) | 1.08 km (kartclass.com listing) | - |

## Pipeline (Tools/IpswichKC)
1. `extract_track.py <work>`: OSM seed -> 3 passes re-centring on the asphalt strip measured across the track every 1 m on
   the aerial (asphalt / paint / kerb pixel classes) -> per-station centre, width, kerb strips (0.70 m), LiDAR elevation
   and cross-slope. Diagnostics: diag_edges.jpg.
2. `make_layout.py <work>`: smoothing spline (sigma 0.15 m) + 6.5 m minimum radius (the NW hairpin's strip flares into
   run-off), widths median-filtered and clamped 8.0-10.5 m at junctions, control points every 4 m
   `[x east, y south, camber, elevation, width]`, kerb runs per driver side, both directions, LiDAR terrain grid
   (`IpswichKartClub_terrain.bin`, 1085 x 891 @ 0.5 m, heights relative to the lowest centre-line point, 51.82 m AHD).
3. `extract_scenery.py <work>`: OSM building footprints (heights estimated by type: ASSUMPTION) and 970 tree crowns detected
   on the aerial (eucalypt-tuned colour test; none inside the circuit loop unless clearly shadowed) ->
   `IpswichKartClub_scenery.json`.
Work dir inputs: osm_map.xml (OSM API map call), osm_track.json, ortho.jpg + dem.tif (exportImage, bbox in bbox.txt).

## Engine support (generic, backward compatible)
* Per-control-point width (5th value) -> SampleHalfW used by asphalt, kerbs, verges, banking, timing lines, barriers,
  terrain blend; AI racing line uses the layout "width" (10th percentile, 8.46 m).
* Kerb sides "left" / "right" (driver's view) and "kerbWidth".
* "terrain": measured heightfield replaces the procedural hills (3 m blend from the track shoulders, 1.25 m mesh).
* "scenery": measured buildings (oriented boxes / shelters) and trees (crown-scaled) instead of the procedural pit,
  fences, woods and tyre walls. Scenery components are now Movable (static ones failed to attach to the movable root).

## Result
* Lap 1077-1084 m (published 1.08 km). Elevation 2.44 m (51.85 m low at the SW corner, 54.2 m plateau on the east
  side, falling down the main straight). Camber -1.9..+3.6 deg. Width 8.2-10.5 m. 25 kerb runs per direction.
* Start/finish: full-width painted line on the paddock straight (station 551.75 m), grid boxes behind it (clockwise).

## Not verified / open
* Lap time vs the club records (Rotax Light 46.862 s, KZ2 45.411 s): the conservative autopilot (1.4 g) laps ~68 s; it
  is a regression driver, not a pace reference. A proper comparison needs the AI at full grip on the racing line.
* Which infield links the B / K layouts use is ASSUMED (full outer loop for both; the records are near-identical).
* Building heights, the tyre walls / fencing (not reproduced), pit lane geometry, link roads (not modelled).
* Test-circuit regression after the track-code changes was not re-run (window closed by the user).
