# JSP Trajectory Format (v2)

Binary little-endian format produced by `jupedsim` with deflate-compressed
frame payloads (libdeflate).

Version 2 adds a per-agent floor column and the source model block. Readers
should accept version 1 as well: it is the same stream with 24 byte records and
no floor column, which then reads as floor `0`.

## Header (52 bytes)

1. `char[4] magic` = `JSP1` (unchanged in v2)
2. `u32 version` = `2`
3. `u32 flags` bit0 = deflate compression
4. `f64 dt`
5. `u32 record_size` = `28` (v1: `24`)
6. `u32 every_nth_frame`
7. `u64 frame_count`
8. `u64 index_offset`
9. `u32 compression_level` (1..12)
10. `u32 reserved` (0)

## Frame Payload Blocks

Each frame payload is a deflate-compressed byte block containing:

- repeated records of 28 bytes each:
  - `u64 agent_id`
  - `f32 x`
  - `f32 y`
  - `f32 ori_x`
  - `f32 ori_y`
  - `u32 floor_id` (v2 only, `0` for single-floor scenarios)

Uncompressed payload size = `agent_count * record_size`.

## Frame Index Entries (48 bytes each)

Stored at `index_offset`, `frame_count` entries:

1. `u64 iteration`
2. `f64 time_seconds`
3. `u32 agent_count`
4. `u32 reserved` (0)
5. `u64 data_offset`
6. `u64 compressed_size`
7. `u64 uncompressed_size`

Use `data_offset` + `compressed_size` to read each compressed frame payload.

## Optional Trailer Blocks

Optional blocks follow the frame index section, each introduced by its own
magic. Their order is not significant. A reader that meets an unknown magic
stops parsing trailers and keeps what it has read so far.

### Agent Metadata Block

1. `char[4] magic` = `JSPM`
2. `u32 metadata_version` = `1`
3. `u32 metadata_record_size` = `24`
4. `u32 metadata_agent_count`

Then `metadata_agent_count` records (24 bytes each):

- `u64 agent_id`
- `u8 age_group_code` (`0` unknown, `1` young, `2` adult, `3` elderly)
- `u8 avatar_hint_code` (`0` unknown, `1` young, `2` adult, `3` grandpa, `4` grandma)
- `u16 reserved` (`0`)
- `f32 desired_speed`
- `f32 time_gap`
- `f32 radius`

### Source Model Block

Written when the scenario carries `flowsimpro_*` attributes on `<scenario>`. It
states where the simulated storey sits inside the source building model, so a
trajectory can be placed correctly without its scenario XML next to it.

1. `char[4] magic` = `JSPF`
2. `u32 version` = `1`
3. `u32 record_size` = `16`

Then one record (16 bytes):

- `f32 storey_elevation_m` (from `flowsimpro_storey_elevation`)
- `f32 centering_shift_x` (from `flowsimpro_centering_x`)
- `f32 centering_shift_y` (from `flowsimpro_centering_y`)
- `f32 centering_shift_z` (from `flowsimpro_centering_z`)

### FDS Smoke Coupling Block

Written when the trajectory was simulated with an `<fds_hazard>` configuration.
It lets an importing application recognize the coupling, locate the Smokeview
case, and synchronize the smoke with the evacuation timeline. The SMV path is
UTF-8 and normally relative to the JSP file so the result remains movable.

1. `char[4] magic` = `JSPH`
2. `u32 version` = `1`
3. `u32 payload_size` = `56 + smv_path_length`
4. `u32 flags`
   - bit 0: FDS Smoke3D soot coupling was applied
   - bit 1: `smv_path` is relative to the JSP directory
5. `u32 smv_path_length`
6. `f32 sample_eye_z_m`
7. `f32 z_tolerance_m`
8. `f32 coordinate_offset_x_m`
9. `f32 coordinate_offset_y_m`
10. `f64 time_offset_s` (currently `0`)
11. `f32 update_interval_s`
12. `f32 awareness_below_m`
13. `f32 severe_below_m`
14. `f32 minimum_speed_factor`
15. `f32 visibility_factor`
16. `f32 maximum_visibility_m`
17. `u8[smv_path_length] smv_path` (UTF-8, without a null terminator)

If the resolved SMV file is unavailable, an importer should keep the JSP
trajectory usable and offer the user a file picker instead of rejecting it.
