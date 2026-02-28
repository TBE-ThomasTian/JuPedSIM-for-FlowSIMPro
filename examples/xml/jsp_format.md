# JSP Trajectory Format (v1)

Binary little-endian format produced by `jupedsim-cli` with deflate-compressed
frame payloads (libdeflate).

## Header (52 bytes)

1. `char[4] magic` = `JSP1`
2. `u32 version` = `1`
3. `u32 flags` bit0 = deflate compression
4. `f64 dt`
5. `u32 record_size` = `24`
6. `u32 every_nth_frame`
7. `u64 frame_count`
8. `u64 index_offset`
9. `u32 compression_level` (1..12)
10. `u32 reserved` (0)

## Frame Payload Blocks

Each frame payload is a deflate-compressed byte block containing:

- repeated records of 24 bytes each:
  - `u64 agent_id`
  - `f32 x`
  - `f32 y`
  - `f32 ori_x`
  - `f32 ori_y`

Uncompressed payload size = `agent_count * 24`.

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

## Optional Agent Metadata Block

If present, this block is appended directly after the frame index section:

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
