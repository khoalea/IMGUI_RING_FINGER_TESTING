# XL330 HAL-style surface extraction and implementation checklist

Source reviewed: `docs/xl330_hal_oop_spec.md`, `Core/Inc/dxl/dxl_xl330_driver.h`, `Core/Src/dxl/dxl_xl330_driver.c`.

## Agreed HAL-style surface

The spec locks an Option-A, motor-manager handle API with HAL-style names and staged writes:

```c
XL330_HandleTypeDef hxl330;
XL330_Init(&hxl330, &huart1);
XL330_AddMotor(&hxl330, 5);
XL330_AddMotor(&hxl330, 2);
XL330_SetMode(&hxl330, XL330_MODE_POS);
XL330_SetTorque(&hxl330, XL330_ENABLE);
XL330_SetGoalPos(&hxl330, 5, 2300);
XL330_SetGoalPos(&hxl330, 2, 1800);
XL330_Commit(&hxl330);
```

Core concepts to preserve:

- Opaque-ish `XL330_HandleTypeDef`, initialized from storage/config and not exposed as a direct field contract.
- One top-level `XL330_StatusTypeDef` or equivalent, with richer per-motor report access.
- ID-primary motor addressing, with index APIs planned too.
- Staged setters only for core public control: mode, torque, goal current/velocity/position, later PWM.
- `XL330_Commit()` applies mixed staged changes in safe order: mode, torque, goal.
- Empty commit returns a specific no-pending-changes status.
- `XL330_Update()` reads a named layout into cached state and optionally user output buffer.
- Read items are short names like `XL330_POS`, `XL330_VEL`, `XL330_CUR`.
- Layouts have numeric ID plus optional short name, registered once and used by ID/name.
- Built-in default layout and preset layouts exist for quick start.
- Library may auto-select single, sync, fast sync, or bulk read/write strategies.
- Config/common helpers are part of milestone 1, with both per-motor and all-motor helpers.
- Discovery includes standalone scan and handle-based rescan.
- Expert raw access is separate from the normal staged manager surface.

## Current driver surface observed

`dxl_xl330_driver.h/.c` already contains three layers:

1. Single-motor immediate API: `XL330_Motor`, `xl330_init`, `xl330_set_torque`, `xl330_set_operating_mode`, `xl330_set_goal_*`, raw `xl330_read/write_u*`, state reads.
2. Group immediate API: `XL330_Group`, `xl330_group_*`, sync read state, sync write goal current/velocity/position, torque/mode all, scan.
3. Staged controller API: `XL330_Controller`, `xl330_begin`, `xl330_add`, `xl330_mode`, `xl330_torque`, `xl330_goal_*`, `xl330_commit`, `xl330_update`, `xl330_state`, `xl330_report`.

The staged controller is the closest implementation substrate for the HAL manager.

## Mismatches to resolve

- Naming is lower-case driver style, not HAL style. Target calls are `XL330_Init`, `XL330_AddMotor`, `XL330_SetMode`, `XL330_SetTorque`, `XL330_SetGoalPos`, `XL330_Commit`, `XL330_Update`.
- Public handle is currently `XL330_Controller` with exposed fields. Spec wants an opaque-ish `XL330_HandleTypeDef` with bind/init APIs and no direct field contract.
- `xl330_begin()` takes only UART or half-duplex pins. Spec wants storage/config binding and one-step init, plus default quick init helper.
- `xl330_mode()` is all-motors only. Spec allows per-motor modes plus `SetModeAll` convenience.
- `xl330_torque()` is all-motors only. Spec wants config target APIs with per-motor and all-motors helpers.
- Staged goals use one global `goal_kind`, so mixed goal families cannot be committed together. Spec allows mode capability masks and modes that permit multiple goal families.
- Staged goals do not track per-motor dirty bits. A goal sync write sends arrays for every motor, including unchanged default zeros.
- Empty `xl330_commit()` currently returns `DXL2_OK` when there are no staged changes. Spec requires a specific no-pending-changes status.
- Commit has safe order mostly present, but mode change forces `torque_dirty = 1`, then writes the stored torque state. Confirm this matches desired torque semantics after a mode change.
- `xl330_set_operating_mode()` and `xl330_group_mode_all()` immediately disable torque as part of the low-level operation. The HAL staged commit should own the safe order explicitly.
- Reads are fixed to the contiguous present current/velocity/position block. There are no named read items, custom layouts, layout IDs/names, built-in presets, user output buffers, or auto-selected read strategies.
- `xl330_update()` has no layout argument and only updates cached state.
- Status model is still `DXL2_Status` at the controller layer. Spec wants one top-level HAL status enum with detail in reports.
- Reports exist, but exact fields and report accessors are not aligned to a final HAL report type.
- Standalone scan exists, and group scan exists. Handle-based rescan is missing.
- Expert raw access is mixed into the main single-motor public surface rather than separated in a documented expert section.
- Async APIs are absent. Spec includes async, but blocking implementation is first priority.
- Motion profile helpers are absent from `dxl_xl330_driver.h/.c`.

## Concrete implementation checklist

1. Keep `XL330_Controller` as the internal substrate. Add or finish a HAL wrapper layer rather than renaming the driver internals.
2. Define the public HAL header surface around `XL330_HandleTypeDef`, `XL330_StatusTypeDef`, `XL330_InitTypeDef`, `XL330_StateTypeDef`, and `XL330_ReportTypeDef`.
3. Implement status mapping from `DXL2_Status` to HAL status, including `XL330_HAL_NO_PENDING_CHANGES`.
4. Add staged dirty tracking to `XL330_Controller`: mode dirty per motor or all flag, torque dirty per motor or all flag, goal dirty per motor and goal family.
5. Change commit logic to detect no staged work and return no-pending-changes through the HAL API.
6. Make commit write in exact order: mode changes first, torque changes second, goal changes last.
7. Avoid unintended goal writes. Commit only motors with dirty goals, or intentionally build sync-write item lists from dirty motors.
8. Support mixed goal families if the chosen mode capability mask allows it. Otherwise fail invalid usage before sending bus writes.
9. Add HAL functions: `XL330_Init`, `XL330_InitEx`, `XL330_AddMotor`, `XL330_SetMode`, `XL330_SetModeAll`, `XL330_SetTorque`, goal setters, `XL330_Commit`, `XL330_Update`, state/report accessors.
10. Decide and document whether `XL330_SetTorque` means all motors by default or whether a per-motor torque API is required in milestone 1.
11. Add common config helpers for milestone 1: LED, profile velocity, profile acceleration, and combined profile helper if accepted.
12. Add read item enum/constants for at least `XL330_POS`, `XL330_VEL`, `XL330_CUR`.
13. Add layout registry storage: numeric ID, optional short name, item list, decoded byte spans, built-in default/presets.
14. Extend update API to `Update(handle, layout_id/name, optional_user_buf)` while preserving a quick-start default update helper if desired.
15. Implement initial read strategy selection simply: default contiguous state block uses sync read, unsupported custom layouts may initially return unsupported until bulk/fast paths are added.
16. Add handle-based rescan that updates or reports discovered motor IDs without requiring callers to build a temporary group.
17. Move or document raw `read/write_u*` as expert APIs, not the primary HAL usage path.
18. Add short README examples matching the spec: quick start, staged mode/torque/goal commit, custom read layout, common config.
19. Add focused tests or host-side stubs for: staged dirty behavior, empty commit status, safe order, invalid ID, mixed goal validation, layout decode metadata.

## Suggested phased cut

Milestone 1 should target the spec's Option-B implementation priority: HAL wrapper, staged control, common config, cached default update, reports, scan/rescan, and no-pending commit semantics. Leave async, fast sync/bulk optimization, named-layout string lookup, and full mixed-mode capability masks for follow-up if time is tight.
