# XL330 HAL-style OOP Spec Draft

Status: draft, spec-first, no implementation yet.

## Locked decisions

| Topic | Decision |
|---|---|
| Top API shape | Option A, one motor-focused manager handle |
| Naming style | Pure HAL-style verbs |
| Write behavior | Stage locally, then `Commit()` |
| Core public API | Low-level staged setters only |
| Commit behavior | Mixed staged changes allowed, internal safe order: mode -> torque -> goal |
| Read model | Named read items, user-defined layouts |
| Read layouts | Multiple named layouts |
| Read optimization | Library may choose Sync Read, Fast Sync Read, or Bulk Read based on layout |
| Item naming | Short names like `XL330_POS`, `XL330_VEL`, `XL330_CUR` |
| Layout usage | Register once, use by name |
| Goal API | Explicit per type: pos, vel, cur, pwm later |
| Motor selection | Both ID and index APIs, ID is primary |
| Error model | One top-level status enum, deeper detail in reports |
| Async support | Included in spec, blocking first in implementation priority |
| Storage policy | User-selectable storage, library core stays stable |
| Handle visibility | Opaque-ish handle, bind/init APIs, no direct field contract |
| State access | Both cached state and optional user output buffer |
| Docs | Short README/doc, tables, comparisons, mode usage examples |
| Storage binding | Config struct |
| Init flow | One-step init with storage config |
| Quick start | Also support default quick init helper |
| Layout identity | Numeric ID + optional short name |
| Commit strategy | One smart `Commit()` auto-selects strategy |
| Update strategy | One smart `Update()` auto-selects single/sync/fast/bulk reads |
| Write classes | Internal separation between config path and realtime goal path |
| Public surface | Spec includes full management surface, implementation may be phased |
| Config target API | Both per-motor and all-motors helpers |
| Mode model | Per-motor modes allowed, with `SetModeAll` convenience helper |
| Error/fallback policy | Fail on invalid usage, fallback on optimization choice |
| Goal validation | Mode capability mask, so modes may allow multiple goal families like POS+CUR |
| Raw access | Separate expert raw-access API section |
| Discovery API | Both standalone scan and handle-based rescan APIs |
| Empty commit behavior | Return specific no-pending-changes status, not plain OK or error |
| Default read layout | Built-in default layout for quick start, custom layouts optional |
| Preset layouts | A few built-in presets plus custom layouts |
| Motion profiles | Both separate profile setters and combined profile helper |
| README format | Compact tables plus short scenario examples |
| First implementation milestone | Option B: core control plus common config |

## Target usage direction

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

## Read layout direction

```c
XL330_Item fast_items[] = { XL330_POS, XL330_VEL, XL330_CUR };
XL330_AddLayout(&hxl330, XL330_FAST, fast_items, 3);
XL330_Update(&hxl330, XL330_FAST, user_buf);
```

## Not decided yet

- Exact storage binding API shape
- Exact report struct fields
- Exact async state machine API names
- Exact layout buffer decode format
- Exact default preset layouts
- Exact naming of status enum and report accessors
