# Frenet hold / secondary selection redundancy audit

Date: 2026-05-28

Branch checked: `rollback/frenet-063ef43`

## Scope

This audit checks whether the current Frenet branch still contains redundant logic after the cleanup request:

- Remove planning-before-hold behavior.
- Remove temporal/profile secondary candidate selection.
- Remove clearance-preference secondary selection.
- Merge clearance preference into the main candidate cost.
- Keep only clearly bounded fallback behavior when the current planning cycle has no safe candidate.

## Implementation state

### Removed production logic

The following production paths are removed:

- `_publish_held_path_if_safe()` planning-before-hold path in `node.py`.
- `_log_hold_replan()` hold-specific logging in `node.py`.
- `_select_consistent_candidate()` node-level secondary selection.
- `select_temporally_consistent_candidate()` planner-level secondary selection.
- `lateral_profile_error()` and its private profile helper usage.
- `held_path_*` runtime parameters.
- `candidate_profile_*` runtime parameters.
- `candidate_channel_memory_timeout_s`.
- `clearance_preference_*` secondary-selection parameters.
- shell defaults and launch arguments for the removed parameters.

The normal active-planning flow is now:

1. Generate Frenet candidates.
2. Apply hard filters: progress, heading jump, swept collision, minimum clearance, curvature.
3. Score every safe candidate with `score_candidate()`.
4. Sort by `candidate.cost`.
5. Publish the lowest-cost candidate.

### Clearance preference is now inside cost

Clearance preference is no longer a post-processing selector. It is encoded in `score_candidate()` as a normalized soft penalty:

```python
clearance_deficit = max(0.0, config.safe_clearance - min_clearance)
clearance_scale = max(config.safe_clearance, 1e-6)
clearance_cost = (clearance_deficit / clearance_scale) ** 2
cost += config.weight_obstacle_clearance * clearance_cost
```

This means:

- `min_clearance_m` is still a hard rejection threshold.
- `safe_clearance_m` is the desired clearance used by the soft cost.
- A candidate above `safe_clearance_m` gets no additional clearance penalty.
- A candidate between `min_clearance_m` and `safe_clearance_m` remains valid but receives a cost penalty.

Current key parameters:

| Parameter | Current default | Meaning |
|---|---:|---|
| `min_clearance_m` | `0.05` | Hard minimum clearance; lower values reject the candidate |
| `safe_clearance_m` | `0.35` | Desired clearance for soft cost |
| `weight_obstacle_clearance` | `24.0` | Weight on normalized clearance deficit |

## Remaining fallback behavior

`reuse_last_candidate_timeout_s` and `_reuse_last_candidate_if_fresh()` intentionally remain.

This is not the deleted hold behavior. The distinction is:

| Behavior | Status | When it runs | Effect |
|---|---|---|---|
| Planning-before hold | Removed | Before generating new Frenet candidates | Could block a new better candidate from being selected |
| Secondary candidate selection | Removed | After raw best candidate was found | Could override the lowest-cost candidate |
| No-candidate reuse fallback | Kept | Only after current planning cycle returns no safe candidate | Short-time fallback if the cached path is still collision-free and above `min_clearance_m` |

If the design goal changes to "never publish any cached candidate", then `_reuse_last_candidate_if_fresh()`, `_safe_cached_candidate()`, `_trim_cached_candidate()`, `last_safe_candidate_*`, and `reuse_last_candidate_timeout_s` should be removed too. That is a stricter requirement than removing hold and secondary selection.

## Redundancy search

Main searched paths:

- `code/pnc_rc/frenet/`
- `test/`
- `launch/`
- `docs/`
- `run_frenet_test.sh`
- `run_frenet_random_obstacle_robustness.sh`

Main searched keywords:

- `hold`, `held`, `held_path`, `_publish_held`
- `candidate_profile`, `profile_consistency`, `profile_lookahead`, `candidate_channel`
- `temporal`, `temporally`, `select_temporally`, `same_profile`
- `lateral_profile_error`, `_unique_monotonic_profile`
- `last_selected`, `last_candidate_selection`
- `clearance_preference`, `select_clearance`
- `FRENET_HOLD`, `hold_replan`, `hold_min`
- `二次选择`, `二次筛选`, `通道连续`, `同侧稳定`

## Findings

| Area | Result | Severity |
|---|---|---|
| Production hold logic | No active planning-before-hold function or call remains | None |
| Production temporal/profile secondary selection | No active selector function, state memory, launch parameter, or call remains | None |
| Production clearance-preference secondary selection | No active secondary selector remains; preference is in `score_candidate()` | None |
| Tests | Old selector tests removed; runner test asserts removed params are absent | None |
| Scripts / launch args | Removed hold/profile/clearance-preference arguments from Frenet preset and shell runner | None |
| `docs/current_frenet_vs_basic_notes.md` | Updated to describe current single-cost selection and removed old parameter table rows | Fixed |
| `docs/frenet_branch_progress_report_2026-05-28.md` | Contains historical references to hold/temporal selection while explaining the branch evolution | Acceptable |
| `code/outputs/` | Historical experiment logs may still contain old parameter names and directory names | Acceptable archive noise |
| `reuse_last_candidate` | Still present by design as no-candidate fallback, not normal hold | Deliberate residual |

## Verification commands

```bash
rg -n "_publish_held|held_path|candidate_profile|profile_consistency|profile_lookahead|candidate_channel|select_temporally|temporally|lateral_profile_error|last_selected|last_candidate_selection|clearance_preference|select_clearance|FRENET_HOLD|hold_replan|hold_min|same_profile|higher_clearance|unlock_clearance" code/pnc_rc/frenet test run_frenet_test.sh run_frenet_random_obstacle_robustness.sh launch docs --glob '!code/outputs/**' --glob '!docs/frenet_redundancy_audit_2026-05-28.md'
```

Expected remaining hits:

- Negative assertions in `test/test_frenet_random_robustness.py`.
- Historical notes in `docs/frenet_branch_progress_report_2026-05-28.md`.
- The positive clearance-cost unit test name `test_candidate_score_prefers_higher_clearance`.

```bash
rg -n "_unique_monotonic_profile|用于 profile 连续性比较|最低代价/同侧稳定" code/pnc_rc/frenet test docs --glob '!code/outputs/**' --glob '!docs/frenet_redundancy_audit_2026-05-28.md'
```

Expected result: no matches.

Unit checks used for this cleanup:

```bash
PYTHONPYCACHEPREFIX=/tmp/frenet_pycache python3 -m py_compile code/pnc_rc/frenet/planner.py code/pnc_rc/frenet/node.py code/pnc_rc/frenet/preset.py test/test_frenet_planner.py test/test_frenet_random_robustness.py
docker run --rm -v "$PWD:/sim_ws/src/f1tenth_gym_ros" f1tenth_gym_ros:latest -lc "cd /sim_ws/src/f1tenth_gym_ros && python3 -m pytest test/test_frenet_planner.py test/test_frenet_random_robustness.py -q"
```

Observed result:

- Python compile check passed.
- Container unit tests passed: `41 passed`.

## Conclusion

The active Frenet candidate-selection path has been simplified to one hard-filter stage plus one cost-ranking stage. The old hold path and secondary selectors are no longer part of production execution.

The only remaining old-path-like behavior is the explicitly bounded no-candidate reuse fallback. It should be treated as a separate design decision, not as the removed hold mechanism.
