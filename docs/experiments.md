# Experiments

## Tasks

| Task | Dataset | Notes |
|------|---------|-------|
| Place coffee cup | `andnetdeboer/vtam_place_coffee_cup` | |
| Setup cup | `andnetdeboer/vtam_setup_cup` | |

---

## Training Runs

| Date | Policy | Dataset | Steps | Notes |
|------|--------|---------|-------|-------|
| 2026-02-26 | `stretch_real_diffusion` | `vtam_place_coffee_cup` | 200k | First diffusion run |
| 2026-03-11 | `stretch_act_stationary` | `vtam_place_coffee_cup` | 60k | ACT, start coffee |
| 2026-03-11 | `stretch_act_stationary` | `vtam_setup_cup` | — | ACT, setup cup |

---

## Notes

**Why relative actions (Δq)?**
Absolute joint positions only work if the robot starts at exactly `neutral_q`. Any deviation means every commanded position is wrong in absolute space. Storing joint deltas `Δq[t] = q[t+1] - q[t]` makes the policy object-centric: the wrist camera encodes relative gripper-to-object pose, so the policy learns "given what I see, move this much" regardless of starting configuration.

At inference: `q[t+1] = q_robot_current + Δq_predicted`, where `q_robot_current` comes from live robot state.
