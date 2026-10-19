# Template checklist (delete this file before publishing)

Search the whole repository for `TODO(you)`: `grep -rn "TODO(you)" .`

## Before anything else
- [ ] Ask your supervisor (Mrs. Bouraine) whether the code and report may be published; choose **private** until she agrees.
- [ ] Read the license of `unitree_rl_gym`; keep any required notice in `src/deploy_mujoco_ros.py`.
- [ ] Never commit `motion.pt`, `scene.xml` or any Unitree file (the `.gitignore` already blocks `*.pt`).

## Code
- [ ] Test the script once from the repository copy: `python3 deploy_mujoco_ros.py g1.yaml` (path from `LEGGED_GYM_ROOT_DIR`).
- [ ] Test the `--body-frame` option (turning command, look at `/g1/base_twist` in `rqt_plot`); update `docs/known_issues.md`.
- [ ] Confirm that `src/deploy_mujoco_ros.py` is the version you used for the report (only differences: path from the
      environment, optional `--body-frame`, cleaner imports).
- [ ] Be able to explain every line (a jury or recruiter may ask).

## Documentation
- [ ] README: username in the clone URL, versions table, demo image, contact, credits.
- [ ] `docs/observation_vector.md`: confirm the gains against your `g1.yaml`; list differences with Unitree's file.
- [ ] `docs/images/`: add the images listed in its README (fix `steps.png` first).
- [ ] `results/`: add `tracking_vx.png`, test dates and machine specs.
- [ ] `LICENSE`: put your name; keep the note about Unitree files until you delete this checklist.
- [ ] `report/report.pdf`: add the final report if allowed.

## Publishing
- [ ] `git init`, first commit, create the GitHub repository (private first), push.
- [ ] Add topics: `ros2`, `mujoco`, `unitree-g1`, `humanoid`, `reinforcement-learning`.
- [ ] Check that the README renders (the mermaid diagram) and that every link works.
- [ ] Add the repository link to the report (appendix or bibliography).
