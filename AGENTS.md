# Repository Guidelines

## Project Structure & Module Organization
- `main.py` is the primary CLI entry point for stereo scanning.
- `measure_distance.py` is the live distance-only utility.
- `scanner/` contains scan orchestration and result flow.
- `detection/` contains blue-object detection and shape classification.
- `stereo/` contains dual-camera capture and disparity-based distance math.
- `servo/` contains PWM servo control.
- `utils/` contains shared image and helper utilities.
- `config.py` defines dataclass-based runtime config; `hparams.py` holds tunable constants.
- Runtime outputs are written to `outputs/` (or custom `--outdir`); do not commit generated images.

## Build, Test, and Development Commands
- `python main.py --help`: list scanner options.
- `python main.py --target cube --view`: run full scanner with live preview.
- `python measure_distance.py --help`: list distance tool options.
- `python measure_distance.py --baseline-m 0.075 --fx-px 1893`: run distance measurement.
- `python -m compileall -q .`: quick syntax validation across the repo.
- `python -m pytest -q`: run tests (currently no committed test suite).

## Coding Style & Naming Conventions
- Use Python 3 with type hints and dataclasses (existing project pattern).
- Indentation: 4 spaces; keep lines readable and avoid dense one-liners.
- Naming: `snake_case` for modules/functions/variables, `PascalCase` for classes, `UPPER_CASE` for constants in `hparams.py`.
- Keep hardware and algorithm defaults centralized in `hparams.py`/`config.py`; avoid hard-coded magic values in feature modules.
- No formatter/linter config is committed; follow PEP 8 and keep imports grouped as stdlib, third-party, local.

## Testing Guidelines
- There is no enforced coverage threshold today.
- Add new automated tests with `pytest` under `tests/`, named `test_*.py`.
- Prefer unit tests for pure logic (`detection`, `stereo/distance`, argument parsing) and mock hardware boundaries (`picamera2`, PWM).
- In every PR, include manual validation commands run and observed results.

## Commit & Pull Request Guidelines
- Recent history includes both descriptive commits and placeholder messages; use clear, imperative commit subjects going forward.
- Recommended format: `<area>: <imperative summary>` (example: `scanner: return only confirmed detections`).
- PRs should include what changed and why.
- PRs should include a linked issue when applicable.
- PRs should include the hardware setup used (cameras/servo).
- PRs should include CLI commands executed for validation.
- PRs should include screenshots or log snippets for preview/detection behavior changes.
