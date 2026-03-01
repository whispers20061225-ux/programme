# Project Structure

This repository follows a common GitHub Python layout:

- `src/`: application source code packages
- `tests/`: automated test code
- `examples/`: demo scripts and visualization examples
- `config/`: runtime and environment configuration files
- `docs/`: project documentation
- `scripts/`: runnable helper scripts
- `models/`: model and mesh assets
- `data/`: sample/input data
- `robotic_arm/`: standalone robotic arm Python package
- `stm32_bridge/`: STM32 side firmware sources

Notes:

- The root `main.py` keeps backward compatibility and adds `src/` to `PYTHONPATH` at runtime.
- New modules should be added under `src/`.
- Python dependencies are managed via `requirements.txt` and `environment.yml`.
