.PHONY: install verify-official-files check test all release

install:
	uv sync
	pre-commit install

verify-official-files:
	.venv/bin/python tools/verify_official_model_files.py --offline

check:
	.venv/bin/python -m ruff check .
	.venv/bin/python tools/format_xml.py --check mobile_fr3_duo.xml mobile_fr3_duo_with_sensors.xml mobile_fr3_duo_position.xml mobile_fr3_duo_reduced.xml mobile_fr3_duo_planar_debug.xml scene.xml scene_with_sensors.xml scene_position.xml
	.venv/bin/python tools/validate_assets.py
	.venv/bin/python tools/verify_official_model_files.py --offline

test:
	env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -v

all: check test

release:
	.venv/bin/python tools/package_release.py
