.PHONY: install verify-official-files check test all release

install:
	uv sync
	pre-commit install

verify-official-files:
	.venv/bin/python tools/verify_official_model_files.py --offline

check:
	.venv/bin/python -m ruff check .
	find models -name '*.xml' -print0 | xargs -0 .venv/bin/python tools/format_xml.py --check
	.venv/bin/python tools/validate.py
	.venv/bin/python tools/verify_official_model_files.py --offline

test:
	env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -v

all: check test

release:
	.venv/bin/python tools/package_release.py
