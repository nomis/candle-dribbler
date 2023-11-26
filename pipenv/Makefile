.PHONY: clean

.venv/bin/python: Pipfile
	rm -rf .venv
	mkdir .venv
	pipenv install --skip-lock

clean:
	rm -rf .venv
