# -- Project information -----------------------------------------------------
project = 'EM1500 ADS Bridge'
copyright = '2025, Elias Landro, Marcus Axelsen Wold and Ole-Morten Nyheim'
author = 'Elias Landro, Marcus Axelsen Wold and Ole-Morten Nyheim'
release = '1.0'

# -- General configuration ---------------------------------------------------
extensions = [
    "myst_parser",
    "sphinx.ext.todo",
    "sphinx.ext.autosectionlabel",
]

source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

templates_path = ['_templates']
exclude_patterns = []

# -- HTML output -------------------------------------------------------------
html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
