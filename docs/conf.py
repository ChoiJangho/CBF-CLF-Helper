# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
from pathlib import Path
sys.path.append('.')
from github_linkcode import github_linkcode_resolve
# import sys
# sys.path.insert(0, os.path.abspath('.'))
# sys.path.insert(0, os.path.abspath('../


# -- Project information -----------------------------------------------------

project = 'CBF-CLF-Helper'
copyright = '2022, Jason Choi'
author = 'Jason Choi'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.linkcode',
    'sphinxcontrib.matlab'
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# MATLAB
matlab_keep_package_prefix = False
matlab_src_dir = os.path.dirname(os.path.abspath('.'))
primary_domain = 'mat'
autodoc_member_order = 'bysource'
# autoclass_content = 'init'

# Napoleon
napoleon_use_rtype = False

# Linking to source code
def linkcode_resolve(domain, info):
    return github_linkcode_resolve(
        domain=domain,
        info=info,
        allowed_module_names=[matlab_src_dir],
        github_org_id='ChoiJangho',
        github_repo_id='CBF-CLF-Helper',
        branch='RABBIT',
        source_prefix=''
        )
