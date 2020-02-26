# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SOURCEDIR     = .
BUILDDIR      = _build
PUBLISHDIR    = ../docs

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

# Integrate the sphinx-autobuild
# See: https://github.com/GaretJax/sphinx-autobuild
livehtml:
	sphinx-autobuild -b html $(SOURCEDIR) $(BUILDDIR)/html

# Copy the files from "docs_source/_build/html/" to "docs/"
publish:
	# Remove the old publish directory
	rm -rf $(PUBLISHDIR)
	# Copy the files
	cp -r $(BUILDDIR)/html $(PUBLISHDIR)
	# Turn off Jekyll
	# See: https://help.github.com/en/articles/files-that-start-with-an-underscore-are-missing
	touch $(PUBLISHDIR)/.nojekyll

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
