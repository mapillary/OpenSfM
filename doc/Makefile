# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = source
BUILDDIR      = ../build/doc
PORT          ?= 8000

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile serve clean

# Serve the documentation locally
serve:
	@echo "Serving documentation at http://localhost:$(PORT)/"
	@echo "Press Ctrl+C to stop the server"
	@python -m http.server --directory $(BUILDDIR)/html $(PORT)

# Clean build artifacts
clean:
	@echo "Cleaning build directory: $(BUILDDIR)"
	@rm -rf $(BUILDDIR)
	@echo "Done"

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
