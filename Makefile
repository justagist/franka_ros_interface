# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line.
SPHINXOPTS    =
SPHINXBUILD   = sphinx-build
SOURCEDIR     = .
BUILDDIR      = ../docs_build
PDFBUILDDIR   = /tmp
PDF           = ../documentation.pdf

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

latexpdf:
	$(SPHINXBUILD) -b latex $(SOURCEDIR) $(PDFBUILDDIR)/latex
	#                                          ^^^
	@echo "Running LaTeX files through pdflatex..."
	make -C $(PDFBUILDDIR)/latex all-pdf
	#         ^^^
	cp $(PDFBUILDDIR)/latex/*.pdf $(PDF)
	#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	@echo "pdflatex finished; see $(PDF)"

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)