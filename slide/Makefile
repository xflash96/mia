MAIN := vfx
all: $(MAIN).pdf

$(MAIN).pdf: $(MAIN).tex
	pdflatex $^ -o $@

.PHONY: install
install:
	evince $(MAIN).pdf &

.PHONY: clean
clean:
	rm -rf *.aux *.log *.nav *.out *.snm *.toc *.bbl *.blg *.dvi

.PHONY: bib
bib: $(MAIN).tex sdp.bib
	latex $(MAIN).tex
	bibtex $(MAIN).aux
	latex $(MAIN).tex
	latex $(MAIN).tex
	pdflatex $(MAIN).tex
