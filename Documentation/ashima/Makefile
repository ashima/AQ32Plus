
DOCS:=$(strip $(subst /, ,$(dir $(wildcard */main.tex))))
#TARGETS:=$(foreach n,$(DOCS),$(subst %,$n,%/%.pdf))
TARGETS:=$(DOCS:%=build/%.pdf)

B=build

all: $(TARGETS)
	@echo DOCS = $(DOCS)
	@echo TARGETS = $(TARGETS)

$B/%.pdf: %/main.tex | $B
	@echo working on $@  
	cd $(basename $(notdir $@)) ; ../build.sh ../$B

$B:
	-mkdir -p $@
