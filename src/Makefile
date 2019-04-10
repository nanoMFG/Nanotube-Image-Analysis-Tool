
APP = FT2D_GUI
MFILES = 
matlabversion = 2018a
DELETE_FILES =
BIN = ../bin

$(APP): $(APP).m $(MFILES)
	mcc -v -C -m -R -nodesktop $(APP).m $(MFILES)
	patchrunscript.sh run_$(APP).sh run_$(APP).sh

all:
	. /etc/environ.sh; use -e -r matlab-$(matlabversion); make $(APP)

install: all
	install --mode 0755 -D run_$(APP).sh $(BIN)
	install --mode 0755 -D $(APP) $(BIN)
	install --mode 0755 -D link_examples.sh $(BIN)/link_examples
	install --mode 0644 -D FT2D_GUI.fig $(BIN)/FT2D_GUI.fig
	install --mode 0644 -D readme.txt $(BIN)/$(APP).readme
	if [ -e $(APP).ctf ] ; then \
        mv $(APP).ctf $(BIN); \
        cd $(BIN); \
        . /etc/environ.sh; use -e -r matlab-$(matlabversion); extractCTF $(APP).ctf; \
    fi

clean:
	rm -f $(APP) $(APP).ctf \
         $(APP).prj run_$(APP).sh \
         $(APP)_main.c $(APP)_mcc_component_data.c \
         mccExcludedFiles.log requiredMCRProducts.txt \
         readme.txt $(DELETE_FILES)

distclean: clean
	rm -rf $(BIN)/$(APP) \
          $(BIN)/$(APP).ctf \
          $(BIN)/$(APP).readme \
          $(BIN)/$(APP)_mcr \
          $(BIN)/run_$(APP).sh \
		  $(BIN)/FT2D_GUI.fig \
		  $(BIN)/link_examples

