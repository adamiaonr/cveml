TO_ZIP_DIRS = $(filter %/, $(wildcard libraries/*/)) 	# find all directories in libraries/
TO_ZIP_NAMES = $(patsubst %/,%,$(TO_ZIP_DIRS))  			# remove trailing /
ZIP_TARGETS = $(addsuffix .zip,$(TO_ZIP_NAMES))  			# add .zip

.SECONDEXPANSION:

$(ZIP_TARGETS): %.zip : $$(shell find % -type f ! -path "%/.*")
	cd $(basename $@)/.. && zip -FSr $(notdir $@) $(notdir $(basename $@)) -x $(notdir $(basename $@))/.\*

zip_projects: $(ZIP_TARGETS)
