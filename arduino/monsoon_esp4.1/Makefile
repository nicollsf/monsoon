BUILDDIR:= build/esp32.esp32.esp32
BINFILE := monsoon_esp4.1.ino.bin
JSONFILE := $(BUILDDIR)/monsoon_esp4.1.json
#HOSTDIR:= fred_nicolls@34.89.101.249:/var/www/html/monsoon
HOSTDIR:= nicolls@dip.ee.uct.ac.za:/home/nicolls/.public_html/ota/monsoon
VER := $(shell grep "define OTA_VERSION" ota.h | sed -n 's/\(.*\)\"\(.*\)\"\(.*\)/\2/p')
SCP:= scp -i /Users/nicolls/.ssh/id_rsa_gcloud

$(JSONFILE):  ota.h
	@echo "Creating $(JSONFILE)"
	@/bin/rm -f $(JSONFILE)
	@/bin/cat ota/otastart.json > $(JSONFILE)
	@/bin/echo -n "	\"Version\": \"" >> $(JSONFILE)
	@/bin/echo -n $(VER) >> $(JSONFILE)
	@/bin/echo "\"," >> $(JSONFILE)
	@/bin/cat ota/otaend.json >> $(JSONFILE)

install: $(JSONFILE)
	$(SCP) $(BUILDDIR)/$(BINFILE) $(HOSTDIR)
	$(SCP) $(JSONFILE) $(HOSTDIR)
