# Uncomment lines below if you have problems with /usr/local/sbin:/usr/local/bin:/usr/bin:/var/lib/flatpak/exports/bin:/usr/bin/site_perl:/usr/bin/vendor_perl:/usr/bin/core_perl:/home/arc/.local/bin
# SHELL := /bin/bash
# PATH := /usr/local/bin:

all:
	pio -f -c vim run

upload:
	pio -f -c vim run --target upload

monitor:
	pio -f -c vim run --target monitor

clean:
	pio -f -c vim run --target clean

program:
	pio -f -c vim run --target program

uploadfs:
	pio -f -c vim run --target uploadfs

update:
	pio -f -c vim update
