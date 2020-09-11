#!/usr/bin/env bash

sudo rm -rf hbuild \
	&& mkdir hbuild \
	&& cd hbuild \
	&& mkdir logs \
	&& cmake .. \
	&& make
