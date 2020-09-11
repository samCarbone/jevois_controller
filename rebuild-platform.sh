#!/usr/bin/env bash

sudo rm -rf pbuild \
	&& mkdir pbuild \
	&& cd pbuild \
	&& cmake -DJEVOIS_PLATFORM=ON .. \
	&& make
