#!/bin/bash
readelf -s main.elf | grep OBJECT | cut -d' ' -f5- | sort
