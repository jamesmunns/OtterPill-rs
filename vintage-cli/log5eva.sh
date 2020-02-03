#!/bin/bash

until cargo run --release -- log
do
	echo "Crash :("
	sleep 3
done
