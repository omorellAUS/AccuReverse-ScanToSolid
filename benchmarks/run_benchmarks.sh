#!/bin/bash
for file in tests/*.stl tests/*.obj tests/*.igs; do
  ./build/Release/accureverse config.json $file
  python3 benchmarks/validate.py $file output/simplified.stl
done