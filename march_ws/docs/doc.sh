#!/usr/bin/env bash

rm -rf build/


cwd=$(pwd)
base_output_dir="$cwd/build/"

for directory in $(find -O3 -L ../src/ -name "CMakeLists.txt")
do

    if [[ "$directory" == *"march_"* ]]
    then
            package_name=$(basename $(dirname "${directory}"))
            dir_name=$(dirname "${directory}")
            output_dir="$base_output_dir$package_name"
            doxyfile="$dir_name/docs/Doxyfile"

            if [ ! -f $doxyfile ]; then
                echo "Skipping package $package_name, no Doxyfile found at $doxyfile."
            else
                echo "Building documentation for package $package_name, Doxyfile found at $doxyfile."
                mkdir -p build/$package_name
                cd "$dir_name/docs"

                ( cat Doxyfile ; echo "OUTPUT_DIRECTORY=$output_dir" ) | doxygen -
                cd "$cwd"

                mkdir -p build/html/$package_name
                mv  build/$package_name/html/*  build/html/$package_name
                rmdir build/$package_name/html
                rmdir build/$package_name/
            fi
    fi
done

doxygen Doxyfile