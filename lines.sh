#!/bin/bash
find . \( -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.java' \) -a \! \( -wholename '*CMakeFiles*' -o -wholename '*cpp/include*' \) | xargs wc -l
