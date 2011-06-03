#!/bin/bash
find . \( -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.java' -o -name '*.py' -o -name '*.pl' \) -a \! \( -wholename '*CMakeFiles*' -o -wholename '*cpp/include*' -o -wholename '*/_*' \) | xargs wc -l
