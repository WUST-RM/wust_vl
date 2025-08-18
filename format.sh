#!/bin/bash
find ./include -name '*.cpp' -o -name '*.hpp'  -o -name '*.h'| xargs clang-format -i
find ./src -name '*.cpp' -o -name '*.hpp'  -o -name '*.h'| xargs clang-format -i
find . -name '*.yaml' -o -name '*.yml' | xargs prettier --write
find . -name '*.py' | xargs black
find . -name '*.html' | xargs prettier --write
