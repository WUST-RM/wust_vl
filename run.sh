#!/bin/bash
source env.bash
blue="\033[1;34m"
yellow="\033[1;33m"
red="\033[1;31m"
reset="\033[0m"

BUILD_DIR="build"

# --------------------- ensure build dir ---------------------
if [ ! -d "$BUILD_DIR" ]; then 
    mkdir "$BUILD_DIR"
fi

# --------------------- parse options ---------------------
while getopts ":ritdg:" opt; do
    case $opt in
        r)
            echo -e "${yellow}<<<--- rebuild (clean build folder) --->>>${reset}"
            cd "$BUILD_DIR"
            sudo ninja uninstall 2>/dev/null
            cd ..
            sudo rm -rf "$BUILD_DIR"
            mkdir "$BUILD_DIR"
            shift
            ;;

        i)
            echo -e "${yellow}<<<--- reinstall --->>>${reset}"
            cd "$BUILD_DIR"
            sudo ninja uninstall 2>/dev/null
            cd ..
            shift
            ;;

        d)
            echo -e "${yellow}Will delete the following:${reset}"
            sudo find "$(pwd)" -maxdepth 1 -name "build"
            sudo find /usr/local/ -name "*wust_vl*"

            echo -e "${red}Are you sure? (y/n): ${reset}"
            read ans
            if [[ "$ans" =~ ^[Yy]$ ]]; then
                echo -e "${yellow}Deleting...${reset}"
                sudo find "$(pwd)" -maxdepth 1 -name "build" -exec rm -rf {} +
                sudo find /usr/local/ -name "*wust_vl*" -exec rm -rf {} +
            else
                echo -e "${yellow}Canceled.${reset}"
            fi
            exit 0
            ;;
        \?)
            echo -e "${red}Invalid param: -$OPTARG${reset}"
            ;;
        :)
            echo -e "${red}param -$OPTARG requires a value${reset}"
            ;;
    esac
done

# ------------------ Main: build + install ------------------

echo -e "${yellow}\n<<<--- CMake (Ninja) --->>>${reset}"
cd "$BUILD_DIR"
cmake -G Ninja ..

echo -e "${yellow}\n<<<--- Ninja Build --->>>${reset}"
ninja   # 自动满线程

echo -e "${yellow}\n<<<--- Install --->>>${reset}"
sudo ninja install

sudo rm /usr/lib/wust_vl_* 2>/dev/null
sudo ln -s /usr/local/lib/wust_vl_* /usr/lib 2>/dev/null

# ------------------ Count Lines ----------------------------
echo -e "${yellow}\n<--- Total Lines --->${reset}"
total=$(find .. \
    -type d \( \
        -path ../build -o \
        -path ../hikSDK -o \
        -path ../model -o \
        -path ../3rdparty -o \
        -path ../.cache \
    \) -prune -o \
    -type f \( \
        -name "*.cpp" -o -name "*.hpp" -o -name "*.c" -o -name "*.h" \
        -o -name "*.py" -o -name "*.html" -o -name "*.sh" -o -name "*.md" \
        -o -name "*.yaml" -o -name "*.json" -o -name "*.css" -o -name "*.js" \
        -o -name "*.cu" -o -name "*.txt" \
    \) -exec wc -l {} + | awk 'END{print $1}')
echo -e "${blue}        $total${reset}"

echo -e "${yellow}\n<<<--- Welcome WUST_VL --->>>${reset}"
