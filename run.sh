#!/bin/bash

blue="\033[1;34m"
yellow="\033[1;33m"
red="\033[1;31m"
reset="\033[0m"


max_threads=$(cat /proc/cpuinfo | grep "processor" | wc -l)


if [ ! -d "build" ]; then 
    mkdir build
fi

while getopts ":ritdg:" opt; do
    case $opt in
        r)
            echo -e "${yellow}<<<--- rebuild --->>>\n${reset}"
            cd build
            sudo make uninstall
            cd ..
            sudo rm -rf build
            mkdir build
            shift
            ;;

        i)
            echo -e "${yellow}<<<--- reinstall --->>>\n${reset}"
            cd build
            sudo make uninstall
            cd ..
            shift
            ;;
        t)
            echo -e "${yellow}<<<--- terminal --->>>\n${reset}"
            cd build
            cmake ..
            sudo make install
            cd ../terminal
            if [ ! -d "build" ]; then 
                mkdir build
            fi
            cd build
            cmake ..
            make -j "$max_threads"
            make -j "$max_threads"
            sudo make install
            cd ../..
            exit 0
            shift
            ;;
        d)
            echo -e "${yellow}\nThe following files and directories will be deleted:${reset}"
            sudo find "$(pwd)" -maxdepth 1 -name "build"
            sudo find /usr/local/ -name "*wust_vl*"
            
            echo -e "${red}Are you sure you want to delete wust_vl? (y/n): ${reset}"
            read answer
            if [ "$answer" == "y" ] || [ "$answer" == "Y" ]; then
                echo -e "${yellow}\n<<<--- delete --->>>\n${reset}"
                sudo find "$(pwd)" -maxdepth 1 -name "build" -exec rm -rf {} +
                sudo find /usr/local/ -name "*wust_vl*" -exec rm -rf {} +
            else
                echo -e "${yellow}Deletion operation canceled${reset}"
            fi
            exit 0
            ;;
        g)
            git_message=$OPTARG
            echo -e "${yellow}\n<--- Git $git_message --->${reset}"
            git pull
            git add -A
            git commit -m "$git_message"
            git push
            exit 0
            shift
            ;;
        \?)
            echo -e "${red}\n--- Unavailable param: -$OPTARG ---\n${reset}"
            ;;
        :)
            echo -e "${red}\n--- param -$OPTARG need a value ---\n${reset}"
            ;;
    esac
done



echo -e "${yellow}\n<<<--- start cmake --->>>\n${reset}"
cd build
cmake ..

echo -e "${yellow}\n<<<--- start make --->>>\n${reset}"
make -j "$max_threads"

echo -e "${yellow}\n<<<--- start install --->>>\n${reset}"
sudo make install
sudo rm /usr/lib/wust_vl/*
sudo ln -s /usr/local/lib/wust_vl/* /usr/lib

echo -e "${yellow}\n<<<--- Total Lines --->>>${reset}"
echo -e "${blue}           $total${reset}"

echo -e "${yellow}\n<<<--- Welcome WUST_VL--->>>\n${reset}"
