cd app_my_controller
make clean && make &&  python3 -m cfloader flash build/cf2.bin stm32-fw -w radio://0/100/2M/E7E7E7E704
cd -