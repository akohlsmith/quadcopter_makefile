#!/bin/sh
# Fichier "flash_terminal.sh"

openocd -f openocd.cfg \
   -c init \
   -c targets \
   -c "reset halt" \
   -c "flash write_image erase  build/stm32f4_usb_cdc.hex 0 ihex" \
   -c "verify_image build/stm32f4_usb_cdc.hex" \
   -c "shutdown"



#   -c "flash erase_sector 0 0 11" \
