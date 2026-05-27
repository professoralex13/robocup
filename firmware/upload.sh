rust-objcopy -O ihex $1 /tmp/firmware.hex
tycmd upload /tmp/firmware.hex
rm /tmp/firmware.hex
tycmd monitor