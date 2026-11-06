# !/bin/bash

source prj.cfg

USE_ST_FLASH=0
NS_FLASH_ADDR=0x8000000
S_FLASH_ADDR=0x0C000000
WRITE_FILE=${PWD}/build/stm32l5_non_secure_bsp.bin

function utils_check_ret()
{
	if [ $1 -eq 0 ]; then
        echo "[INFO] $2 done!"
    else
        echo "[ERR] Failed on $2!"
        exit -1
    fi
}

# Fall back to st-flash when STM32CubeProgrammer is not installed.
if [[ "$USE_ST_FLASH" != "1" ]] && ! command -v STM32_Programmer_CLI >/dev/null 2>&1 \
        && command -v st-flash >/dev/null 2>&1; then
    USE_ST_FLASH=1
fi

# No sudo on Windows (Git Bash / MSYS).
SUDO=sudo
[[ "$(uname)" == *MINGW* || "$(uname)" == *MSYS* || "$(uname)" == *CYGWIN* ]] && SUDO=

if [[ "$USE_ST_FLASH" == "1" ]]; then
    echo "[INFO] use st-flash"
    ${SUDO} st-flash --reset --connect-under-reset write ${WRITE_FILE} ${NS_FLASH_ADDR} || \
        utils_check_ret $? "flash write ${WRITE_FILE} ${NS_FLASH_ADDR}"
else
    STM32_Programmer_CLI -c port=SWD -d ${WRITE_FILE} ${NS_FLASH_ADDR} -s || \
        utils_check_ret $? "flash write ${WRITE_FILE} ${NS_FLASH_ADDR}"
fi
