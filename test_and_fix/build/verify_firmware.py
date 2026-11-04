"""Compile and link firmware with ARM GCC, without generated IDE makefiles.

Run from any directory. Outputs go to build/refactor_verify; never flashes hardware.
Requires ARM GCC 11+; MQTT credentials are needed only with uplink enabled.
"""
from pathlib import Path
import argparse
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "build" / "refactor_verify"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="arm-none-eabi-gcc", help="ARM GCC executable (GCC 11 or newer)")
    args = parser.parse_args()
    cc = shutil.which(args.cc)
    if not cc:
        raise SystemExit("arm-none-eabi-gcc must be on PATH")
    OUT.mkdir(parents=True, exist_ok=True)
    make = (ROOT / "STM32Make.make").read_text(encoding="utf-8-sig")
    sources = re.findall(r"^(?:Core|Drivers|Middlewares)/[^\s]+\.[cs](?=\s|$)", make, re.M)
    # Generated makefiles can lag behind newly added application modules.
    # CubeIDE builds the Core source directory; mirror that discovery here.
    sources = sorted(set(sources) | {
        p.relative_to(ROOT).as_posix() for p in (ROOT / "Core" / "Src").glob("*.c")
    })
    includes = ["Core/Inc", "Drivers/STM32F4xx_HAL_Driver/Inc",
                "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy", "Drivers/CMSIS/Include",
                "Drivers/CMSIS/Device/ST/STM32F4xx/Include",
                "Middlewares/Third_Party/FreeRTOS/Source/include",
                "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2",
                "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"]
    cpu = ["-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard"]
    flags = cpu + ["-std=gnu11", "-DDEBUG", "-DUSE_HAL_DRIVER", "-DSTM32F407xx",
                   "-O0", "-g3", "-ffunction-sections", "-fdata-sections", "-Wall",
                   "-Werror=implicit-function-declaration", "-fstack-usage",
                   "--specs=nano.specs"] + ["-I" + p for p in includes]
    objects = []
    for source in sources:
        obj = OUT / Path(source).with_suffix(".o")
        obj.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run([cc, *flags, "-c", source, "-o", str(obj)], cwd=ROOT, check=True)
        objects.append(obj)
    # The local response file is a build artifact, not tracked configuration.
    rsp = OUT / "link.tmp"
    rsp.write_text("\n".join('"' + p.as_posix() + '"' for p in objects), encoding="utf-8")
    elf = OUT / "firmware.elf"
    subprocess.run([cc, *cpu, "@" + str(rsp), "-o", str(elf),
                    "-TSTM32F407ZETX_FLASH.ld", "--specs=nosys.specs", "--specs=nano.specs",
                    # Check the backend links even when the publish task is disabled.
                    "-Wl,-u,Modem_Init,-u,Modem_Send,-u,Modem_OnRxEvent",
                    "-Wl,--gc-sections", "-Wl,-Map=" + str(OUT / "firmware.map"),
                    "-static", "-Wl,--start-group", "-lc", "-lm", "-Wl,--end-group"],
                   cwd=ROOT, check=True)
    print(f"PASS: compiled {len(sources)} sources and linked {elf}")
    size = shutil.which(str(Path(cc).with_name("arm-none-eabi-size" + Path(cc).suffix)))
    if size:
        subprocess.run([size, str(elf)], check=True)


if __name__ == "__main__":
    main()
