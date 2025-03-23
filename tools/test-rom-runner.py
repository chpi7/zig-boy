import subprocess
import argparse
from pathlib import Path
import time
from multiprocessing import Process, Pool
import os
import signal

def delayed_kill(delay, pid):
    print(f"Wait for {delay}")
    time.sleep(delay)
    print("Killing...")
    os.kill(pid, signal.SIGKILL)

def run_rom(args: argparse.Namespace, rom: Path):
    print(f"Running {rom.name}")

    executable = args.binary_name
    run_args = [ executable, rom ]

    tmpfile_name = f"output-{rom.name}.txt"
    tmpfile = open(tmpfile_name, "w")

    proc = subprocess.Popen(
        run_args,
        stdout=tmpfile, stderr=tmpfile,
        bufsize=1, universal_newlines=True)

    retcode = proc.wait()

    result_line = ""
    with open(tmpfile_name, "rt") as rf:
        for l in rf.readlines():
            if "Passed" in l:
                result_line = l.replace("[serial] ", "").strip()
                break
            elif "Failed" in l:
                result_line = l.replace("[serial] ", "").strip()
                break

    print(f"{rom.name} done");
    return result_line


def main():
    p = argparse.ArgumentParser()
    p.add_argument("-b", "--binary-name", type=str, default="gb_emulator")
    p.add_argument("--rom-path", type=Path, default=Path("external/gb-test-roms/cpu_instrs/individual"))
    args = p.parse_args()

    rom_folder: Path = args.rom_path
    roms = list(rom_folder.glob("*.gb"))

    run_args = []
    for rom in sorted(roms):
        run_args += [(args, rom)]

    pool = Pool(8)
    results = pool.starmap(run_rom, run_args)
    print('\n'.join(results))

if __name__ == "__main__":
    main()
