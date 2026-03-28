"""generate target partition table header from partitions.csv for ota migration

Runs gen_esp32part.py to convert partitions.csv to binary,
then convert it to C array in src/target_ptable.h
"""
import subprocess, sys, os

Import("env")

def generate_ptable_header(source, target, env):
    project_dir = env.get("PROJECT_DIR", ".")
    csv_path = os.path.join(project_dir, "partitions.csv")
    header_path = os.path.join(project_dir, "src", "target_ptable.h")

    framework_dir = env.PioPlatform().get_package_dir("framework-arduinoespressif32")
    gen_part = os.path.join(framework_dir, "tools", "gen_esp32part.py")

    if not os.path.exists(gen_part):
        print(f"[ptable] ERROR: gen_esp32part.py not found at {gen_part}")
        sys.exit(1)

    result = subprocess.run(
        [env.subst("$PYTHONEXE"), gen_part, csv_path],
        capture_output=True
    )
    if result.returncode != 0:
        print(f"[ptable] ERROR: gen_esp32part.py failed: {result.stderr.decode()}")
        sys.exit(1)

    data = result.stdout

    with open(header_path, "w") as f:
        f.write("// Auto-generated from partitions.csv — do not edit\n")
        f.write("#pragma once\n")
        f.write("#include <stdint.h>\n\n")
        f.write(f"#define TARGET_PTABLE_SIZE {len(data)}\n\n")
        f.write("static const uint8_t TARGET_PTABLE[] PROGMEM = {\n")
        for i in range(0, len(data), 12):
            chunk = data[i:i+12]
            f.write("    " + ", ".join(f"0x{b:02x}" for b in chunk) + ",\n")
        f.write("};\n")

    print(f"[ptable] Generated {header_path} ({len(data)} bytes from {csv_path})")

env.AddPreAction("$BUILD_DIR/src/migrate.cpp.o", generate_ptable_header)
