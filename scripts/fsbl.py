#!/usr/bin/env python3
# Replacement for scripts/fsbl.tcl for Vitis 2026.1+, where xsct is disabled.
# Uses the Vitis Python API to generate and build the zynq_fsbl boot domain.
#
# Usage (via Makefile): vitis -s scripts/fsbl.py <project_name> <proc_name>
#
# Steps:
#   1. Create a Vitis platform from the project's .xsa.
#      For Zynq, this automatically generates an FSBL boot domain with
#      sources in <workspace>/<platform>/zynq_fsbl/.
#   2. Patch fsbl_hooks.c (adds SetMacAddress declaration + call).
#   3. Add red_pitaya_fsbl_hooks.c to the build via UserConfig.cmake.
#   4. Build the platform (compiles BSP + FSBL).
#   5. Copy <workspace>/<platform>/zynq_fsbl/build/fsbl.elf to
#      tmp/<project>.fsbl/executable.elf.

import sys
import os
import shutil
import glob

import vitis


def patch_fsbl_hooks(path):
    with open(path, 'r') as f:
        content = f.read()

    # Hunk 1: insert SetMacAddress() forward declaration in Function Prototypes section.
    # The template has two blank lines between the header and the next block comment.
    old = (
        "/************************** Function Prototypes"
        " ******************************/\n\n\n"
    )
    new = (
        "/************************** Function Prototypes"
        " ******************************/\n\n"
        "u32 SetMacAddress();\n\n"
    )
    if old not in content:
        print(f"WARNING: fsbl_hooks.c prototype section not found as expected; "
              f"patch hunk 1 skipped")
    else:
        content = content.replace(old, new, 1)

    # Hunk 2: call SetMacAddress() in FsblHookBeforeHandoff before the return.
    old = (
        '\tfsbl_printf(DEBUG_INFO,"In FsblHookBeforeHandoff function \\r\\n");\n'
        '\n'
        '\treturn (Status);\n'
        '}'
    )
    new = (
        '\tfsbl_printf(DEBUG_INFO,"In FsblHookBeforeHandoff function \\r\\n");\n'
        '\tStatus = SetMacAddress();\n'
        '\n'
        '\treturn (Status);\n'
        '}'
    )
    if old not in content:
        print(f"WARNING: FsblHookBeforeHandoff return site not found as expected; "
              f"patch hunk 2 skipped")
    else:
        content = content.replace(old, new, 1)

    with open(path, 'w') as f:
        f.write(content)


def add_source_to_userconfig(userconfig_path, src_filename):
    with open(userconfig_path, 'r') as f:
        content = f.read()

    old = "set(USER_COMPILE_SOURCES\n)"
    new = f"set(USER_COMPILE_SOURCES\n${{CMAKE_CURRENT_SOURCE_DIR}}/{src_filename}\n)"
    if old not in content:
        print(f"WARNING: USER_COMPILE_SOURCES not found in {userconfig_path}; "
              f"skipping source addition")
    else:
        content = content.replace(old, new, 1)
        with open(userconfig_path, 'w') as f:
            f.write(content)


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <project_name> <proc_name>")
        sys.exit(1)

    project_name = sys.argv[1]
    proc_name = sys.argv[2]  # noqa: F841 — kept for future use / documentation

    # All paths are relative to the project root (cwd when make runs).
    xsa_path = os.path.abspath(f"tmp/{project_name}.xsa")
    output_dir = os.path.abspath(f"tmp/{project_name}.fsbl")
    output_elf = os.path.join(output_dir, "executable.elf")
    workspace = os.path.abspath(f"tmp/{project_name}.workspace")
    hooks_patch_src = os.path.abspath("patches/red_pitaya_fsbl_hooks.c")

    platform_name = f"{project_name}_platform"

    if os.path.exists(workspace):
        shutil.rmtree(workspace)

    client = vitis.create_client()
    try:
        client.set_workspace(path=workspace)

        # create_platform_component for Zynq automatically creates an FSBL
        # boot domain.  Sources land in <workspace>/<platform>/zynq_fsbl/
        # before the platform is built.
        platform = client.create_platform_component(
            name=platform_name,
            hw_design=xsa_path,
        )

        # The FSBL source directory is created synchronously by platform
        # creation; patch it before building.
        fsbl_src_dir = os.path.join(workspace, platform_name, "zynq_fsbl")
        fsbl_hooks = os.path.join(fsbl_src_dir, "fsbl_hooks.c")
        userconfig = os.path.join(fsbl_src_dir, "UserConfig.cmake")

        patch_fsbl_hooks(fsbl_hooks)
        shutil.copy(hooks_patch_src, fsbl_src_dir)
        add_source_to_userconfig(userconfig, "red_pitaya_fsbl_hooks.c")

        platform.build()

        # The ELF produced by the platform boot domain build.
        elf_files = glob.glob(
            os.path.join(fsbl_src_dir, "**", "*.elf"),
            recursive=True,
        )
        if not elf_files:
            print("ERROR: built ELF not found under", fsbl_src_dir)
            sys.exit(1)

        os.makedirs(output_dir, exist_ok=True)
        shutil.copy(elf_files[0], output_elf)
        print(f"FSBL ELF written to: {output_elf}")

    finally:
        vitis.dispose()


if __name__ == "__main__":
    main()
