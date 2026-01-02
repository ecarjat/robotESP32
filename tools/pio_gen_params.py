Import("env")

import os
import subprocess
import sys


def run_generator():
    project_dir = env["PROJECT_DIR"]
    script = os.path.join(project_dir, "tools", "gen_params_json.py")
    if not os.path.exists(script):
        print("[param-map] generator missing:", script)
        env.Exit(1)
    try:
        subprocess.check_call([sys.executable, script], cwd=project_dir)
    except subprocess.CalledProcessError:
        print("[param-map] generator failed")
        env.Exit(1)


run_generator()
