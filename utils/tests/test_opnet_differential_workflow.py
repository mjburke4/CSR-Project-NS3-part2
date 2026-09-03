#!/usr/bin/env python3
"""End-to-end tests for run-opnet-differential.py."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import sys
import tempfile
import textwrap
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "utils" / "run-opnet-differential.py"
COMPARATOR = ROOT / "utils" / "compare-opnet-ns3-traces.py"


def load_uncached_module(name: str, path: Path):
    """Execute a temporary workflow copy without generating import bytecode."""

    module = types.ModuleType(name)
    module.__file__ = str(path)
    module.__package__ = ""
    sys.modules[name] = module
    try:
        exec(compile(path.read_bytes(), str(path), "exec"), module.__dict__)
    finally:
        del sys.modules[name]
    return module


def write_instrumented_comparator(destination: Path) -> None:
    """Copy the child comparator and record its effective interpreter flags."""

    marker = '\nif __name__ == "__main__":\n'
    program = COMPARATOR.read_text(encoding="utf-8")
    probe = textwrap.dedent(
        """

        sys.path.insert(0, str(Path(__file__).resolve().parent))
        import child_execution_probe
        child_execution_probe.record(Path(__file__).name)
        """
    )
    if marker not in program:
        raise RuntimeError(f"cannot instrument {COMPARATOR}")
    destination.write_text(program.replace(marker, probe + marker), encoding="utf-8")


class DifferentialWorkflowIsolationTests(unittest.TestCase):
    def test_comparator_is_isolated_and_bytecode_free(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            tools = root / "tools"
            tools.mkdir()
            workflow_path = tools / SCRIPT.name
            shutil.copy2(SCRIPT, workflow_path)
            comparator_path = tools / COMPARATOR.name
            write_instrumented_comparator(comparator_path)
            (tools / "child_execution_probe.py").write_text(
                textwrap.dedent(
                    """\
                    import json
                    from pathlib import Path
                    import sys

                    def record(tool):
                        state = {
                            "tool": tool,
                            "isolated": sys.flags.isolated,
                            "dont_write_bytecode": sys.dont_write_bytecode,
                        }
                        Path(__file__).with_name("child-state.json").write_text(
                            json.dumps(state, sort_keys=True) + "\\n",
                            encoding="utf-8",
                        )
                    """
                ),
                encoding="utf-8",
            )
            workflow = load_uncached_module(
                "csr_opnet_differential_workflow_isolation_test",
                workflow_path,
            )

            scenario = root / "scenario.csv"
            scenario.write_text("fixture\n", encoding="utf-8")
            opnet_trace = root / "opnet.csv"
            trace_text = "time_s,event,node\n1,tx,1\n"
            opnet_trace.write_text(trace_text, encoding="utf-8")
            runner = root / "fake-runner"
            runner.write_text(
                textwrap.dedent(
                    """\
                    #!/bin/sh
                    trace=
                    for argument in "$@"; do
                        case "$argument" in
                            --trace=*) trace=${argument#--trace=} ;;
                        esac
                    done
                    test -n "$trace" || exit 64
                    printf 'time_s,event,node\\n1,tx,1\\n' > "$trace"
                    """
                ),
                encoding="utf-8",
            )
            os.chmod(runner, 0o755)
            output = root / "run"
            argv = [
                str(workflow_path),
                "--scenario",
                str(scenario),
                "--runner",
                str(runner),
                "--opnet-trace",
                str(opnet_trace),
                "--output-dir",
                str(output),
            ]
            with mock.patch.object(sys, "argv", argv):
                with self.assertRaises(SystemExit) as result:
                    workflow.main()
            self.assertEqual(result.exception.code, 0)

            manifest = json.loads(
                (output / "run-manifest.json").read_text(encoding="utf-8")
            )
            compare_command = manifest["comparator_command"]
            self.assertEqual(compare_command[:3], [sys.executable, "-B", "-I"])
            self.assertEqual(Path(compare_command[3]), comparator_path)
            state = json.loads(
                (tools / "child-state.json").read_text(encoding="utf-8")
            )
            self.assertEqual(state["tool"], COMPARATOR.name)
            self.assertEqual(state["isolated"], 1)
            self.assertTrue(state["dont_write_bytecode"])
            self.assertEqual(list(tools.rglob("*.pyc")), [])
            self.assertEqual(list(tools.rglob("__pycache__")), [])


if __name__ == "__main__":
    unittest.main()
