#!/usr/bin/env python3
"""Compile and check the actual LCD conversion helper on a Linux host."""

from pathlib import Path
import re
import subprocess
import tempfile
import unittest


class DisplayFormatTests(unittest.TestCase):
    def test_two_digit_display_without_repeated_allocation(self):
        source = (
            Path(__file__).resolve().parents[1] / "firmware/discovery/main.c"
        ).read_text(encoding="utf-8")
        match = re.search(
            r"static char \*int_to_string\(unsigned int number\)\s*\{.*?^\}",
            source,
            flags=re.MULTILINE | re.DOTALL,
        )
        self.assertIsNotNone(match, "Cannot locate the firmware helper")
        helper = match.group(0)
        harness = r"""
#include <assert.h>
#include <stdio.h>
#include <string.h>

HELPER

int main(void)
{
    char *buffer = int_to_string(0);
    assert(buffer != NULL);
    assert(strcmp(buffer, "00") == 0);
    for (unsigned int n = 0; n < 100; ++n)
    {
        char *value = int_to_string(n);
        assert(value == buffer);
        assert(strlen(value) == 2);
        assert(value[0] == (char)('0' + n / 10));
        assert(value[1] == (char)('0' + n % 10));
    }
    for (unsigned int n = 0; n < 100000; ++n)
    {
        assert(int_to_string(n % 7) == buffer);
    }
    assert(strcmp(int_to_string(6), "06") == 0);
    assert(strcmp(int_to_string(0), "00") == 0);
    return 0;
}
""".replace("HELPER", helper)
        with tempfile.TemporaryDirectory(prefix="parking-display-check-") as temp:
            c_file = Path(temp) / "display_check.c"
            executable = Path(temp) / "display_check"
            c_file.write_text(harness, encoding="utf-8")
            subprocess.run(
                [
                    "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                    "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                    str(c_file), "-o", str(executable),
                ],
                check=True,
            )
            subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
