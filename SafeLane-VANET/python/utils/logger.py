from __future__ import annotations
import csv
from pathlib import Path

class CsvLogger:
    def __init__(self, path: Path, fieldnames: list[str]):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.fieldnames = fieldnames
        self._fh = open(self.path, "w", newline="", encoding="utf-8")
        self._w = csv.DictWriter(self._fh, fieldnames=self.fieldnames)
        self._w.writeheader()

    def write(self, row: dict):
        out = {k: row.get(k, "") for k in self.fieldnames}
        self._w.writerow(out)

    def close(self):
        try:
            self._fh.close()
        except Exception:
            pass
