import sys
import toml


class MultilineLiteralTomlEncoder(toml.TomlEncoder):
    def __init__(self, _dict=dict):
        super().__init__(_dict)
        self.super_dump_str = self.dump_funcs[str]
        self.dump_funcs[str] = self._dump_str

    def _dump_str(self, v: str) -> str:
        # Decide when to use multiline: contains newline or carriage return
        if ("\n" in v) or ("\r" in v):
            # For TOML basic multiline strings ('''...'''):
            # The input string has been interpreted by load before, so the value will always be a "raw" string, hence the triple single quotes
            return "'''" + v + "'''"
        # Fallback to the library's default string handling

        return self.super_dump_str(v)


def deep_merge(old, new):
    for k, v in new.items():
        if k in old:
            # Old existing values take priority, so we should check whether old contains a dict separately for recursion
            if isinstance(old[k], dict) and isinstance(v, dict):
                deep_merge(old[k], v)
        else:
            old[k] = v
    return old


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: merge-configuration-files.py <old_file> <new_file> <out_file>")
        sys.exit(1)
    old_file, new_file, out_file = sys.argv[1:4]
    old_cfg = toml.load(open(old_file))
    new_cfg = toml.load(open(new_file))
    merged = deep_merge(old_cfg, new_cfg)
    with open(out_file, "w") as f:
        toml.dump(merged, f, encoder=MultilineLiteralTomlEncoder())
