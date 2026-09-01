"""Minimal reader for the boost property-tree INFO files used by OCS2.

Only the subset the plotting script needs: nested `key { ... }` blocks and `key value` leaves,
with `;` comments. Values are returned as strings; the caller converts.
"""


def _strip_comment(line):
    # `;` only starts a comment outside of a quoted string.
    out = []
    in_quotes = False
    for ch in line:
        if ch == '"':
            in_quotes = not in_quotes
        if ch == ";" and not in_quotes:
            break
        out.append(ch)
    return "".join(out).strip()


def parse_info(text):
    root = {}
    stack = [root]
    pending_key = None

    for raw in text.splitlines():
        line = _strip_comment(raw)
        if not line:
            continue

        while line:
            if line.startswith("{"):
                child = {}
                key = pending_key if pending_key is not None else ""
                stack[-1][key] = child
                stack.append(child)
                pending_key = None
                line = line[1:].strip()
                continue
            if line.startswith("}"):
                if len(stack) > 1:
                    stack.pop()
                line = line[1:].strip()
                continue

            # key [value]
            if line[0] == '"':
                end = line.index('"', 1)
                key = line[1:end]
                rest = line[end + 1 :].strip()
            else:
                parts = line.split(None, 1)
                key = parts[0]
                rest = parts[1].strip() if len(parts) > 1 else ""

            if not rest or rest.startswith("{"):
                pending_key = key
                line = rest
                if not line:
                    # The opening brace is on the next line.
                    break
                continue

            if rest[0] == '"':
                end = rest.index('"', 1)
                value = rest[1 : end]
                line = rest[end + 1 :].strip()
            else:
                parts = rest.split(None, 1)
                value = parts[0]
                line = parts[1].strip() if len(parts) > 1 else ""

            stack[-1][key] = value

    return root


def load_info(path):
    with open(path, "r") as f:
        return parse_info(f.read())
