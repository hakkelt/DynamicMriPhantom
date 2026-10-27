"""MkDocs hook: point links to repository files outside docs/ at GitHub, so the
Markdown pages stay readable on GitHub too."""
import os
import re

BLOB = "https://github.com/hakkelt/DynamicMriPhantom/blob/master/"
LINK = re.compile(r"\]\((?![a-z]+:|#)([^)\s]+)\)")


def on_page_markdown(markdown, page, config, files):
    def fix(m):
        path = os.path.normpath(os.path.join("docs", m.group(1))).replace(os.sep, "/")
        if path.startswith("docs/"):
            return f"]({path[len('docs/'):]})"
        return f"]({BLOB}{path})"

    return LINK.sub(fix, markdown)
