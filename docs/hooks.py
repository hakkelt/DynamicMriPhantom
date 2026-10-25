"""MkDocs hooks: publish the root README as the Reference page and point links
to repository files outside docs/ at GitHub, so the Markdown stays readable on
GitHub too."""
import os
import re

from mkdocs.structure.files import File

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BLOB = "https://github.com/hakkelt/DynamicMriPhantom/blob/master/"
LINK = re.compile(r"\]\((?![a-z]+:|#)([^)\s]+)\)")
REFERENCE = "reference.md"


def on_files(files, config):
    with open(os.path.join(ROOT, "README.md"), encoding="utf-8") as f:
        files.append(File.generated(config, REFERENCE, content=f.read()))
    return files


def on_page_markdown(markdown, page, config, files):
    # page sources live in docs/, except the root README published as REFERENCE
    base = "" if page.file.src_uri == REFERENCE else "docs"

    def fix(m):
        path = os.path.normpath(os.path.join(base, m.group(1))).replace(os.sep, "/")
        if path == "README.md":
            return f"]({REFERENCE})"
        if path.startswith("docs/"):
            return f"]({path[len('docs/'):]})"
        return f"]({BLOB}{path})"

    return LINK.sub(fix, markdown)
