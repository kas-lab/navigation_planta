#!/usr/bin/env python3
"""Generate all navigation paper data, TeX plots/tables, and PDF/PNG previews.

Consumes a completed all_experiments suite; never runs planners or edits manuscript sources.
"""

import argparse
import json
from pathlib import Path
import sys

from helpers.paper_export import generate_paper_artifacts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suite_dir', type=Path)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--dry-run', action='store_true', help='Validate and report without writing')
    parser.add_argument('--overwrite', action='store_true', help='Replace an unmodified exporter-owned bundle')
    parser.add_argument('--no-previews', action='store_true',
                        help='Omit PDF/PNG previews and standalone wrappers; no TeX tools needed')
    parser.add_argument('--latex-prefix', default='',
                        help='Relative bundle path from the LaTeX project root, e.g. navigation_experiment')
    args = parser.parse_args()
    try:
        report = generate_paper_artifacts(args.suite_dir, args.output_dir,
                                          overwrite=args.overwrite, dry_run=args.dry_run,
                                          no_previews=args.no_previews, latex_prefix=args.latex_prefix)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'Paper export failed: {exc}', file=sys.stderr)
        return 1
    if args.dry_run:
        print(json.dumps(report, indent=2, ensure_ascii=False))
    else:
        print(f'Paper artifacts saved to {report["output_directory"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
