from pathlib import Path
import sys

import matplotlib


# Import directly from the checkout; the repository is not an installed package.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))


matplotlib.use("Agg")
