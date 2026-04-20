from pathlib import Path

NO_PLAN = -1


def count_plan_actions(plan_file: Path) -> int:
    """Count actions in an FD plan file, ignoring comment/cost lines.

    FD writes plans to {plan_file}.1, .2, ... — take the last (best for
    satisficing search). Returns NO_PLAN (-1) if no plan file is found.
    """
    candidates = sorted(plan_file.parent.glob(plan_file.name + '.*'))
    target = candidates[-1] if candidates else (plan_file if plan_file.exists() else None)
    if target is None:
        return NO_PLAN
    lines = target.read_text().splitlines()
    return sum(1 for line in lines if line.strip() and not line.startswith(';'))
