"""Export tests use synthetic suite results; no planners are invoked."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/export_camara_paper_data.py'
spec = importlib.util.spec_from_file_location('export_camara_paper_data', SCRIPT)
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)


class ExportCamaraTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.suite = self.root / 'suite'
        self.out = self.root / 'export'
        self.sources = {}
        for label, search in exporter.STRATEGIES.items():
            folder = self.suite / label
            folder.mkdir(parents=True)
            (folder / 'search.txt').write_text(search + '\n')
            source = folder / 'camara_discretized/adaptive/planning_times.csv'
            source.parent.mkdir(parents=True)
            source.write_text(self.csv(pddl=True))
            self.sources[label] = source
        source = self.suite / 'camara_prism/planning_times.csv'
        source.parent.mkdir(parents=True)
        source.write_text(self.csv(pddl=False))
        self.sources['prism'] = source

    @staticmethod
    def csv(pddl=True, pairs=('wp1_wp2', 'wp2_wp1'), repeats=2):
        columns = list(exporter.COMMON_COLUMNS)
        if pddl:
            columns.append('comparable_action_count')
        rows = [','.join(columns)]
        for pair in pairs:
            rows.extend(','.join([pair, '0.4', '4', '3', '1', '12.7', '32000', '3.5']
                                 + (['2'] if pddl else [])) for _ in range(repeats))
        return '\n'.join(rows) + '\n'

    def export(self, strategies=None, **kwargs):
        return exporter.export_datasets(self.suite, self.out,
                                        strategies or ['astar', 'eager'], **kwargs)

    def test_both_separate_and_byte_preserving(self):
        self.assertEqual(len(self.export()), 4)
        for label in ('astar', 'eager'):
            dest = self.out / f'camara_map_pddl_experiment_{label}.csv'
            self.assertEqual(dest.read_bytes(), self.sources[label].read_bytes())
        self.assertFalse((self.out / 'camara_map_pddl_experiment.csv').exists())
        manifest = json.loads((self.out / exporter.MANIFEST_NAME).read_text())
        self.assertFalse(manifest['pooled_strategies'])
        self.assertEqual([x['rows'] for x in manifest['datasets']], [4, 4, 4])
        self.assertEqual(manifest['datasets'][0]['search'], 'astar(blind())')

    def test_single_strategy_legacy_filename(self):
        self.export(['eager'])
        self.assertEqual((self.out / 'camara_map_pddl_experiment.csv').read_bytes(),
                         self.sources['eager'].read_bytes())
        self.assertTrue((self.out / 'camara_map_prism_experiment.csv').exists())

    def test_refuse_overwrite_before_writing(self):
        self.out.mkdir()
        existing = self.out / 'camara_map_prism_experiment.csv'
        existing.write_text('keep me')
        with self.assertRaises(FileExistsError):
            self.export()
        self.assertEqual(existing.read_text(), 'keep me')
        self.assertEqual(list(self.out.iterdir()), [existing])
        self.export(overwrite=True)
        self.assertEqual(existing.read_bytes(), self.sources['prism'].read_bytes())

    def test_coverage_and_repetition_mismatch(self):
        for pairs, repeats in [(('wp1_wp2',), 2), (('wp1_wp2', 'wp2_wp1'), 1)]:
            with self.subTest(pairs=pairs, repeats=repeats):
                self.sources['prism'].write_text(self.csv(False, pairs, repeats))
                with self.assertRaisesRegex(ValueError, 'do not match'):
                    self.export()
                self.assertFalse(self.out.exists())

    def test_invalid_metrics_and_missing_columns(self):
        original = self.csv()
        for bad in [original.replace('0.4', 'nan'), original.replace('0.4', '-1'),
                    original.replace(',4,3,', ',0,3,'), original.replace('distance', 'other'),
                    original.replace(',4,3,', ',4.5,3,')]:
            with self.subTest(bad=bad):
                self.sources['astar'].write_text(bad)
                with self.assertRaises(ValueError):
                    self.export()
                self.assertFalse(self.out.exists())

    def test_missing_or_ambiguous_strategy(self):
        search_file = self.suite / 'astar/search.txt'
        search_file.unlink()
        with self.assertRaisesRegex(ValueError, 'found 0'):
            self.export()
        search_file.write_text(exporter.STRATEGIES['astar'])
        duplicate = self.suite / 'duplicate'
        duplicate.mkdir()
        (duplicate / 'search.txt').write_text(exporter.STRATEGIES['astar'])
        with self.assertRaisesRegex(ValueError, 'found 2'):
            self.export()

    def test_missing_prism(self):
        self.sources['prism'].unlink()
        with self.assertRaises(FileNotFoundError):
            self.export()
        self.assertFalse(self.out.exists())


if __name__ == '__main__':
    unittest.main()
