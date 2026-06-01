# Smart Grid Control Notes

This repository stores public reading notes and writing resources for smart grid control research.

## Folder Layout

- `00_private_unpublished/`: unpublished manuscripts, review notes, and collaboration materials. This folder is local-only and is excluded from Git.
- `literature/core_ieee_paper/`: reading notes for a published IEEE paper, with downloaded paper PDFs excluded from Git.
- `templates/latex/`: LaTeX templates collected for future writing.
- `tools/`: small research-automation scripts.

## arXiv Watch

Run the lightweight arXiv watcher to collect recent papers related to smart-grid control, optimization, energy storage, and distributed algorithms:

```powershell
python tools/arxiv_watch.py
```

By default, the script scans recent papers from `eess.SY`, `math.OC`, and `eess.SP`, first requires a power-system topical match, then ranks the remaining papers with control, optimization, and algorithm keywords. It writes a Markdown digest to `paper_watch/arxiv-digest.md`.

Useful options:

```powershell
python tools/arxiv_watch.py --days 30 --min-score 7 --max-results 200
python tools/arxiv_watch.py --power-keywords "power system" microgrid "energy storage" "wind farm"
python tools/arxiv_watch.py --method-keywords "distributed control" ADMM KKT Lyapunov
python tools/arxiv_watch.py --include-general-methods --keywords "stochastic approximation"
python tools/arxiv_watch.py --categories eess.SY math.OC eess.SP cs.LG stat.ML --per-category
python tools/arxiv_watch.py --output paper_watch/weekly-digest.md
```

## Git Safety

The `.gitignore` is configured to exclude unpublished materials, downloaded paper PDFs, and generated note exports by default. Public commits should contain source notes such as Markdown files rather than private drafts or generated PDFs.
