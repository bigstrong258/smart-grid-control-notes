#!/usr/bin/env python3
"""Fetch and rank recent arXiv papers for smart-grid control research."""

from __future__ import annotations

import argparse
import datetime as dt
import html
import textwrap
import time
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


API_URL = "https://export.arxiv.org/api/query"
ATOM_NS = "{http://www.w3.org/2005/Atom}"
ARXIV_NS = "{http://arxiv.org/schemas/atom}"

DEFAULT_CATEGORIES = (
    "eess.SY",
    "math.OC",
    "cs.LG",
    "stat.ML",
    "eess.SP",
)

DEFAULT_KEYWORDS = (
    "power system",
    "smart grid",
    "microgrid",
    "distributed control",
    "secondary control",
    "optimal power flow",
    "OPF",
    "frequency regulation",
    "voltage regulation",
    "wind farm",
    "energy storage",
    "battery storage",
    "DFIG",
    "renewable energy",
    "communication topology",
    "graph theory",
    "consensus",
    "ADMM",
    "KKT",
    "Lyapunov",
    "primal-dual",
    "finite-time",
)

DEFAULT_EXCLUDE = (
    "quantum",
    "medical",
    "image segmentation",
    "large language model",
)


@dataclass(frozen=True)
class Paper:
    title: str
    authors: tuple[str, ...]
    abstract: str
    published: str
    updated: str
    arxiv_id: str
    pdf_url: str
    categories: tuple[str, ...]


@dataclass(frozen=True)
class RankedPaper:
    paper: Paper
    score: int
    hits: tuple[str, ...]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Fetch recent arXiv papers and rank them with keyword rules for "
            "smart-grid control, optimization, and energy-storage topics."
        )
    )
    parser.add_argument(
        "--categories",
        nargs="+",
        default=list(DEFAULT_CATEGORIES),
        help="arXiv categories to scan. Default: %(default)s",
    )
    parser.add_argument(
        "--keywords",
        nargs="+",
        default=list(DEFAULT_KEYWORDS),
        help="Keywords or phrases used for local ranking.",
    )
    parser.add_argument(
        "--exclude",
        nargs="+",
        default=list(DEFAULT_EXCLUDE),
        help="Keywords or phrases that subtract from the score.",
    )
    parser.add_argument(
        "--max-results",
        type=int,
        default=120,
        help="Maximum arXiv records to fetch before local filtering.",
    )
    parser.add_argument(
        "--min-score",
        type=int,
        default=2,
        help="Only include papers with score at least this value.",
    )
    parser.add_argument(
        "--days",
        type=int,
        default=14,
        help="Only include papers published or updated within this many days.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("paper_watch/arxiv-digest.md"),
        help="Markdown output path.",
    )
    parser.add_argument(
        "--user-agent",
        default="smart-grid-control-notes/0.1 (personal research alert)",
        help="User-Agent sent to arXiv API.",
    )
    parser.add_argument(
        "--sleep",
        type=float,
        default=3.0,
        help="Delay before the request, useful when scripting repeated runs.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=60.0,
        help="HTTP timeout in seconds for the arXiv API request.",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=2,
        help="Number of retries after a failed arXiv API request.",
    )
    return parser.parse_args()


def build_query(categories: list[str]) -> str:
    if not categories:
        raise ValueError("At least one arXiv category is required.")
    return " OR ".join(f"cat:{category}" for category in categories)


def fetch_feed(
    query: str,
    max_results: int,
    user_agent: str,
    sleep_seconds: float,
    timeout: float,
    retries: int,
) -> bytes:
    if max_results < 1:
        raise ValueError("--max-results must be positive.")

    params = {
        "search_query": query,
        "start": "0",
        "max_results": str(max_results),
        "sortBy": "lastUpdatedDate",
        "sortOrder": "descending",
    }
    url = f"{API_URL}?{urllib.parse.urlencode(params)}"
    request = urllib.request.Request(url, headers={"User-Agent": user_agent})

    attempts = max(1, retries + 1)
    for attempt in range(1, attempts + 1):
        if sleep_seconds > 0:
            time.sleep(sleep_seconds)
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                return response.read()
        except (TimeoutError, urllib.error.URLError) as error:
            if attempt >= attempts:
                raise RuntimeError(
                    f"arXiv API request failed after {attempts} attempt(s): {error}"
                ) from error
            wait_seconds = max(sleep_seconds, 3.0) * attempt
            print(
                f"arXiv API request failed on attempt {attempt}/{attempts}: {error}. "
                f"Retrying in {wait_seconds:.0f}s..."
            )
            time.sleep(wait_seconds)

    raise RuntimeError("Unexpected arXiv API retry state.")


def node_text(node: ET.Element | None) -> str:
    if node is None or node.text is None:
        return ""
    return " ".join(html.unescape(node.text).split())


def parse_feed(feed_xml: bytes) -> list[Paper]:
    root = ET.fromstring(feed_xml)
    papers: list[Paper] = []

    for entry in root.findall(f"{ATOM_NS}entry"):
        title = node_text(entry.find(f"{ATOM_NS}title"))
        abstract = node_text(entry.find(f"{ATOM_NS}summary"))
        published = node_text(entry.find(f"{ATOM_NS}published"))
        updated = node_text(entry.find(f"{ATOM_NS}updated"))
        authors = tuple(
            node_text(author.find(f"{ATOM_NS}name"))
            for author in entry.findall(f"{ATOM_NS}author")
        )
        categories = tuple(
            category.attrib.get("term", "")
            for category in entry.findall(f"{ATOM_NS}category")
            if category.attrib.get("term")
        )

        entry_id = node_text(entry.find(f"{ATOM_NS}id"))
        arxiv_id = entry_id.rsplit("/", maxsplit=1)[-1] if entry_id else ""
        pdf_url = ""
        for link in entry.findall(f"{ATOM_NS}link"):
            if link.attrib.get("title") == "pdf":
                pdf_url = link.attrib.get("href", "")
                break
        if not pdf_url and arxiv_id:
            pdf_url = f"https://arxiv.org/pdf/{arxiv_id}"

        papers.append(
            Paper(
                title=title,
                authors=authors,
                abstract=abstract,
                published=published,
                updated=updated,
                arxiv_id=arxiv_id,
                pdf_url=pdf_url,
                categories=categories,
            )
        )
    return papers


def parse_arxiv_date(value: str) -> dt.datetime | None:
    if not value:
        return None
    try:
        return dt.datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None


def is_recent(paper: Paper, days: int, now: dt.datetime) -> bool:
    if days <= 0:
        return True
    threshold = now - dt.timedelta(days=days)
    candidates = [
        parsed
        for parsed in (parse_arxiv_date(paper.published), parse_arxiv_date(paper.updated))
        if parsed is not None
    ]
    return bool(candidates) and max(candidates) >= threshold


def rank_paper(paper: Paper, keywords: list[str], exclude: list[str]) -> RankedPaper:
    searchable = f"{paper.title}\n{paper.abstract}\n{' '.join(paper.categories)}".lower()
    hits: list[str] = []
    score = 0

    for keyword in keywords:
        normalized = keyword.lower()
        if normalized in searchable:
            hits.append(keyword)
            score += 2 if " " in normalized or keyword.isupper() else 1

    for keyword in exclude:
        if keyword.lower() in searchable:
            score -= 2

    return RankedPaper(paper=paper, score=score, hits=tuple(hits))


def markdown_escape(text: str) -> str:
    return text.replace("|", "\\|")


def format_date(value: str) -> str:
    parsed = parse_arxiv_date(value)
    if parsed is None:
        return value or "unknown"
    return parsed.date().isoformat()


def render_markdown(
    ranked: list[RankedPaper],
    query: str,
    categories: list[str],
    keywords: list[str],
    min_score: int,
    days: int,
    now: dt.datetime,
) -> str:
    lines = [
        "# arXiv Smart Grid Control Watch",
        "",
        f"- Generated: {now.astimezone(dt.timezone.utc).strftime('%Y-%m-%d %H:%M UTC')}",
        f"- Window: last {days} days" if days > 0 else "- Window: all fetched records",
        f"- Categories: {', '.join(categories)}",
        f"- Minimum score: {min_score}",
        f"- Query: `{query}`",
        f"- Keywords: {', '.join(keywords)}",
        "",
    ]

    if not ranked:
        lines.extend(
            [
                "No papers matched the current filters.",
                "",
                "Try lowering `--min-score`, increasing `--days`, or adding broader keywords.",
                "",
            ]
        )
        return "\n".join(lines)

    lines.extend(
        [
            "| Score | Date | Title | Hits |",
            "| ---: | --- | --- | --- |",
        ]
    )
    for item in ranked:
        paper = item.paper
        title = markdown_escape(paper.title)
        link = f"https://arxiv.org/abs/{paper.arxiv_id}" if paper.arxiv_id else paper.pdf_url
        hits = markdown_escape(", ".join(item.hits))
        lines.append(
            f"| {item.score} | {format_date(paper.published)} | [{title}]({link}) | {hits} |"
        )

    lines.append("")
    lines.append("## Details")
    lines.append("")
    for index, item in enumerate(ranked, start=1):
        paper = item.paper
        authors = ", ".join(paper.authors[:6])
        if len(paper.authors) > 6:
            authors += ", et al."
        abstract = textwrap.fill(paper.abstract, width=100)

        lines.extend(
            [
                f"### {index}. {paper.title}",
                "",
                f"- Score: {item.score}",
                f"- Published: {format_date(paper.published)}",
                f"- Updated: {format_date(paper.updated)}",
                f"- Authors: {authors or 'unknown'}",
                f"- Categories: {', '.join(paper.categories) or 'unknown'}",
                f"- Hits: {', '.join(item.hits) or 'none'}",
                f"- Abstract: {abstract}",
                f"- Links: [abstract](https://arxiv.org/abs/{paper.arxiv_id}) | [pdf]({paper.pdf_url})",
                "",
            ]
        )

    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    now = dt.datetime.now(dt.timezone.utc)
    query = build_query(args.categories)
    feed_xml = fetch_feed(
        query=query,
        max_results=args.max_results,
        user_agent=args.user_agent,
        sleep_seconds=args.sleep,
        timeout=args.timeout,
        retries=args.retries,
    )
    papers = parse_feed(feed_xml)

    ranked = [
        rank_paper(paper, args.keywords, args.exclude)
        for paper in papers
        if is_recent(paper, args.days, now)
    ]
    ranked = [
        item
        for item in ranked
        if item.score >= args.min_score and item.hits
    ]
    ranked.sort(
        key=lambda item: (
            item.score,
            parse_arxiv_date(item.paper.updated) or dt.datetime.min.replace(tzinfo=dt.timezone.utc),
        ),
        reverse=True,
    )

    output = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        render_markdown(
            ranked=ranked,
            query=query,
            categories=args.categories,
            keywords=args.keywords,
            min_score=args.min_score,
            days=args.days,
            now=now,
        ),
        encoding="utf-8",
    )

    print(f"Wrote {len(ranked)} matched papers to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
