#!/usr/bin/env python3
"""Fetch and rank recent arXiv papers for smart-grid control research."""

from __future__ import annotations

import argparse
import datetime as dt
import html
import re
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

DEFAULT_CATEGORIES = (
    "eess.SY",
    "math.OC",
    "eess.SP",
)

DEFAULT_POWER_KEYWORDS = (
    "power system",
    "power systems",
    "smart grid",
    "microgrid",
    "microgrids",
    "electric grid",
    "power grid",
    "distribution network",
    "transmission network",
    "optimal power flow",
    "power flow",
    "OPF",
    "frequency regulation",
    "voltage regulation",
    "wind farm",
    "wind farms",
    "energy storage",
    "battery storage",
    "DFIG",
    "renewable energy",
    "photovoltaic",
    "inverter",
    "converter",
    "converters",
    "HVDC",
    "fault ride through",
    "FRT",
    "grid code",
    "droop control",
    "load demand",
    "unit commitment",
    "economic dispatch",
    "distributed energy resource",
    "DER",
)

DEFAULT_METHOD_KEYWORDS = (
    "distributed control",
    "distributed optimization",
    "secondary control",
    "coordination control",
    "control strategy",
    "model predictive control",
    "MPC",
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
    "large language model",
    "reinforcement learning",
    "image segmentation",
)

DEFAULT_HARD_EXCLUDE = (
    "quantum",
    "medical",
    "postoperative",
    "pancreatic",
    "computed tomography",
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
    power_score: int
    method_score: int
    penalty: int
    hits: tuple[str, ...]
    power_hits: tuple[str, ...]
    method_hits: tuple[str, ...]


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
        "--power-keywords",
        nargs="+",
        default=list(DEFAULT_POWER_KEYWORDS),
        help="Power-system keywords required for topical filtering.",
    )
    parser.add_argument(
        "--method-keywords",
        nargs="+",
        default=list(DEFAULT_METHOD_KEYWORDS),
        help="Control, optimization, and algorithm keywords used for ranking.",
    )
    parser.add_argument(
        "--keywords",
        nargs="+",
        default=[],
        help="Extra custom keywords added to the ranking.",
    )
    parser.add_argument(
        "--exclude",
        nargs="+",
        default=list(DEFAULT_EXCLUDE),
        help="Keywords or phrases that subtract from the score.",
    )
    parser.add_argument(
        "--hard-exclude",
        nargs="+",
        default=list(DEFAULT_HARD_EXCLUDE),
        help="Keywords or phrases that remove a paper entirely.",
    )
    parser.add_argument(
        "--min-power-score",
        type=int,
        default=3,
        help="Required power-system topical score before method terms can pass.",
    )
    parser.add_argument(
        "--include-general-methods",
        action="store_true",
        help="Allow method-only papers without the power-system topical gate.",
    )
    parser.add_argument(
        "--max-results",
        type=int,
        default=120,
        help="Maximum arXiv records to fetch before local filtering.",
    )
    parser.add_argument(
        "--per-category",
        action="store_true",
        help="Fetch each category separately, then merge and deduplicate results.",
    )
    parser.add_argument(
        "--min-score",
        type=int,
        default=7,
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


def build_queries(categories: list[str], per_category: bool) -> list[str]:
    if per_category:
        return [f"cat:{category}" for category in categories]
    return [build_query(categories)]


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
            retry_after = 0.0
            if isinstance(error, urllib.error.HTTPError):
                try:
                    retry_after = float(error.headers.get("Retry-After", "0"))
                except ValueError:
                    retry_after = 0.0
            if isinstance(error, urllib.error.HTTPError) and error.code == 429:
                wait_seconds = max(retry_after, 60.0 * attempt)
            else:
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


def keyword_pattern(keyword: str) -> re.Pattern[str]:
    parts = [part for part in re.split(r"[\s-]+", keyword.strip()) if part]
    if len(parts) > 1:
        flexible_phrase = r"[\s-]+".join(re.escape(part) for part in parts)
    else:
        flexible_phrase = re.escape(keyword.strip())
    return re.compile(
        rf"(?<![A-Za-z0-9]){flexible_phrase}(?![A-Za-z0-9])",
        flags=re.IGNORECASE,
    )


def keyword_matches(searchable: str, keyword: str) -> bool:
    return bool(keyword.strip()) and bool(keyword_pattern(keyword).search(searchable))


def keyword_weight(keyword: str, group: str) -> int:
    strong_power_terms = {
        "power system",
        "power systems",
        "smart grid",
        "microgrid",
        "microgrids",
        "electric grid",
        "power grid",
        "optimal power flow",
        "frequency regulation",
        "voltage regulation",
        "wind farm",
        "wind farms",
        "energy storage",
        "battery storage",
        "HVDC",
        "fault ride through",
        "FRT",
        "renewable energy",
        "DFIG",
    }
    medium_power_terms = {
        "distribution network",
        "transmission network",
        "power flow",
        "droop control",
        "converter",
        "converters",
        "grid code",
        "unit commitment",
        "economic dispatch",
        "distributed energy resource",
        "DER",
    }
    strong_method_terms = {
        "distributed control",
        "distributed optimization",
        "coordination control",
        "control strategy",
        "model predictive control",
        "MPC",
        "communication topology",
        "ADMM",
        "primal-dual",
    }
    strong_power_terms = {term.lower() for term in strong_power_terms}
    medium_power_terms = {term.lower() for term in medium_power_terms}
    strong_method_terms = {term.lower() for term in strong_method_terms}

    normalized = keyword.strip().lower()
    if group == "power":
        if normalized in strong_power_terms:
            return 4
        if normalized in medium_power_terms:
            return 3
        return 2 if " " in normalized or keyword.isupper() else 1
    if group == "method":
        if normalized in strong_method_terms:
            return 3
        return 2 if " " in normalized or keyword.isupper() else 1
    return 2 if " " in normalized or keyword.isupper() else 1


def canonical_keyword(keyword: str) -> str:
    words = [word for word in re.split(r"[\s-]+", keyword.strip().lower()) if word]
    if not words:
        return ""
    if words[-1].endswith("s") and len(words[-1]) > 3:
        words[-1] = words[-1][:-1]
    return " ".join(words)


def score_keywords(
    searchable: str,
    keywords: list[str],
    group: str,
) -> tuple[int, tuple[str, ...]]:
    hits: list[str] = []
    seen_concepts: set[str] = set()
    score = 0
    for keyword in keywords:
        concept = canonical_keyword(keyword)
        if concept in seen_concepts:
            continue
        if keyword_matches(searchable, keyword):
            seen_concepts.add(concept)
            hits.append(keyword)
            score += keyword_weight(keyword, group)
    return score, tuple(hits)


def rank_paper(
    paper: Paper,
    power_keywords: list[str],
    method_keywords: list[str],
    extra_keywords: list[str],
    exclude: list[str],
    hard_exclude: list[str],
    min_power_score: int,
    include_general_methods: bool,
) -> RankedPaper | None:
    searchable = f"{paper.title}\n{paper.abstract}\n{' '.join(paper.categories)}"

    if any(keyword_matches(searchable, keyword) for keyword in hard_exclude):
        return None

    power_score, power_hits = score_keywords(searchable, power_keywords, "power")
    method_score, method_hits = score_keywords(searchable, method_keywords, "method")
    extra_score, extra_hits = score_keywords(searchable, extra_keywords, "extra")

    if not include_general_methods and power_score < min_power_score:
        return None

    penalty = sum(2 for keyword in exclude if keyword_matches(searchable, keyword))
    score = power_score + method_score + extra_score - penalty
    hits = power_hits + method_hits + extra_hits

    return RankedPaper(
        paper=paper,
        score=score,
        power_score=power_score,
        method_score=method_score + extra_score,
        penalty=penalty,
        hits=hits,
        power_hits=power_hits,
        method_hits=method_hits + extra_hits,
    )


def markdown_escape(text: str) -> str:
    return text.replace("|", "\\|")


def format_date(value: str) -> str:
    parsed = parse_arxiv_date(value)
    if parsed is None:
        return value or "unknown"
    return parsed.date().isoformat()


def render_markdown(
    ranked: list[RankedPaper],
    queries: list[str],
    categories: list[str],
    power_keywords: list[str],
    method_keywords: list[str],
    extra_keywords: list[str],
    min_score: int,
    min_power_score: int,
    include_general_methods: bool,
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
        f"- Minimum power-topic score: {min_power_score}",
        f"- Method-only papers allowed: {'yes' if include_general_methods else 'no'}",
        f"- Queries: {'; '.join(f'`{query}`' for query in queries)}",
        f"- Power keywords: {', '.join(power_keywords)}",
        f"- Method keywords: {', '.join(method_keywords)}",
        f"- Extra keywords: {', '.join(extra_keywords) if extra_keywords else 'none'}",
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
            "| Score | Power | Method | Date | Title | Hits |",
            "| ---: | ---: | ---: | --- | --- | --- |",
        ]
    )
    for item in ranked:
        paper = item.paper
        title = markdown_escape(paper.title)
        link = f"https://arxiv.org/abs/{paper.arxiv_id}" if paper.arxiv_id else paper.pdf_url
        hits = markdown_escape(", ".join(item.hits))
        lines.append(
            f"| {item.score} | {item.power_score} | {item.method_score} | "
            f"{format_date(paper.published)} | [{title}]({link}) | {hits} |"
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
                f"- Power score: {item.power_score}",
                f"- Method score: {item.method_score}",
                f"- Penalty: {item.penalty}",
                f"- Published: {format_date(paper.published)}",
                f"- Updated: {format_date(paper.updated)}",
                f"- Authors: {authors or 'unknown'}",
                f"- Categories: {', '.join(paper.categories) or 'unknown'}",
                f"- Power hits: {', '.join(item.power_hits) or 'none'}",
                f"- Method hits: {', '.join(item.method_hits) or 'none'}",
                f"- Abstract: {abstract}",
                f"- Links: [abstract](https://arxiv.org/abs/{paper.arxiv_id}) | [pdf]({paper.pdf_url})",
                "",
            ]
        )

    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    now = dt.datetime.now(dt.timezone.utc)
    queries = build_queries(args.categories, args.per_category)
    papers_by_id: dict[str, Paper] = {}
    for query in queries:
        feed_xml = fetch_feed(
            query=query,
            max_results=args.max_results,
            user_agent=args.user_agent,
            sleep_seconds=args.sleep,
            timeout=args.timeout,
            retries=args.retries,
        )
        for paper in parse_feed(feed_xml):
            key = paper.arxiv_id or paper.pdf_url or paper.title
            papers_by_id[key] = paper
    papers = list(papers_by_id.values())

    ranked = []
    for paper in papers:
        if not is_recent(paper, args.days, now):
            continue
        item = rank_paper(
            paper=paper,
            power_keywords=args.power_keywords,
            method_keywords=args.method_keywords,
            extra_keywords=args.keywords,
            exclude=args.exclude,
            hard_exclude=args.hard_exclude,
            min_power_score=args.min_power_score,
            include_general_methods=args.include_general_methods,
        )
        if item is not None:
            ranked.append(item)

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
            queries=queries,
            categories=args.categories,
            power_keywords=args.power_keywords,
            method_keywords=args.method_keywords,
            extra_keywords=args.keywords,
            min_score=args.min_score,
            min_power_score=args.min_power_score,
            include_general_methods=args.include_general_methods,
            days=args.days,
            now=now,
        ),
        encoding="utf-8",
    )

    print(f"Wrote {len(ranked)} matched papers to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
