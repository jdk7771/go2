#!/usr/bin/env python3
"""Build a GE6001 revision from the supplied DOCX, using only Python's stdlib.

The tracked Word document contains native w:ins/w:del revisions. A separate
static markup document is used for PDF export so the PDF always shows changes.
"""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
from difflib import SequenceMatcher
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET
from zipfile import ZIP_DEFLATED, ZipFile


ROOT = Path(__file__).resolve().parent.parent
SOURCE_DEFAULT = Path('/home/jiang/Downloads/Homework-final.docx')
W = 'http://schemas.openxmlformats.org/wordprocessingml/2006/main'
NS = {'w': W}
XML_SPACE = '{http://www.w3.org/XML/1998/namespace}space'
DATE = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace('+00:00', 'Z')
ET.register_namespace('w', W)

REVISED = [
    '''Cloud computing provides applications, computing resources, and data storage over the Internet. Enterprises deploy software and data on platforms such as Amazon Web Services (AWS), Microsoft Azure, Google Cloud, and Alibaba Cloud to reduce infrastructure costs, scale resources, and support collaboration. Virtualization supports these services by abstracting physical server resources into virtual environments. Cloud networking connects the virtual nodes in these environments; connectivity failures can therefore interrupt application services. In large cloud deployments, the number of virtual nodes and the complexity of their communication relationships make faults difficult to localize. Tracing packet forwarding paths helps administrators determine where communication fails. Tenants can also use connection-status information to guide load-balancing and fault-tolerance decisions.''',
    '''In traditional networks, administrators diagnose connectivity problems by inspecting router configurations and using tools such as ping and traceroute. Repeating these checks across networks with millions of nodes is labor-intensive and difficult to scale. In cloud networks built with software-defined networking (SDN) and Virtual Extensible LAN (VXLAN) [4], the diagnostic task extends to communication between virtual nodes in the overlay. SDN provides a centralized controller with network-wide visibility and the ability to configure virtual switches and routers. The controller can therefore coordinate tracing across virtual forwarding devices.''',
    '''SDN research has addressed network operation [6, 7, 9, 35] as well as monitoring and debugging. Some diagnostic systems require substantial network instrumentation [3, 11, 12, 37], which can complicate deployment in large cloud networks. Packet-tracing methods include modifying virtual forwarding devices [13, 33], deploying decision engines on end hosts [8, 27], and mirroring packets [1, 15].

We present VTrace, a network debugging tool for tracing virtual flows in cloud overlay networks. VTrace uses the SDN controller to configure virtual forwarding devices for flow tracing. It then reconstructs forwarding paths from the collected tracing information to help locate connectivity failures.''',
    '''The contributions of this paper are as follows:

(1) We develop VTrace, a network traceback system for large-scale cloud overlay networks. The system uses forwarding-path information to help localize connectivity failures.

(2) We describe a controller-based mechanism that uses network-wide visibility to configure virtual forwarding devices for tracing arbitrary virtual flows. The mechanism is designed to limit tracing overhead and interference with production traffic.

(3) We implement a distributed stream-processing system and algorithms for reconstructing and analyzing virtual flows. The system supports concurrent tracing tasks.''',
]


def node(tag, parent=None, **attrs):
    out = ET.Element(f'{{{W}}}{tag}', {f'{{{W}}}{k}': str(v) for k, v in attrs.items()})
    if parent is not None:
        parent.append(out)
    return out


def run(parent, text, kind='equal', native=False, bold=False, size=None):
    r = node('r', parent)
    props = node('rPr', r)
    node('rFonts', props, ascii='Liberation Serif', hAnsi='Liberation Serif')
    if bold:
        node('b', props)
    if size:
        node('sz', props, val=size)
    if not native and kind != 'equal':
        node('color', props, val='B00020' if kind == 'delete' else '1453A0')
        node('strike' if kind == 'delete' else 'u', props, **({} if kind == 'delete' else {'val': 'single'}))
    t = node('delText' if native and kind == 'delete' else 't', r)
    t.set(XML_SPACE, 'preserve')
    t.text = text
    return r


def paragraph(parent, text=None, style='Normal', size=None, bold=False):
    p = node('p', parent)
    pp = node('pPr', p)
    node('pStyle', pp, val=style)
    if style == 'Normal':
        node('jc', pp, val='both')
    if text is not None:
        run(p, text, size=size, bold=bold)
    return p


def tokenise(text):
    # Keep citation groups intact so the redline cannot split a reference number.
    # Paragraph boundaries are tokens; other whitespace belongs to its word.
    return re.findall(r'\[[\d, ]+\][^\S\n]*|\n\n|[^\s]+[^\S\n]*', text)


def changes(old, new):
    a, b = tokenise(old), tokenise(new)
    out = []
    for tag, i, j, k, l in SequenceMatcher(None, a, b, autojunk=False).get_opcodes():
        if tag == 'equal':
            out.append(('equal', ''.join(a[i:j])))
        else:
            if tag in ('delete', 'replace'):
                out.append(('delete', ''.join(a[i:j])))
            if tag in ('insert', 'replace'):
                out.append(('insert', ''.join(b[k:l])))
    return out


def normalise(s):
    return ' '.join(s.split())


def get_original(path):
    with ZipFile(path) as z:
        root = ET.fromstring(z.read('word/document.xml'))
    pars = [''.join(p.itertext()) for p in root.findall('./w:body/w:p', NS)]
    original = [p for p in pars if p.startswith(('Cloud computing ', 'Unfortunately, ', 'And we ', 'The contributions '))]
    if len(original) != 4:
        raise ValueError(f'Expected four introduction paragraphs, found {len(original)}')
    return original


def create_docx(path, original, mode):
    doc = node('document')
    body = node('body', doc)
    paragraph(body, 'GE6001 | Academic Writing, Norms and Ethics', 'Title', size=30, bold=True)
    subtitle = 'Introduction — Clean Revision' if mode == 'clean' else 'Introduction — Tracked Revision'
    paragraph(body, subtitle, 'Subtitle', size=25, bold=True)
    if mode != 'clean':
        p = paragraph(body, style='Legend')
        run(p, 'Deleted text', kind='delete', size=20)
        run(p, '   |   ', size=20)
        run(p, 'Inserted text', kind='insert', size=20)
        paragraph(body, 'All reference numbers are retained from the supplied text.', 'Legend', size=19)
    else:
        paragraph(body, 'All reference numbers are retained from the supplied text.', 'Legend', size=19)
    revision_id = 1
    for old, new in zip(original, REVISED):
        if mode == 'clean':
            for part in new.split('\n\n'):
                paragraph(body, part)
            continue
        p = paragraph(body)
        for kind, value in changes(old, new):
            pieces = value.split('\n\n')
            for i, piece in enumerate(pieces):
                if i:
                    p = paragraph(body)
                    # Mark inserted paragraph breaks as revisions as well.
                    if mode == 'tracked' and kind == 'insert':
                        pp = p.find(f'{{{W}}}pPr')
                        rp = node('rPr', pp)
                        node('ins', rp, id=revision_id, author='Editor', date=DATE)
                        revision_id += 1
                if not piece:
                    continue
                target = p
                if mode == 'tracked' and kind != 'equal':
                    target = node('del' if kind == 'delete' else 'ins', p,
                                  id=revision_id, author='Editor', date=DATE)
                    revision_id += 1
                run(target, piece, kind, native=(mode == 'tracked'))
    sect = node('sectPr', body)
    node('pgSz', sect, w=11906, h=16838)
    node('pgMar', sect, top=1134, right=1190, bottom=1134, left=1190, header=567, footer=567, gutter=0)
    footerref = node('footerReference', sect, type='default')
    footerref.set('{http://schemas.openxmlformats.org/officeDocument/2006/relationships}id', 'rIdFooter')

    footer = node('ftr')
    p = paragraph(footer, style='Footer')
    pp = p.find(f'{{{W}}}pPr')
    node('jc', pp, val='center')
    run(p, 'GE6001  |  ')
    field = node('fldSimple', p, instr='PAGE')
    run(field, '1')

    styles = node('styles')
    defaults = node('docDefaults', styles)
    rp = node('rPr', node('rPrDefault', defaults))
    node('rFonts', rp, ascii='Liberation Serif', hAnsi='Liberation Serif')
    node('sz', rp, val=23)
    node('lang', rp, val='en-US')
    pp = node('pPr', node('pPrDefault', defaults))
    node('spacing', pp, after=150, line=290, lineRule='auto')
    node('widowControl', pp)
    for name in ('Normal', 'Title', 'Subtitle', 'Legend', 'Footer'):
        st = node('style', styles, type='paragraph', styleId=name)
        node('name', st, val=name)
        if name == 'Normal':
            st.set(f'{{{W}}}default', '1')
        else:
            node('basedOn', st, val='Normal')
        if name != 'Normal':
            pp = node('pPr', st)
            node('spacing', pp, after=110 if name in ('Title', 'Subtitle') else 130)
            if name in ('Title', 'Subtitle', 'Legend'):
                node('keepNext', pp)

    settings = node('settings')
    if mode == 'tracked':
        node('revisionView', settings, markup='true', insDel='true', formatting='true')
        node('trackRevisions', settings)
        node('doNotTrackFormatting', settings)
    node('defaultTabStop', settings, val=720)

    def serial(e):
        return ET.tostring(e, encoding='UTF-8', xml_declaration=True)

    package = {
        '[Content_Types].xml': b'''<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="xml" ContentType="application/xml"/><Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/><Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/><Override PartName="/word/settings.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.settings+xml"/><Override PartName="/word/footer1.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.footer+xml"/></Types>''',
        '_rels/.rels': b'''<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/></Relationships>''',
        'word/_rels/document.xml.rels': b'''<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Id="rIdStyles" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/><Relationship Id="rIdSettings" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/settings" Target="settings.xml"/><Relationship Id="rIdFooter" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/footer" Target="footer1.xml"/></Relationships>''',
        'word/document.xml': serial(doc),
        'word/styles.xml': serial(styles),
        'word/settings.xml': serial(settings),
        'word/footer1.xml': serial(footer),
    }
    with ZipFile(path, 'w', ZIP_DEFLATED) as z:
        for name, value in package.items():
            z.writestr(name, value)


def tex_escape(s):
    replacements = {'\\': r'\textbackslash{}', '&': r'\&', '%': r'\%', '$': r'\$',
                    '#': r'\#', '_': r'\_', '{': r'\{', '}': r'\}', '~': r'\textasciitilde{}',
                    '^': r'\textasciicircum{}', '—': '---', '–': '--', '’': "'"}
    return ''.join(replacements.get(c, c) for c in s)


def create_tex(original):
    marked = []
    for old, new in zip(original, REVISED):
        parts = []
        for kind, value in changes(old, new):
            chunks = value.split('\n\n')
            for i, chunk in enumerate(chunks):
                if i:
                    parts.append('\n\n')
                if chunk:
                    escaped = tex_escape(chunk)
                    parts.append(escaped if kind == 'equal' else '\\' + ('deleted' if kind == 'delete' else 'added') + '{' + escaped + '}')
        marked.append(''.join(parts))
    preamble = r'''% Compile twice with pdfLaTeX, XeLaTeX, or LuaLaTeX.
% Default: a marked revision followed by a clean reading copy.
% For only the marked revision: change \cleansectiontrue to \cleansectionfalse.
% For only the clean revision: change \markedsectiontrue to \markedsectionfalse.
% Citation numbers are copied verbatim; the source did not include a bibliography.
\documentclass[11pt,a4paper]{article}
\usepackage[T1]{fontenc}
\usepackage[utf8]{inputenc}
\usepackage{lmodern}
\usepackage[margin=23mm]{geometry}
\usepackage{xcolor}
\usepackage[normalem]{ulem}
\usepackage{microtype}
\usepackage{fancyhdr}
\definecolor{deletion}{HTML}{B00020}
\definecolor{insertion}{HTML}{1453A0}
\newcommand{\deleted}[1]{{\color{deletion}\sout{#1}}}
\newcommand{\added}[1]{{\color{insertion}\uline{#1}}}
\newif\ifmarkedsection
\newif\ifcleansection
\markedsectiontrue
\cleansectiontrue
\setlength{\parindent}{0pt}
\setlength{\parskip}{0.65\baselineskip}
\linespread{1.08}
\setlength{\emergencystretch}{3em}
\pagestyle{fancy}
\fancyhf{}
\fancyfoot[C]{\small GE6001\quad |\quad \thepage}
\renewcommand{\headrulewidth}{0pt}
\begin{document}
\ifmarkedsection
{\Large\bfseries GE6001 | Academic Writing, Norms and Ethics\par}
{\large\bfseries Introduction --- Tracked Revision\par}
{\small \deleted{Deleted text}\quad |\quad \added{Inserted text}\par}
{\small All reference numbers are retained from the supplied text.\par}

'''
    middle = r'''
\fi
\ifcleansection
\ifmarkedsection\clearpage\fi
{\Large\bfseries GE6001 | Academic Writing, Norms and Ethics\par}
{\large\bfseries Introduction --- Clean Revision\par}
{\small All reference numbers are retained from the supplied text.\par}

'''
    content = (preamble + '% BEGIN MARKED INTRODUCTION\n' + '\n\n'.join(marked)
               + '\n% END MARKED INTRODUCTION\n' + middle
               + '% BEGIN CLEAN INTRODUCTION\n' + '\n\n'.join(tex_escape(x) for x in REVISED)
               + '\n% END CLEAN INTRODUCTION\n\\fi\n\\end{document}\n')
    (ROOT / 'Homework-final.tex').write_text(content)


def validate_tex(original):
    """Check source integrity without claiming a TeX compilation."""
    content = (ROOT / 'Homework-final.tex').read_text()
    marked = content.split('% BEGIN MARKED INTRODUCTION\n', 1)[1].split('\n% END MARKED INTRODUCTION', 1)[0]
    clean = content.split('% BEGIN CLEAN INTRODUCTION\n', 1)[1].split('\n% END CLEAN INTRODUCTION', 1)[0]

    def resolve_markup(value, accept):
        out = []
        i = 0
        while i < len(value):
            match = re.match(r'\\(added|deleted)\{', value[i:])
            if not match:
                out.append(value[i])
                i += 1
                continue
            start = i + match.end()
            end = start
            depth = 1
            while end < len(value) and depth:
                if value[end] == '{' and (end == 0 or value[end - 1] != '\\'):
                    depth += 1
                elif value[end] == '}' and (end == 0 or value[end - 1] != '\\'):
                    depth -= 1
                end += 1
            assert depth == 0, 'Unclosed change-markup argument'
            if (match[1] == 'added') == accept:
                out.append(value[start:end - 1])
            i = end
        return ''.join(out)

    expected_old = normalise('\n\n'.join(tex_escape(x) for x in original))
    expected_new = normalise('\n\n'.join(tex_escape(x) for x in REVISED))
    assert normalise(resolve_markup(marked, False)) == expected_old, 'TeX redline does not reconstruct the original'
    assert normalise(resolve_markup(marked, True)) == expected_new, 'TeX redline does not reconstruct the revision'
    assert normalise(clean) == expected_new, 'TeX clean section differs from revision'

    # Detect missing boolean definitions and unbalanced source braces.
    code = re.sub(r'(?m)%.*$', '', content)
    definitions = set(re.findall(r'\\newif\\(if\w+)', code))
    without_definitions = re.sub(r'\\newif\\if\w+', '', code)
    used = set(re.findall(r'\\(ifmarkedsection|ifcleansection)\b', without_definitions))
    assert used <= definitions, f'Undefined conditional: {used - definitions}'
    depth = 0
    for token in re.findall(r'\\(?:ifmarkedsection|ifcleansection|fi)\b', without_definitions):
        depth += -1 if token == r'\fi' else 1
        assert depth >= 0, 'Unmatched \\fi'
    assert depth == 0, 'Unclosed conditional'
    depth = 0
    for match in re.finditer(r'(?<!\\)[{}]', code):
        depth += 1 if match[0] == '{' else -1
        assert depth >= 0, 'Unmatched closing brace'
    assert depth == 0, 'Unclosed brace'
    assert content.count(r'\begin{document}') == content.count(r'\end{document}') == 1
    assert Counter(re.findall(r'\[[\d, ]+\]', '\n'.join(original))) == Counter(re.findall(r'\[[\d, ]+\]', '\n'.join(REVISED)))
    report = {'original_reconstructs_from_tex': True, 'revision_reconstructs_from_tex': True,
              'clean_section_matches': True, 'conditionals_defined_and_balanced': True,
              'braces_balanced': True, 'citations_preserved': True, 'latex_compiled': False,
              'original_words': sum(len(x.split()) for x in original),
              'revised_words': sum(len(x.split()) for x in REVISED)}
    (ROOT / 'build/latex-validation.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


def validate(original):
    reports = []
    for i, (old, new) in enumerate(zip(original, REVISED), 1):
        diff = changes(old, new)
        rejected = ''.join(v for k, v in diff if k != 'insert')
        accepted = ''.join(v for k, v in diff if k != 'delete')
        assert normalise(rejected) == normalise(old), f'Original reconstruction failed: {i}'
        assert normalise(accepted) == normalise(new), f'Revision reconstruction failed: {i}'
        old_cites, new_cites = re.findall(r'\[[\d, ]+\]', old), re.findall(r'\[[\d, ]+\]', new)
        assert Counter(old_cites) == Counter(new_cites), f'Citations changed: {i}'
        reports.append({'paragraph': i, 'original_words': len(old.split()), 'revised_words': len(new.split()),
                        'reconstruct_original': True, 'reconstruct_revision': True, 'citations_preserved': True})
    with ZipFile(ROOT / 'Homework-final-tracked.docx') as z:
        doc = ET.fromstring(z.read('word/document.xml'))
        counts = {kind: len(doc.findall(f'.//w:{kind}', NS)) for kind in ('ins', 'del')}
        assert counts['ins'] and counts['del']
        def extract(accept):
            pieces = []
            for p in doc.findall('./w:body/w:p', NS)[4:]:
                def walk(e):
                    if e.tag == f'{{{W}}}' + ('del' if accept else 'ins'):
                        return ''
                    if e.tag in (f'{{{W}}}t', f'{{{W}}}delText'):
                        return e.text or ''
                    return ''.join(walk(c) for c in e)
                pieces.append(walk(p))
            return normalise(' '.join(pieces))
        assert extract(True) == normalise(' '.join(REVISED)), 'DOCX accept all did not reproduce revision'
        assert extract(False) == normalise(' '.join(original)), 'DOCX reject all did not reproduce original'
    summary = {'checks': reports, 'native_word_revisions': counts, 'docx_accept_all_correct': True,
               'docx_reject_all_correct': True}
    (ROOT / 'build/validation.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    print(json.dumps(summary, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('original_docx', type=Path, nargs='?', default=SOURCE_DEFAULT)
    parser.add_argument('--latex-only', action='store_true', help='Generate and check only the requested LaTeX deliverable')
    args = parser.parse_args()
    ROOT.mkdir(exist_ok=True)
    (ROOT / 'build').mkdir(exist_ok=True)
    original = get_original(args.original_docx)
    if not args.latex_only:
        create_docx(ROOT / 'Homework-final-tracked.docx', original, 'tracked')
        create_docx(ROOT / 'build/Homework-final-marked.docx', original, 'marked')
        create_docx(ROOT / 'Homework-final-clean.docx', original, 'clean')
    create_tex(original)
    (ROOT / 'source/revised_introduction.txt').write_text('\n\n'.join(REVISED) + '\n')
    (ROOT / 'build/revisions.json').write_text(json.dumps({'original': original, 'revised': REVISED}, ensure_ascii=False, indent=2))
    validate_tex(original)
    if not args.latex_only:
        validate(original)


if __name__ == '__main__':
    main()
