#!/usr/bin/env python3
"""Extract BOM / net-label summary from a JLCEDA Pro .epro2 project file.

.epro2 is a ZIP; inside it, <project>.epru is a line-oriented stream:
    {"type": "...", ...}||{...json body...}
Documents are separated by DOCHEAD records (docType SCH / SCH_PAGE / PCB / DEVICE ...).

Usage:
    python parse_epro2.py <project.epro2> [out_csv]

The script is read-only: it unzips into a temp dir and never touches the project.
"""
import csv
import json
import sys
import zipfile
from collections import Counter


def read_docs_text(text):
    docs, cur = [], None
    for line in text.splitlines():
        line = line.strip()
        if not line or '||' not in line:
            continue
        head_s, body_s = line.split('||', 1)
        try:
            head = json.loads(head_s)
        except json.JSONDecodeError:
            continue
        body = None
        body_s = body_s.rstrip('|').strip()
        if body_s:
            try:
                body = json.loads(body_s)
            except json.JSONDecodeError:
                body = None
        if head.get('type') == 'DOCHEAD':
            cur = {'head': body, 'entries': []}
            docs.append(cur)
        elif cur is not None:
            cur['entries'].append((head.get('type'), head.get('id'), body))
    return docs


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    proj = sys.argv[1]
    out_csv = sys.argv[2] if len(sys.argv) > 2 else 'bom.csv'

    with zipfile.ZipFile(proj) as z:
        eprus = [n for n in z.namelist() if n.lower().endswith('.epru')]
        if not eprus:
            sys.exit('no .epru inside project')
        text = z.read(eprus[0]).decode('utf-8', 'replace')
    docs = read_docs_text(text)

    dev_meta = {}
    for d in docs:
        h = d['head'] or {}
        if h.get('docType') == 'DEVICE':
            for t, _, b in d['entries']:
                if t == 'META' and b:
                    dev_meta[h.get('uuid')] = b

    for d in docs:
        h = d['head'] or {}
        if h.get('docType') != 'SCH_PAGE':
            continue
        comps, attrs = {}, {}
        for t, i, b in d['entries']:
            if t == 'COMPONENT' and b:
                comps[i] = b
            elif t == 'ATTR' and b:
                attrs.setdefault(b.get('parentId'), {})[b.get('key')] = b.get('value')
        rows = []
        for cid, c in comps.items():
            a = attrs.get(cid, {})
            dm = dev_meta.get(a.get('Device', ''), {})
            da = dm.get('attributes', {})
            rows.append({
                'Designator': a.get('Designator', ''),
                'Name': a.get('Name', ''),
                'Value': a.get('Value', ''),
                'Device': da.get('Name') or dm.get('title', ''),
                'Footprint': da.get('Footprint', ''),
                'Manufacturer': da.get('Manufacturer', ''),
                'MPN': da.get('Manufacturer Part', ''),
                'Supplier': da.get('Supplier', ''),
                'SPN': da.get('Supplier Part', ''),
                'Description': (da.get('Description') or a.get('Description') or '')
                .replace('\n', ' ')[:80],
            })
        rows.sort(key=lambda r: (r['Designator'] == '', r['Designator']))
        with open(out_csv, 'w', newline='', encoding='utf-8-sig') as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)
        print('wrote %d rows -> %s' % (len(rows), out_csv))

        nets = Counter()
        for t, i, b in d['entries']:
            if t == 'ATTR' and b and b.get('key') == 'NET':
                nets[b.get('value')] += 1
        print('net labels:', ', '.join('%s(%d)' % (k, v) for k, v in nets.most_common()))
        return
    sys.exit('no SCH_PAGE found')


if __name__ == '__main__':
    main()
