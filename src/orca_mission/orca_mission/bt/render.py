"""行為樹 YAML → 流程圖。

    ros2 run orca_mission render_tree gate_pass                 # 寫 trees/gate_pass.md（Mermaid）
    ros2 run orca_mission render_tree gate_pass --svg out.svg   # 另外用 graphviz 畫 SVG
    ros2 run orca_mission render_tree --check                   # 每棵樹的 .md 是否跟 YAML 一致（CI 用）

Mermaid 在 GitHub、VS Code 的 Markdown 預覽都能直接顯示，所以每棵樹旁邊放一份產生出來的 .md。
執行中的即時狀態另外看：py-trees-tree-watcher（見 README）。
"""
import argparse
import dataclasses
import glob
import math
import os
import sys

from .loader import COMMON, DECORATORS, TreeError, build, find_tree, kinds, leaf_params, read

SHAPES = {
    'Sequence': ('["→ {}"]', 'seq'),
    'Selector': ('["? {}"]', 'sel'),
    'Parallel': ('["⇉ {}"]', 'par'),
}


def _fmt(v):
    if isinstance(v, float):
        return 'NaN' if math.isnan(v) else f'{v:g}'
    if isinstance(v, (list, tuple)):
        return '/'.join(_fmt(x) for x in v)
    return str(v)


def _esc(text):
    return text.replace('"', '#quot;')


def _label(node):
    kind = node['type']
    title = str(node.get('name', kind))
    if title != kind:
        title = f'{title}（{kind}）'
    defaults = leaf_params(kind)
    lines = [f'<b>{_esc(title)}</b>']
    for k, v in node.items():
        if k in COMMON or (k in defaults and defaults[k] is not dataclasses.MISSING and defaults[k] == v):
            continue
        if kind == 'Confirm' and k == 'checklist':
            lines.append(f'檢查清單 {len(v)} 項')
            continue
        lines.append(_esc(f'{k} = {_fmt(v)}'))
    return '<br/>'.join(lines)


def mermaid(spec):
    out = ['flowchart TD']
    counter = [0]
    all_kinds = kinds()

    def add(node, parent, edge_label=''):
        def emit(text, cls):
            nid = f'n{counter[0]}'
            counter[0] += 1
            out.append(f'  {nid}{text}:::{cls}')
            if parent is not None:
                out.append(f'  {parent} -->{edge_label} {nid}' if not edge_label
                           else f'  {parent} -->|{edge_label}| {nid}')
            return nid

        # 簡寫的 timeout、retry 畫成外層的裝飾節點
        if 'retry' in node:
            parent = emit(f'{{{{"↻ 最多失敗 {node["retry"]} 次"}}}}', 'dec')
            edge_label = ''
        if 'timeout' in node:
            parent = emit(f'{{{{"⏱ {_fmt(node["timeout"])} s"}}}}', 'dec')
            edge_label = ''
        kind = node['type']
        name = str(node.get('name', kind))
        if kind in SHAPES:
            shape, cls = SHAPES[kind]
            extra = '' if kind == 'Parallel' else ('' if node.get('memory', True) else '（反應式）')
            if kind == 'Parallel':
                extra = '（全部成功）' if node.get('policy', 'all') == 'all' else '（一個成功）'
            nid = emit(shape.format(_esc(name + extra)), cls)
            for i, c in enumerate(node['children']):
                add(c, nid, str(i + 1))
        elif kind in DECORATORS:
            args = ' '.join(f'{_fmt(node[k])}' for k in DECORATORS[kind])
            text = {'Timeout': f'⏱ {args} s', 'Retry': f'↻ 最多失敗 {args} 次',
                    'Inverter': '¬ 反轉', 'ForceSuccess': '✓ 失敗也算成功'}[kind]
            nid = emit(f'{{{{"{_esc(text)}"}}}}', 'dec')
            add(node['child'], nid)
        elif all_kinds[kind] == 'task':
            emit(f'(["{_label(node)}"])', 'task')
        elif kind == 'Seen':
            emit(f'{{"{_label(node)}"}}', 'cond')
        else:
            emit(f'["{_label(node)}"]', 'mission')

    add(spec['root'], None)
    out += [
        '  classDef seq fill:#dbeafe,stroke:#1d4ed8,color:#0f172a',
        '  classDef sel fill:#fef3c7,stroke:#b45309,color:#0f172a',
        '  classDef par fill:#ede9fe,stroke:#6d28d9,color:#0f172a',
        '  classDef dec fill:#f1f5f9,stroke:#64748b,color:#0f172a',
        '  classDef task fill:#dcfce7,stroke:#15803d,color:#0f172a',
        '  classDef mission fill:#fee2e2,stroke:#b91c1c,color:#0f172a',
        '  classDef cond fill:#fff,stroke:#0f172a,color:#0f172a',
    ]
    return '\n'.join(out)


LEGEND = """\
圖例：→ Sequence（依序，一個失敗就失敗）、? Selector（依序嘗試，一個成功就成功）、
⇉ Parallel、⏱ 逾時、↻ 重試；綠色圓角 = 執行器（tasks/，經 TaskRunner 控制推進器）、
紅色 = 任務層節點（就緒檢查、確認、歸零、解鎖、上鎖）、菱形 = 條件。
邊上的數字是執行順序。樹結束（成功或失敗）時一律 MANUAL、全 0、DISARM。"""


def markdown(spec, source):
    title = spec.get('name', os.path.splitext(os.path.basename(source))[0])
    desc = spec.get('description', '')
    vars_ = spec.get('vars') or {}
    lines = [f'# {title}', '',
             f'<!-- 由 render_tree 從 {os.path.basename(source)} 產生，不要手改 -->', '']
    if desc:
        lines += [desc.strip(), '']
    if vars_:
        lines += ['| 變數（-p 覆寫） | 預設 |', '|---|---|']
        lines += [f'| `{k}` | `{_fmt(v)}` |' for k, v in vars_.items()]
        lines.append('')
    lines += ['```mermaid', mermaid(spec), '```', '', LEGEND, '']
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description='行為樹 YAML → 流程圖')
    ap.add_argument('trees', nargs='*', help='樹的名稱或 YAML 路徑；不給 = trees/ 底下全部')
    ap.add_argument('--svg', help='另外用 graphviz 畫 SVG 到這個路徑（只能給一棵樹）')
    ap.add_argument('--check', action='store_true', help='只檢查 .md 是否跟 YAML 一致，不寫檔')
    ap.add_argument('--stdout', action='store_true', help='印到螢幕，不寫檔')
    args = ap.parse_args(argv)

    paths = [find_tree(t) for t in args.trees]
    if not paths:
        here = os.environ.get('ORCA_TREES_DIR') or os.path.join(os.path.dirname(__file__), '..', '..', 'trees')
        paths = sorted(glob.glob(os.path.join(here, '*.yaml')))
    stale = []
    for path in paths:
        try:
            spec = read(path)
        except TreeError as e:
            print(f'{path}: {e}', file=sys.stderr)
            return 1
        text = markdown(spec, path)
        md = os.path.splitext(path)[0] + '.md'
        if args.stdout:
            print(text)
        elif args.check:
            current = open(md, encoding='utf-8').read() if os.path.exists(md) else ''
            if current != text:
                stale.append(md)
        else:
            with open(md, 'w', encoding='utf-8') as f:
                f.write(text)
            print(f'寫入 {md}')
        if args.svg:
            import py_trees
            root = build(spec, None)
            py_trees.display.dot_tree(root).write_svg(args.svg)
            print(f'寫入 {args.svg}')
    if stale:
        print('這些圖跟 YAML 不一致，請跑 render_tree 重新產生：\n  ' + '\n  '.join(stale), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
