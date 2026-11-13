"""行為樹 YAML → py_trees。

格式（完整例子見 trees/gate_pass.yaml）：

    name: gate_pass
    description: 一句話
    vars:                       # 可在指令列用 -p depth=0.5 覆寫；節點參數寫 $depth 引用
      depth: 0.30
    root:
      type: Sequence
      children:
        - {type: Dive, depth: $depth, timeout: 40}

每個節點是一個 mapping，type 必填：
- 組合：Sequence、Selector（memory 預設 true）、Parallel（policy: all | one）→ children
- 裝飾：Timeout（seconds）、Retry（times）、Inverter、ForceSuccess → child
- 任務層節點：WaitReady、Confirm、Zero、Arm、Finish、Seen（bt/behaviours.py）
- 執行器：Hold、Dive、TurnTo、Cruise、AlignTo、ApproachGate（tasks/）
任何節點都可以加：
- name：顯示的名字（預設是 type）
- timeout: 秒 → 外面包一層 Timeout；retry: 次數 → 再外面包一層 Retry（每次嘗試各自計時）
其餘的 key 是該節點的參數；寫錯名字、缺必填參數，載入時就報錯（附上在樹裡的位置）。
"""
import copy
import dataclasses
import os

import py_trees
import yaml

from ..tasks import TASKS, params_of
from .behaviours import BEHAVIOURS, TaskAction, Timeout

COMPOSITES = ('Sequence', 'Selector', 'Parallel')
DECORATORS = {'Timeout': ('seconds',), 'Retry': ('times',), 'Inverter': (), 'ForceSuccess': ()}
COMMON = ('type', 'name', 'timeout', 'retry')


class TreeError(ValueError):
    pass


def kinds():
    """所有可用的 type → 種類。"""
    out = {t: 'composite' for t in COMPOSITES}
    out.update({t: 'decorator' for t in DECORATORS})
    out.update({t: 'behaviour' for t in BEHAVIOURS})
    out.update({t: 'task' for t in TASKS})
    return out


def leaf_params(kind):
    """{參數: 預設值}，必填的預設值是 dataclasses.MISSING。"""
    if kind in TASKS:
        return params_of(TASKS[kind])
    cls = BEHAVIOURS[kind]
    out = dict(cls.PARAMS)
    out.update({k: dataclasses.MISSING for k in cls.REQUIRED})
    return out


def numeric_params(kind):
    """要填數字的參數名稱（執行器看 Params 的型別標註，任務層節點看預設值）。"""
    if kind in TASKS:
        return {f.name for f in dataclasses.fields(TASKS[kind].Params) if f.type in (float, int, 'float', 'int')}
    return {k for k, v in BEHAVIOURS[kind].PARAMS.items()
            if isinstance(v, (int, float)) and not isinstance(v, bool)}


# ---------- 讀檔、代換變數、檢查 ----------

def read(path_or_spec, overrides=None):
    """讀 YAML（或已經是 dict），代換 $變數，檢查格式。回傳整理好的 spec（dict）。"""
    if isinstance(path_or_spec, dict):
        spec = copy.deepcopy(path_or_spec)
    else:
        with open(path_or_spec, encoding='utf-8') as f:
            spec = yaml.safe_load(f)
    if not isinstance(spec, dict) or 'root' not in spec:
        raise TreeError('最上層要有 root')
    variables = dict(spec.get('vars') or {})
    for k, v in (overrides or {}).items():
        if k not in variables:
            raise TreeError(f'-p {k}：樹裡沒有這個變數（可用：{", ".join(variables) or "無"}）')
        variables[k] = v
    spec['vars'] = variables
    spec['root'] = _check(_subst(spec['root'], variables, 'root'), 'root')
    return spec


def _subst(node, variables, path):
    if isinstance(node, dict):
        return {k: _subst(v, variables, f'{path}.{k}') for k, v in node.items()}
    if isinstance(node, list):
        return [_subst(v, variables, f'{path}[{i}]') for i, v in enumerate(node)]
    if isinstance(node, str) and node.startswith('$'):
        if node[1:] not in variables:
            raise TreeError(f'{path}：沒有變數 {node}')
        return variables[node[1:]]
    return node


def _check(node, path):
    if not isinstance(node, dict) or 'type' not in node:
        raise TreeError(f'{path}：每個節點要是有 type 的 mapping')
    kind = node['type']
    all_kinds = kinds()
    if kind not in all_kinds:
        raise TreeError(f'{path}：沒有 {kind} 這種節點（可用：{", ".join(all_kinds)}）')
    where = f'{path}（{kind}）'
    for k in ('timeout', 'retry'):
        if k in node and not (isinstance(node[k], (int, float)) and node[k] > 0):
            raise TreeError(f'{where}：{k} 要是正數')
    extra = {k: v for k, v in node.items() if k not in COMMON}

    if kind in COMPOSITES:
        allowed = {'children', 'memory'} if kind != 'Parallel' else {'children', 'policy'}
        _no_unknown(extra, allowed, where)
        children = extra.get('children')
        if not isinstance(children, list) or not children:
            raise TreeError(f'{where}：children 要是非空的列表')
        if kind == 'Parallel' and extra.get('policy', 'all') not in ('all', 'one'):
            raise TreeError(f'{where}：policy 只能是 all 或 one')
        node['children'] = [_check(c, f'{path}.children[{i}]') for i, c in enumerate(children)]
    elif kind in DECORATORS:
        _no_unknown(extra, {'child', *DECORATORS[kind]}, where)
        for k in DECORATORS[kind]:
            if not isinstance(extra.get(k), (int, float)) or extra[k] <= 0:
                raise TreeError(f'{where}：{k} 要是正數')
        if 'child' not in extra:
            raise TreeError(f'{where}：要有 child')
        node['child'] = _check(extra['child'], f'{path}.child')
    else:
        params = leaf_params(kind)
        _no_unknown(extra, set(params), where)
        missing = [k for k, v in params.items() if v is dataclasses.MISSING and k not in extra]
        if missing:
            raise TreeError(f'{where}：缺少參數 {", ".join(missing)}')
        numeric = numeric_params(kind)
        for k, v in extra.items():
            if k in numeric and not (isinstance(v, (int, float)) and not isinstance(v, bool)):
                raise TreeError(f'{where}：{k} 要是數字，拿到 {v!r}')
    return node


def _no_unknown(extra, allowed, where):
    unknown = set(extra) - set(allowed)
    if unknown:
        raise TreeError(f'{where}：不認得的參數 {", ".join(sorted(unknown))}'
                        f'（可用：{", ".join(sorted(allowed)) or "無"}）')


# ---------- 建樹 ----------

def build(spec, ctx):
    """spec（read() 的結果）→ py_trees 的根節點。ctx 可以是 None（只畫圖時）。"""
    return _build(spec['root'], ctx)


def _build(node, ctx):
    kind = node['type']
    name = str(node.get('name', kind))
    params = {k: v for k, v in node.items() if k not in COMMON + ('children', 'child')}
    if kind == 'Sequence':
        b = py_trees.composites.Sequence(name, memory=node.get('memory', True),
                                         children=[_build(c, ctx) for c in node['children']])
    elif kind == 'Selector':
        b = py_trees.composites.Selector(name, memory=node.get('memory', True),
                                         children=[_build(c, ctx) for c in node['children']])
    elif kind == 'Parallel':
        policy = (py_trees.common.ParallelPolicy.SuccessOnOne() if node.get('policy') == 'one'
                  else py_trees.common.ParallelPolicy.SuccessOnAll())
        b = py_trees.composites.Parallel(name, policy=policy,
                                         children=[_build(c, ctx) for c in node['children']])
    elif kind == 'Timeout':
        b = Timeout(name, _build(node['child'], ctx), float(node['seconds']), ctx)
    elif kind == 'Retry':
        b = py_trees.decorators.Retry(name, _build(node['child'], ctx), num_failures=int(node['times']))
    elif kind == 'Inverter':
        b = py_trees.decorators.Inverter(name, _build(node['child'], ctx))
    elif kind == 'ForceSuccess':
        b = py_trees.decorators.FailureIsSuccess(name, _build(node['child'], ctx))
    elif kind in TASKS:
        b = TaskAction(name, ctx, TASKS[kind], **params)
    else:
        b = BEHAVIOURS[kind](name, ctx, **params)
    # 每一次嘗試各自有 timeout：Retry(Timeout(節點))
    if 'timeout' in node:
        b = Timeout(f'逾時 {node["timeout"]} s', b, float(node['timeout']), ctx)
    if 'retry' in node:
        b = py_trees.decorators.Retry(f'最多失敗 {node["retry"]} 次', b, num_failures=int(node['retry']))
    return b


# ---------- 找樹檔 ----------

def find_tree(name):
    """名稱（gate_pass）或路徑 → YAML 檔的路徑。名稱依序找 $ORCA_TREES_DIR、套件安裝的 trees/。"""
    if os.path.sep in name or name.endswith(('.yaml', '.yml')):
        return name
    dirs = [os.environ.get('ORCA_TREES_DIR', '')]
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(os.path.join(get_package_share_directory('orca_mission'), 'trees'))
    except Exception:
        pass
    dirs.append(os.path.join(os.path.dirname(__file__), '..', '..', 'trees'))
    for d in dirs:
        p = os.path.join(d, name + '.yaml')
        if d and os.path.isfile(p):
            return p
    raise TreeError(f'找不到行為樹 {name}（找過：{", ".join(d for d in dirs if d)}）')
