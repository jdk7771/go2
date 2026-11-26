"""Pointwise overhead decisions and upward candidates on observed functions."""


def classify_stack(options,minimum_passage_m,step_limit_m,raw_support_m=.03):
    n=len(options)
    if not n:return []
    if n==1:
        option=options[0]
        option.update(overhead_passable=1,overhead_clearance_m=float('inf'),
            raw_supported=int(option.get('raw_distance_m',0.)<=raw_support_m),
            ceiling_owner=None,ceiling_z_m=float('inf'),layer_index=0,upward_candidates=[])
        return options
    ordered=sorted(options,key=lambda o:o['point'][2])
    zs=[float(o['point'][2]) for o in ordered]
    for k,option in enumerate(ordered):
        zk=zs[k]
        higher=[p for p,zp in zip(ordered[k+1:],zs[k+1:]) if zp-zk>.02]
        ceiling=float(higher[0]['point'][2]) if higher else float('inf')
        gap=ceiling-zk
        option.update(overhead_passable=int(gap+1e-7>=minimum_passage_m),overhead_clearance_m=gap,
            raw_supported=int(option.get('raw_distance_m',0.)<=raw_support_m),
            ceiling_owner=higher[0]['owner'] if higher else None,ceiling_z_m=ceiling,
            layer_index=k,upward_candidates=[dict(owner=p['owner'],height_m=float(p['point'][2]),
                delta_z_m=float(p['point'][2]-zk),
                capability_passed=int(p['point'][2]-zk<=step_limit_m+1e-6)) for p in higher])
    lookup={p['owner']:p for p in ordered}
    for option in ordered:
        for candidate in option['upward_candidates']:
            target=lookup[candidate['owner']]
            candidate['overhead_passable']=target['overhead_passable']
            candidate['raw_supported']=target['raw_supported']
            candidate['switchable']=int(candidate['capability_passed'] and candidate['overhead_passable'] and candidate['raw_supported'])
    return ordered


def describe_stack(xy,options):
    """An upward chain at this XY; A* separately exports its branching tree."""
    ordered=sorted(options,key=lambda o:o['point'][2]);layers=[]
    for k,p in enumerate(ordered):
        layers.append(dict(owner=p['owner'],z_m=float(p['point'][2]),parent_owner=ordered[k-1]['owner'] if k else None,
            children=[ordered[k+1]['owner']] if k+1<len(ordered) else [],
            overhead_passable=p['overhead_passable'],overhead_clearance_m=p['overhead_clearance_m'],
            raw_supported=p['raw_supported'],
            upward_candidates=p['upward_candidates']))
    return dict(xy_m=list(map(float,xy)),layers=layers)
