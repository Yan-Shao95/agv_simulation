import math
from .kinematics import ChassisTwist

def _solve3(a,b):
    m=[list(a[i])+[b[i]] for i in range(3)]
    for col in range(3):
        pivot=max(range(col,3),key=lambda r:abs(m[r][col]))
        if abs(m[pivot][col])<1e-10: raise ValueError('运动学矩阵奇异')
        m[col],m[pivot]=m[pivot],m[col]
        scale=m[col][col]; m[col]=[v/scale for v in m[col]]
        for r in range(3):
            if r!=col:
                scale=m[r][col]; m[r]=[m[r][c]-scale*m[col][c] for c in range(4)]
    return tuple(m[i][3] for i in range(3))

def estimate_twist(module_angles,module_speeds,module_positions):
    if not (len(module_angles)==len(module_speeds)==len(module_positions)==4): raise ValueError('需要四个总成状态')
    rows=[]; values=[]
    for angle,speed,(x,y) in zip(module_angles,module_speeds,module_positions):
        c,s=math.cos(angle),math.sin(angle)
        rows.extend(((1.0,0.0,-y),(0.0,1.0,x)))
        values.extend((speed*c,speed*s))
    ata=[[sum(row[i]*row[j] for row in rows) for j in range(3)] for i in range(3)]
    atb=[sum(row[i]*v for row,v in zip(rows,values)) for i in range(3)]
    return ChassisTwist(*_solve3(ata,atb))
