import json, os, math
ROBOT_BUFFER = max(0.7112/2, 0.6604/2)
FIELD_W, FIELD_H = 16.54, 8.052
tw_dist_x = 120.0*0.0254 + 23.5*0.0254
tw_dist_y = 73.0*0.0254 + 23.5*0.0254 + 6*0.0254
tw_cx, tw_cy = 8.27, 4.035
tw_hw, tw_hh = 53*0.0254/2, 12*0.0254/2
obstacles = []
for n,dx,dy in [("BotLeft",-1,-1),("BotRight",1,-1),("TopLeft",-1,1),("TopRight",1,1)]:
    obstacles.append(("TrenchWall-"+n, tw_cx+dx*tw_dist_x, tw_cy+dy*tw_dist_y, tw_hw, tw_hh))
obstacles.append(("BlueHub", 4.5974, 4.034536, 47*0.0254/2, 47*0.0254/2))
obstacles.append(("RedHub", 11.938, 4.034536, 47*0.0254/2, 47*0.0254/2))
obstacles.append(("BluePole", 42*0.0254, 159*0.0254, 2*0.0254/2, 47*0.0254/2))
obstacles.append(("RedPole", (651-42)*0.0254, 170*0.0254, 2*0.0254/2, 47*0.0254/2))
print("OBSTACLE MAP:")
for n,cx,cy,hw,hh in obstacles:
    print(f"  {n}: center=({cx:.3f},{cy:.3f}) nogo=[{cx-hw-ROBOT_BUFFER:.3f},{cx+hw+ROBOT_BUFFER:.3f}]x[{cy-hh-ROBOT_BUFFER:.3f},{cy+hh+ROBOT_BUFFER:.3f}]")
print(f"  Field walls: safe x=[{ROBOT_BUFFER:.3f},{FIELD_W-ROBOT_BUFFER:.3f}] y=[{ROBOT_BUFFER:.3f},{FIELD_H-ROBOT_BUFFER:.3f}]")
print()
def chk(px,py,lbl):
    issues=[]
    if py<ROBOT_BUFFER: issues.append(f"  X {lbl}({px:.3f},{py:.3f}): below field (min y={ROBOT_BUFFER:.3f})")
    if py>FIELD_H-ROBOT_BUFFER: issues.append(f"  X {lbl}({px:.3f},{py:.3f}): above field (max y={FIELD_H-ROBOT_BUFFER:.3f})")
    if px<ROBOT_BUFFER: issues.append(f"  X {lbl}({px:.3f},{py:.3f}): left of field (min x={ROBOT_BUFFER:.3f})")
    if px>FIELD_W-ROBOT_BUFFER: issues.append(f"  X {lbl}({px:.3f},{py:.3f}): right of field")
    for on,cx,cy,hw,hh in obstacles:
        if abs(px-cx)<hw+ROBOT_BUFFER and abs(py-cy)<hh+ROBOT_BUFFER:
            issues.append(f"  X {lbl}({px:.3f},{py:.3f}): HIT {on} (need {ROBOT_BUFFER:.3f}m clear)")
    return issues
def bezier(p0,p1,p2,p3,n=30):
    pts=[]
    for i in range(n+1):
        t=i/n; m=1-t
        pts.append((m**3*p0[0]+3*m**2*t*p1[0]+3*m*t**2*p2[0]+t**3*p3[0], m**3*p0[1]+3*m**2*t*p1[1]+3*m*t**2*p2[1]+t**3*p3[1]))
    return pts
pdir=r"c:\team9522_2026\src\main\deploy\pathplanner\paths"
total=0
for fn in sorted(os.listdir(pdir)):
    if not fn.endswith('.path'): continue
    with open(os.path.join(pdir,fn)) as f: d=json.load(f)
    wp=d['waypoints']; issues=[]
    for i,w in enumerate(wp):
        issues.extend(chk(w['anchor']['x'],w['anchor']['y'],f"wp{i}"))
        if w.get('prevControl'): issues.extend(chk(w['prevControl']['x'],w['prevControl']['y'],f"wp{i}prev"))
        if w.get('nextControl'): issues.extend(chk(w['nextControl']['x'],w['nextControl']['y'],f"wp{i}next"))
    for i in range(len(wp)-1):
        a=wp[i]; b=wp[i+1]
        p0=(a['anchor']['x'],a['anchor']['y'])
        p1=(a['nextControl']['x'],a['nextControl']['y']) if a.get('nextControl') else p0
        p2=(b['prevControl']['x'],b['prevControl']['y']) if b.get('prevControl') else (b['anchor']['x'],b['anchor']['y'])
        p3=(b['anchor']['x'],b['anchor']['y'])
        for j,(cx,cy) in enumerate(bezier(p0,p1,p2,p3)):
            issues.extend(chk(cx,cy,f"seg{i}t{j/30:.2f}"))
    seen=set()
    unique=[]
    for iss in issues:
        obs=iss.split("HIT ")[-1].split(" (")[0] if "HIT" in iss else iss.split("):")[1].strip()[:20]
        if obs not in seen: seen.add(obs); unique.append(iss)
    print(f"{'X' if unique else 'OK'} {fn}")
    for u in unique: print(u)
    total+=len(unique)
print(f"\nTOTAL ISSUES: {total}")
