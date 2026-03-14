# Team 9522 - 2026 REBUILT Match Strategy

## Table of Contents
1. [Robot Capability Assessment](#1-robot-capability-assessment)
2. [Game Mechanics Deep Dive](#2-game-mechanics-deep-dive)
3. [Autonomous Strategy](#3-autonomous-strategy)
4. [Teleop Strategy](#4-teleop-strategy)
5. [Endgame Strategy](#5-endgame-strategy)
6. [Ranking Point Strategy](#6-ranking-point-strategy)
7. [Alliance Selection Strategy](#7-alliance-selection-strategy)
8. [Scouting Priorities](#8-scouting-priorities)
9. [Match-by-Match Adaptation](#9-match-by-match-adaptation)
10. [Driver Practice Plan](#10-driver-practice-plan)

---

## 1. Robot Capability Assessment

### Strengths
| Capability | Details | Strategic Advantage |
|------------|---------|---------------------|
| **Swerve drive** | 4.46 m/s max, 3.0 m/s² accel | Fast field traversal, strafe while shooting |
| **Turret** | ±90° rotation, NEO + 40:1 | Shoot from any robot heading — no need to face the hub |
| **Shoot-on-the-move** | Auto-aim with vision + turret | Drastically reduces cycle time — never stop to aim |
| **Dual-NEO shooter** | 3 speed presets (2700/3300/4200 RPM) | Range flexibility from close to far |
| **Ground intake** | NEO Vortex, deploys to 148° | Fast ground pickup, runs at full duty cycle |
| **112 lbs** | Under the 115 lb limit | Room for additions (climber later) |

### Weaknesses (Honest Assessment)
| Constraint | Impact | Mitigation |
|------------|--------|------------|
| **1.5" too tall for trench** | Cannot use trench shortcut — must path around, adding ~2-3 seconds per cycle | Compensate with shoot-on-the-move to reduce wasted travel time |
| **Fixed hood (55°)** | Cannot optimize launch angle per distance — limited effective shooting range | Identify 2-3 sweet-spot distances and orbit those zones |
| **No climber** | Cannot earn Tower points or contribute to Traversal RP | Must be an elite FUEL scorer to justify alliance pick; plan to add climber before district champs |
| **No beam breaks** | Cannot auto-count balls or auto-feed | Operator must manually manage feeding — practice coordination drills |
| **Single rear camera** | Limited pose estimation, blind spots | Drive with rear facing hub when possible; supplement with odometry |
| **No ball count** | Can't tell when magazine is full/empty | Operator/driver communication is critical |

### Effective Shooting Zones (Fixed 55° Hood)
With a fixed 55° hood, the effective shooting range is determined purely by flywheel speed. Based on the three presets:

| Preset | RPM | Approximate Range | Best Use |
|--------|-----|-------------------|----------|
| **Close** | 2700 | 1.5 - 2.5 m from hub | Right next to hub ramp, highest accuracy |
| **Mid** | 3300 | 2.5 - 4.0 m from hub | Primary scoring zone — balance of speed and accuracy |
| **Far** | 4200 | 4.0 - 6.0 m from hub | Cross-field shots, opportunistic scoring |

**Recommendation:** The **mid-range zone (2.5-4.0 m)** should be the primary operating distance. It offers the best balance of accuracy and cycle time. Close range forces you onto the hub ramp (congested). Far range with a fixed hood has low accuracy.

---

## 2. Game Mechanics Deep Dive

### The Shift System — The Heart of Strategy

The match is divided into phases with alternating hub activation:

```
 0:00          0:20     0:30        0:55        1:20        1:45        2:10    2:40
  |-- AUTO --|--Trans--|--Shift 1--|--Shift 2--|--Shift 3--|--Shift 4--|--End--|
  | Both ON  | Both ON | ONE on    | OTHER on  | ONE on    | OTHER on  | Both  |
```

**Critical rule:** The alliance that scores MORE FUEL in AUTO has their hub go INACTIVE first (Shifts 1 & 3). The alliance that scores LESS in AUTO is ACTIVE first (Shifts 1 & 3).

**This creates a paradox:** Winning AUTO means your hub is off first. This is NOT necessarily bad — it depends on your alliance's cycle time:

| Scenario | If you WIN AUTO | If you LOSE AUTO |
|----------|----------------|------------------|
| Hub active | Shifts 2, 4 + AUTO + Transition + Endgame | Shifts 1, 3 + AUTO + Transition + Endgame |
| Hub inactive | Shifts 1, 3 | Shifts 2, 4 |
| Total active time | ~100 seconds | ~100 seconds |

**Key insight:** Total active time is the same regardless! The difference is WHEN your windows are. Winning AUTO gives you the first active window later (Shift 2 starts at ~1:20 remaining), meaning your alliance has less time to warm up. Losing AUTO gives you the first active window immediately (Shift 1 at ~2:10 remaining).

### What To Do During Inactive Shifts (25 seconds each)

This is where strategy separates good teams from great ones:

1. **Collect FUEL** — Pick up balls during inactive time so you're loaded when your hub activates
2. **Reposition** — Move to optimal shooting position for when your hub turns on
3. **Feed alliance partners** — If a partner is closer to their shot, pass balls
4. **Defense (risky)** — Block opponents from scoring during THEIR active shift, but beware penalties

**For Team 9522:** Use inactive shifts to:
- Collect 3-5 FUEL from the field/depot
- Position at your mid-range sweet spot (2.5-4.0 m from hub)
- Have shooter spun up before your shift starts
- **Be shooting within 1-2 seconds of hub activation**

---

## 3. Autonomous Strategy

### Priority Order for Auto Routines

AUTO is 20 seconds with both hubs active. Every FUEL counts double strategically (counts toward shift scheduling AND is guaranteed to score).

#### Tier 1: Score-and-Collect (Ideal)
1. Start with preloaded FUEL (up to 8)
2. Shoot preloaded FUEL immediately (turret aims while driving)
3. Drive to nearest FUEL cluster
4. Collect 3-5 additional FUEL
5. Shoot collected FUEL before AUTO ends

**Target:** 8-13 FUEL scored in AUTO

#### Tier 2: Score-and-Taxi (Reliable)
1. Start with preloaded FUEL
2. Shoot all preloaded FUEL
3. Taxi out of starting zone for any taxi bonus

**Target:** 5-8 FUEL scored in AUTO

#### Tier 3: Taxi Only (Fallback)
1. Drive out of starting zone
2. Collect FUEL for teleop

**Target:** 0 FUEL scored, but positioned for teleop

### Starting Position Recommendations

Since the turret starts at -90° (stowed right) and the camera faces backward:

| Position | Pros | Cons | Best For |
|----------|------|------|----------|
| **Bottom (near wall)** | Clear path to depot FUEL, less congestion | Farther from center FUEL | Score preload + collect from depot |
| **Center** | Access to center field FUEL grid | Contested by all 6 robots | Score preload + grab center FUEL |
| **Top (near outpost)** | Human player can feed you, less traffic | Farther from hub (may need far shot) | Score preload + receive from human player |

**Recommendation:** Bottom or center start. These give the shortest path to FUEL and back to the mid-range shooting zone.

### Auto Path Design (No Trench Available)

Since we CANNOT go through the trench, our auto paths must route around it:

```
    Hub
     |
     |--- Mid-range shooting zone (2.5-4m)
     |
     |     [TRENCH - CAN'T GO UNDER]
     |
     +--- Route AROUND trench (add ~1.5s to cycle)
     |
     FUEL pickup zone (center field)
```

**Critical:** PathPlanner paths must avoid trench zones entirely. Add waypoints that route around trench walls with at least 0.5m clearance.

---

## 4. Teleop Strategy

### Optimal Cycle Pattern

A "cycle" is: Pick up FUEL → Drive to shooting position → Score FUEL → Repeat.

#### Without Trench Access (Our Reality)
```
Estimated cycle time breakdown:
  Intake FUEL:          2-3 seconds (ground pickup while moving)
  Drive to shot zone:   3-5 seconds (around trench, swerve helps)
  Aim + shoot:          1-2 seconds (turret auto-aims, shoot on the move)
  ────────────────────────────────
  Total cycle:          6-10 seconds per cycle
  Balls per cycle:      3-5 FUEL

  Per 25-second active shift: 2-3 full cycles = 6-15 FUEL scored
  Four active shifts + AUTO + Endgame: 24-60+ FUEL per robot
```

#### Key Optimization: Shoot On The Move

Our turret + swerve + auto-aim combination means we should **almost never stop moving**. The ideal pattern:

1. **Drive toward FUEL** while turret tracks hub
2. **Deploy intake** as you approach FUEL (right bumper)
3. **Drive through FUEL** — intake collects while moving
4. **Arc toward hub** — turret continuously aims
5. **Feed and shoot** while still driving (A button for feed, Y for shoot)
6. **Continue to next FUEL cluster** — never fully stop

This "figure-8" or "orbit" pattern around the mid-range zone maximizes balls per second.

### Field Zones & Pathing

```
  ┌─────────────────────────────────────────────┐
  │                                             │
  │  OUTPOST                            DEPOT   │
  │     ●                                  ●    │
  │                                             │
  │         ┌─────┐          ┌─────┐            │
  │         │TRNCH│   FUEL   │TRNCH│            │
  │         │     │  GRID    │     │            │
  │  HUB ◆  └─────┘          └─────┘   ◆ HUB   │
  │  (BLUE)  ┌─────┐          ┌─────┐  (RED)   │
  │         │TRNCH│   FUEL   │TRNCH│            │
  │         │     │  GRID    │     │            │
  │         └─────┘          └─────┘            │
  │     ●                                  ●    │
  │  DEPOT                          OUTPOST     │
  │                                             │
  └─────────────────────────────────────────────┘
```

**Primary scoring path (avoid trench):**
1. Collect FUEL from your alliance's side of center field
2. Arc around the outside of the trench walls
3. Enter the mid-range zone (2.5-4m from YOUR hub)
4. Shoot while moving through the zone
5. Continue past hub toward next FUEL cluster

**Avoid:**
- Getting stuck on the hub ramp (congested, slow to exit)
- Driving through trench walls (collision, stuck)
- Crossing the full field unnecessarily (wastes 3-4 seconds)

### Shooting Positions Ranked

| Position | Distance to Hub | Congestion | Accuracy | Cycle Time | Rating |
|----------|----------------|------------|----------|------------|--------|
| **Mid-range orbit** | 2.5-4.0 m | Low | High | Fast | Best |
| **Close hub ramp** | 1.0-2.0 m | High | Very High | Slow (exit time) | Good for emptying magazine |
| **Far cross-field** | 5.0-7.0 m | None | Low (fixed hood) | Fast | Emergency only |
| **Near trench wall** | 3.0-5.0 m | Medium | Medium | Medium | Avoid (tight space) |

### Operator Coordination

Without beam breaks, the operator is critical:

| Operator Action | When | Controller Input |
|-----------------|------|-----------------|
| Deploy intake | Approaching FUEL | Right Bumper (hold) |
| Stow intake | Done collecting | Release Right Bumper / D-pad Down |
| Aim turret | Before shooting | D-pad or auto-aim (Left Bumper toggle) |
| Feed to shooter | Shooter spun up + aimed | A Button (hold) |
| Spin up shooter | Approaching shot zone | Y Button |
| Reverse (unjam) | Ball stuck | B Button |

**Recommended flow:**
1. Driver calls "INTAKING" → Operator holds Right Bumper
2. Driver calls "SHOOTING" → Operator presses Y (spin up), waits for green indicator, presses A (feed)
3. Driver calls "JAMMED" → Operator presses B (reverse feed)

---

## 5. Endgame Strategy

### Critical Decision: Both Hubs Active (Last 30 Seconds)

Endgame is the most valuable 30 seconds because BOTH hubs are always active. This means:
- No wasted time during inactive shifts
- Maximum scoring opportunity
- All 6 robots can score simultaneously

### Our Endgame Plan (No Climber)

Since we have **no climbing mechanism**, our endgame is simple: **Keep scoring FUEL for the full 30 seconds.**

While other robots peel off to climb the tower (typically starting at ~45-30 seconds remaining), we keep shooting. This means:
- Less field congestion (opponents are climbing, not competing for FUEL)
- Dedicated access to remaining FUEL on the field
- Potentially 10-20 extra FUEL in endgame alone

### When Alliance Partners Climb

Communicate with alliance before the match:
- "We will score FUEL the entire match including endgame — we cannot climb"
- "You two handle climbing for the Traversal RP"
- "If you need FUEL cleared from near the tower, we can collect and score it"

### Adding a Climber (Future)

A climber should be the **#1 priority upgrade** after the first event:
- Level 1 (10 pts) is achievable with a simple hook mechanism
- Level 2 (20 pts) is worth the effort for Traversal RP contribution
- At 112 lbs, you have 3 lbs of weight budget (or 23 lbs with bumpers)
- Even a basic Level 1 climber makes you significantly more valuable in alliance selection

---

## 6. Ranking Point Strategy

### RP Breakdown Per Match

| RP | Source | Max | Our Contribution |
|----|--------|-----|------------------|
| Win/Tie | Match result | 2 | Full contributor |
| Energized | 100 FUEL total (alliance) | 1 | Primary contributor — our main strength |
| Supercharged | 360 FUEL total (alliance) | 1 | Stretch goal — need elite alliance |
| Traversal | 50 Tower points (alliance) | 1 | **CANNOT contribute** — rely on partners |

### Realistic RP Targets

**Weeks 1-3 (Early Season):**
- **Win RP (2):** Achievable in ~50-60% of matches with strong FUEL scoring
- **Energized RP (1):** Very achievable — 100 FUEL across 3 robots = ~33 each. We should score 40-60+ FUEL per match
- **Supercharged RP (1):** Unlikely early season — 360 FUEL = 120 per robot average. Only top alliances will hit this
- **Traversal RP (1):** Depends entirely on partners — we contribute 0 Tower points

**Target: 2.5-3.5 RP per match average** (competitive for district qualification)

### Maximizing RP Without a Climber

Since we can't contribute to Traversal RP, we MUST over-contribute to Energized RP:

1. **Be so good at scoring FUEL that partners want to pick you**
2. **Target 50+ FUEL per match** — this carries the Energized RP threshold even if partners score few
3. **In alliance selection, pair with climbers** — our FUEL output + their climbing = 4 RP matches

### The RP Math

```
Best realistic match:
  Our FUEL:        50-60
  Partner 1 FUEL:  20-30
  Partner 2 FUEL:  10-20
  Total FUEL:      80-110 → Energized RP likely, Supercharged RP unlikely

  Partner 1 climb: Level 2 (20 pts)
  Partner 2 climb: Level 3 (30 pts)
  Total climb:     50 pts → Traversal RP achieved

  Win:             2 RP
  Energized:       1 RP
  Traversal:       1 RP
  ─────────────────────
  Total:           4 RP (excellent match)
```

---

## 7. Alliance Selection Strategy

### What We Offer
- **Elite FUEL scoring** with turret + shoot-on-the-move
- **Reliable autonomous** FUEL scoring
- **Full match FUEL scoring** (no time lost climbing)
- **Consistent, high-volume FUEL output**

### What We Need In Partners

#### Must-Have (1st Priority Pick)
- **Climber (Level 2+)** — We cannot climb, so partner MUST
- **Reasonable FUEL scoring (20+)** — Don't need a superstar, just consistent

#### Ideal Partner Profile
| Priority | Trait | Why |
|----------|-------|-----|
| 1 | Level 2 or 3 climber | Traversal RP requires 50 tower points |
| 2 | Can score 20+ FUEL | Helps reach Energized RP threshold |
| 3 | Can go under trench | Covers the field zones we can't access quickly |
| 4 | Has intake | Can collect their own FUEL (not dependent on human player only) |

#### Dream Alliance
- **Us (9522):** 50+ FUEL scorer, no climb
- **Partner 1:** 30+ FUEL scorer with Level 3 climb
- **Partner 2:** 20+ FUEL scorer with Level 2 climb
- **Result:** 100+ FUEL (Energized RP) + 50 Tower (Traversal RP) + likely win = 4 RP

### If We Are Alliance Captain (Top 8)
1. **Pick the best climber who can also score 20+ FUEL**
2. Look for Level 2-3 climbers specifically
3. Avoid picking another non-climber — you'll never get Traversal RP

### If We Are Picked (Outside Top 8)
- Emphasize our FUEL scoring volume
- Tell captains: "We score 50+ FUEL and never stop — you and your 2nd pick handle climbing, we handle FUEL"

---

## 8. Scouting Priorities

### What to Scout (Ranked)

| Priority | Data Point | Why |
|----------|------------|-----|
| 1 | **Climber level achieved** | We need climbing partners |
| 2 | **FUEL scored per match** | Quantifies scoring capability |
| 3 | **Can traverse trench?** | Identifies complementary partners |
| 4 | **Auto FUEL scored** | Impacts shift scheduling |
| 5 | **Cycle time** | Separates fast from slow robots |
| 6 | **Reliability / disconnects** | Avoid unreliable partners |
| 7 | **Defense capability** | Useful for Shift strategy |
| 8 | **Human player accuracy** | Outpost FUEL scoring adds up |

### Scouting Form Template

```
Match #: ___  Team #: ___  Alliance: Red / Blue

AUTO:
  [ ] Taxi?
  [ ] FUEL scored: ___
  [ ] Auto path: Bottom / Center / Top

TELEOP:
  [ ] FUEL scored: ___
  [ ] Shooting position: Close / Mid / Far
  [ ] Can go under trench? Y / N
  [ ] Cycle time (est): ___ seconds
  [ ] Consistent? Y / Somewhat / N

ENDGAME:
  [ ] Climb level: None / L1 / L2 / L3
  [ ] Climb time: ___ seconds
  [ ] Climb success rate: Always / Usually / Sometimes / Rarely

NOTES:
  ___________________________________
```

### Key Opponents to Track
- Teams that can score 60+ FUEL AND climb Level 2+ — these are the teams to beat
- Teams with accurate far-range shooters (adjustable hood) — they score during our inactive shifts more effectively
- Fast defensive robots — they can disrupt our orbit pattern

---

## 9. Match-by-Match Adaptation

### Pre-Match Checklist (2 Minutes Before Match)
1. Turret physically positioned at -90° (stowed right, inside frame)
2. Up to 8 FUEL preloaded
3. Confirm auto routine selection with alliance
4. Discuss shift strategy: "When our hub is inactive, we collect. When active, we dump everything"
5. Confirm who is climbing (NOT us)
6. Camera lens clean, vision processing confirmed

### In-Match Decision Framework

#### If We're Winning AUTO (scoring more FUEL):
- Our hub goes inactive Shift 1
- Use Shift 1 (25s) to collect FUEL and reposition
- Be ready to fire immediately when Shift 2 activates
- **Don't panic** — active time is the same either way

#### If We're Losing AUTO:
- Our hub goes active Shift 1
- Start shooting IMMEDIATELY in Shift 1
- Use Shift 2 (inactive) to reload and reposition
- This is slightly advantageous — we get to shoot first while opponents reload

#### If Alliance Partner Breaks Down:
- Shift to "volume scorer" mode — collect more aggressively
- Take wider field coverage to compensate
- Consider whether a 2-robot Energized RP (100 FUEL) is still achievable
- Two robots at ~50 FUEL each can still hit it

#### If We're Behind Late in Match:
- Focus on high-percentage close shots (2700 RPM, near hub)
- Skip optimization — just get balls in
- Don't waste time traveling to collect — shoot what you have

#### If We're Ahead Late in Match:
- Maintain current strategy — don't get conservative
- Consider playing light defense during opponent's active shift (risky — only if comfortable with rules)
- Help partners position for climbing if they need it

### Between Matches
1. Check turret and intake for mechanical issues
2. Review shot accuracy — are we consistently making mid-range?
3. Adjust flywheel speeds if shots are consistently long/short
4. Update scouting data
5. Discuss strategy adjustments with drive team

---

## 10. Driver Practice Plan

### Priority Drills (Pre-Competition)

#### Drill 1: Orbit Shooting (Most Important)
- Set up FUEL at 3 known locations
- Practice driving a smooth arc through the mid-range zone (2.5-4m from hub)
- Score while moving — never stop
- **Goal:** 3 FUEL scored in under 8 seconds per orbit

#### Drill 2: Collect-and-Shoot Cycle
- Start at the far end of the field
- Drive to FUEL, collect with intake
- Route AROUND trench markers
- Score at mid-range
- **Goal:** Full cycle in under 10 seconds

#### Drill 3: Shift Transition
- Practice the inactive→active transition:
  1. During "inactive" practice: collect FUEL, spin up shooter, position at mid-range
  2. On "active" signal: immediately start scoring
  - **Goal:** First ball scored within 2 seconds of hub activation

#### Drill 4: Driver-Operator Coordination
- Practice call-outs: "INTAKING" → "SHOOTING" → "CLEAR"
- Operator practices the Right Bumper → Y → A sequence blindly
- **Goal:** Smooth coordination without looking at each other

#### Drill 5: Recovery from Jam
- Intentionally create jams (double-feed)
- Practice: B button reverse → A button forward → resume shooting
- **Goal:** Clear jam in under 3 seconds

### Practice Match Simulation
Run full 2:40 practice matches with shift timing:
- 0:00-0:20: AUTO (both hubs on) → Score preloaded FUEL
- 0:20-0:30: Transition → Collect
- 0:30-0:55: Shift 1 → Practice for BOTH active and inactive scenarios
- Continue through all shifts
- 2:10-2:40: Endgame → Maximum FUEL scoring

---

## Appendix A: Quick Reference Card (Print for Driver Station)

```
╔══════════════════════════════════════════════════╗
║  TEAM 9522 - REBUILT QUICK REFERENCE             ║
╠══════════════════════════════════════════════════╣
║                                                  ║
║  SHOOTER PRESETS:                                ║
║    Close: 2700 RPM  (1.5-2.5m from hub)         ║
║    Mid:   3300 RPM  (2.5-4.0m from hub) ← USE   ║
║    Far:   4200 RPM  (4.0-6.0m from hub)         ║
║                                                  ║
║  SHIFT TIMING:                                   ║
║    AUTO:    0:00-0:20  BOTH ON                   ║
║    Trans:   0:20-0:30  BOTH ON                   ║
║    Shift 1: 0:30-0:55  (depends on AUTO result)  ║
║    Shift 2: 0:55-1:20  (opposite of Shift 1)     ║
║    Shift 3: 1:20-1:45  (same as Shift 1)         ║
║    Shift 4: 1:45-2:10  (same as Shift 2)         ║
║    Endgame: 2:10-2:40  BOTH ON                   ║
║                                                  ║
║  INACTIVE SHIFT PLAN:                            ║
║    1. Collect FUEL                               ║
║    2. Reposition to mid-range                    ║
║    3. Spin up shooter                            ║
║    4. Be ready to fire at shift change           ║
║                                                  ║
║  WE DO NOT CLIMB — KEEP SCORING FUEL             ║
║                                                  ║
╚══════════════════════════════════════════════════╝
```

## Appendix B: Known Limitations & Workarounds

| Limitation | Workaround | Long-term Fix |
|------------|------------|---------------|
| Can't go under trench | Route around — adds ~2s per cycle | Lower robot (different swerve modules) or accept the trade-off |
| Fixed hood (55°) | Stay in mid-range zone where 55° works best | Add motorized hood for range flexibility |
| No climber | Score FUEL all match, rely on partners for Tower RP | Build and add a Level 2 climber before champs |
| No beam breaks | Operator manually manages feeding | Wire up beam breaks at competition if time allows |
| Single rear camera | Keep hub in camera FOV when possible | Add front camera for full-field pose estimation |
| No ball count | Driver/operator verbal coordination | Add beam break counter to dashboard |

## Appendix C: Upgrade Priority List (Between Events)

| Priority | Upgrade | Impact | Difficulty |
|----------|---------|--------|------------|
| 1 | **Add Level 1-2 climber** | Enables Traversal RP contribution — dramatically increases alliance value | High |
| 2 | **Add beam break sensors** | Automates feeding, enables ball counting | Low |
| 3 | **Add front camera** | Better pose estimation, fewer blind spots | Medium |
| 4 | **Motorized hood** | Extends effective shooting range | High |
| 5 | **Lower robot height** | Enables trench traversal — faster cycles | Very High (requires frame changes) |

---

*Strategy document prepared for Team 9522 — 2026 REBUILT season. Update after each event with match data and lessons learned.*
