# viz_autovp.py
# -*- coding: utf-8 -*-
import io, math
from collections import defaultdict

import matplotlib.pyplot as plt
import networkx as nx
import simpy
from PIL import Image
import random

from props import (
    EDGE_TRAVEL_TIME, ARRIVAL_GAP, REPLAN_DELAY, PARK_TIME_RANGE,
    CONNECT_TIME, DT, FLOOR_MAP_B1, FLOOR_MAP_B2
)

TRAV = set("0GR-|")


# ===== 예약 테이블 =====
class ReservationTable:
    def __init__(self):
        from collections import defaultdict
        self.book_edge = defaultdict(list)
        self.book_node = defaultdict(list)
        self.book_conn = []
    @staticmethod
    def ek(u,v): return tuple(sorted((u,v)))
    @staticmethod
    def _ok(arr,t0,t1):
        for (a,b,_) in arr:
            if not (t1<=a or b<=t0): return False
        return True
    def can_use_edge(self,fid,ek,t0,t1): return self._ok(self.book_edge[(fid,ek)],t0,t1)
    def can_use_node(self,fid,node,t0,t1): return self._ok(self.book_node[(fid,node)],t0,t1)
    def can_use_connector(self,t0,t1): return self._ok(self.book_conn,t0,t1)
    def commit_edge(self,fid,ek,t0,t1,vid): self.book_edge[(fid,ek)].append((t0,t1,vid))
    def commit_node(self,fid,node,t0,t1,vid): self.book_node[(fid,node)].append((t0,t1,vid))
    def commit_connector(self,t0,t1,vid): self.book_conn.append((t0,t1,vid))

# ===== 관제/차량 =====
class ControlTower:
    def __init__(self, env, floors):
        self.env = env
        self.floors = floors
        self.resv  = ReservationTable()
        self.traj  = {"B1": [], "B2": []}
        self.occ   = {"B1": defaultdict(list), "B2": defaultdict(list)}
        self.claimed = {"B1": set(), "B2": set()}
        # 스팟별 락 (SimPy Resource)
        self.spot_locks = {"B1": {}, "B2": {}}

        for fid, info in floors.items():
            G = info["G"]
            for n in find_nodes(G, "R"):
                self.occ[fid][n].append((0.0, math.inf))

    # 스팟 락 getter
    def get_spot_lock(self, fid, spot):
        Ls = self.spot_locks[fid]
        if spot not in Ls:
            Ls[spot] = simpy.Resource(self.env, capacity=1)
        return Ls[spot]

    # 후보 스팟
    def spot_candidates(self, fid):
        G = self.floors[fid]["G"]
        now = self.env.now
        out = []
        for sp in find_nodes(G, "G"):
            if sp in self.claimed[fid]:
                continue
            if self.is_occupied_now(fid, sp, now):
                continue
            out.append(sp)
        return out

    # 글로벌 입구/게이트 접근자
    @property
    def B1_global_entr(self):
        return self.floors["B1"]["entr_top"]

    @property
    def B1_bottom_gate(self):
        return self.floors["B1"]["gate_bot"]

    # @property
    # def B2_top_entr(self):
    #     return self.floors["B2"]["entr_top"]
    
    @property
    def B1_top_exit(self):
        return self.floors["B1"]["exit_top"]
    @property
    def B1_bottom_exit(self):
        return self.floors["B1"]["exit_bot"]
    @property
    def B2_top_entr(self):
        return self.floors["B2"]["entr_top"]
    @property
    def B2_top_exit(self):
        return self.floors["B2"]["exit_top"]

    # 입차/출차용 복합 경로 생성 (세그먼트 리스트)
    # 반환 형식: [(fid, u, v, dur), ...] + 커넥터는 ( "XFER", None, None, CONNECT_TIME ) 형태
    def build_composite_path_to_spot(self, fid_dest, spot):
        # 목적지가 B1인 경우: B1(global_entr) → B1(spot)
        if fid_dest == "B1":
            G1 = self.floors["B1"]["G"]
            SG1 = build_route_subgraph(G1, src=None, dst=spot)
            p1  = astar_path_on(SG1, self.B1_global_entr, spot)
            if not p1: return None
            segs1 = [("B1", u, v, G1.edges[u, v].get("w", EDGE_TRAVEL_TIME)) for u, v in zip(p1[:-1], p1[1:])]
            return segs1

        # 목적지가 B2인 경우:
        # 1) B1(global_entr) → B1(bottom_gate)
        G1 = self.floors["B1"]["G"]
        SG1 = build_route_subgraph(G1, src=None, dst=self.B1_bottom_gate)
        p1  = astar_path_on(SG1, self.B1_global_entr, self.B1_bottom_gate)
        if not p1: return None
        segs1 = [("B1", u, v, G1.edges[u, v].get("w", EDGE_TRAVEL_TIME)) for u, v in zip(p1[:-1], p1[1:])]

        # 2) [커넥터] B1(bottom_gate) → B2(top_entr)
        xfer = [("XFER", None, None, CONNECT_TIME)]

        # 3) B2(top_entr) → B2(spot)
        G2 = self.floors["B2"]["G"]
        SG2 = build_route_subgraph(G2, src=None, dst=spot)
        p2  = astar_path_on(SG2, self.B2_top_entr, spot)
        if not p2: return None
        segs2 = [("B2", u, v, G2.edges[u, v].get("w", EDGE_TRAVEL_TIME)) for u, v in zip(p2[:-1], p2[1:])]

        return segs1 + xfer + segs2

    def build_composite_path_to_exit(self, fid_src, spot):
        # B1에서 출차: spot → B1 우상단 출구
        if fid_src == "B1":
            G1 = self.floors["B1"]["G"]
            SG1 = build_route_subgraph(G1, src=spot, dst=self.B1_top_exit)
            p1  = astar_path_on(SG1, spot, self.B1_top_exit)
            if not p1: return None
            segs1 = [("B1", u, v, G1.edges[u, v].get("w", EDGE_TRAVEL_TIME)) for u, v in zip(p1[:-1], p1[1:])]
            return segs1

        # B2에서 출차:
        #   1) B2 spot → B2 우상단 출구
        G2 = self.floors["B2"]["G"]
        SG2 = build_route_subgraph(G2, src=spot, dst=self.B2_top_exit)
        p2  = astar_path_on(SG2, spot, self.B2_top_exit)
        if not p2: return None
        segs2 = [("B2", u, v, G2.edges[u, v].get("w", EDGE_TRAVEL_TIME)) for u, v in zip(p2[:-1], p2[1:])]

        #   2) [커넥터] B2(우상 출구) → B1(우하 출구쪽 진입점)
        xfer = [("XFER", None, None, CONNECT_TIME)]

        #   3) B1 우하단 0 → B1 우상단 출구
        G1 = self.floors["B1"]["G"]
        SG1 = build_route_subgraph(G1, src=self.B1_bottom_exit, dst=self.B1_top_exit)
        p1  = astar_path_on(SG1, self.B1_bottom_exit, self.B1_top_exit)
        if not p1: return None
        segs1 = [("B1", u, v, G1.edges[u, v].get("w", EDGE_TRAVEL_TIME)) for u, v in zip(p1[:-1], p1[1:])]
        return segs2 + xfer + segs1

    # 스팟 선택 (두 층 후보 중 총 소요시간 최소)
    def pick_spot(self):
        best = None
        best_time = float('inf')
        for fid in ("B1", "B2"):
            G = self.floors[fid]["G"]
            for sp in self.spot_candidates(fid):
                comp = self.build_composite_path_to_spot(fid, sp)
                if not comp: 
                    continue
                total = sum(d for _,_,_,d in comp)
                if total < best_time:
                    best_time = total
                    best = (fid, sp, total)
        return best

    # 예약 수행(복합 경로). 성공 시 시각화용 traj 기록까지.
    def reserve_composite(self, comp, vid, exiting=False):
        t = self.env.now
        # 1) 가능성 체크
        for (fid, u, v, dur) in comp:
            t0, t1 = t, t + dur
            if fid == "XFER":
                if not self.resv.can_use_connector(t0, t1):
                    return None
            else:
                ek = ReservationTable.ek(u, v)
                if not self.resv.can_use_edge(fid, ek, t0, t1):
                    return None
                # 목적 노드 점유 (도착 직전/직후 margin)
                node_margin = 0.2 * dur
                if not self.resv.can_use_node(fid, v, t1 - node_margin, t1 + node_margin):
                    return None
            t = t1

        # 2) 커밋 + traj 기록
        t = self.env.now
        for (fid, u, v, dur) in comp:
            t0, t1 = t, t + dur
            if fid == "XFER":
                self.resv.commit_connector(t0, t1, vid)
                # 시각화: 점프(렌더 생략). 원하면 연결 애니메이션 추가 가능.
            else:
                ek = ReservationTable.ek(u, v)
                self.resv.commit_edge(fid, ek, t0, t1, vid)
                node_margin = 0.2 * dur
                self.resv.commit_node(fid, v, t1 - node_margin, t1 + node_margin, vid)
                # traj 기록
                G = self.floors[fid]["G"]
                ui, uj = G.nodes[u]["ij"]; vi, vj = G.nodes[v]["ij"]
                # 커밋 루프 내부 (fid != "XFER" 인 경우) ─ 기존 append 코드에 'exiting' 추가
                self.traj[fid].append({
                    "vid": vid, "t0": t0, "t1": t1,
                    "ui": ui, "uj": uj, "vi": vi, "vj": vj,
                    "exiting": exiting 
                })
            t = t1
        return True

    # 점유 구간 관리
    def add_occupancy(self, fid, spot, t0, t1):
        self.occ[fid][spot].append((t0, t1))

    def is_occupied_now(self, fid, spot, t_now):
        for (a, b) in self.occ[fid].get(spot, []):
            if a <= t_now <= b:
                return True
        return False

    def try_claim(self, fid, spot, t_now):
        if self.is_occupied_now(fid, spot, t_now):
            return False
        if spot in self.claimed[fid]:
            return False
        self.claimed[fid].add(spot)
        return True

    def unclaim(self, fid, spot):
        self.claimed[fid].discard(spot)

class Vehicle:
    def __init__(self, env, vid, tower, logs):
        self.env = env
        self.vid = vid
        self.tower = tower
        self.logs = logs
        env.process(self.run())

    def L(self, msg):
        line = f"[T={self.env.now:6.1f}] car#{self.vid:<2d} {msg}"
        self.logs.append(line)
        print(line)

    def run(self):
        lock_req = None 
        locked_fid = None
        locked_spot = None

        try:
            # 1) 스팟 선택 루프
            pick = self.tower.pick_spot()
            if not pick:
                self.L("배정 가능한 스팟 없음 → 종료"); return
            fid, spot, _ = pick
            self.L(f"배정 층={fid}, 스팟={spot}")

            while True:
                # 스팟 락 먼저 요청 (대기 없이 즉시 or 큐에서 순서대로 보장)
                lock = self.tower.get_spot_lock(fid, spot)
                lock_req = lock.request()
                yield lock_req    # 용량=1 → 동시에 두 대가 여기서 동시 통과 불가
                locked_fid, locked_spot = fid, spot

                # 이후 선점(soft) — 중복 pick 차단(빠른 재탐색 유도)
                if not self.tower.try_claim(fid, spot, self.env.now):
                    self.L("스팟 선점 실패 → 재탐색")
                    # 락 해제
                    lock.release(lock_req); lock_req = None; locked_fid = locked_spot = None
                    new_pick = self.tower.pick_spot()
                    if not new_pick:
                        self.L("재배정 불가 → 대기 후 재시도")
                        yield self.env.timeout(REPLAN_DELAY); continue
                    fid, spot, _ = new_pick
                    self.L(f"재배정 층={fid}, 스팟={spot}")
                    continue

                # 2) 입차 경로(복합) & 예약
                comp_in = self.tower.build_composite_path_to_spot(fid, spot)
                if not comp_in:
                    self.L("입차 경로 없음 → 선점 해제 후 재시도")
                    self.tower.unclaim(fid, spot)
                    # 락 해제
                    lock.release(lock_req); lock_req = None; locked_fid = locked_spot = None
                    yield self.env.timeout(REPLAN_DELAY); continue

                ok = self.tower.reserve_composite(comp_in, self.vid)
                if not ok:
                    self.L("입차 예약 충돌 → 선점 해제 후 대기")
                    self.tower.unclaim(fid, spot)
                    # 락 해제
                    lock.release(lock_req); lock_req = None; locked_fid = locked_spot = None
                    yield self.env.timeout(REPLAN_DELAY); continue

                # 3) 이동
                travel = sum(d for _,_,_,d in comp_in)
                self.L(f"입차 이동 시작 (예상 {travel:.1f}s)")
                yield self.env.timeout(travel)
                break

            # 4) 주차
            park_start = self.env.now
            dwell = random.randint(*PARK_TIME_RANGE)
            self.L(f"체류 시작 ({dwell}s)")
            G = self.tower.floors[fid]["G"]
            si, sj = G.nodes[spot]["ij"]
            self.tower.traj[fid].append({"vid": self.vid, "t0": park_start, "t1": park_start + dwell,
                                         "ui": si, "uj": sj, "vi": si, "vj": sj})
            self.tower.add_occupancy(fid, spot, park_start, park_start + dwell)
            yield self.env.timeout(dwell)

            # 5) 출차(복합) & 예약
            self.L("출차 요청")
            while True:
                comp_out = self.tower.build_composite_path_to_exit(fid, spot)
                if not comp_out:
                    # 경로가 당장 없을 때도 정지 세그먼트를 추가하여 점이 유지되도록
                    wait_t0 = self.env.now
                    wait_dt = REPLAN_DELAY
                    # 시각화용 정지 세그먼트(주차칸에 그대로)
                    G = self.tower.floors[fid]["G"]
                    si, sj = G.nodes[spot]["ij"]
                    self.tower.traj[fid].append({
                        "vid": self.vid, "t0": wait_t0, "t1": wait_t0 + wait_dt,
                        "ui": si, "uj": sj, "vi": si, "vj": sj, "exiting": True 
                    })
                    # 점유도 동일 구간 연장
                    self.tower.add_occupancy(fid, spot, wait_t0, wait_t0 + wait_dt)

                    self.L("출차 경로 없음 → 주차칸 대기 후 재시도")
                    yield self.env.timeout(wait_dt)
                    continue

                ok = self.tower.reserve_composite(comp_out, self.vid, exiting=True)
                if not ok:
                    # 예약이 아직 안 잡히면 같은 방식으로 정지 세그먼트/점유 연장
                    wait_t0 = self.env.now
                    wait_dt = REPLAN_DELAY
                    G = self.tower.floors[fid]["G"]
                    si, sj = G.nodes[spot]["ij"]
                    self.tower.traj[fid].append({
                        "vid": self.vid, "t0": wait_t0, "t1": wait_t0 + wait_dt,
                        "ui": si, "uj": sj, "vi": si, "vj": sj
                    })
                    self.tower.add_occupancy(fid, spot, wait_t0, wait_t0 + wait_dt)

                    self.L("출차 예약 충돌(앞 도로 점유) → 주차칸 대기")
                    yield self.env.timeout(wait_dt)
                    continue

                travel = sum(d for _, _, _, d in comp_out)
                self.L(f"출차 이동 시작 (예상 {travel:.1f}s)")
                yield self.env.timeout(travel)
                break

            self.tower.unclaim(fid, spot)
            self.L("글로벌 입구 도착 / 스팟 해제")

        finally:
            # 어떤 이유로든 종료 시 락이 잡혀 있으면 반드시 해제
            if lock_req is not None and locked_fid is not None and locked_spot is not None:
                self.tower.get_spot_lock(locked_fid, locked_spot).release(lock_req)

# ===== 도착 프로세스 =====
def arrival(env, tower, logs):
    vid = 0
    while True:
        vid += 1
        Vehicle(env, vid, tower, logs)
        yield env.timeout(ARRIVAL_GAP)

# ===== 결합 GIF 렌더 =====
def render_combined_gif(floors, traj_dict, occ_dict, sim_time, dt, out_path):
    G1 = floors["B1"]["G"]; G2 = floors["B2"]["G"]
    coords1 = [G1.nodes[n]["ij"] for n in G1.nodes]
    coords2 = [G2.nodes[n]["ij"] for n in G2.nodes]
    max_i1 = max(i for i, j in coords1)
    max_j1 = max(j for i, j in coords1)
    max_i2 = max(i for i, j in coords2)
    max_j2 = max(j for i, j in coords2)

    offset_y = max_i1 + 4  # B2를 아래로 내리는 오프셋
    frames = []
    t = 0.0

    while t <= sim_time + 1e-6:
        fig = plt.figure(figsize=(8, 12))
        ax = plt.gca()
        ax.set_aspect("equal")
        ax.set_xlim(-0.5, max(max_j1, max_j2) + 1.5)
        ax.set_ylim(max_i1 + max_i2 + 8, -0.5)
        ax.set_xticks([]); ax.set_yticks([])
        ax.set_title(f"Combined Simulation (t={int(t)}s)")

        def draw_floor(G, occ, traj, offset, label):
            # 타일
            for n in G.nodes:
                i, j = G.nodes[n]["ij"]; base = G.nodes[n]["ch"]
                i += offset
                # 점유 상태(시간 기반)
                occupied_now = False
                if base in {"G","R"}:
                    for (a,b) in occ.get(n, []):
                        if a <= t <= b: occupied_now = True; break
                chp = "R" if occupied_now else base
                if chp == "0": fc=(0.82,0.86,1.00)
                elif chp == "G": fc=(0.85,1.00,0.85)
                elif chp == "R": fc=(1.00,0.85,0.85)
                else: fc=(1,1,1)
                ax.add_patch(plt.Rectangle((j,i),1,1,facecolor=fc,edgecolor='0.85',linewidth=0.8))
                ax.text(j+0.5,i+0.55,base,ha="center",va="center",fontsize=8,alpha=0.6)

            # 간선 (G/R에 닿는 간선은 가독성을 위해 숨김)
            for u,v in G.edges():
                cu,cv=G.nodes[u]["ch"],G.nodes[v]["ch"]
                if cu in {"G","R"} or cv in {"G","R"}: continue
                ui,uj=G.nodes[u]["ij"]; vi,vj=G.nodes[v]["ij"]
                ui+=offset; vi+=offset
                ax.plot([uj+0.5,vj+0.5],[ui+0.5,vi+0.5],linewidth=1.2,alpha=0.35,color="k")

            # 차량
            for seg in traj:
                t0,t1=seg["t0"],seg["t1"]
                if t0 <= t <= t1:
                    ui,uj,vi,vj = seg["ui"],seg["uj"],seg["vi"],seg["vj"]
                    ui+=offset; vi+=offset
                    a = (t-t0)/max(1e-9,(t1-t0))
                    ci=ui+a*(vi-ui); cj=uj+a*(vj-uj)
                    # 색상 선택: 출차 플래그면 빨강, 아니면 파랑
                    is_exiting = bool(seg.get("exiting", False))
                    face = (1.00, 0.30, 0.30) if is_exiting else (0.20, 0.40, 1.00)
                    circ = plt.Circle((cj+0.5, ci+0.5), 0.28, alpha=0.95,
                                    facecolor=face, edgecolor="black", linewidth=0.6)
                    ax.add_patch(circ)
                    ax.text(cj+0.5, ci+0.2, f"{seg['vid']}", ha='center', va='top', fontsize=8)

            ax.text(0, offset-1.5, f"Floor {label}", fontsize=12, weight='bold', color='black')

        draw_floor(G1, occ_dict["B1"], traj_dict["B1"], 0, "B1")
        draw_floor(G2, occ_dict["B2"], traj_dict["B2"], offset_y, "B2")

        buf = io.BytesIO()
        plt.savefig(buf, format="png", bbox_inches="tight")
        buf.seek(0)
        img = Image.open(buf).convert("RGBA").copy().convert("P", palette=Image.ADAPTIVE)
        buf.close(); plt.close(fig)
        frames.append(img)
        t += dt

    frames[0].save(out_path, save_all=True, append_images=frames[1:], duration=int(dt*1000), loop=0)
    print(f"[ok] saved {out_path}")


# ===== 그래프/유틸 =====
def build_grid_graph(lines):
    H = len(lines); W = max(len(r) for r in lines)
    lines = [r + " " * (W - len(r)) for r in lines]
    G = nx.Graph()
    nid = lambda i,j: f"N{i}_{j}"
    for i in range(H):
        for j in range(W):
            c = lines[i][j]
            if c in TRAV:
                G.add_node(nid(i,j), ch=c, ij=(i,j))

    def can_connect(ch_from, ch_to, direction):
        if direction == 'R':
            if ch_from in {'G','R'} and ch_to == '-': return True
            if ch_from == '-' and ch_to in {'G','R','-','0','|'}: return True
            if ch_from == '0' and ch_to == '-': return True
            if ch_from == '|' and ch_to == '-': return True
            return False
        if direction == 'D':
            if ch_from == '|' and ch_to in {'|','0'}: return True
            if ch_from == '0' and ch_to == '|': return True
            return False
        return False

    for i in range(H):
        for j in range(W):
            ch = lines[i][j]
            if ch not in TRAV: continue
            u = nid(i,j)
            if j+1 < W:
                chR = lines[i][j+1]
                if chR in TRAV and can_connect(ch, chR, 'R'):
                    G.add_edge(u, nid(i,j+1), w=EDGE_TRAVEL_TIME)
            if i+1 < H:
                chD = lines[i+1][j]
                if chD in TRAV and can_connect(ch, chD, 'D'):
                    G.add_edge(u, nid(i+1,j), w=EDGE_TRAVEL_TIME)
    return G

def find_nodes(G, ch):
    return [n for n,d in G.nodes(data=True) if d.get("ch")==ch]

# ===== 출입구 선택기 =====
def _pick_zero(Gx, row="top", col="left"):
    zs = [(n, Gx.nodes[n]["ij"]) for n in find_nodes(Gx,"0")]
    if not zs: return None
    rsel = min if row=="top" else max
    target_r = rsel(r for _,(r,_) in zs)
    cand = [(n,ij) for n,ij in zs if ij[0]==target_r]
    cand.sort(key=(lambda x: x[1][1]) if col=="left" else (lambda x: -x[1][1]))
    return cand[0][0]

def build_floors_from_single_map(b1, b2):
    G1 = build_grid_graph(b1); G2 = build_grid_graph(b2)
    floors = {
        "B1": {
            "G": G1,
            "entr_top": _pick_zero(G1,"top","left"),
            "gate_bot": _pick_zero(G1,"bottom","left"),
            "exit_top": _pick_zero(G1,"top","right"),
            "exit_bot": _pick_zero(G1,"bottom","right"),
        },
        "B2": {
            "G": G2,
            "entr_top": _pick_zero(G2,"top","left"),
            "exit_top": _pick_zero(G2,"top","right"),
        }
    }
    return floors

def build_route_subgraph(G, src=None, dst=None):
    base_allow={'0','-','|'}
    allowed=set(n for n,d in G.nodes(data=True) if d['ch'] in base_allow)
    if src is not None: allowed.add(src)
    if dst is not None: allowed.add(dst)
    return G.subgraph(allowed).copy()

def astar_path_on(Gx, s, t):
    def xy(n): return Gx.nodes[n]["ij"]
    def h(a,b):
        (ai,aj),(bi,bj)=xy(a),xy(b); return abs(ai-bi)+abs(aj-bj)
    try:
        return nx.astar_path(Gx, s, t, heuristic=h, weight=lambda u,v,ed: ed.get("w", EDGE_TRAVEL_TIME))
    except nx.NetworkXNoPath:
        return None
    
# 편의 함수 (러너에서 쓰기 좋게)
def make_world(env):
    floors = build_floors_from_single_map(FLOOR_MAP_B1, FLOOR_MAP_B2)
    tower = ControlTower(env, floors)
    return floors, tower
